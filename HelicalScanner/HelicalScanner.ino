#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <math.h>           // floorf

// ───────────────────────── 핀 배치 ─────────────────────────
#define ENC_A     2     // INT0  (엔코더 A)
#define ENC_B     3     // INT1  (엔코더 B)
#define DC_IN3    4     // L298N IN3 (방향)
#define DC_IN4    5     // L298N IN4 (방향)
#define DC_ENB    6     // L298N ENB (PWM, Timer0)

#define TFMINI_RX 9     // Arduino RX  <- TFmini TX
#define TFMINI_TX 8     // Arduino TX  -> TFmini RX

#define STP_IN1   10
#define STP_IN2   11
#define STP_IN3   12
#define STP_IN4   13

SoftwareSerial tfSerial(TFMINI_RX, TFMINI_TX);
AccelStepper tiltStepper(AccelStepper::HALF4WIRE, STP_IN1, STP_IN3, STP_IN2, STP_IN4);

// ──────────────────── 턴테이블 (속도 PID) ───────────────────
const float COUNTS_PER_TABLE_REV = 1317.0f;   // ★UI 에서 맞춘 cpr 과 반드시 동일하게! (pan 정렬값)
                                              //  이제 tilt 위상도 이 값으로 계산되니 정확해야 함.
const float TABLE_RPS            = 0.50f;      // 목표 회전수 [rev/s]  (안정 우선값)
const float TARGET_CPS           = COUNTS_PER_TABLE_REV * TABLE_RPS;  // 목표 속도 [counts/s]
const float DEG_PER_COUNT        = 360.0f / COUNTS_PER_TABLE_REV;

float Kp = 0.045f, Ki = 0.180f, Kd = 0.000f;  // 시작점 (튜닝 필요)
const int   PWM_MAX        = 180;
const int   PWM_MIN_KICK   = 45;               // deadband 보상: 최소 구동 PWM
const float INTEGRAL_MAX   = 4000.0f;
const int   MOTOR_DIRECTION   = 1;             // 반대로 돌면 -1
const int   ENCODER_DIRECTION = 1;             // 회전은 정방향인데 pan 이 감소하면 -1

// ──────────────────── LiDAR tilt (pan 종속 연속 왕복) ──────────
//  tilt 위상을 '시간'이 아니라 '엔코더 바퀴수'에 종속시킨다:
//    phase = panRevs * TILT_CYCLES_PER_REV    (panRevs = encCount / COUNTS_PER_TABLE_REV)
//  => 23 pan바퀴마다 tilt 가 정확히 20사이클 => 실제 RPS/PID 와 무관하게 패턴 반복이 정확.
//     (시간 기반(TILT_SWING_HZ)은 RPS 가 0.50 에서 조금만 어긋나도 위상이 누적으로 밀렸음)
//  precession 은 20/23 비정수 비율이라 그대로 유지.
const long  TILT_MIN_STEPS      = 0;
const long  TILT_MAX_STEPS      = 512;                  // 45° (half-step, 4096 step/rev)
const float TILT_DEG_PER_STEP   = 45.0f / (float)TILT_MAX_STEPS;
const int   TILT_DIRECTION      = 1;                    // 위/아래 반대면 -1
const float TILT_CYCLES_PER_REV = 20.0f / 23.0f;        // pan 1회전당 tilt 사이클 수 (= 위상비)
//  최대속도는 정상상태 peak( = TABLE_RPS * TILT_CYCLES_PER_REV * 2 * MAX_STEPS ) 위로 여유.
//  TABLE_RPS=0.5 -> peak ~445 step/s. 700 이면 추종 여유 OK. (table 더 빠르게 하면 같이 올릴 것)
const float TILT_MAX_SPEED_SPS  = 700.0f;
const float TILT_ACCEL_SPS2     = 2500.0f;

// ─────────────────── 엔코더 (인터럽트 + 레지스터) ───────────
// 4x 쿼드러처 상태천이 LUT : index = (이전상태<<2) | 현재상태
static const int8_t QDEC[16] = {  0, -1,  1,  0,
                                  1,  0,  0, -1,
                                 -1,  0,  0,  1,
                                  0,  1, -1,  0 };
volatile long    encCount = 0;
volatile uint8_t encState = 0;

// Uno 에서 D2=PD2(bit2), D3=PD3(bit3) → 핀번호가 곧 PORTD 비트 위치라 그대로 시프트
static inline void encUpdate() {
  uint8_t p = PIND;                                              // PORTD 직접 read
  uint8_t s = (((p >> ENC_A) & 1) << 1) | ((p >> ENC_B) & 1);    // (A<<1)|B
  encCount += QDEC[(encState << 2) | s];
  encState  = s;
}
ISR(INT0_vect) { encUpdate(); }   // D2(A) 에지
ISR(INT1_vect) { encUpdate(); }   // D3(B) 에지

long readEncCount() {
  noInterrupts();
  long c = encCount;
  interrupts();
  return c * ENCODER_DIRECTION;
}

// ───────────────────────── TFmini 상태 ─────────────────────
int  tfDistanceCm = 0;
int  tfStrength   = 0;
bool tfValid      = false;
unsigned long tfLastUpdateMs = 0;

// ───────────────────────── 스캔/제어 상태 ──────────────────
bool  scanning   = false;
bool  tiltAtTop  = false;
unsigned long pidPrevUs = 0;
long  pidPrevCount = 0;
float velIntegral  = 0.0f;
float prevErr      = 0.0f;

// ============================================================
void setup() {
  Serial.begin(115200);
  tfSerial.begin(19200);      // TFmini 를 19200 으로 영구 설정해둔 상태 (무손실, ~100/s)

  // ── L298N ──
  pinMode(DC_IN3, OUTPUT);
  pinMode(DC_IN4, OUTPUT);
  pinMode(DC_ENB, OUTPUT);
  driveMotor(0);

  // ── 엔코더 : 입력 + 내부 풀업 (PORTD 직접) ──
  DDRD  &= ~((1 << ENC_A) | (1 << ENC_B));    // 입력
  PORTD |=  ((1 << ENC_A) | (1 << ENC_B));    // 풀업
  // ── 외부 인터럽트 INT0/INT1 : 양쪽 에지(any change) ──
  EICRA = (EICRA & ~0x0F) | (1 << ISC00) | (1 << ISC10);  // ISCx1:x0 = 01
  EIFR  = (1 << INTF0) | (1 << INTF1);        // 대기중 플래그 클리어
  EIMSK |= (1 << INT0) | (1 << INT1);         // 인터럽트 활성화
  encState = (((PIND >> ENC_A) & 1) << 1) | ((PIND >> ENC_B) & 1);

  // ── 스텝모터 ──
  tiltStepper.setMaxSpeed(TILT_MAX_SPEED_SPS);
  tiltStepper.setAcceleration(TILT_ACCEL_SPS2);
  tiltStepper.setCurrentPosition(0);
  tiltStepper.disableOutputs();               // 대기 중 코일 OFF (발열 방지)

  // ── TFmini 통신 설정 (19200) ──────────────────────────────
  //  TFmini-S 를 별도 스케치(TFmini_SetBaud19200.ino)로 19200 영구 설정해둔 상태.
  //  SoftwareSerial 은 19200 을 거의 무손실로 받아 ~100/s 확보됨 (115200 은 ~40/s).
  //  공장초기화/펌웨어 교체로 115200 으로 되돌아가면 그 스케치 다시 돌릴 것.
  //  지원 baud: 9600/14400/19200/56000/115200/460800/921600 (38400 은 미지원이었음)
  delay(100);
  tfSetFrameRate100();             // 출력 100Hz 확인
  delay(100);
  while (tfSerial.available()) tfSerial.read();   // 설정 ACK 프레임 버림

  pidPrevUs = micros();
  Serial.println(F("# READY"));
}

// ============================================================
void loop() {
  handlePcCommands();

  bool gotFrame = readTfmini();                 // 항상 파싱 (프레임 누락 최소화)

  if (scanning && gotFrame && tfValid) {
    streamSample();                             // 프레임 도착 즉시 pan/tilt 캡처
  }

  if (scanning) {
    updateVelPid();                             // 턴테이블 속도 PID (~200Hz)
    updateTilt();                               // tilt 왕복 (AccelStepper.run)
  }
}

// ─────────────────────── DC 모터 구동 ──────────────────────
void driveMotor(int pwm) {
  pwm *= MOTOR_DIRECTION;
  bool forward = (pwm >= 0);
  int  mag = forward ? pwm : -pwm;
  if (mag > PWM_MAX) mag = PWM_MAX;
  if (mag > 0 && mag < PWM_MIN_KICK) mag = PWM_MIN_KICK;   // deadband 보상
  if (forward) { digitalWrite(DC_IN3, LOW);  digitalWrite(DC_IN4, HIGH); }
  else         { digitalWrite(DC_IN3, HIGH); digitalWrite(DC_IN4, LOW);  }
  analogWrite(DC_ENB, mag);
}

// ───────────────────── 턴테이블 속도 PID ───────────────────
void updateVelPid() {
  unsigned long now = micros();
  float dt = (now - pidPrevUs) * 1e-6f;
  if (dt < 0.005f) return;                      // ~200 Hz 주기
  pidPrevUs = now;

  long  c   = readEncCount();
  float vel = (float)(c - pidPrevCount) / dt;   // 현재 속도 [counts/s]
  pidPrevCount = c;

  float err = TARGET_CPS - vel;
  velIntegral += err * dt;
  if (velIntegral >  INTEGRAL_MAX) velIntegral =  INTEGRAL_MAX;
  if (velIntegral < -INTEGRAL_MAX) velIntegral = -INTEGRAL_MAX;
  float deriv = (err - prevErr) / dt;
  prevErr = err;

  float out = Kp * err + Ki * velIntegral + Kd * deriv;
  if (out < 0)       out = 0;                   // 정방향 전용 (역회전 jerk 방지)
  if (out > PWM_MAX) out = PWM_MAX;
  driveMotor((int)out);
}

// ─────────────────────── tilt 연속 왕복 ────────────────────
void updateTilt() {
  // tilt 목표 step 을 pan(엔코더) 위치에서 직접 계산 -> 0..MAX 삼각파.
  // 목표 갱신은 ~330Hz 로 충분(pan 이 느림), step pulse 는 매 루프 생성.
  static unsigned long lastUs = 0;
  unsigned long now = micros();
  if (now - lastUs >= 3000) {
    lastUs = now;
    float panRevs = (float)readEncCount() / COUNTS_PER_TABLE_REV;  // 부호 포함 누적 바퀴수
    float phase   = panRevs * TILT_CYCLES_PER_REV;                 // tilt 사이클 위상
    phase -= floorf(phase);                                        // [0,1)
    if (phase < 0.0f) phase += 1.0f;                               // 역회전 보정
    float frac = (phase < 0.5f) ? (phase * 2.0f)            // 0 -> 1 (올라감)
                                : ((1.0f - phase) * 2.0f);   // 1 -> 0 (내려감)
    long target = (long)(TILT_MAX_STEPS * frac) * TILT_DIRECTION;
    tiltStepper.moveTo(target);                                    // pan 에 슬레이브
  }
  tiltStepper.run();                            // 필요한 만큼만 step pulse 생성 후 즉시 반환
}

// ──────────────────── TFmini 9-byte 프레임 파싱 ────────────
bool readTfmini() {
  static uint8_t buf[9];
  static uint8_t idx = 0;
  bool got = false;

  while (tfSerial.available()) {
    uint8_t b = tfSerial.read();
    if (idx == 0)      { if (b == 0x59) buf[idx++] = b; }
    else if (idx == 1) { if (b == 0x59) buf[idx++] = b; else idx = 0; }
    else {
      buf[idx++] = b;
      if (idx == 9) {
        idx = 0;
        uint16_t sum = 0;
        for (int i = 0; i < 8; i++) sum += buf[i];
        if ((sum & 0xFF) == buf[8]) {           // checksum 통과한 프레임만 인정
          tfDistanceCm = buf[2] | (buf[3] << 8);
          tfStrength   = buf[4] | (buf[5] << 8);
          tfValid = true;
          tfLastUpdateMs = millis();
          got = true;
        }
      }
    }
  }
  return got;
}

// ──── 한 샘플 출력 : tilt_deg, enc_count(raw), dist, strength ────
//  pan 각도는 Python(GUI)에서 'enc_count × 360 / 한바퀴카운트' 로 계산.
//  → 한 바퀴 카운트를 UI 에서 실시간으로 바꿔 누적오차(precession) 보정 가능.
void streamSample() {
  long  c       = readEncCount();                                // 누적 엔코더 카운트(부호有)
  long  tiltStp = tiltStepper.currentPosition() * TILT_DIRECTION; // 논리 step (0..512)
  float tiltDeg = tiltStp * TILT_DEG_PER_STEP;

  Serial.print(tiltDeg, 2);    Serial.print(',');
  Serial.print(c);             Serial.print(',');   // raw 카운트 (Python 에서 각도 변환)
  Serial.print(tfDistanceCm);  Serial.print(',');
  Serial.println(tfStrength);
}

// ───────────────────────── PC 명령 처리 ────────────────────
void handlePcCommands() {
  while (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 's': case 'S': {                     // 시작: 현재 자세를 pan0°/tilt0° 로 가정
        noInterrupts(); encCount = 0; interrupts();
        encState = (((PIND >> ENC_A) & 1) << 1) | ((PIND >> ENC_B) & 1);
        tiltStepper.enableOutputs();
        tiltStepper.setCurrentPosition(0);
        tiltAtTop = false;
        velIntegral = 0; prevErr = 0;
        pidPrevCount = 0; pidPrevUs = micros();
        scanning = true;
        Serial.println(F("# SCAN_START"));
      } break;
      case 'h': case 'H': {                     // 정지
        scanning = false;
        driveMotor(0);
        tiltStepper.disableOutputs();
        Serial.println(F("# SCAN_HALT"));
      } break;
      case 'z': case 'Z': {                     // 현재 위치를 0 으로
        noInterrupts(); encCount = 0; interrupts();
        encState = (((PIND >> ENC_A) & 1) << 1) | ((PIND >> ENC_B) & 1);
        tiltStepper.setCurrentPosition(0);
        tiltAtTop = false;
        Serial.println(F("# ZEROED"));
      } break;
      case 'p': case 'P': {                     // 상태 출력
        long c = readEncCount();
        Serial.print(F("# STATUS enc="));   Serial.print(c);
        Serial.print(F(" pan_deg="));        Serial.print(c * DEG_PER_COUNT, 1);
        Serial.print(F(" tilt_step="));      Serial.print(tiltStepper.currentPosition());
        Serial.print(F(" scanning="));       Serial.println(scanning ? 1 : 0);
      } break;
      default: break;
    }
  }
}

// ──────────────────── TFmini 설정 명령 (5A 프로토콜) ────────
// UART 보레이트 변경 : 5A 08 06 <baud LE 4byte> <checksum>  (save 안 함 = 휘발성)
void tfSetBaudRate(unsigned long baud) {
  uint8_t cmd[8];
  cmd[0] = 0x5A; cmd[1] = 0x08; cmd[2] = 0x06;
  cmd[3] = (uint8_t)( baud        & 0xFF);
  cmd[4] = (uint8_t)((baud >> 8)  & 0xFF);
  cmd[5] = (uint8_t)((baud >> 16) & 0xFF);
  cmd[6] = (uint8_t)((baud >> 24) & 0xFF);
  uint16_t sum = 0;
  for (int i = 0; i < 7; i++) sum += cmd[i];
  cmd[7] = (uint8_t)(sum & 0xFF);              // 체크섬 = 앞 7바이트 합의 하위 1바이트
  tfSerial.write(cmd, 8);
  delay(50);
}

void tfSetFrameRate100() {                      // 5A 06 03 64 00 C7  (100 Hz)
  uint8_t cmd[6] = { 0x5A, 0x06, 0x03, 0x64, 0x00, 0xC7 };
  tfSerial.write(cmd, 6);
  delay(50);
}
void tfSaveSettings() {                         // 5A 04 11 6F
  uint8_t cmd[4] = { 0x5A, 0x04, 0x11, 0x6F };
  tfSerial.write(cmd, 4);
  delay(50);
}
