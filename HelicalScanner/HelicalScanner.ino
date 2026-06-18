#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <math.h>

//핀 배치
#define ENC_A     2     // INT0  (엔코더 A)
#define ENC_B     3     // INT1  (엔코더 B)
#define DC_IN3    4     // L298N IN3
#define DC_IN4    5     // L298N IN4
#define DC_ENB    6     // L298N ENB (PWM, Timer0)

#define TFMINI_RX 9     // Arduino RX  <- TFmini TX
#define TFMINI_TX 8     // Arduino TX  -> TFmini RX
SoftwareSerial tfSerial(TFMINI_RX, TFMINI_TX);

#define STP_IN1   10
#define STP_IN2   11
#define STP_IN3   12
#define STP_IN4   13
AccelStepper tiltStepper(AccelStepper::HALF4WIRE, STP_IN1, STP_IN3, STP_IN2, STP_IN4);

//턴테이블
const float COUNTS_PER_TABLE_REV = 1320.0f;   // 턴테이블 엔코더 한바퀴(정확히 측정해야하나 실험적으로 찾음)
const float TABLE_RPS            = 0.50f;      // 목표 회전수 [rev/s]  (안정 우선값)
const float TARGET_CPS           = COUNTS_PER_TABLE_REV * TABLE_RPS;  // 목표 속도 [counts/s]
const float DEG_PER_COUNT        = 360.0f / COUNTS_PER_TABLE_REV;

float Kp = 0.045f, Ki = 0.180f, Kd = 0.000f;
const int   PWM_MAX        = 180;
const int   PWM_MIN_KICK   = 45;               // deadband 보상: 최소 구동 PWM
const float INTEGRAL_MAX   = 4000.0f;
const int   MOTOR_DIRECTION   = 1;             // 반대로 돌면 -1
const int   ENCODER_DIRECTION = 1;             // 회전은 정방향인데 pan 이 감소하면 -1

const long  TILT_MIN_STEPS      = 0;
const long  TILT_MAX_STEPS      = 512;                  // 45° (half-step, 4096 step/rev)
const float TILT_DEG_PER_STEP   = 45.0f / (float)TILT_MAX_STEPS;
const int   TILT_DIRECTION      = 1;                    // 위/아래 반대면 -1
const float TILT_CYCLES_PER_REV = 20.0f / 23.0f;        // pan 1회전당 tilt 사이클 수 (= 위상비)
const float TILT_MAX_SPEED_SPS  = 700.0f;
const float TILT_ACCEL_SPS2     = 2500.0f;

//엔코더
static const int8_t PRECALC[16] = {  0, -1,  1,  0, 1,  0,  0, -1, -1,  0,  0,  1, 0,  1, -1,  0 };
volatile long    encoder_count = 0;
volatile uint8_t encState = 0;
static inline void encoder_interrupts() {
  uint8_t p = PIND;
  uint8_t s = (((p >> ENC_A) & 1) << 1) | ((p >> ENC_B) & 1);    // (A<<1)|B
  encoder_count += PRECALC[(encState << 2) | s];
  encState  = s;
}
ISR(INT0_vect) { encoder_interrupts(); }   // D2(A)
ISR(INT1_vect) { encoder_interrupts(); }   // D3(B)

long read_encoder_count() {
  noInterrupts();
  long c = encoder_count;
  interrupts();
  return c * ENCODER_DIRECTION;
}

//TFmini 설정
const long TF_STRENGTH_MIN = 100;    // 신뢰 가능한 최소 strength (매뉴얼 기준)
const long TF_DIST_MIN_CM  = 10;     // blind zone 하한
const long TF_DIST_MAX_CM  = 1200;   // 유효 거리 상한 (과도값/에러코드 컷)

long tfDistanceCm = 0;               // cm, 부호없이 파싱 (0..65535)
long tfStrength   = 0;               // 0..65535
bool tfValid      = false;
unsigned long tfLastUpdateMs = 0;

//스캔/제어 상태
const long SKIP_REVS = 2;            // 시작 후 무시할 회전 수 (PID 등속 정착 전 데이터 버림)
bool  scanning   = false;
bool  rawStream  = false;            // 디버그: 모터 정지 + 모든 유효 프레임 즉시 스트리밍(정지 측정)
bool  streamArmed = false;           // SKIP_REVS 회전 지나면 true -> 그때부터 스트리밍
bool  tiltAtTop  = false;
unsigned long pidPrevUs = 0;
long  pidPrevCount = 0;
float velIntegral  = 0.0f;
float prevErr      = 0.0f;

void setup() {
  Serial.begin(115200);
  tfSerial.begin(19200);      // TFmini 19200으로 설정돼 있어 포트도 19200

  //L298N
  pinMode(DC_IN3, OUTPUT);
  pinMode(DC_IN4, OUTPUT);
  pinMode(DC_ENB, OUTPUT);
  driveMotor(0);

  //엔코더 입력 + 내부 풀업
  DDRD  &= ~((1 << ENC_A) | (1 << ENC_B));
  PORTD |=  ((1 << ENC_A) | (1 << ENC_B));
  //인터럽트 INT0/INT1 -> any change
  EICRA = (EICRA & ~0x0F) | (1 << ISC00) | (1 << ISC10);  // ISCx1:x0 = 01
  EIFR  = (1 << INTF0) | (1 << INTF1);        // 대기중 플래그 클리어
  EIMSK |= (1 << INT0) | (1 << INT1);         // 인터럽트 활성화
  encState = (((PIND >> ENC_A) & 1) << 1) | ((PIND >> ENC_B) & 1);

  //스텝모터
  tiltStepper.setMaxSpeed(TILT_MAX_SPEED_SPS);
  tiltStepper.setAcceleration(TILT_ACCEL_SPS2);
  tiltStepper.setCurrentPosition(0);
  tiltStepper.disableOutputs();

  delay(100);
  while (tfSerial.available()) tfSerial.read();

  pidPrevUs = micros();
  Serial.println(F("# READY"));
}

// ============================================================
void loop() {
  handlePcCommands();

  bool gotFrame = readTfmini();                 // 항상 파싱 (프레임 누락 최소화)
  if (scanning) {
    if (!streamArmed) {                         // PID 등속 정착 전 SKIP_REVS 회전은 버림
      long c = read_encoder_count(); if (c < 0) c = -c;
      if (c >= (long)(SKIP_REVS * COUNTS_PER_TABLE_REV)) {
        streamArmed = true;
        Serial.println(F("# ARMED"));           // 정착 완료, 이제부터 스트리밍
      }
    }
    updateVelPid();                             // 턴테이블 속도 PID (~200Hz)
    updateTilt();                               // tilt (pan 종속, AccelStepper.run)
  }
}

//DC 모터 구동
void driveMotor(int pwm) {
  pwm *= MOTOR_DIRECTION;
  bool forward = (pwm >= 0);
  int mag = pwm;
  if (forward < 0) mag *= -1;
  if (mag > PWM_MAX) mag = PWM_MAX;
  if (mag > 0 && mag < PWM_MIN_KICK) mag = PWM_MIN_KICK;
  if (forward) { digitalWrite(DC_IN3, LOW);  digitalWrite(DC_IN4, HIGH); }
  else         { digitalWrite(DC_IN3, HIGH); digitalWrite(DC_IN4, LOW);  }
  analogWrite(DC_ENB, mag);
}

// ───────────────────── 턴테이블 속도 PID ───────────────────
void updateVelPid() {
  unsigned long now = micros();
  float dt = (now - pidPrevUs) * 1e-6f;
  if (dt < 0.005f) return;
  pidPrevUs = now;

  long  c   = read_encoder_count();
  float vel = (float)(c - pidPrevCount) / dt;
  pidPrevCount = c;

  float err = TARGET_CPS - vel;
  velIntegral += err * dt;
  if (velIntegral >  INTEGRAL_MAX) velIntegral =  INTEGRAL_MAX;
  if (velIntegral < -INTEGRAL_MAX) velIntegral = -INTEGRAL_MAX;
  float deriv = (err - prevErr) / dt;
  prevErr = err;

  float out = Kp * err + Ki * velIntegral + Kd * deriv;
  if (out < 0)       out = 0; // 정방향 전용
  if (out > PWM_MAX) out = PWM_MAX;
  driveMotor((int)out);
}


void updateTilt() {
  static unsigned long lastUs = 0;
  unsigned long now = micros();
  if (now - lastUs >= 3000) {
    lastUs = now;
    float panRevs = (float)read_encoder_count() / COUNTS_PER_TABLE_REV;
    float phase   = panRevs * TILT_CYCLES_PER_REV;
    phase -= floorf(phase);
    if (phase < 0.0f) phase += 1.0f;
    float frac;
    if (phase < 0.5f) {
      frac = phase * 2.0f;
    } else {
      frac = (1.0f - phase) * 2.0f;
    }
    long target = (long)(TILT_MAX_STEPS * frac) * TILT_DIRECTION;
    tiltStepper.moveTo(target);
  }
  tiltStepper.run();
}

//TFmini 9-byte 프레임 파싱
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
        if ((sum & 0xFF) == buf[8]) {           // checksum 통과
          long d = (uint16_t)(buf[2] | (buf[3] << 8));   // 부호없이 0..65535
          long s = (uint16_t)(buf[4] | (buf[5] << 8));
          if (s >= TF_STRENGTH_MIN && d >= TF_DIST_MIN_CM && d <= TF_DIST_MAX_CM) {
            tfDistanceCm = d;
            tfStrength   = s;
            tfValid = true;
            tfLastUpdateMs = millis();
            got = true;
          }
        }
      }
    }
  }
  return got;
}

// ───────────────────────── PC 명령 처리 ────────────────────
void handlePcCommands() {
  while (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 's': case 'S': {                     // 시작: 현재 자세를 pan0°/tilt0° 로 가정
        noInterrupts(); encoder_count = 0; interrupts();
        encState = (((PIND >> ENC_A) & 1) << 1) | ((PIND >> ENC_B) & 1);
        tiltStepper.enableOutputs();
        tiltStepper.setCurrentPosition(0);
        tiltAtTop = false;
        velIntegral = 0; prevErr = 0;
        pidPrevCount = 0; pidPrevUs = micros();
        streamArmed = false;                     // 시작 후 SKIP_REVS 회전은 다시 버림
        scanning = true;
        Serial.println(F("# SCAN_START"));
      } break;
      case 'h': case 'H': {                     // 정지 (모든 모드 OFF)
        scanning = false;
        rawStream = false;
        driveMotor(0);
        tiltStepper.disableOutputs();
        Serial.println(F("# SCAN_HALT"));
      } break;
      case 'z': case 'Z': {                     // 현재 위치를 0 으로
        noInterrupts(); encoder_count = 0; interrupts();
        encState = (((PIND >> ENC_A) & 1) << 1) | ((PIND >> ENC_B) & 1);
        tiltStepper.setCurrentPosition(0);
        tiltAtTop = false;
        Serial.println(F("# ZEROED"));
      } break;
      case 'p': case 'P': {                     // 상태 출력
        long c = read_encoder_count();
        Serial.print(F("# STATUS enc="));   Serial.print(c);
        Serial.print(F(" pan_deg="));        Serial.print(c * DEG_PER_COUNT, 1);
        Serial.print(F(" tilt_step="));      Serial.print(tiltStepper.currentPosition());
        Serial.print(F(" scanning="));       Serial.println(scanning ? 1 : 0);
      } break;
      default: break;
    }
  }
}
