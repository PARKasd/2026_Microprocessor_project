#define ENC_A  2
#define ENC_B  3

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
}

void loop() {
  uint8_t p = PIND;
  int a = (p >> ENC_A) & 1;
  int b = (p >> ENC_B) & 1;

  Serial.print(a);
  Serial.print('\t');
  Serial.println(b + 2);
}
