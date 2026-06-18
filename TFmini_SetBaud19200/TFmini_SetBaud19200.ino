#include <SoftwareSerial.h>

#define TFMINI_RX 9     
#define TFMINI_TX 8     
SoftwareSerial tf(TFMINI_RX, TFMINI_TX);
void setup() {
  Serial.begin(115200);
  tf.begin(115200UL); delay(100);
  uint8_t baud[8] = {0x5A, 0x08, 0x06, 0x00, 0x4B, 0x00, 0x00, 0xB3}; 
  tf.write(baud, 8); 
  delay(200);
  uint8_t save[4] = {0x5A,0x04,0x11,0x6F}; 
  tf.write(save,4); 
  delay(100); 
}

void loop() {}
