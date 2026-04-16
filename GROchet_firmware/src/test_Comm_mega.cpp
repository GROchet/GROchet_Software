#include <Arduino.h>

#define SerialGripper Serial2

#define BTN_FERMER 22  // Press to close gripper
#define BTN_OUVRIR 23  // Press to open gripper

uint8_t crc8(uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}

void gripperSend(uint8_t msg) {
  uint8_t frame[2] = { msg, crc8(&msg, 1) };
  SerialGripper.write(frame, 2);
  Serial.print("Sent: 0x");
  Serial.print(msg, HEX);
  Serial.print(" CRC: 0x");
  Serial.println(frame[1], HEX);
}

void setup() {
  Serial.begin(115200);
  SerialGripper.begin(9600);

  pinMode(BTN_FERMER, INPUT_PULLUP);
  pinMode(BTN_OUVRIR, INPUT_PULLUP);

  Serial.println("Ready. Press button to send gripper command.");
}

void loop() {
  if (digitalRead(BTN_FERMER) == LOW) {
    Serial.println("Fermer pince...");
    gripperSend(0x02);
    delay(300); // debounce
  }

  if (digitalRead(BTN_OUVRIR) == LOW) {
    Serial.println("Ouvrir pince...");
    gripperSend(0x01);
    delay(300); // debounce
  }
}