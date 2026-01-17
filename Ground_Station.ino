/*
 * Flight Computer Software
 * Developed by: Selim Yılmaz & Hikmet Berat Ünverdi
 */

#include <Arduino.h>

// === RFD1 ===
#define HEADER1_0 0xFF
#define HEADER1_1 0xFF
#define HEADER1_2 0x54
#define HEADER1_3 0x52
#define PACKET1_SIZE 78

uint8_t buffer1[PACKET1_SIZE];
uint8_t lastBuffer1[PACKET1_SIZE];
uint8_t index1 = 0;

// === RFD2 ===
#define HEADER2 0xFA
#define FOOTER2 0xEF
#define PACKET2_SIZE 33

uint8_t buffer2[PACKET2_SIZE];
uint8_t lastBuffer2[PACKET2_SIZE];
uint8_t index2 = 0;
bool paketBasladi2 = false;

// --- RFD1 ---
bool checkFooter1(uint8_t *buf) { return (buf[76] == 0x0D && buf[77] == 0x0A); }
bool checkChecksum1(uint8_t *buf) {
  uint8_t sum = 0;
  for (int i = 4; i <= 74; i++) sum = (sum + buf[i]) % 256;
  return (sum == buf[75]);
}

bool readPacketRFD1() {
  while (Serial3.available()) {
    uint8_t incoming = Serial3.read();
    if (index1 == 0 && incoming == HEADER1_0) buffer1[index1++] = incoming;
    else if (index1 == 1 && incoming == HEADER1_1) buffer1[index1++] = incoming;
    else if (index1 == 2 && incoming == HEADER1_2) buffer1[index1++] = incoming;
    else if (index1 == 3 && incoming == HEADER1_3) buffer1[index1++] = incoming;
    else if (index1 >= 4) {
      buffer1[index1++] = incoming;
      if (index1 == PACKET1_SIZE) {
        if (!checkFooter1(buffer1)) { index1 = 0; return false; }
        if (!checkChecksum1(buffer1)) { index1 = 0; return false; }
        index1 = 0;
        return true;
      }
    } else index1 = 0;
  }
  return false;
}

// --- RFD2 ---
bool readPacketRFD2() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (!paketBasladi2) {
      if (b == HEADER2) {
        paketBasladi2 = true;
        index2 = 0;
        buffer2[index2++] = b;
      }
    } else {
      buffer2[index2++] = b;
      if (index2 >= PACKET2_SIZE) {
        paketBasladi2 = false;
        if (buffer2[PACKET2_SIZE - 1] != FOOTER2) return false;
        uint8_t checksum = 0;
        for (int i = 1; i <= 30; i++) checksum ^= buffer2[i];
        if (checksum != buffer2[31]) return false;
        index2 = 0;
        return true;
      }
    }
  }
  return false;
}

// --- Tek satır veriler ---
void printCombinedData() {
  // RFD1 Floats F0..F16
  for (int i = 0; i < 17; i++) {
    union { float f; uint8_t b[4]; } u;
    int baseIdx = 6 + i*4;
    for (int j = 0; j < 4; j++) u.b[j] = lastBuffer1[baseIdx+j];
    Serial.print("F"); Serial.print(i); Serial.print(":"); Serial.print(u.f,6); Serial.print(" ");
  }

  // RFD2 Floats P0..P6
  float p[7];
  memcpy(&p[0], &lastBuffer2[2], 4);
  memcpy(&p[1], &lastBuffer2[6], 4);
  memcpy(&p[2], &lastBuffer2[10],4);
  memcpy(&p[3], &lastBuffer2[14],4);
  memcpy(&p[4], &lastBuffer2[18],4);
  memcpy(&p[5], &lastBuffer2[22],4);
  memcpy(&p[6], &lastBuffer2[26],4);

  for (int i=0;i<7;i++){
    Serial.print("P"); Serial.print(i); Serial.print(":"); Serial.print(p[i],6); Serial.print(" ");
  }

  Serial.println();
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Serial3.begin(57600);
  Serial1.begin(57600);
}

// --- Loop ---
void loop() {
  if (readPacketRFD1()) memcpy(lastBuffer1, buffer1, PACKET1_SIZE);
  if (readPacketRFD2()) memcpy(lastBuffer2, buffer2, PACKET2_SIZE);

  printCombinedData();
  delay(50); // ~20 Hz, isteğe bağlı azaltılabilir
}
