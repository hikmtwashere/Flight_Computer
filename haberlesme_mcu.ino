#include <Arduino.h>
#include <TinyGPS++.h>

// ==== Ayarlar ====
#define DEBUG

#define HEADER    0xFA
#define FOOTER_1  0xBE
#define FOOTER_2  0xEF
#define PACKET_IN_SIZE   53
#define PACKET_OUT_SIZE  78  

// ==== GPS ====
TinyGPSPlus gps;

// ==== Değişkenler ====
float sensorData[12];
float measuredAlt, pitch, roll, accX, accY, accZ, batteryV, batteryPct, lpsT, lpsP, angleDeg, yaw;

uint8_t durum_global = 1;

// ==== MCU_2'dan Veri Al ====
bool MCU_2_paket_al() {
  static uint8_t buffer[PACKET_IN_SIZE];
  static uint8_t index = 0;

  while (Serial3.available()) {
    uint8_t incoming = Serial3.read();

    if (index == 0) {
      if (incoming == HEADER) {
        buffer[index++] = incoming;
      }
    } else {
      buffer[index++] = incoming;

      if (index == PACKET_IN_SIZE) {
        if (buffer[51] != FOOTER_1 || buffer[52] != FOOTER_2) {
#ifdef DEBUG
          Serial.println(">> Hatalı footer");
#endif
          index = 0;
          return false;
        }

        uint8_t sum = 0;
        for (int i = 1; i <= 49; i++) sum += buffer[i];
        if (sum != buffer[50]) {
#ifdef DEBUG
          Serial.println(">> Hatalı checksum");
#endif
          index = 0;
          return false;
        }

        for (int i = 0; i < 12; i++) {
          union { float f; uint8_t b[4]; } u;
          u.b[3] = buffer[1 + i * 4];
          u.b[2] = buffer[2 + i * 4];
          u.b[1] = buffer[3 + i * 4];
          u.b[0] = buffer[4 + i * 4];
          sensorData[i] = u.f;
        }

        measuredAlt = sensorData[0];
        pitch       = sensorData[1];
        roll        = sensorData[2];
        accX        = sensorData[3];
        accY        = sensorData[4];
        accZ        = sensorData[5];
        batteryV    = sensorData[6];
        batteryPct  = sensorData[7];
        lpsT        = sensorData[8];
        lpsP        = sensorData[9];
        angleDeg    = sensorData[10];
        yaw         = sensorData[11];

        durum_global = buffer[49];

#ifdef DEBUG
        Serial.println("=== MCU_2 Paket Alındı ===");
        for (int i = 0; i < 12; i++) {
          Serial.print("Data["); Serial.print(i); Serial.print("]: ");
          Serial.println(sensorData[i], 4);
        }
        Serial.print("Durum Byte: ");
        Serial.println(durum_global);
        Serial.println("===========================");
#endif

        index = 0;
        return true;
      }
    }
  }
  return false;
}

// ==== Telemetri Verisi Gönder ====
void RFD_paket_gonder() {
  static uint8_t sayac = 0;
  const uint8_t takim_id = 1;

  uint8_t durum = durum_global; 

  float gps_enlem = 0.0, gps_boylam = 0.0, gps_irtifa = 0.0;
  if (gps.location.isValid()) {
    gps_enlem = gps.location.lat();
    gps_boylam = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    gps_irtifa = gps.altitude.meters();
  }

  float dataOut[17] = {
    measuredAlt,
    gps_irtifa,
    gps_enlem,
    gps_boylam,
    gps_irtifa,
    gps_enlem,
    gps_boylam,
    0.0,
    0.0,
    0.0,
    accX,
    accY,
    accZ,
    pitch,
    roll,
    yaw,
    angleDeg
  };

  uint8_t packet[PACKET_OUT_SIZE];
  int idx = 0;

  packet[idx++] = 0xFF;
  packet[idx++] = 0xFF;
  packet[idx++] = 0x54;  
  packet[idx++] = 0x52;  

  packet[idx++] = takim_id;
  packet[idx++] = sayac++;

  for (int i = 0; i < 17; i++) {
    union { float f; uint8_t b[4]; } u;
    u.f = dataOut[i];
    for (int j = 0; j < 4; j++) {
      packet[idx++] = u.b[j];
    }
  }

  packet[idx++] = durum;

  uint8_t checksum = 0;
  for (int i = 4; i <= 74; i++) {
    checksum += packet[i];
  }
  packet[idx++] = checksum;

  packet[idx++] = 0x0D;
  packet[idx++] = 0x0A;

  Serial2.write(packet, PACKET_OUT_SIZE);

#ifdef DEBUG
  Serial.println(">> RFD paketi gönderildi.");
#endif
}

// ==== SETUP ====
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println(">> Başlatılıyor...");
#endif

  Serial3.begin(115200);  // MCU_2
  Serial2.begin(57600);   // RFD
  Serial1.begin(9600);    // GPS

#ifdef DEBUG
  Serial.println("GPS başlatıldı.");
#endif
}

// ==== LOOP ====
void loop() {
  // === GPS OKUMA ===
  while (Serial1.available()) {  // GPS Serial1'den okunacak
    gps.encode(Serial1.read());
  }

  // === MCU_2 Veri ===
  if (MCU_2_paket_al()) {
    RFD_paket_gonder();
  }

#ifdef DEBUG
  if (gps.location.isUpdated()) {
    Serial.print("GPS Enlem: "); Serial.println(gps.location.lat(), 7);
    Serial.print("GPS Boylam: "); Serial.println(gps.location.lng(), 7);
    Serial.print("GPS Yükseklik: "); Serial.println(gps.altitude.meters(), 2);
    Serial.print("GPS Hız: "); Serial.println(gps.speed.kmph(), 2);
    Serial.println("----------------------");
  }
#endif
}
