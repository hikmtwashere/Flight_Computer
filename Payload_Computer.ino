/*
 * Flight Computer Software
 * Developed by: Selim Yılmaz & Hikmet Berat Ünverdi
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>

// ----------- BMP280 Tanımlamaları -----------
Adafruit_BMP280 bmp; // I2C bağlantı için
#define SEA_LEVEL_PRESSURE   1013.25

// ----------- SD Kart Tanımlamaları -----------
const uint8_t SD_CS_PIN = 10;
SdFat SD;
SdFile telemetryFile;

float referansYukseklik = 0;
bool bmpCalibrated = false;

// ----------- GPS Tanımlamaları -----------
TinyGPSPlus gps;

// ----------- RFD900X Tanımlamaları -----------
#define RFD_SERIAL Serial2  // RFD900X modülü bağlı Serial
#define HEADER 0xFA
#define FOOTER 0xEF

uint8_t paketSayac = 0;

// ----------- Fonksiyon Prototipleri -----------
void initBMP280();
void readBMP280(float &pressure_hPa, float &temperature_C, float &altitude_m, float &currentAlt);
void readGPS(float &lat, float &lon, float &gpsAlt);
void sendTelemetryRFD(float altitude, float lat, float lon, float gpsAlt, float g1, float g2, float g3);

void setup(){
  Serial.begin(115200);
  Serial1.begin(9600);    // GPS
  RFD_SERIAL.begin(57600); // RFD900X
  Wire.begin();
  initBMP280();
  delay(100);

  pinMode(SD_CS_PIN, OUTPUT);
  SPI.begin();

  if (!SD.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
    Serial.print(F("SdFat init failed, CS="));
    Serial.println(SD_CS_PIN);
  } else {
    if (!telemetryFile.open("telemetry.csv", O_CREAT | O_WRITE | O_TRUNC)) {
      Serial.println(F("telemetry.csv açılamadı!"));
    } else {
      telemetryFile.println(F("ALT,LAT,LON,GPS_ALT,GEIGER1,GEIGER2,GEIGER3"));
      telemetryFile.close();
      Serial.println(F("telemetry.csv oluşturuldu."));
    }
  }
}

void loop(){
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // ---- Sensör verilerini oku ----
  float pressure, temperature, altitude, currentAlt;
  readBMP280(pressure, temperature, altitude, currentAlt);

  float lat = 0, lon = 0, gpsAlt = 0;
  readGPS(lat, lon, gpsAlt);

  // ---- Geiger Counter Dummy Değerleri ----
  float geiger1 = millis();
  float geiger2 = random(5, 20);
  float geiger3 = random(5, 20);

  // ---- CSV'ye yaz ----
  if (telemetryFile.open("telemetry.csv", O_WRITE | O_APPEND)) {
    telemetryFile.print(altitude); telemetryFile.print(",");
    telemetryFile.print(lat, 6); telemetryFile.print(",");
    telemetryFile.print(lon, 6); telemetryFile.print(",");
    telemetryFile.print(gpsAlt); telemetryFile.print(",");
    telemetryFile.print(geiger1); telemetryFile.print(",");
    telemetryFile.print(geiger2); telemetryFile.print(",");
    telemetryFile.println(geiger3);
    telemetryFile.close();
  }

  // ---- RFD900X ile gönder ----
  sendTelemetryRFD(altitude, lat, lon, gpsAlt, geiger1, geiger2, geiger3);

  // ---- Debug ----
  Serial.print(F("ALT: ")); Serial.print(altitude);
  Serial.print(F(" | GPS_ENLEM: ")); Serial.print(lat, 6);
  Serial.print(F(" | GPS_BOYLAM: ")); Serial.print(lon, 6);
  Serial.print(F(" | GPS_ALT: ")); Serial.print(gpsAlt);
  Serial.print(F(" | Geiger1: ")); Serial.print(geiger1);
  Serial.print(F(" | Geiger2: ")); Serial.print(geiger2);
  Serial.print(F(" | Geiger3: ")); Serial.println(geiger3);

  delay(50);
}

// ----------- GPS Okuma Fonksiyonu ----------- 
void readGPS(float &lat, float &lon, float &gpsAlt) {
  if (gps.location.isValid()) {
    lat = gps.location.lat();
    lon = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    gpsAlt = gps.altitude.meters();
  }
}

// ----------- BMP280 Başlatma ------------ 
void initBMP280() {
  if (!bmp.begin(0x76)) { // BMP280'in varsayılan I2C adresi 0x76'dır
    Serial.println(F("BMP280 sensörü bulunamadı!"));
    if (!bmp.begin(0x77)) { // Alternatif adres 0x77
      Serial.println(F("BMP280 sensörü 0x77 adresinde de bulunamadı!"));
      // while (1); // Hata durumunda döngüde kalma
    }
  }
  
  // BMP280 ayarları
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Çalışma modu
                  Adafruit_BMP280::SAMPLING_X2,     // Sıcaklık oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Basınç oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtreleme
                  Adafruit_BMP280::STANDBY_MS_500); // Standby süresi
  
  Serial.println(F("BMP280 başlatıldı."));
}

// ----------- BMP280 Okuma ------------ 
void readBMP280(float &pressure_hPa, float &temperature_C, float &altitude_m, float &currentAlt) {
  temperature_C = bmp.readTemperature();
  pressure_hPa = bmp.readPressure() / 100.0; // Pa'dan hPa'ya çevir
  currentAlt = bmp.readAltitude(SEA_LEVEL_PRESSURE);
  
  // İlk okumada referans yüksekliği ayarla
  if (!bmpCalibrated) {
    referansYukseklik = currentAlt;
    bmpCalibrated = true;
    Serial.print(F("BMP280 referans irtifa: ")); Serial.println(referansYukseklik);
  }
  
  altitude_m = currentAlt - referansYukseklik;
}

// ----------- Telemetri Paket Gönderimi ----------- 
void sendTelemetryRFD(float altitude, float lat, float lon, float gpsAlt, float g1, float g2, float g3) {
  uint8_t *ptr;
  uint8_t paket[33]; // Header + Sayaç + 7 float (28) + Checksum + Footer = 33

  paket[0] = HEADER;          // Header
  paket[1] = paketSayac++;    // Sayaç

  // Altitude
  ptr = (uint8_t*)&altitude;
  for (int i = 0; i < 4; i++) paket[2 + i] = ptr[i];

  // Latitude
  ptr = (uint8_t*)&lat;
  for (int i = 0; i < 4; i++) paket[6 + i] = ptr[i];

  // Longitude
  ptr = (uint8_t*)&lon;
  for (int i = 0; i < 4; i++) paket[10 + i] = ptr[i];

  // GPS Altitude
  ptr = (uint8_t*)&gpsAlt;
  for (int i = 0; i < 4; i++) paket[14 + i] = ptr[i];

  // Geiger1
  ptr = (uint8_t*)&g1;
  for (int i = 0; i < 4; i++) paket[18 + i] = ptr[i];

  // Geiger2
  ptr = (uint8_t*)&g2;
  for (int i = 0; i < 4; i++) paket[22 + i] = ptr[i];

  // Geiger3
  ptr = (uint8_t*)&g3;
  for (int i = 0; i < 4; i++) paket[26 + i] = ptr[i];

  // Checksum (XOR)
  uint8_t checksum = 0;
  for (int i = 1; i <= 30; i++) checksum ^= paket[i];
  paket[31] = checksum;

  paket[32] = FOOTER; // Footer

  // Gönder
  RFD_SERIAL.write(paket, 33);
}
