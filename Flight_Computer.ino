
/*
 * Flight Computer Software
 * Developed by:Yavuz Selim Yılmaz & Hikmet Berat Ünverdi
 */

/*SON GÜNCELLEME
    05.07.25
BMP280 için başlatma ve veri çekme eklendi.
*/
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>

//Pyro süre kontrol değişkenleri
bool dragTimerActive = false;
unsigned long dragStartTime = 0;
const unsigned long dragDuration = 3000; // 3 saniye

bool mainTimerActive = false;
unsigned long mainStartTime = 0;
const unsigned long mainDuration = 3000; // 3 saniye

// Kalman filtresi nesneleri
// BMP280 için (irtifa)
SimpleKalmanFilter kalmanCurrentAlt(1, 1, 0.5); // mutlak yükseklik
SimpleKalmanFilter kalmanAltitude(1, 1, 0.5);   // referans yükseklik
SimpleKalmanFilter kalmanPressure(1, 1, 0.5);   // basınç

// BNO055 için (ivme)
SimpleKalmanFilter kalmanAccX(2, 2, 0.05);
SimpleKalmanFilter kalmanAccY(2, 2, 0.05);
SimpleKalmanFilter kalmanAccZ(2, 2, 0.05);

// BNO055 için (açı)
SimpleKalmanFilter kalmanYaw(2, 2, 0.05);
SimpleKalmanFilter kalmanPitch(2, 2, 0.05);
SimpleKalmanFilter kalmanRoll(2, 2, 0.05);


//---------- SİT ve SUT -----------
#include <Arduino.h>
// --- Protokol sabitleri ---
#define HEADER_CMD 0xAA
#define FOOTER_1 0x0D
#define FOOTER_2 0x0A
#define TELEMETRI_HEADER 0xAB
#define TELEMETRI_PACKET_SIZE 36
unsigned long lastSendTimeSUT = 0;
unsigned long lastSendTimeSIT = 0;
bool sit_flag = false;
bool sut_flag = false;
float sutVerileri[8];

//-------------SERİAL PORT TANIMLAMALARI--------------
#define RS232 Serial1
#define HABERLESME_BILGISAYARI Serial2
// ----------- HABERLEŞME BİLGİSAYARI İLETİM -----------
#define MCU_TELEMETRI_HEADER 0xFA
#define MCU_2_FOOTER_1 0xBE
#define MCU_2_FOOTER_2 0xEF
float sensorData[12] = { 0 };

uint8_t durum = 1;  // örnek durum, bunu gerçek durum değişkeni ile değiştir

// ----------- SD Kart Tanımlamaları -----------
const uint8_t SD_CS_PIN = 53;  // SD kart CS pini: Mega'da D10'a bağlayın
SdFat SD;
SdFile telemetryFile;

// ----------- Sensör Nesneleri -----------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP280 bmp;  // BMP280 sensörü

// ----------- Değişkenler -----------
float sut_irtifa = 0;
float sut_basinc = 0;
float sut_ivmeX = 0;
float sut_ivmeY = 0;
float sut_ivmeZ = 0;
float sut_aciX = 0;
float sut_aciY = 0;
float sut_aciZ = 0;
bool ana_parasut_durumu = false;       // Ana paraşütü durumu
bool drag_parasutu_durumu = false;     // Drogue paraşütü açıldı mı?
bool alcalma_durumu = false;           // Alçalma durumu
bool aci_durumu = false;               // Açı durumu
bool belirlenen_irtifa_esigi = false;  // Belirlenen irtifa eşiği
bool minimum_irtifa_esigi = false;     // Minimum irtifa eşiği
bool kalkis_algilama = false;          // KALKIŞ ALGILAMA DURUMU
bool burnout = false;                  // Motor itkisinin bittiğini algılamak için
bool drag_acildi_mi = false;
bool ana_parasut_acildi_mi = false;
bool inis_gerceklesti = false;
float referansYukseklik = 0;
float referansBasinc = 0;
bool bmpCalibrated = false;
bool gyro_onayi;
float onceki_irtifa = 0;
int negatif_deger_sayaci = 0;
float dragDeployAltitude = 0.0;
float mainDeployAltitude = 0.0;

// Pil durumu için gerilim aralığı (LiPo örneği)
const float BATT_MIN_V = 3.0;
const float BATT_MAX_V = 4.2;
const float SEA_LEVEL_PRESSURE = 1013.25;

// Maksimum irtifa takibi
float measuredAlt = 0.0;
float maxAltitude = 0.0;
bool summaryLogged = false;

// RTC olmadığı için basit zaman takibi
unsigned long startTime = 0;

// Pil değişkenleri
float batteryV = 12.6;    // Örnek değer
float batteryPct = 85.0;  // Örnek değer

// SİT ve SUT Fonksiyon prototipleri
void komutKontrol();
void veriPaketiGonder(float bmpRakim, float bmpP, float accX, float accY, float accZ,
                      float yaw, float pitch, float roll);
void sitBaslat();
void durdur();
void sut_durum_bilgisi();

// --- Fonksiyon prototipleri ---
void MCU_2_paket_gonder();
void komutKontrol();
void sutTesti();
void veriPaketiGonder();
void sutVeriAl();
void sitBaslat();
void sutBaslat();
void durdur();
void sitTesti(float measuredAlt, float bmpP, float accX, float accY, float accZ, float yaw, float pitch, float roll);

// ----------- Fonksiyon Prototipleri -----------
void sutBaslat();
void readAcceleration(float &accX, float &accY, float &accZ);
void initBMP280();
void initBNO();
void readBMP280(float &pressure_hPa, float &temperature_C, float &altitude_m, float &currentAlt);
void readBNO(float &yaw, float &pitch, float &roll);
void checkApogee(float filtAlt, float filtPitch, float filtRoll, float filtAccX, float filtAccY, float accZ,
                 float sut_irtifa, float sut_basinc, float sut_ivmeX, float sut_ivmeY, float sut_ivmeZ, float sut_aciX, float sut_aciY, float sut_aciZ);
void deployDrogue(float anlik_irtifa);
void deployMain(float anlik_irtifa);
void logData(float filtAlt, float filtYaw, float filtPitch, float filtRoll, float filtAccX, float filtAccY, float filtAccZ, float voltage, float percent, unsigned long currentTime);
void setupCommunication();
void buzzer_aktiflestirme();
void recordFlightSummary();
void printTime(unsigned long currentTime);
void updatePyros();
// ----------- Setup ------------
void setup() {
  Serial.begin(115200);
  RS232.begin(115200);
  HABERLESME_BILGISAYARI.begin(115200);
  Wire.begin();
  delay(100);

  startTime = millis();  // Başlangıç zamanını kaydet

  pinMode(SD_CS_PIN, OUTPUT);
  SPI.begin();

  if (!SD.begin(SD_CS_PIN, SD_SCK_MHZ(50))) {
    Serial.print(F("SdFat init failed, CS="));
    Serial.println(SD_CS_PIN);
    //while (1);
  }
  Serial.print(F("SdFat kart baslatildi, CS="));
  Serial.println(SD_CS_PIN);

  // ===== DOSYA TEMİZLEME VE YENİ HEADER =====
  // Eski bozuk dosyayı sil
  if (SD.exists("telemetry.csv")) {
    SD.remove("telemetry.csv");
    Serial.println(F("Eski dosya silindi"));
  }

  // Yeni dosya oluştur - düzeltilmiş header ile
  if (!telemetryFile.open("telemetry.csv", O_CREAT | O_WRITE)) {
    Serial.println(F("telemetry.csv açılamadi!"));
  } else {
    // DÜZELTILMIŞ HEADER - Türkçe karakter yok
    telemetryFile.println(F("ALT,YAW,PITCH,ROLL,ACCX,ACCY,ACCZ,VOLTAGE,BATT_PERCENT,TIME,DROGUE_ALT,MAIN_ALT"));
    telemetryFile.close();
    Serial.println(F("telemetry.csv oluşturuldu."));
  }

  initBMP280();
  initBNO();

  Serial.print(F("Baslangic Pil: "));
  Serial.print(batteryV, 2);
  Serial.print(F(" V, Durum: "));
  Serial.print(batteryPct, 1);
  Serial.println(F("%"));

  setupCommunication();

  pinMode(10, OUTPUT);  // Drogue pyro d10 
  pinMode(11, OUTPUT);  // Main pyro d11
  pinMode(8, OUTPUT);  // Buzzer
  pinMode(9,OUTPUT); // buzzer acılıs
  digitalWrite(9,HIGH);
  delay(2000);
  digitalWrite(9,LOW);

  Serial.println(F("Tüm sensörler başlatıldı, sistem hazır."));
}
// ----------- Loop ------------
void loop() {
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  float gX = gravity.x();
  float gY = gravity.y();
  float gZ = gravity.z();

  float dot = gZ;
  float mag = sqrt(gX * gX + gY * gY + gZ * gZ);

  float angleRad = acos(dot / mag);
  float angleDeg = angleRad * 180.0 / PI;

  // Serial.print("Ag (Açı): ");
  //Serial.println(angleDeg, 2);

  float bmpP, bmpT, bmpAlt, bmpRakim;
  float yaw, pitch, roll;
  float accX, accY, accZ;
  unsigned long now = millis();

  // Sensörlerden ham veriler
  readAcceleration(accX, accY, accZ);
  readBMP280(bmpP, bmpT, bmpAlt, bmpRakim);
  readBNO(yaw, pitch, roll);

  // Kalman filtresi uygulanmış veriler
 float filtRakim = kalmanCurrentAlt.updateEstimate(bmpRakim);
float filtAlt = kalmanAltitude.updateEstimate(bmpAlt);
float filtBasinc = kalmanPressure.updateEstimate(bmpP);
  float filtAccX = kalmanAccX.updateEstimate(accX);
  float filtAccY = kalmanAccY.updateEstimate(accY);
  float filtAccZ = kalmanAccZ.updateEstimate(accZ);
  float filtYaw = kalmanYaw.updateEstimate(yaw);
  float filtPitch = kalmanPitch.updateEstimate(pitch);
  float filtRoll = kalmanRoll.updateEstimate(roll);

  // BMP280'den aldığımız irtifayı kullan
  measuredAlt = bmpAlt;

  // Maksimum irtifayı güncelle
  if (filtAlt > maxAltitude) {
    maxAltitude = filtAlt;
  }

  komutKontrol();                                                // Gelen komutları kontrol et
  sitTesti(filtRakim, filtBasinc, filtAccX, filtAccY, filtAccZ, filtYaw, filtPitch, filtRoll);  // SİT verisi gönderimi
  sutTesti();

  checkApogee(filtAlt, filtPitch, filtRoll, filtAccX, filtAccY, filtAccZ, sut_irtifa, sut_basinc, sut_ivmeX, sut_ivmeY, sut_ivmeZ, sut_aciX, sut_aciY, sut_aciZ);
  updatePyros();
  // Telemetriyi logla
  logData(filtAlt, filtYaw, filtPitch, filtRoll, filtAccX, filtAccY, filtAccZ, batteryV, batteryPct, now);

  // Sensör verilerini MCU 2 için hazırla
  sensorData[0] = filtAlt;
  sensorData[1] = filtPitch;
  sensorData[2] = filtRoll;
  sensorData[3] = filtAccX;
  sensorData[4] = filtAccY;
  sensorData[5] = filtAccZ;
  sensorData[6] = batteryV;
  sensorData[7] = batteryPct;
  sensorData[8] = bmpT;
  sensorData[9] = bmpP;
  sensorData[10] = angleDeg;
  sensorData[11] = yaw;

  MCU_2_paket_gonder();  // bu satır eksikse gönderim olmaz
  printTime(now);
  delay(100);
}

// ----------- BMP280 Başlatma ------------
void initBMP280() {
  if (!bmp.begin(0x76)) {  // BMP280'in varsayılan I2C adresi 0x76
    Serial.println(F("BMP280 bulunamadi!"));
    if (!bmp.begin(0x77)) {  // Alternatif adres 0x77
      Serial.println(F("BMP280 alternatif adres de bulunamadi!"));
      while (1)
        ;
    }
  }

  // BMP280 ayarları
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,     // sıcaklık oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // basınç oversampling
                  Adafruit_BMP280::FILTER_X16,      // IIR filtre
                  Adafruit_BMP280::STANDBY_MS_63);  // ~10 Hz


  // Referans basınç ve yükseklik ayarla
  delay(100);
  referansBasinc = bmp.readPressure() / 100.0;  // hPa cinsine çevir

  Serial.print(F("BMP280 baslatildi. Referans basinc: "));
  Serial.print(referansBasinc, 2);
  Serial.println(F(" hPa"));
}

// ----------- BNO055 Başlatma ------------
void initBNO() {
  if (!bno.begin()) {
    Serial.println(F("BNO055 bulunamadi!"));
    while (1)
      ;
  }
  bno.setExtCrystalUse(true);
  Serial.println(F("BNO055 baslatildi."));
}

// ----------- BMP280 Okuma ------------
void readBMP280(float &pressure_hPa, float &temperature_C, float &altitude_m, float &currentAlt) {
  pressure_hPa = bmp.readPressure() / 100.0;  // Pa'dan hPa'ya çevir
  temperature_C = bmp.readTemperature();

  // Deniz seviyesinden yükseklik hesaplama
  currentAlt = 44330.0 * (1.0 - pow(pressure_hPa / SEA_LEVEL_PRESSURE, 0.1903));

  if (!bmpCalibrated) {
    referansYukseklik = currentAlt;
    bmpCalibrated = true;
    Serial.print(F("BMP280 referans irtifa: "));
    Serial.println(referansYukseklik);
  }

  altitude_m = currentAlt - referansYukseklik;
}

// ----------- BNO055 Okuma ------------
void readBNO(float &yaw, float &pitch, float &roll) {
  sensors_event_t ev;
  bno.getEvent(&ev);
  yaw = ev.orientation.x;
  pitch = ev.orientation.y;
  roll = ev.orientation.z;
}

// ---------BNO055 İvme okuma ----------
void readAcceleration(float &accX, float &accY, float &accZ) {
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accX = accel.x();
  accY = accel.y();
  accZ = accel.z();
}

// ----------- Zamanı Seri Monitöre Yazdırma -----------
void printTime(unsigned long currentTime) {
  unsigned long seconds = (currentTime - startTime) / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;

  seconds = seconds % 60;
  minutes = minutes % 60;

  char buf[20];
  sprintf(buf, "%02lu:%02lu:%02lu", hours, minutes, seconds);
  Serial.println(buf);
}

// ----------- Apogee Kontrol ------------
void checkApogee(float filtAlt, float filtPitch, float filtRoll, float filtAccX, float filtAccY, float filtAccZ,
                 float sut_irtifa, float sut_basinc, float sut_ivmeX, float sut_ivmeY, float sut_ivmeZ, float sut_aciX, float sut_aciY, float sut_aciZ) {
  float alg_anlik_irtifa, alg_pitch, alg_roll, alg_accX, alg_accY, alg_accZ;

  if (sut_flag) {
    // SUT verilerinden kullan
    alg_anlik_irtifa = sut_irtifa;
    alg_pitch = sut_aciX;
    alg_roll = sut_aciY;
    alg_accX = sut_ivmeX;
    alg_accY = sut_ivmeY;
    alg_accZ = sut_ivmeZ;
  } else {
    // Normal sensör verilerini kullan
    alg_anlik_irtifa = filtAlt;
    alg_pitch = filtPitch;
    alg_roll = filtRoll;
    alg_accX = filtAccX;
    alg_accY = filtAccY;
    alg_accZ = filtAccZ;
  }

  /*Serial.print("Irtifa: ");
  Serial.println(alg_anlik_irtifa);
  Serial.print("Pitch: ");
  Serial.println(alg_pitch);*/

  //KALKIŞ ALGILAMA KARAR BLOĞU
  if (alg_accZ > 0.1 && !kalkis_algilama) {
    kalkis_algilama = true;
    // KALKIŞ ALGILAMA DURUMU TRUE OLDUĞUNDA SUT DURUM BİLGİSİ BURADA GÖNDERİLECEK
  }

  //MOTOR İTKİSİ BİTİMİNİ İVME İLE ALGILAMA BLOĞU
  if (alg_accZ < -10 && !burnout) {  //3 değeri örnek olarak verildi gerçeği temsil etmemektedir
    burnout = true;
    //MOTOR İTKİSİ BİTİMİ ALGILANDIĞINDA SUT DURUM BİLGİSİ BURADA GÖNDERİLECEK
  }

  if (alg_anlik_irtifa > 3000 && !drag_acildi_mi && !ana_parasut_acildi_mi && !minimum_irtifa_esigi) {
    minimum_irtifa_esigi = true;
    //MİNİMUM İRTİFA EŞİĞİ GEÇİLDİĞİNDE SUT DURUM BİLGİSİ BURADA GÖNDERİLECEK
  }

  if (alg_anlik_irtifa < 600 && !ana_parasut_acildi_mi && drag_acildi_mi && !belirlenen_irtifa_esigi) {
    belirlenen_irtifa_esigi = true;
    //belirlenen irtifa eşiği geçildiğinde SUT durum bilgisi burada gönderilecek
  }
  if (drag_acildi_mi && !ana_parasut_acildi_mi) {
    durum = 2;
  }
  if (!drag_acildi_mi && ana_parasut_acildi_mi) {
    durum = 3;
  }
  if (drag_acildi_mi && ana_parasut_acildi_mi) {
    durum = 4;
  }

  // gyro kilidi sürüklenme için
  if ((abs((float)alg_pitch) > 45.00) || (abs((float)alg_roll) > 45.00)) {
    gyro_onayi = true;
    if (!aci_durumu) {
      aci_durumu = true;
      //AÇI ONAYI VERİLMESİ BİT KAYDIRMA İŞLEMİ BURADA YAPILACAK
    }
  } else
    gyro_onayi = false;

  // İrtifa farkı hesapla;
  float irtifa_fark = alg_anlik_irtifa - onceki_irtifa;
  if (irtifa_fark < -0.8)  // düşüş varsa
    negatif_deger_sayaci++;
  else
    negatif_deger_sayaci = 0;


  if (!drag_acildi_mi && negatif_deger_sayaci >= 4 && gyro_onayi) {
    if (!alcalma_durumu) {
      alcalma_durumu = true;
      //ALÇALMA DURUM ONAYI BURADA BİT KAYDIRILACAK
    }
    deployDrogue(alg_anlik_irtifa);
  }

  if (drag_acildi_mi && !ana_parasut_acildi_mi && (alg_anlik_irtifa < 600))
    deployMain(alg_anlik_irtifa);

  // Ana paraşüt açıldıysa ve özet henüz kaydedilmediyse
  if (ana_parasut_acildi_mi && !summaryLogged) {
    recordFlightSummary();
    summaryLogged = true;
  }

  onceki_irtifa = alg_anlik_irtifa;

  if (ana_parasut_acildi_mi) {
    buzzer_aktiflestirme();
  }

  if (sut_flag) {
    //sut durum bilgisi gönderme fonksiyonu
    sut_durum_bilgisi();
  }
}

// ----------- Drogue Açma ------------
void deployDrogue(float altitude) {
  digitalWrite(10, HIGH);
  drag_acildi_mi = true;
  dragTimerActive = true;
  dragStartTime = millis();

  dragDeployAltitude = altitude;
  Serial.print(F("Drogue paraşütü açıldı! İrtifa: "));
  Serial.print(dragDeployAltitude, 2);
  Serial.println(F(" m"));
  if (!drag_parasutu_durumu) {
    drag_parasutu_durumu = true;
    // DROGUE PARAŞÜT AÇILDIĞINDA SUT DURUM BİLGİSİ BURADA GÖNDERİLECEK
  }
}

// ----------- Main Açma ------------
void deployMain(float altitude) {
  digitalWrite(11, HIGH);
  ana_parasut_acildi_mi = true;
  mainTimerActive = true;
  mainStartTime = millis();

  mainDeployAltitude = altitude;
  Serial.print(F("Ana paraşüt açıldı! İrtifa: "));
  Serial.print(mainDeployAltitude, 2);
  Serial.println(F(" m"));
  if (!ana_parasut_durumu) {
    ana_parasut_durumu = true;
    // ANA PARAŞÜT AÇILDIĞINDA SUT DURUM BİLGİSİ BURADA GÖNDERİLECEK
  }
}


// ----------- Telemetri Kaydı ------------
void logData(float filtAlt, float filtYaw, float filtPitch, float filtRoll, 
             float filtAccX, float filtAccY, float filtAccZ, 
             float voltage, float percent, unsigned long currentTime) {
  // Zaman hesaplama aynı...
  
  // SERİ MONITÖR - DÜZELTILMIŞ SIRALAMA

  Serial.print("Irtifa:"); Serial.print(filtAlt, 2);        Serial.print(",");    // A: ALT
  Serial.print(",YAW:"); Serial.print(filtYaw, 1);        Serial.print(",");    // B: YAW  
  Serial.print(",PIT:"); Serial.print(filtPitch, 1);      Serial.print(",");    // C: PITCH
 Serial.print(",ROL:"); Serial.print(filtRoll, 1);       Serial.print(",");    // D: ROLL
  Serial.print(",AccX:"); Serial.print(filtAccX, 2);       Serial.print(",");    // E: ACCX
   Serial.print(",AccY:");Serial.print(filtAccY, 2);       Serial.print(",");    // F: ACCY
  Serial.print(",AccZ:"); Serial.print(filtAccZ, 2);       Serial.print(",");    // G: ACCZ
  //Serial.print(voltage, 2);        Serial.print(",");    // H: VOLTAGE
  //Serial.print(percent, 0);        Serial.print(",");    // I: BATT_PERCENT
  //Serial.print(currentTime);           Serial.print(",");    // J: TIME
 Serial.print("Drag yuksekligi:"); Serial.print(dragDeployAltitude, 2); Serial.print(","); // K: DROGUE_ALT
 Serial.print("Ana Parasut yuksekligi:"); Serial.println(mainDeployAltitude, 2);                  // L: MAIN_ALT

  // SD KART - AYNI SIRALAMA
  if (telemetryFile.open("telemetry.csv", O_APPEND | O_WRITE)) {
    telemetryFile.print(filtAlt, 2);    telemetryFile.print(',');
    telemetryFile.print(filtYaw, 1);    telemetryFile.print(',');
    telemetryFile.print(filtPitch, 1);  telemetryFile.print(',');
    telemetryFile.print(filtRoll, 1);   telemetryFile.print(',');
    telemetryFile.print(filtAccX, 2);   telemetryFile.print(',');
    telemetryFile.print(filtAccY, 2);   telemetryFile.print(',');
    telemetryFile.print(filtAccZ, 2);   telemetryFile.print(',');
    telemetryFile.print(voltage, 2);    telemetryFile.print(',');
    telemetryFile.print(percent, 0);    telemetryFile.print(',');
    telemetryFile.print(currentTime);       telemetryFile.print(',');
    telemetryFile.print(dragDeployAltitude, 2); telemetryFile.print(',');
    telemetryFile.println(mainDeployAltitude, 2);
    telemetryFile.sync();
    telemetryFile.close();
  }
}

// ----------- Özet Kaydetme ------------
void recordFlightSummary() {
  SdFile summary;
  if (summary.open("summary.csv", O_CREAT | O_WRITE)) {
    summary.println(F("PARAMETRE,DEĞER"));
    summary.print(F("MAKS_IRTIFA,"));
    summary.print(maxAltitude, 2);
    summary.println(F(" m"));
    summary.print(F("DROGUE_IRTIFASI,"));
    summary.print(dragDeployAltitude, 2);
    summary.println(F(" m"));
    summary.print(F("MAIN_IRTIFASI,"));
    summary.print(mainDeployAltitude, 2);
    summary.println(F(" m"));
    summary.close();
    Serial.println(F("Flight summary kaydedildi."));
  } else {
    Serial.println(F("Özet dosyası açılamadı!"));
  }
}

// ----------- Haberleşme ------------
void setupCommunication() {
  // Komünikasyon protokolleri burada başlatılır
}

// ----------- Buzzer Aktifleştirme ------------
void buzzer_aktiflestirme() {
  digitalWrite(8, HIGH);
  inis_gerceklesti = true;
  Serial.println(F("inis gerceklesti. Buzzer aktiflesti."));
}

// --- Komut Kontrol Fonksiyonu ---
void komutKontrol() {
  // SUT aktifse ve komut başlığı yoksa kontrolü bırak
  if (sut_flag && RS232.peek() != HEADER_CMD) return;

  while (RS232.available() >= 5) {
    if (RS232.peek() != HEADER_CMD) {
      RS232.read();  // Başlık değilse atla
      continue;
    }

    if (RS232.available() < 5) return;

    uint8_t header = RS232.read();
    uint8_t cmd = RS232.read();
    uint8_t cs = RS232.read();
    uint8_t f1 = RS232.read();
    uint8_t f2 = RS232.read();

    uint8_t cs_calc = (header + cmd) % 256;

    if ((cmd == 0x20 || cmd == 0x22 || cmd == 0x24) && cs == cs_calc && f1 == FOOTER_1 && f2 == FOOTER_2) {

      switch (cmd) {
        case 0x20: sitBaslat(); break;
        case 0x22: sutBaslat(); break;
        case 0x24: durdur(); break;
      }
    } else {
      // Geçersiz komut gibi görünüyorsa atla
      continue;
    }
  }
}

// --- SİT Başlat Fonksiyonu ---
void sitBaslat() {
  if (!sit_flag) {
    Serial.println("SİT Başlatılıyor");
    delay(1000);
    sit_flag = true;
  }
}

void sutBaslat() {
  if (!sut_flag) {
    Serial.println("SUT Başlatılıyor");
    delay(1000);
    sut_flag = true;
  }
}

// --- Durdurma Fonksiyonu ---
void durdur() {
  Serial.println("Durduruluyor");
  sit_flag = false;
  sut_flag = false;
  // Gerekirse durdurma işlemleri eklenir
}

// --- SİT Testi Veri Gönderimi ---
void sitTesti(float bmpRakim, float bmpP, float accX, float accY, float accZ,
              float yaw, float pitch, float roll) {
  if (sit_flag && millis() - lastSendTimeSIT >= 100) {
    lastSendTimeSIT = millis();
    veriPaketiGonder(bmpRakim, bmpP, accX, accY, accZ,
                     yaw, pitch, roll);
  }
}

// --- SUT veri alma ---
void sutTesti() {
  if (sut_flag && millis() - lastSendTimeSUT >= 90) {
    lastSendTimeSUT = millis();
    sutVeriAl();
  }
}

// --- Telemetri Paketi Oluşturma ve Gönderme ---
void veriPaketiGonder(float bmpRakim, float bmpP, float accX, float accY, float accZ,
                      float yaw, float pitch, float roll) {
  //Serial.print("rakim: ");
  // Serial.print(bmpRakim, 2); Serial.print(",");

  float veriler[8] = {
    bmpRakim,  // irtifa
    bmpP,      // basınç
    accX,      // ivmeX
    accY,      // ivmeY
    accZ,      // ivmeZ
    roll,      // gyroX
    pitch,     // gyroY
    yaw,       // gyroZ
  };

  const int pktSize = 1 + 8 * 4 + 1 + 2;
  byte pkt[pktSize];
  int i = 0;

  pkt[i++] = TELEMETRI_HEADER;
  for (int j = 0; j < 8; j++) {
    byte *p = (byte *)(&veriler[j]);
    // Big Endian
    for (int b = 3; b >= 0; b--) {
      pkt[i++] = p[b];
    }
  }

  // Checksum (mod 255)
  byte cs2 = 0;
  for (int j = 0; j < 1 + 8 * 4; j++) {
    cs2 = (cs2 + pkt[j]) % 0xFF;
  }
  pkt[i++] = cs2;

  pkt[i++] = FOOTER_1;
  pkt[i++] = FOOTER_2;

  RS232.write(pkt, pktSize);
}

/*float truncate2Decimals(float value) {
  return (int)(value * 100) / 100.0;
}*/

// --- SUT telemetri verisini al ---
void sutVeriAl() {
  while (RS232.available() >= TELEMETRI_PACKET_SIZE) {
    // 1) Başlık kontrolü
    if (RS232.peek() != TELEMETRI_HEADER) {
      RS232.read();
      continue;
    }

    // 2) Tam paket gelene kadar bekle
    if (RS232.available() < TELEMETRI_PACKET_SIZE) {
      break;
    }

    // 3) Paketi oku
    uint8_t buffer[TELEMETRI_PACKET_SIZE];
    RS232.readBytes(buffer, TELEMETRI_PACKET_SIZE);

    // 5) Checksum hesapla
    uint8_t checksum_calc = 0;
    for (int i = 0; i < TELEMETRI_PACKET_SIZE - 3; i++) {
      checksum_calc = (checksum_calc + buffer[i]) % 256;
    }

    // 6) Footer & checksum kontrolü
    if (buffer[33] != checksum_calc || buffer[34] != FOOTER_1 || buffer[35] != FOOTER_2) {
      Serial.println(">>> HATALI SUT PAKET <<<");
      Serial.println();
      continue;
    }

    // 7) Float'ları çöz
    for (int i = 0; i < 8; i++) {
      uint8_t *f = &buffer[1 + i * 4];
      uint32_t asInt = ((uint32_t)f[0] << 24) | ((uint32_t)f[1] << 16) | ((uint32_t)f[2] << 8) | (uint32_t)f[3];
      memcpy(&sutVerileri[i], &asInt, sizeof(float));
    }

    // 7b) sutVerileri'den anlamlı değişkenlere ata
    sut_irtifa = sutVerileri[0];
    sut_basinc = sutVerileri[1];
    sut_ivmeX = sutVerileri[2];
    sut_ivmeY = sutVerileri[3];
    sut_ivmeZ = sutVerileri[4];
    sut_aciX = sutVerileri[5];
    sut_aciY = sutVerileri[6];
    sut_aciZ = sutVerileri[7];

    Serial.println();

    // Anlamlı değişkenlerle yazdır
    /* Serial.print("İrtifa: "); Serial.println(sut_irtifa, 2);
    Serial.print("Basınç: "); Serial.println(sut_basinc, 2);
    Serial.print("İvme X: "); Serial.println(sut_ivmeX, 2);
    Serial.print("İvme Y: "); Serial.println(sut_ivmeY, 2);
    Serial.print("İvme Z: "); Serial.println(sut_ivmeZ, 2);
    Serial.print("Açı X: "); Serial.println(sut_aciX, 2);
    Serial.print("Açı Y: "); Serial.println(sut_aciY, 2);
    Serial.print("Açı Z: "); Serial.println(sut_aciZ, 2);*/

    //Serial.println(); // Okunabilirlik için boş satır
  }
}

void sut_durum_bilgisi() {
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime < 100) return;  // 10Hz kontrol
  lastSendTime = millis();

  uint16_t durum = 0;

  if (kalkis_algilama) durum |= (1 << 0);
  if (burnout) durum |= (1 << 1);
  if (minimum_irtifa_esigi) durum |= (1 << 2);
  if (aci_durumu) durum |= (1 << 3);
  if (alcalma_durumu) durum |= (1 << 4);
  if (drag_parasutu_durumu) durum |= (1 << 5);
  if (belirlenen_irtifa_esigi) durum |= (1 << 6);
  if (ana_parasut_durumu) durum |= (1 << 7);
  // Bit 8-15: Şimdilik boş

  uint8_t data1 = durum & 0xFF;
  uint8_t data2 = (durum >> 8) & 0xFF;
  uint8_t checksum = data1 + data2;

  RS232.write(0xAA);
  RS232.write(data1);
  RS232.write(data2);
  RS232.write(checksum);
  RS232.write(0x0D);
  RS232.write(0x0A);
}

void MCU_2_paket_gonder() {
  uint8_t packet[53];  // 1 header + 12*4 float + 1 durum + 1 checksum + 2 footer = 53
  packet[0] = 0xFA;    // header

  for (int i = 0; i < 12; i++) {
    union {
      float f;
      uint8_t b[4];
    } u;
    u.f = sensorData[i];
    // Big endian
    packet[1 + i * 4] = u.b[3];
    packet[2 + i * 4] = u.b[2];
    packet[3 + i * 4] = u.b[1];
    packet[4 + i * 4] = u.b[0];
  }


  packet[49] = durum;  // durum byte'ı

  // checksum hesapla: 1'den 49'a kadar (float + durum)
  uint8_t sum = 0;
  for (int i = 1; i <= 49; i++) sum += packet[i];
  packet[50] = sum;

  packet[51] = 0xBE;  // footer1
  packet[52] = 0xEF;  // footer2

  HABERLESME_BILGISAYARI.write(packet, 53);  // uygun seri port ile gönder
}

//Pyro kızartma süresi non-blocking delay ile
void updatePyros() {
  if (dragTimerActive && (millis() - dragStartTime >= dragDuration)) {
    digitalWrite(10, LOW);
    dragTimerActive = false;
   // Serial.println("Drogue pini kapatıldı (3 sn bitti)");
  }

  if (mainTimerActive && (millis() - mainStartTime >= mainDuration)) {
    digitalWrite(11, LOW);
    mainTimerActive = false;
   // Serial.println("Main pini kapatıldı (3 sn bitti)");
  }
}
