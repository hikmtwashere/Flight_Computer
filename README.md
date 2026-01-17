Flight Computer Software

Bu depo, model roket ve sonda roketi uygulamaları için geliştirilen gömülü bir uçuş bilgisayarı yazılımını içermektedir. Yazılım; uçuş esnasında sensör verilerinin toplanması, filtrelenmesi, telemetri olarak iletilmesi, paraşüt tetikleme kararlarının verilmesi ve uçuş verilerinin SD karta kaydedilmesi amacıyla tasarlanmıştır.

Geliştiriciler

Yavuz Selim Yılmaz

Hikmet Berat Ünverdi

Genel Özellikler

BMP280 ve BNO055 sensörlerinden veri okuma

Kalman filtresi ile irtifa, ivme ve yönelim verilerinin filtrelenmesi

Apogee (tepe noktası) tespiti

Drogue ve ana paraşüt için bağımsız tetikleme algoritmaları

Non-blocking (bloklamasız) pyro sürme yapısı

SD kart üzerine CSV formatında telemetri kaydı

Uçuş sonunda özet (maksimum irtifa, paraşüt açılma irtifaları) kaydı

İniş sonrası buzzer ile konum bildirimi

Test Modları

Yazılım, TEKNOFEST Roket Yarışması test gereksinimlerine uygun olarak iki farklı test modunu destekler:

SİT (Sensör İzleme Testi)

Sensörlerden okunan veriler 10 Hz hızla seri port üzerinden gönderilir

Telemetri paketleri özel başlık–checksum–footer yapısı içerir

SUT (Sentetik Uçuş Testi)

Harici test cihazından gelen sentetik uçuş verileri kullanılır

Gerçek sensör verileri yerine SUT verileri ile apogee ve paraşüt algoritmaları çalıştırılır

SUT sırasında komut ve telemetri çakışmasını önleyen ayrıştırılmış protokol yapısı mevcuttur

Haberleşme Mimarisi

RS232 (Serial1): SİT / SUT test haberleşmesi

Serial2: İkinci MCU’ya telemetri iletimi

Özel Paket Formatları:

Big-endian float veri yapısı

Header / Checksum / Footer doğrulaması

Durum bitleri ile uçuş fazı bilgisi aktarımı

SD Kart Kayıtları

telemetry.csv

Anlık uçuş verileri (irtifa, ivme, açı, pil durumu vb.)

summary.csv

Maksimum irtifa

Drogue ve ana paraşüt açılma irtifaları

Donanım Gereksinimleri

Arduino Mega (veya eşdeğer çoklu UART destekli MCU)

BMP280 (basınç / irtifa)

BNO055 (ivme + yönelim)

SD kart modülü

Pyro çıkışları (drogue & main)

Buzzer

Notlar

Kod, yarışma ve akademik kullanım amacıyla geliştirilmiştir

Parametreler (irtifa eşikleri, ivme limitleri vb.) görev profiline göre güncellenmelidir

Gerçek uçuş öncesi SİT ve SUT testlerinin eksiksiz yapılması önerilir
