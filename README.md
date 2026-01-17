# Flight Computer System

Bu depo, model roket ve sonda roketi uygulamaları için geliştirilmiş **modüler ve çoklu mikrodenetleyici mimarisine sahip bir uçuş bilgisayarı sistemini** içermektedir. Sistem; uçuş kontrolü, haberleşme, görev yükü ve yer istasyonu olmak üzere **dört bağımsız yazılımdan** oluşur.

Amaç; uçuş güvenliğini korurken, telemetri kararlılığını artırmak ve bilimsel görevleri uçuş kontrolünden izole etmektir.

---

## Geliştiriciler

* **Yavuz Selim Yılmaz**
* **Hikmet Berat Ünverdi**

---

## Sistem Genel Bakışı

Sistem, görevlerin birbirini etkilememesi için **ayrık sorumluluk prensibiyle** tasarlanmıştır.

* Uçuş kararları ve kurtarma algoritmaları ayrı bir MCU’da çalışır
* Haberleşme ve RF yükü ayrı bir MCU’ya alınmıştır
* Bilimsel görev yükü bağımsızdır
* Yer istasyonu yalnızca izleme ve veri birleştirme yapar

Bu yapı, yarışma ve gerçek uçuş senaryolarında **kararlılık ve güvenlik** sağlar.

---

## Yazılım Bileşenleri

Sistem toplam **4 ana yazılımdan** oluşur:

---

## 1. Uçuş Bilgisayarı (Flight Computer)

Uçuş bilgisayarı, roketin **uçuş boyunca karar veren merkezi birimidir**.

### Görevleri

* Sensör verilerini toplamak ve filtrelemek
* Apogee (tepe noktası) tespiti yapmak
* Drogue ve ana paraşüt açılma kararlarını vermek
* Uçuş durumunu (state) belirlemek
* Telemetri verisini haberleşme MCU’suna iletmek

### Özellikler

* Bloklamasız (non-blocking) yapı
* Test modları (SİT / SUT) ile yarışma uyumluluğu
* Uçuş güvenliğini önceliklendiren karar mekanizması

Uçuş bilgisayarı **RF, GPS veya yer istasyonu bağımlılığı olmadan** çalışır.

---

## 2. Haberleşme MCU (Telemetry Controller)

Haberleşme MCU, uçuş bilgisayarı ile yer istasyonu arasındaki **telemetri köprüsüdür**.

### Görevleri

* Uçuş bilgisayarından gelen verileri almak
* Paket bütünlüğünü (header / checksum / footer) doğrulamak
* GPS verisi ile telemetriyi zenginleştirmek
* RFD900 üzerinden yer istasyonuna veri göndermek

### Temel İlke

Bu MCU:

* ❌ Uçuş kararı vermez
* ❌ Paraşüt tetiklemez

Sadece:

> **Veri alır – doğrular – paketler – iletir**

Bu sayede RF ve GPS işlemleri uçuş güvenliğini etkilemez.

---

## 3. Görev Yükü MCU (Payload Computer)

Görev yükü MCU, uçuş sırasında **bilimsel ve çevresel verilerin toplanmasından** sorumludur.

### Görevleri

* Basınç sensörü ile irtifa ölçümü
* GPS ile konum ve irtifa takibi
* Bilimsel sensör verilerinin toplanması
* Tüm verilerin SD karta CSV formatında kaydı
* Görev yükü telemetrisinin yer istasyonuna iletilmesi

### Özellikler

* Referans irtifa kalibrasyonu
* Uçuş sonrası analiz için sürekli veri kaydı
* Uçuş sistemlerinden tamamen bağımsız yapı

Görev yükü, uçuş kontrol sistemlerinden **tamamen izoledir**.

---

## 4. Yer İstasyonu (Ground Station)

Yer istasyonu, uçuş boyunca tüm telemetriyi **eş zamanlı olarak izlemek ve birleştirmek** için tasarlanmıştır.

### Görevleri

* Ana telemetriyi (uçuş + haberleşme MCU) almak
* Görev yükü telemetrisini almak
* Paketleri ayrı ayrı doğrulamak
* Son geçerli verileri birleştirerek tek çıktı sunmak

### Veri Kaynakları

* **RFD1:** Ana uçuş telemetrisi
* **RFD2:** Görev yükü telemetrisi

Yer istasyonu:

* ❌ Uçuşa müdahale etmez
* ❌ Komut göndermez

Sadece **izleme ve analiz** amaçlıdır.

---

## Sistem Mimarisi

Sistem mimarisi aşağıdaki prensiplere dayanır:

* Modülerlik
* Hata izolasyonu
* Yarışma uyumluluğu
* Gerçek uçuş güvenliği

Her bir yazılım, diğerlerinden bağımsız olarak çalışabilecek şekilde tasarlanmıştır.

---

## Donanım Varsayımları

* Çoklu UART destekli mikrodenetleyiciler (Arduino Mega / eşdeğeri)
* RFD900 / RFD900X modemler
* BMP280 basınç sensörü
* GPS modülü
* SD kart modülü

---

## Notlar

* Bu proje **yarışma ve akademik kullanım** amacıyla geliştirilmiştir
* Uçuş öncesi tüm test modlarının çalıştırılması önerilir
* Parametreler görev profiline göre ayarlanmalıdır

---

## Sonuç

Bu sistem;

* Güvenli
* Modüler
* Yarışma odaklı
* Genişletilebilir

bir uçuş bilgisayarı mimarisi sunar.
