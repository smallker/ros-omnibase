# ESP32
### Thanks to <a href="https://github.com/GuiRitter">Guilherme Alan Ritter </a>for the awesome guide and straight to the point odometry formula 
# 
## Catatan :
### Upload sketch ke ESP32 menggunakan <a href="https://code.visualstudio.com/">VScode</a> dengan ekstensi <a href="https://platformio.org/install/ide?install=vscode">PlatformIO</a>
### 1. Sesuaikan pinout di main.h
### 2. Mode komunikasi ROS dapat diganti ke Serial, selengkapnya buka file **ros_setup.h**
### 3. Sebelum menjalankan program utama, pastikan bahwa RPM motor bernilai positif saat motor berputar clockwise (CW), dan bernilai negatif saat berputar counter clockwise (CCW). Cara untuk menjalankan motor secara manual dengan memanggil fungsi **speed** objek motor, contoh:
```
#include <Arduino.h>
#include <kinematic.h>

#define M1_A    4
#define M1_B    16
#define M1_PWM  17
#define EN1_A   26
#define EN1_B   35
#define EN1_PPR 700
#define D_WHEEL 0.06

const int sampling_time_ms = 5;

Motor m1(M1_A, M1_B, M1_PWM, EN1_A, EN1_B, EN1_PPR, D_WHEEL);

void ICACHE_RAM_ATTR EN1_ISR()
{
  m1.isrHandler();
}

void setup(){
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(m1.en_a), EN1_ISR, FALLING);
  m1.pid(2, 0.05, 1, 255); // kP, kI, kD, windup
}

void loop(){
  m1.speed(30); // 30 RPM clockwise
  m1.calculateRpm(sampling_time_ms);
  Serial.println(m1.rpm); // Harus bernilai 30 positif
  delay(sampling_time_ms);
}
```
### 4. Pastikan juga kalibrasi nilai PPR di **main.h** dan nilai PID motor di fungsi **void setup main.cpp**
### 5. Posisi motor **M1, M2, dan M3** (atau V1, V2, V3 berurutan) seperti di bawah
<div>
<center><img src="https://cdn.rawgit.com/GuiRitter/OpenBase/master/images/geometry.svg" width=300></center>
</div> 

### 6. Jika menggunakan wifi, atur SSID, password dan alamat IP di file **wifi_setup.h**.
```
IPAddress local_ip(192, 168, 43, 100);
IPAddress gateway(192, 168, 43, 100);
IPAddress subnet(255, 255, 255, 0);
WiFi.softAP(SSID, PASS);
WiFi.softAPConfig(local_ip, gateway, subnet);
```
### Pastikan juga telah mengatur alamat IP PC yang telah terinstal ROS di file **ros_setup.h**
```
IPAddress server(192, 168, 43, 101); // IP PC yang terinstal ROS
```
### 7. Uncomment baris ini di file **platformio.ini** jika ingin menggunakan WiFi OTAA (sesuaikan upload_port dengan IP ESP32)
```
upload_protocol = espota
upload_port = 192.168.43.100
```
### 8. Wifi telah diset aktif dengan ssid **openbase** dan password **openbase**


