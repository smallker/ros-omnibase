#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

/*
    #define AP berfungsi untuk mendefinisikan
    ESP32 sebagai akses poin, untuk menjadikan
    ESP32 sebagai client, cukup comment pada baris
    tsb lalu ganti SSID, PASS, dan IP Address sesuai
    pengaturan jaringan yang akan digunakan 
*/
#define SSID "openbase"
#define PASS "openbase"

#define AP

void wifiSetup(void *parameters)
{
    Serial.println("Booting");
    IPAddress local_ip(192, 168, 43, 100);
    IPAddress gateway(192, 168, 43, 100);
    IPAddress subnet(255, 255, 255, 0);
#ifdef AP
    WiFi.softAP(SSID, PASS);
    WiFi.softAPConfig(local_ip, gateway, subnet);
#else
    WiFi.config(local_ip, gateway, subnet);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
#endif
    Serial.println("IP : " + (String)WiFi.localIP().toString());

/*
    Menyalakan fungsi OTA pada ESP32 sehingga
    ESP32 dapat di program melalui wifi
    NB: Setup awal harus menggunakan kabel
*/
    ArduinoOTA
        .onStart([]()
                 {
                     String type;
                     if (ArduinoOTA.getCommand() == U_FLASH)
                         type = "sketch";
                     else // U_SPIFFS
                         type = "filesystem";

                     // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                     Serial.println("Start updating " + type);
                 })
        .onEnd([]()
               { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total)
                    { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
        .onError([](ota_error_t error)
                 {
                     Serial.printf("Error[%u]: ", error);
                     if (error == OTA_AUTH_ERROR)
                         Serial.println("Auth Failed");
                     else if (error == OTA_BEGIN_ERROR)
                         Serial.println("Begin Failed");
                     else if (error == OTA_CONNECT_ERROR)
                         Serial.println("Connect Failed");
                     else if (error == OTA_RECEIVE_ERROR)
                         Serial.println("Receive Failed");
                     else if (error == OTA_END_ERROR)
                         Serial.println("End Failed");
                 });
    ArduinoOTA.begin();
    for (;;)
    {
        ArduinoOTA.handle();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}