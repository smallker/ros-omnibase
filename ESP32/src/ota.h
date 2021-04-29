#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#define SSID "bolt"
#define PASS "11111111"

void setupOta()
{
    Serial.println("Booting");
    IPAddress local_IP(192, 168, 43, 100);
    IPAddress gateway(192, 168, 43, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.config(local_IP, gateway, subnet);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    
    ArduinoOTA.begin();
}