#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#define SSID "bolt"
#define PASS "11111111"

#define AP

void setupOta(void *parameters)
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
    // ArduinoOTA.begin();
    for(;;){
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}