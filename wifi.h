#pragma once
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

class WiFiManager
{
public:
    WiFiManager();
    void setup();
    void loop();
    void sendBufferedData();
private:
    WiFiMulti wifiMulti;
    WiFiServer wifiServer;
    WiFiUDP wifiUDPServer;
    uint32_t lastBroadcast;
};


