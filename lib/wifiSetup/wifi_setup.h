#pragma once
#include <DNSServer.h>
#include <WiFi.h>
class WifiSetup {
   private:
    DNSServer dnsServer;
    bool mode = false;
    String macAddress = WiFi.macAddress();

   public:
    // void begin();
    WifiSetup();
    // void begin();
    void setupWiFiAP(const char *ssid, const char *password);
    void loopDns();
    void setupWiFiSTA(const char *ssid, const char *pass);
    void connectAP();
    void connectSTA();
    // String getMac() { return macAddress; }
    void disconnectAP();
    void disconnectSTA();
    bool isConnected() { return mode; }
    bool reconnect() { return WiFi.reconnect(); }
    void disconnect();
};
