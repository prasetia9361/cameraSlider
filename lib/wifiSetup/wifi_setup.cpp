#include "wifi_setup.h"

WifiSetup::WifiSetup() {}

void WifiSetup::setupWiFiAP(const char *ssid, const char *password) {
    IPAddress apIP(192, 168, 7, 2);
    IPAddress subNet(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apIP, subNet);
    WiFi.softAP(ssid, password, 6, 0, 4);
    
    dnsServer.setTTL(3600);
    dnsServer.start(53, "*", apIP);
}

void WifiSetup::setupWiFiSTA(const char* ssid, const char* pass){
    WiFi.begin(ssid, pass);

    int retry = 20; // 10 detik timeout
    while (WiFi.status() != WL_CONNECTED && retry > 0) {
        retry--;
        delay(500);
    }
    
    if (WiFi.status() != WL_CONNECTED && retry == 0) {
        mode = false;
        // connectAP();
    } else if(WiFi.status() == WL_CONNECTED){
        mode = true;
    }
}

void WifiSetup::connectAP(){
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
}

void WifiSetup::connectSTA(){
    WiFi.disconnect();
    // WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
}

void WifiSetup::disconnectAP(){
    if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
        WiFi.softAPdisconnect(true);
        dnsServer.stop();
    }
}

void WifiSetup::disconnectSTA(){
    if (WiFi.getMode() == WIFI_STA || WiFi.getMode() == WIFI_AP_STA) {
        WiFi.disconnect(true);
    }
}

void WifiSetup::loopDns() {
    if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
        dnsServer.processNextRequest();
    }
}

void WifiSetup::disconnect(){
    WiFi.disconnect(true);
    if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
       dnsServer.stop();
    }
}
