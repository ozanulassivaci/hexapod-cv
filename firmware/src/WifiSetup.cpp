#include "WifiSetup.h"

#include <Arduino.h>
#include <WiFi.h>

#include "Config.h"
#include "secrets.h"

void WifiSetup::begin() {
    startStation();
}

void WifiSetup::startStation() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
        delay(200);
    }

    if (WiFi.status() == WL_CONNECTED) {
        mode_ = WifiMode::Station;
        Serial.print("WiFi: connected, IP ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi: could not join configured network, falling back to AP");
        startAccessPoint();
    }
}

void WifiSetup::startAccessPoint() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    mode_ = WifiMode::AccessPoint;
    Serial.print("WiFi: hosting fallback AP, IP ");
    Serial.println(WiFi.softAPIP());
}

bool WifiSetup::maintain() {
    if (mode_ != WifiMode::Station) {
        return false;  // already in AP fallback -- stays there until reboot
    }
    if (WiFi.status() == WL_CONNECTED) {
        staLostAtMs_ = 0;
        return false;
    }
    if (staLostAtMs_ == 0) {
        staLostAtMs_ = millis();
        return false;
    }
    if (millis() - staLostAtMs_ >= WIFI_CONNECT_TIMEOUT_MS) {
        startAccessPoint();
        return true;
    }
    return false;
}
