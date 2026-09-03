#include "WifiSetup.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "Config.h"
#include "secrets.h"

namespace {

// Reads the power-save mode back out of the driver rather than trusting
// that the set call above took effect -- "I called esp_wifi_set_ps()" and
// "the radio is actually staying awake" are different claims, and only
// the second one explains link latency. WIFI_PS_NONE is the only one of
// these that keeps the radio up between packets.
void reportPowerSave() {
    wifi_ps_type_t ps = WIFI_PS_NONE;
    esp_err_t err = esp_wifi_get_ps(&ps);
    Serial.print("WiFi: power save = ");
    if (err != ESP_OK) {
        Serial.print("unreadable (esp_wifi_get_ps err 0x");
        Serial.print(err, HEX);
        Serial.println(")");
        return;
    }
    switch (ps) {
        case WIFI_PS_NONE: Serial.println("NONE (radio stays awake -- expected)"); break;
        case WIFI_PS_MIN_MODEM: Serial.println("MIN_MODEM -- modem sleep is ON, expect high link latency"); break;
        case WIFI_PS_MAX_MODEM: Serial.println("MAX_MODEM -- modem sleep is ON, expect high link latency"); break;
        default: Serial.println("unknown"); break;
    }
}

}  // namespace

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
        // Modem sleep (the ESP32 Arduino core's default STA power-save
        // mode) powers the radio down between beacon intervals and takes
        // hundreds of ms to wake for an inbound packet -- measured on
        // this exact robot as an ~883ms, mostly-dropped-packet link, far
        // past LINK_TIMEOUT_S (see docs/HOW_TO_USE.md's link
        // troubleshooting section). This robot runs off mains power,
        // never a battery, so there's no current-draw tradeoff to weigh
        // against responsiveness -- power save is simply off. Must come
        // after the driver is up (WiFi.mode()/WiFi.begin() above), not
        // before.
        esp_wifi_set_ps(WIFI_PS_NONE);
        Serial.print("WiFi: connected, IP ");
        Serial.println(WiFi.localIP());
        reportPowerSave();
    } else {
        Serial.println("WiFi: could not join configured network, falling back to AP");
        startAccessPoint();
    }
}

void WifiSetup::startAccessPoint() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    mode_ = WifiMode::AccessPoint;
    // Same reasoning as startStation() above -- set unconditionally on
    // this path too (first-boot AP fallback and maintain()'s later
    // STA-lost fallback both come through here) rather than relying on
    // startStation()'s call still being in effect after a mode switch.
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.print("WiFi: hosting fallback AP, IP ");
    Serial.println(WiFi.softAPIP());
    reportPowerSave();
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
