// STA-with-AP-fallback, so the robot is always reachable somehow. begin()
// blocks briefly at boot (WIFI_CONNECT_TIMEOUT_MS, one-time startup cost);
// maintain(), called every loop() tick, watches for a STA connection
// dropping later and falls back to AP if it doesn't recover -- "always
// reach it" is a standing guarantee, not just a first-boot behavior.
//
// Deliberately does not attempt to hop back from AP fallback to STA on its
// own once it's fallen back -- that's a real feature this doesn't build,
// not an oversight; a power cycle retries STA from scratch, matching the
// existing link-troubleshooting pattern in docs/HOW_TO_USE.md.
#pragma once

enum class WifiMode { Station, AccessPoint };

class WifiSetup {
public:
    void begin();

    // Returns true if this call caused a fresh transition into AP
    // fallback (an edge, not a level) -- the caller should treat that as
    // safety-relevant and enter safe state, same as a watchdog trip.
    bool maintain();

    WifiMode mode() const { return mode_; }

private:
    void startStation();
    void startAccessPoint();

    WifiMode mode_ = WifiMode::Station;
    unsigned long staLostAtMs_ = 0;
};
