#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>

class Teleplot {
private:
    WiFiUDP udp_;
    IPAddress targetIP_;
    uint16_t targetPort_;
    bool enabled_;

public:
    Teleplot(IPAddress target = IPAddress(192, 168, 1, 100), uint16_t port = 47269);
    void begin();
    void plot(const char* name, float value);
    void plot(const char* name, float value, uint32_t timestamp);
    void setEnabled(bool enabled) { enabled_ = enabled; }
    bool isEnabled() const { return enabled_; }
};
