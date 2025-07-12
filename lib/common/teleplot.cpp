#include "teleplot.h"
#include <stdio.h>


Teleplot::Teleplot(IPAddress ip, uint16_t port)
    : targetIP_(ip), targetPort_(port), enabled_(false) {}

void Teleplot::begin() {
    if (WiFi.status() == WL_CONNECTED) {
        udp_.begin(0);
        enabled_ = true;
    }
}

void Teleplot::plot(const char* name, float value) {
    plot(name, value, millis());
}

void Teleplot::plot(const char* name, float value, uint32_t timestamp) {
    if (!enabled_ || WiFi.status() != WL_CONNECTED) return;

    char buffer[128];
    snprintf(buffer, sizeof(buffer), ">%s:%lu:%f\n", name, timestamp, value);

    udp_.beginPacket(targetIP_, targetPort_);
    udp_.write((uint8_t*)buffer, strlen(buffer));
    udp_.endPacket();
}
