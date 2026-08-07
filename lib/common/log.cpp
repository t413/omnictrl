#include "log.h"
#include <cstdarg>
#include <cstdio>
#ifdef ARDUINO
#include <Arduino.h>
#endif

void logme(const char* fmt, ...) {
#ifdef ARDUINO
    va_list args;
    va_start(args, fmt);
    // HWCDC Serial has no vprintf; format into a buffer and print
    char buf[512];
    vsnprintf(buf, sizeof(buf), fmt, args);
    if (Serial) {
        Serial.print(buf);
    }
    va_end(args);
#else
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);
#endif
}
