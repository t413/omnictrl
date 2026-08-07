#pragma once
#include <stdint.h>

void logme(const char* fmt, ...);

#define D_LOGBASE(fmt, ...) { logme(fmt, ##__VA_ARGS__); }
#define D_LOG(fmt, ...) D_LOGBASE("[dug] " fmt "\n", ##__VA_ARGS__)

#ifndef ARDUINO
uint32_t millis();
void delay(uint32_t ms);
#endif

