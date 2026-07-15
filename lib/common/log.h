#pragma once
#include <stdint.h>

#ifdef ARDUINO
#include <Arduino.h>
#define D_LOGBASE(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)

#else
#include <cstdio>
#define D_LOGBASE(fmt, ...) { printf(fmt, ##__VA_ARGS__); fflush(stdout); }

uint32_t millis();
void delay(uint32_t ms);
#endif

#define D_LOG(fmt, ...) D_LOGBASE("[dug] " fmt "\n", ##__VA_ARGS__)
