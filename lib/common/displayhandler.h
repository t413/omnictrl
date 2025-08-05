#pragma once
#include <Arduino.h>
#include "rfprotocol.h"

// Forward declaration
namespace lgfx { inline namespace v1 { class LGFX_Device; struct IFont; } }

// Common color declarations
extern const uint16_t SUPERDARKRED;
extern const uint16_t SUPERDARKBLUE;

class DisplayHandler {
private:
    lgfx::v1::LGFX_Device* lcd_ = nullptr;
    uint16_t borderWidth_ = 2;
    uint32_t lastClear_ = 0;
    bool redrawRequired_ = false;

public:
    DisplayHandler(lgfx::v1::LGFX_Device* lcd = nullptr, uint16_t borderWidth = 2);

    // Basic display management
    void setLCD(lgfx::v1::LGFX_Device* lcd);
    void setBorderWidth(uint16_t width);
    void requestRedraw();
    void setRotation(int rotation);

    // Layout functions
    void startFrame();
    void endFrame();
    void drawBorder(uint16_t color);
    void clearContent(uint16_t backgroundColor, uint32_t now, bool force = false);
    void setFont(const lgfx::v1::IFont*);

    // Text drawing functions
    void drawTitle(const String& title, uint16_t textColor = 0x0000, uint16_t backgroundColor = 0xFFFF);
    void drawCentered(const char* text, uint16_t backgroundColor);

    // Specialized drawing functions
    void drawTelem(const Telem& telem, uint32_t now, uint16_t backgroundColor = 0x0000);
    void drawVersion(const String& version, uint16_t backgroundColor = 0x0000);

    static uint16_t rainbowColor(float v);
    static uint16_t timeRainbow(uint32_t now);

    // Getters
    uint16_t getBorderWidth() const { return borderWidth_; }
    lgfx::v1::LGFX_Device* getLCD() const { return lcd_; }
    bool isRedrawRequired() const { return redrawRequired_; }
};
