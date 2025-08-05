#include "displayhandler.h"
#include "utils.h"
#ifdef IS_M5
#include <M5Unified.h>
#endif

// Color definitions
const uint16_t SUPERDARKRED = lgfx::color565(50, 0, 0);
const uint16_t SUPERDARKBLUE = lgfx::color565(0, 0, 50);

DisplayHandler::DisplayHandler(lgfx::v1::LGFX_Device* lcd, uint16_t borderWidth)
    : lcd_(lcd), borderWidth_(borderWidth) {
}

void DisplayHandler::setLCD(lgfx::v1::LGFX_Device* lcd) {
    lcd_ = lcd;
}

void DisplayHandler::setBorderWidth(uint16_t width) {
    borderWidth_ = width;
    requestRedraw();
}

void DisplayHandler::requestRedraw() {
    redrawRequired_ = true;
}

void DisplayHandler::setRotation(int rotation) {
  if (lcd_->getRotation() != rotation) {
    lcd_->setRotation(rotation);
    requestRedraw();
  }
}

void DisplayHandler::startFrame() {
    if (lcd_) lcd_->startWrite();
}

void DisplayHandler::endFrame() {
    if (lcd_) lcd_->endWrite();
    redrawRequired_ = false;
}

void DisplayHandler::drawBorder(uint16_t color) {
    if (!lcd_) return;
    // Draw border: left, right, and bottom
    lcd_->fillRect(0, 0, borderWidth_, lcd_->height(), color); // vertical left
    lcd_->fillRect(lcd_->width() - borderWidth_, 0, borderWidth_, lcd_->height(), color); // vertical right
    lcd_->fillRect(0, lcd_->height() - borderWidth_, lcd_->width(), borderWidth_, color); // horizontal bottom
}

void DisplayHandler::setFont(const lgfx::v1::IFont* font) {
    if (lcd_) lcd_->setFont(font);
}

void DisplayHandler::clearContent(uint16_t backgroundColor, uint32_t now, bool force) {
    if (!lcd_) return;
    if (redrawRequired_ || force || (now - lastClear_) > 5000) {
        auto x = lcd_->getCursorX();
        auto y = lcd_->getCursorY();
        lcd_->fillRect(x, y, lcd_->width() - x - borderWidth_, lcd_->height() - y - borderWidth_, backgroundColor);
        lastClear_ = now;
        redrawRequired_ = false;
    }
}

void DisplayHandler::drawTitle(const String& title, uint16_t textColor, uint16_t backgroundColor) {
    if (!lcd_) return;
    lcd_->setCursor(borderWidth_, 0);
    lcd_->setTextColor(textColor, backgroundColor);
    drawCentered(title.c_str(), backgroundColor);
    lcd_->setCursor(borderWidth_, lcd_->getCursorY()); // indent after title
}

void DisplayHandler::drawCentered(const char* text, uint16_t backgroundColor) {
    if (!lcd_) return;

    auto titlewidth = lcd_->textWidth(text);
    auto titleheight = lcd_->fontHeight();
    auto starty = lcd_->getCursorY();

    // Use smaller font if text doesn't fit
    if (titlewidth > lcd_->width() - 4) {
        lcd_->setFont(&FreeMono9pt7b);
        titlewidth = lcd_->textWidth(text);
    }

    // Center the text
    lcd_->setCursor((lcd_->width() - titlewidth) / 2, starty);

    // Fill background on sides
    lcd_->fillRect(borderWidth_, starty, lcd_->getCursorX() - borderWidth_, lcd_->fontHeight(), backgroundColor);
    lcd_->print(text);
    lcd_->fillRect(lcd_->getCursorX(), starty, lcd_->width() - lcd_->getCursorX() - borderWidth_, lcd_->fontHeight(), backgroundColor);

    // Move cursor to next line
    lcd_->setCursor(borderWidth_, starty + titleheight);
}

void DisplayHandler::drawTelem(const Telem& telem, uint32_t now, uint16_t backgroundColor) {
    if (!lcd_) return;
    String status;
    bool stale = false;
    if ((now - telem.timestamp) > 1000) {
        status = "stale";
        stale = true;
    } else {
        status = String(telem.vbus, 1) + "V";
    }
    lcd_->setFont(&FreeSans18pt7b);
    lcd_->setTextColor(WHITE, backgroundColor);
    drawCentered(status.c_str(), backgroundColor);

    // Draw adjustment info if active
    if (!stale && telem.adjustSrc[0]) {
        lcd_->setTextColor(SUPERDARKBLUE, WHITE);
        lcd_->setFont(&FreeSansBold9pt7b);
        String adjustStr = String(telem.adjustSrc) + ":" + String(telem.adjusting, 3);
        drawCentered(adjustStr.c_str(), WHITE);
    }
}

void DisplayHandler::drawVersion(const String& version, uint16_t backgroundColor) {
    if (!lcd_) return;

    // Bottom aligned version text
    lcd_->setFont(&Font0);
    lcd_->setCursor(borderWidth_, lcd_->height() - borderWidth_ - lcd_->fontHeight());
    lcd_->setTextColor(WHITE, backgroundColor);
    drawCentered(version.c_str(), backgroundColor);
}

uint16_t DisplayHandler::timeRainbow(uint32_t now) {
    return rainbowColor((now >> 2) % 1000 / 1000.0);
}

uint16_t DisplayHandler::rainbowColor(float v) {
    float h = v * 6.0;
    float x = 1.0 - fabs(fmod(h, 2.0) - 1.0);
    return lgfx::color565(
        (h < 1 ? 1 : h < 2 ? x : h < 4 ? 0 : h < 5 ? x : 1) * 255,
        (h < 1 ? x : h < 3 ? 1 : h < 4 ? x : 0) * 255,
        (h < 2 ? 0 : h < 3 ? x : h < 5 ? 1 : x) * 255
    );
}
