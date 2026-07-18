// lib/leds/leds.h
#pragma once
#include <FastLED.h>
#include "log.h"

namespace leds {

constexpr uint8_t PANEL_SIZE = 8;
constexpr uint8_t PANEL_LEDS = PANEL_SIZE * PANEL_SIZE;
constexpr uint8_t NUM_PANELS = 4; // eyeL, eyeR, sideL, sideR

// fixed 8x8 heart bitmap, 1 = lit
constexpr uint8_t HEART_BITMAP[8] = {
  0b01100110,
  0b11111111,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100,
  0b00011000,
  0b00000000,
};

// Generalized panel orientation: covers all 8 dihedral transforms.
// swapXY=true, flipX=true, flipY=true reproduces the old hardcoded xy().
struct Orientation {
  bool flipX = false, flipY = false;
  bool swapXY = true;
  Orientation(bool flipXin = false, bool flipYin = false) : flipX(flipXin), flipY(flipYin) {}

  inline uint8_t xy(uint8_t x, uint8_t y) const {
    if (swapXY) { uint8_t t = x; x = y; y = t; }
    if (flipX) x = 7 - x;
    if (flipY) y = 7 - y;
    return y * 8 + x;
  }
};

enum class EyeShape : uint8_t { Circle, Heart };

class Eye {
  static constexpr uint16_t BLINK_MS = 140;
  CRGB* leds_ = nullptr;
  Orientation geom_;
  CRGB color_ = CRGB::Cyan;
  float size_ = 1.0f, targetSize_ = 1.0f;
  float openness_ = 1.0f;
  bool blinking_ = false;
  uint32_t blinkStart_ = 0;
  EyeShape shape_ = EyeShape::Circle;

public:
  void setBuffer(CRGB* l) { leds_ = l; }
  void setOrientation(Orientation o) { geom_ = o; }
  void setColor(CRGB c) { color_ = c; }
  void setSize(float s) { targetSize_ = constrain(s, 0.4f, 1.3f); }
  void widen(bool w) { setSize(w ? 1.3f : 1.0f); }
  void setShape(EyeShape s) { shape_ = s; }

  // blink() is called externally (LedRig owns the shared schedule so both eyes sync)
  void blink() { blinking_ = true; blinkStart_ = millis(); }

  void update(uint32_t now) {
    if (blinking_) {
      float t = (now - blinkStart_) / (float)BLINK_MS;
      if (t >= 1.0f) { blinking_ = false; openness_ = 1.0f; }
      else openness_ = (t < 0.5f) ? (1.0f - t * 2) : ((t - 0.5f) * 2);
    }
    size_ += (targetSize_ - size_) * 0.15f;
  }

  // draws over existing buffer content (fluid bg shows through outside pupil/eyelid)
  void render() {
    if (!leds_) return;
    uint8_t rowsClosed = (uint8_t)((1.0f - openness_) * 4);

    if (shape_ == EyeShape::Heart) {
      // heart ignores continuous size geometry (fixed bitmap); size_ instead pulses brightness
      uint8_t brightness = (uint8_t)constrain(180.0f + size_ * 60.0f, 0.0f, 255.0f);
      CRGB c = color_;
      c.nscale8_video(brightness);
      for (uint8_t y = 0; y < PANEL_SIZE; y++) {
        bool eyelidRow = (y < rowsClosed) || (y >= PANEL_SIZE - rowsClosed);
        for (uint8_t x = 0; x < PANEL_SIZE; x++) {
          if (eyelidRow) { leds_[geom_.xy(x, y)] = CRGB::Black; continue; }
          bool lit = (HEART_BITMAP[y] >> (7 - x)) & 0x01;
          if (lit) leds_[geom_.xy(x, y)] = c;
        }
      }
      return;
    }

    float r = 2.4f * size_;
    for (uint8_t y = 0; y < PANEL_SIZE; y++) {
      bool eyelidRow = (y < rowsClosed) || (y >= PANEL_SIZE - rowsClosed);
      for (uint8_t x = 0; x < PANEL_SIZE; x++) {
        if (eyelidRow) { leds_[geom_.xy(x, y)] = CRGB::Black; continue; }
        float dx = x - 3.5f, dy = y - 3.5f;
        if (sqrtf(dx * dx + dy * dy) <= r) leds_[geom_.xy(x, y)] = color_;
      }
    }
  }
};

// FluidPanel: a stochastic "falling sand" simulation, single or dual panel.
class FluidPanel {
  static constexpr uint8_t H = PANEL_SIZE;
  static constexpr uint8_t MAX_W = PANEL_SIZE * 2;
  static constexpr uint16_t MAX_CELLS = MAX_W * H;
  static constexpr uint32_t TICK_MS = 60;        // simulation step period
  static constexpr float ACCEL_DEADZONE = 0.04f; // ignore tiny noise near 0g

  uint8_t numPanels_ = 1;
  uint8_t w_ = PANEL_SIZE; // 8 or 16

  bool grid_[MAX_CELLS] = {false};
  uint8_t order_[MAX_CELLS] = {0}; // shuffled per-tick processing order

  CRGB* bufs_[2] = {nullptr, nullptr};
  Orientation geom_[2];

  CRGB color_ = CRGB::Blue;
  uint8_t fade_ = 255;

  uint32_t accMs_ = 0;
  float gx_ = 0.0f, gy_ = 1.0f; // continuous, normalized gravity direction

public:
  // single-panel setup
  void setBuffer(CRGB* l, Orientation o = Orientation()) {
    numPanels_ = 1; w_ = PANEL_SIZE;
    bufs_[0] = l; geom_[0] = o;
    seedFill();
  }
  // dual-panel setup: forms one wxH=16x8 grid spanning both buffers side by side
  void setBuffers(CRGB* left, Orientation lo, CRGB* right, Orientation ro) {
    numPanels_ = 2; w_ = PANEL_SIZE * 2;
    bufs_[0] = left;  geom_[0] = lo;
    bufs_[1] = right; geom_[1] = ro;
    seedFill();
  }

  void setColor(CRGB c) { color_ = c; }
  void setFade(uint8_t f) { fade_ = f; }

  // ax/ay: raw acceleration in g's, panel-local axes (whatever two components
  // the caller decides are "horizontal"/"vertical" for this panel).
  void update(float ax, float ay, uint32_t dtMs) {
    float mag = sqrtf(ax * ax + ay * ay);
    if (mag > ACCEL_DEADZONE) {
      gx_ = ax / mag;
      gy_ = ay / mag;
    }
    // if below deadzone, keep the last known direction rather than snapping
    // to (0,0), which would freeze the sim at rest - matches "1/3 glass of
    // water settles and stays put" rather than "glitches when nearly level".

    accMs_ += dtMs;
    if (accMs_ < TICK_MS) return;
    accMs_ = 0;
    step();
  }

  void render() {
    CRGB c = color_;
    c.nscale8_video(fade_);
    for (uint8_t y = 0; y < H; y++)
      for (uint8_t x = 0; x < w_; x++)
        writeCell(x, y, grid_[idx(x, y)] ? c : CRGB::Black);
  }

private:
  static inline uint16_t idx(uint8_t x, uint8_t y) { return (uint16_t)y * MAX_W + x; }

  void seedFill() {
    memset(grid_, 0, sizeof(grid_));
    uint16_t total = (uint16_t)w_ * H;
    uint16_t fillCount = total / 3; // ~1/3 full, "glass of water" starting point
    uint16_t placed = 0;
    for (int y = H - 1; y >= 0 && placed < fillCount; y--)
      for (uint8_t x = 0; x < w_ && placed < fillCount; x++) {
        grid_[idx(x, y)] = true; placed++;
      }
  }

  // routes a grid-local (x,y) to the correct physical buffer + its own orientation
  void writeCell(uint8_t x, uint8_t y, const CRGB& c) {
    if (numPanels_ == 1) { bufs_[0][geom_[0].xy(x, y)] = c; return; }
    if (x < PANEL_SIZE) bufs_[0][geom_[0].xy(x, y)] = c;
    else bufs_[1][geom_[1].xy(x - PANEL_SIZE, y)] = c;
  }

  inline bool inBounds(int x, int y) const {
    return x >= 0 && x < w_ && y >= 0 && y < H;
  }
  inline bool occupied(int x, int y) const {
    if (!inBounds(x, y)) return true; // walls count as occupied
    return grid_[idx((uint8_t)x, (uint8_t)y)];
  }

  // Fisher-Yates shuffle of the cell processing order. Cheap (<=128 elements)
  // and is what removes the scan-direction chain-hop artifact.
  void shuffleOrder(uint16_t total) {
    for (uint16_t i = 0; i < total; i++) order_[i] = (uint8_t)i;
    for (uint16_t i = total; i > 1; i--) {
      uint16_t j = random16(i);
      uint8_t t = order_[i - 1]; order_[i - 1] = order_[j]; order_[j] = t;
    }
  }

  void step() {
    static const int8_t NEIGHBORS[8][2] = {
      {0, 1}, {0, -1}, {1, 0}, {-1, 0},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };

    uint16_t total = (uint16_t)w_ * H;
    shuffleOrder(total);

    bool moved[MAX_CELLS] = {false};

    for (uint16_t k = 0; k < total; k++) {
      // order_ holds indices into a packed w_*H space; convert to x,y directly
      uint8_t x = order_[k] % w_;
      uint8_t y = order_[k] / w_;
      uint16_t cell = idx(x, y);
      if (!grid_[cell] || moved[cell]) continue;

      // score each unoccupied neighbor by alignment with gravity; pick the best.
      int8_t bestDx = 0, bestDy = 0;
      float bestScore = -999.0f;
      bool found = false;
      for (auto& n : NEIGHBORS) {
        int tx = x + n[0], ty = y + n[1];
        if (!inBounds(tx, ty)) continue;
        uint16_t tcell = idx((uint8_t)tx, (uint8_t)ty);
        if (grid_[tcell] || moved[tcell]) continue;
        // normalize neighbor direction (diagonals are length sqrt(2))
        float len = (n[0] != 0 && n[1] != 0) ? 1.4142136f : 1.0f;
        float score = (n[0] * gx_ + n[1] * gy_) / len;
        if (score > bestScore) { bestScore = score; bestDx = n[0]; bestDy = n[1]; found = true; }
      }

      // only move if the best option is actually downhill-ish; a small negative
      // threshold still allows sideways spreading (score ~0) but blocks cells
      // from climbing back "uphill" against gravity.
      if (found && bestScore > -0.2f) {
        int tx = x + bestDx, ty = y + bestDy;
        uint16_t tcell = idx((uint8_t)tx, (uint8_t)ty);
        grid_[cell] = false;
        grid_[tcell] = true;
        moved[tcell] = true;
      }
    }
  }
};

class LedRig {
  CRGB leds_[NUM_PANELS * PANEL_LEDS];
  Eye eyeL_, eyeR_;
  FluidPanel sideL_, sideR_; // single-panel each
  uint32_t lastUpdate_ = 0;

  // shared blink schedule so both eyes blink together
  uint32_t nextBlinkTime_ = 0;

public:
  void setup() {
    FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds_, NUM_PANELS * PANEL_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(8);

    sideL_.setBuffer(&leds_[0 * PANEL_LEDS], Orientation(false, true));
    sideR_.setBuffer(&leds_[1 * PANEL_LEDS]);
    eyeR_.setBuffer(&leds_[2 * PANEL_LEDS]);
    eyeL_.setBuffer(&leds_[3 * PANEL_LEDS]);

    nextBlinkTime_ = millis() + random(2000, 6000);

    D_LOG("%dx leds set up on pin %d", FastLED.size(), PIN_LEDS);
  }

  // ax = fwd/back g, ay = left/right g, az = up/down g (raw, unscaled g's).
  // Eyes face forward: horiz driver = ay, vert driver = az.
  // Side panels are ~perpendicular to face: horiz driver = ax, vert driver = az.
  // NOTE: if fluid pools "up" instead of down at rest, az's sign is inverted
  // for your mounting - flip the sign on the az terms below.
  void update(uint32_t now, float ax, float ay, float az) {
    uint32_t dt = lastUpdate_ ? (now - lastUpdate_) : 20;
    FastLED.clear();

    eyeL_.update(now); eyeR_.update(now);

    if (now > nextBlinkTime_) {
      eyeL_.blink(); eyeR_.blink();
      nextBlinkTime_ = now + random(2000, 6000);
    }

    sideL_.update(ax, az, dt);
    sideR_.update(ax, az, dt);

    eyeL_.render(); eyeR_.render();
    sideL_.render(); sideR_.render();

    FastLED.show();
    lastUpdate_ = now;
  }

  Eye& eyeL() { return eyeL_; }
  Eye& eyeR() { return eyeR_; }
  FluidPanel& sideL() { return sideL_; }
  FluidPanel& sideR() { return sideR_; }

  void setMoodColor(CRGB c, CRGB c2) {
    eyeL_.setColor(c); eyeR_.setColor(c);
    sideL_.setColor(c); sideR_.setColor(c);
  }
  void setEyeShape(EyeShape s) { eyeL_.setShape(s); eyeR_.setShape(s); }

  void set(CRGB c, CRGB c2, EyeShape sh) { setMoodColor(c, c2); setEyeShape(sh); }
};

} // namespace leds
