// lib/leds/leds.h
#pragma once
#include <FastLED.h>
#include "log.h"

namespace leds {

constexpr uint8_t PANEL_SIZE = 8;
constexpr uint8_t PANEL_LEDS = PANEL_SIZE * PANEL_SIZE;
constexpr uint8_t NUM_PANELS = 4; // eyeL, eyeR, sideL, sideR

static inline uint8_t xy(uint8_t x, uint8_t y) {
  uint8_t px = 7 - y;     // rotated x = flipped old y
  uint8_t py = 7 - x;     // rotated y = flipped old x
  return py * 8 + px;     // straight row indexing
}

class Eye {
  static constexpr uint16_t BLINK_MS = 140;
  CRGB* leds_ = nullptr;
  CRGB color_ = CRGB::Cyan;
  float size_ = 1.0f, targetSize_ = 1.0f;
  float openness_ = 1.0f;
  bool blinking_ = false;
  uint32_t blinkStart_ = 0;
  uint32_t nextBlinkTime_ = 0;

public:
  void setBuffer(CRGB* l) { leds_ = l; }
  void setColor(CRGB c) { color_ = c; }
  void setSize(float s) { targetSize_ = constrain(s, 0.4f, 1.3f); }
  void widen(bool w) { setSize(w ? 1.3f : 1.0f); }
  void blink() { blinking_ = true; blinkStart_ = millis(); }

  void update(uint32_t now) {
    if (blinking_) {
      float t = (now - blinkStart_) / (float)BLINK_MS;
      if (t >= 1.0f) { blinking_ = false; openness_ = 1.0f; }
      else openness_ = (t < 0.5f) ? (1.0f - t * 2) : ((t - 0.5f) * 2);
    } else if (now > nextBlinkTime_) {
      blink();
      nextBlinkTime_ = now + random(2000, 6000);
    }
    size_ += (targetSize_ - size_) * 0.15f;
  }

  // draws over existing buffer content (fluid bg shows through outside pupil/eyelid)
  void render() {
    if (!leds_) return;
    float r = 2.6f * size_;
    uint8_t rowsClosed = (uint8_t)((1.0f - openness_) * 4);
    for (uint8_t y = 0; y < PANEL_SIZE; y++) {
      bool eyelidRow = (y < rowsClosed) || (y >= PANEL_SIZE - rowsClosed);
      for (uint8_t x = 0; x < PANEL_SIZE; x++) {
        if (eyelidRow) { leds_[xy(x, y)] = CRGB::Black; continue; }
        float dx = x - 3.5f, dy = y - 3.5f;
        if (sqrtf(dx * dx + dy * dy) <= r) leds_[xy(x, y)] = color_;
      }
    }
  }
};

class FluidPanel {
  struct Particle { float x = 4, y = 4, vx = 0, vy = 0; };
  static constexpr uint8_t NUM_PARTICLES = 40;
  static constexpr float FORCE_SCALE = 3.0f;
  static constexpr float DAMPING = 0.97f;

  Particle particles_[NUM_PARTICLES];
  CRGB* leds_ = nullptr;
  CRGB color_ = CRGB::Blue;
  uint8_t fade_ = 255;
  bool fireMode_ = false;
  uint8_t heat_[PANEL_LEDS] = {0};

public:
  void setBuffer(CRGB* l) { leds_ = l; }
  void setColor(CRGB c) { color_ = c; }
  void setFade(uint8_t f) { fade_ = f; } // 0-255, for dimmed bg-behind-eyes use
  void setFireMode(bool on) { fireMode_ = on; if (on) memset(heat_, 0, sizeof(heat_)); }

  // ax/ay: lateral tilt/accel in roughly [-1,1]. energy: 0-1, how hard we're flinging (turn/speed magnitude)
  void update(float ax, float ay, float energy, uint32_t dtMs) {
    if (fireMode_) { updateFire(energy); return; }
    float dt = dtMs / 1000.0f;
    for (auto& p : particles_) {
      p.vx += ax * FORCE_SCALE * dt;
      p.vy += ay * FORCE_SCALE * dt;
      if (energy > 0.4f) {
        p.vx += (random(-100, 100) / 100.0f) * energy * 3.0f * dt;
        p.vy += (random(-100, 100) / 100.0f) * energy * 3.0f * dt;
      }
      p.vx *= DAMPING; p.vy *= DAMPING;
      p.x += p.vx * dt * 8; p.y += p.vy * dt * 8;
      if (p.x < 0) { p.x = 0; p.vx *= -0.5f; }
      if (p.x > 7) { p.x = 7; p.vx *= -0.5f; }
      if (p.y < 0) { p.y = 0; p.vy *= -0.5f; }
      if (p.y > 7) { p.y = 7; p.vy *= -0.5f; }
    }
  }

  void render() {
    if (!leds_) return;
    fadeToBlackBy(leds_, PANEL_LEDS, 90); // trail
    if (fireMode_) { renderFire(); return; }
    CRGB c = color_;
    c.nscale8_video(fade_);
    for (auto& p : particles_) splat(p.x, p.y, c);
  }

private:
  void addWeighted(int x, int y, float w, const CRGB& c) {
    if (x < 0 || x > 7 || y < 0 || y > 7 || w < 0.02f) return;
    leds_[xy(x, y)] += CRGB(c.r * w, c.g * w, c.b * w);
  }
  void splat(float fx, float fy, const CRGB& c) {
    int x0 = (int)fx, y0 = (int)fy;
    float tx = fx - x0, ty = fy - y0;
    addWeighted(x0,     y0,     (1 - tx) * (1 - ty), c);
    addWeighted(x0 + 1, y0,     tx       * (1 - ty), c);
    addWeighted(x0,     y0 + 1, (1 - tx) * ty,       c);
    addWeighted(x0 + 1, y0 + 1, tx       * ty,       c);
  }

  // Fire2012-style, adapted to 8x8: sparks bottom row, drifts upward, cools
  void updateFire(float energy) {
    uint8_t sparking = 100 + (uint8_t)(energy * 100);
    for (uint8_t x = 0; x < PANEL_SIZE; x++) {
      for (uint8_t y = 0; y < PANEL_SIZE - 1; y++) {
        uint8_t& h = heat_[y * PANEL_SIZE + x];
        h = qsub8(h, random8(0, 30));
      }
      for (uint8_t y = PANEL_SIZE - 1; y >= 2; y--)
        heat_[y * PANEL_SIZE + x] = (heat_[(y - 1) * PANEL_SIZE + x] + heat_[(y - 2) * PANEL_SIZE + x] * 2) / 3;
      if (random8() < sparking)
        heat_[x] = qadd8(heat_[x], random8(160, 255)); // row 0 = spark source
    }
  }

  void renderFire() {
    for (uint8_t x = 0; x < PANEL_SIZE; x++)
      for (uint8_t y = 0; y < PANEL_SIZE; y++)
        leds_[xy(x, PANEL_SIZE - 1 - y)] = HeatColor(heat_[y * PANEL_SIZE + x]); // row0=spark rises to top
  }
};


class LedRig {
  CRGB leds_[NUM_PANELS * PANEL_LEDS];
  Eye eyeL_, eyeR_;
  FluidPanel eyeBgL_, eyeBgR_, sideL_, sideR_;
  uint32_t lastUpdate_ = 0;

public:
  void setup() {
    FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds_, NUM_PANELS * PANEL_LEDS).setCorrection(TypicalLEDStrip); // set pin via template param at call site if neededs
    FastLED.setBrightness(8);

    sideL_.setBuffer(&leds_[0 * PANEL_LEDS]);
    sideR_.setBuffer(&leds_[1 * PANEL_LEDS]);
    eyeL_.setBuffer(&leds_[2 * PANEL_LEDS]);
    eyeR_.setBuffer(&leds_[3 * PANEL_LEDS]);

    eyeBgL_.setBuffer(&leds_[2 * PANEL_LEDS]);
    eyeBgR_.setBuffer(&leds_[3 * PANEL_LEDS]);
    eyeBgL_.setFade(60); eyeBgR_.setFade(60); // dim fluid behind eyes
    D_LOG("%dx leds set up on pin %d", FastLED.size(), PIN_LEDS);
  }

  // ax/ay: lateral/fwd tilt or accel. isBalancing/energy drive fluid behavior.
  void update(uint32_t now, float ax, float ay, bool isBalancing, float energy) {
    uint32_t dt = lastUpdate_ ? (now - lastUpdate_) : 20;

    eyeL_.update(now); eyeR_.update(now);

    // balancing: gentle pooling toward "down" tilt. driving hard: flingy
    eyeBgL_.update(ax, ay, energy, dt);
    eyeBgR_.update(ax, ay, energy, dt);
    sideL_.update(ax, ay, energy, dt);
    sideR_.update(ax, ay, energy, dt);

    eyeBgL_.render(); eyeL_.render();
    eyeBgR_.render(); eyeR_.render();
    sideL_.render();
    sideR_.render();

    FastLED.show();
    lastUpdate_ = now;
  }

  Eye& eyeL() { return eyeL_; }
  Eye& eyeR() { return eyeR_; }
  FluidPanel& sideL() { return sideL_; }
  FluidPanel& sideR() { return sideR_; }

  void setMoodColor(CRGB c) {
    eyeL_.setColor(c); eyeR_.setColor(c);
    eyeBgL_.setColor(c); eyeBgR_.setColor(c);
    sideL_.setColor(c); sideR_.setColor(c);
  }
  void setFireMode(bool on) { sideL_.setFireMode(on); sideR_.setFireMode(on); }

};

} // namespace behavior
