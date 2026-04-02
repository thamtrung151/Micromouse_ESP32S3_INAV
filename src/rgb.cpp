#include "config.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// Khởi tạo strip
static Adafruit_NeoPixel strip(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// Trạng thái hệ thống
enum RGBState {
  RGB_OFF = 0,
  RGB_READY_SOLID,
  RGB_SAVED_MAP_SOLID,
  RGB_START_BLINK,
  RGB_ARRIVED_SOLID,
  RGB_RAINBOW,
  RGB_FINGER_ARMING_BLINK,
  RGB_FINGER_READY_BLINK
};

static volatile RGBState currentState = RGB_OFF;
static bool fcReady = false;
static bool started = false;
static bool arrived = false;
static bool rainbowActive = false;
static bool savedMapReady = false;

// NEW: finger-start overlay states
static bool fingerArming = false;
static bool fingerReady = false;

// Blinking control
static unsigned long lastBlinkMs = 0;
static bool blinkOn = false;

// 5 Hz started/finger-ready blink
static const unsigned long blinkPeriodMs = 200;

// 10 Hz finger-arming blink
static const unsigned long fingerFastBlinkPeriodMs = 100;

// Rainbow control
static unsigned long lastRainbowMs = 0;
static uint8_t rainbowHue = 0; // 0..255
static unsigned long lastStepMs = 0;
static uint8_t rainbowStepIndex = 0;

static inline uint32_t rgbToColor(uint8_t r, uint8_t g, uint8_t b) {
  return strip.Color(r, g, b);
}

static uint32_t wheel(byte wheelPos) {
  wheelPos = 255 - wheelPos;
  if(wheelPos < 85) {
    return strip.Color(255 - wheelPos * 3, 0, wheelPos * 3);
  }
  if(wheelPos < 170) {
    wheelPos -= 85;
    return strip.Color(0, wheelPos * 3, 255 - wheelPos * 3);
  }
  wheelPos -= 170;
  return strip.Color(wheelPos * 3, 255 - wheelPos * 3, 0);
}

static uint32_t cfgColor(int idx) {
  switch(idx) {
    case 1: return rgbToColor(RGB_COLOR1_R, RGB_COLOR1_G, RGB_COLOR1_B);
    case 2: return rgbToColor(RGB_COLOR2_R, RGB_COLOR2_G, RGB_COLOR2_B);
    case 3: return rgbToColor(RGB_COLOR3_R, RGB_COLOR3_G, RGB_COLOR3_B);
    case 4: return rgbToColor(RGB_COLOR4_R, RGB_COLOR4_G, RGB_COLOR4_B);
    default: return rgbToColor(0,0,0);
  }
}

static void fillStrip(uint32_t color) {
  for(uint16_t i=0;i<strip.numPixels();i++) strip.setPixelColor(i, color);
  strip.show();
}

// NEW: resolve visible state by priority
static void RGB_applyState() {
  if (rainbowActive) {
    currentState = RGB_RAINBOW;
    return;
  }

  if (arrived) {
    currentState = RGB_ARRIVED_SOLID;
    fillStrip(cfgColor(2));
    return;
  }

  if (started) {
    currentState = RGB_START_BLINK;
    return;
  }

  if (fingerReady) {
    currentState = RGB_FINGER_READY_BLINK;
    return;
  }

  if (fingerArming) {
    currentState = RGB_FINGER_ARMING_BLINK;
    return;
  }

  if (savedMapReady) {
    currentState = RGB_SAVED_MAP_SOLID;
    fillStrip(cfgColor(4));
    return;
  }

  if (fcReady) {
    currentState = RGB_READY_SOLID;
    fillStrip(cfgColor(1));
    return;
  }

  currentState = RGB_OFF;
  fillStrip(rgbToColor(0,0,0));
}


void RGB_setRainbow(bool active) {
  rainbowActive = active;
  if (active) {
    lastRainbowMs = millis();
    lastStepMs = millis();
    rainbowHue = 0;
    rainbowStepIndex = 0;
    RGB_applyState();
    fillStrip(wheel(rainbowHue));
    return;
  }
  RGB_applyState();
}

void RGB_setSavedMapReady(bool ready) {
  savedMapReady = ready;
  RGB_applyState();
}



void RGB_init() {
  strip.begin();
  uint8_t bri = (uint8_t)constrain((int)(RGB_LED_BRIGHTNESS * 255.0f), 0, 255);
  strip.setBrightness(bri);
  strip.show();
  currentState = RGB_OFF;
  lastBlinkMs = millis();
  lastRainbowMs = millis();
}

void RGB_setFCReady(bool ready) {
  fcReady = ready;
  RGB_applyState();
}

void RGB_setStarted(bool s) {
  started = s;
  if (s) {
    lastBlinkMs = millis();
    blinkOn = true;
  }
  RGB_applyState();
}

void RGB_setArrived(bool a) {
  arrived = a;
  RGB_applyState();
}

// NEW
void RGB_setFingerArming(bool active) {
  fingerArming = active;
  if (active) {
    lastBlinkMs = millis();
    blinkOn = true;
  }
  RGB_applyState();
}

// NEW
void RGB_setFingerReady(bool active) {
  fingerReady = active;
  if (active) {
    lastBlinkMs = millis();
    blinkOn = true;
  }
  RGB_applyState();
}

void RGB_loop() {
  unsigned long now = millis();

  if(currentState == RGB_START_BLINK || currentState == RGB_FINGER_READY_BLINK) {
    if(now - lastBlinkMs >= blinkPeriodMs) {
      lastBlinkMs = now;
      blinkOn = !blinkOn;
      if(blinkOn) {
        fillStrip(savedMapReady ? cfgColor(4) : cfgColor(2));
      } else {
        fillStrip(rgbToColor(0,0,0));
      }
    }
    return;
  }

  if(currentState == RGB_FINGER_ARMING_BLINK) {
    if(now - lastBlinkMs >= fingerFastBlinkPeriodMs) {
      lastBlinkMs = now;
      blinkOn = !blinkOn;
      if(blinkOn) {
        fillStrip(savedMapReady ? cfgColor(4) : cfgColor(1));
      } else {
        fillStrip(rgbToColor(0,0,0));
      }
    }
    return;
  }

  if(currentState == RGB_RAINBOW) {
    if(RGB_RAINBOW_SMOOTH) {
      if(now - lastRainbowMs >= RGB_RAINBOW_SPEED_MS) {
        lastRainbowMs = now;
        rainbowHue++;
        uint32_t c = wheel(rainbowHue);
        fillStrip(c);
      }
    } else {
      if(now - lastStepMs >= RGB_RAINBOW_STEP_MS) {
        lastStepMs = now;
        const uint8_t anchors[7] = {0, 36, 72, 108, 144, 180, 216};
        uint8_t h = anchors[rainbowStepIndex % 7];
        rainbowStepIndex++;
        fillStrip(wheel(h));
      }
    }
    return;
  }
}
