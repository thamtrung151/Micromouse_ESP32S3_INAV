#include "config.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// Khởi tạo strip
static Adafruit_NeoPixel strip(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// Trạng thái hệ thống
enum RGBState {
  RGB_OFF = 0,
  RGB_READY_SOLID,
  RGB_START_BLINK,
  RGB_ARRIVED_SOLID,
  RGB_RAINBOW
};

static volatile RGBState currentState = RGB_OFF;
static bool fcReady = false;
static bool started = false;
static bool arrived = false;

// Blinking control
static unsigned long lastBlinkMs = 0;
static bool blinkOn = false;
static const unsigned long blinkPeriodMs = 200; // 5 Hz -> period 200ms (100ms on / 100ms off)

// Rainbow control
static unsigned long lastRainbowMs = 0;
static uint8_t rainbowHue = 0; // 0..255
static unsigned long lastStepMs = 0;
static uint8_t rainbowStepIndex = 0;

// Helper: convert RGB -> 32-bit color
static inline uint32_t rgbToColor(uint8_t r, uint8_t g, uint8_t b) {
  return strip.Color(r, g, b);
}

// Wheel helper (0..255) -> RGB (nguồn: common NeoPixel wheel)
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

// Lấy màu cấu hình (color1..color4)
static uint32_t cfgColor(int idx) {
  switch(idx) {
    case 1: return rgbToColor(RGB_COLOR1_R, RGB_COLOR1_G, RGB_COLOR1_B);
    case 2: return rgbToColor(RGB_COLOR2_R, RGB_COLOR2_G, RGB_COLOR2_B);
    case 3: return rgbToColor(RGB_COLOR3_R, RGB_COLOR3_G, RGB_COLOR3_B);
    case 4: return rgbToColor(RGB_COLOR4_R, RGB_COLOR4_G, RGB_COLOR4_B);
    default: return rgbToColor(0,0,0);
  }
}

// Thiết lập toàn strip 1 màu
static void fillStrip(uint32_t color) {
  for(uint16_t i=0;i<strip.numPixels();i++) strip.setPixelColor(i, color);
  strip.show();
}

// API được export
void RGB_init() {
  strip.begin();
  uint8_t bri = (uint8_t)constrain((int)(RGB_BRIGHTNESS * 255.0f), 0, 255);
  strip.setBrightness(bri);
  strip.show();
  currentState = RGB_OFF;
  lastBlinkMs = millis();
  lastRainbowMs = millis();
}

void RGB_setFCReady(bool ready) {
  fcReady = ready;
  if(ready) {
    // theo yêu cầu: khi esp đã đọc được tín hiệu đúng từ fc (fc khởi tạo xong)
    // hiện solid màu xanh lá (sử dụng COLOR1)
    currentState = RGB_READY_SOLID;
    fillStrip(cfgColor(1));
  } else {
    currentState = RGB_OFF;
    fillStrip(rgbToColor(0,0,0));
  }
}

void RGB_setStarted(bool s) {
  started = s;
  if(s) {
    // bật blinking xanh lá theo chu kỳ 5Hz
    currentState = RGB_START_BLINK;
    lastBlinkMs = millis();
    blinkOn = true; // bắt đầu bằng ON
  } else {
    // nếu stop -> nếu fc ready thì về ready, nếu đã arrived giữ arrived
    if(arrived) {
      currentState = RGB_ARRIVED_SOLID;
      fillStrip(cfgColor(2)); // xanh dương
    } else if(fcReady) {
      currentState = RGB_READY_SOLID;
      fillStrip(cfgColor(1));
    } else {
      currentState = RGB_OFF;
      fillStrip(rgbToColor(0,0,0));
    }
  }
}

void RGB_setArrived(bool a) {
  arrived = a;
  if(a) {
    currentState = RGB_ARRIVED_SOLID;
    // khi đến đích đổi sang solid xanh dương (sử dụng COLOR2)
    fillStrip(cfgColor(2));
  } else {
    // nếu rời khỏi vùng đến đích
    if(started) {
      currentState = RGB_START_BLINK;
    } else if(fcReady) {
      currentState = RGB_READY_SOLID;
      fillStrip(cfgColor(1));
    } else {
      currentState = RGB_OFF;
      fillStrip(rgbToColor(0,0,0));
    }
  }
}

void RGB_loop() {
  unsigned long now = millis();

  // Blink handling (5Hz -> 200ms period)
  if(currentState == RGB_START_BLINK) {
    if(now - lastBlinkMs >= blinkPeriodMs) {
      lastBlinkMs = now;
      blinkOn = !blinkOn;
      if(blinkOn) {
        fillStrip(cfgColor(2)); // nhấp xanh dương
      } else {
        fillStrip(rgbToColor(0,0,0)); // tắt
      }
    }
    return;
  }

  // Solid states handled in state transition functions, but add fallback for rainbow
  if(currentState == RGB_RAINBOW) {
    if(RGB_RAINBOW_SMOOTH) {
      if(now - lastRainbowMs >= RGB_RAINBOW_SPEED_MS) {
        lastRainbowMs = now;
        rainbowHue++; // 0..255
        uint32_t c = wheel(rainbowHue);
        fillStrip(c);
      }
    } else {
      // step mode (7-color-ish cycle using wheel anchors)
      if(now - lastStepMs >= RGB_RAINBOW_STEP_MS) {
        lastStepMs = now;
        // cycle through 7 anchor hues roughly spaced
        const uint8_t anchors[7] = {0, 36, 72, 108, 144, 180, 216};
        uint8_t h = anchors[rainbowStepIndex % 7];
        rainbowStepIndex++;
        fillStrip(wheel(h));
      }
    }
    return;
  }

  // For other states we do nothing here (they already show solid color)
}
