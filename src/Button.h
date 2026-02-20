#pragma once

#include <Arduino.h>

// Active-low momentary button with debounce and edge detection.
class Button {
public:
  void begin(uint8_t pin, uint32_t debounceMs = 25) {
    _pin = pin;
    _debounceMs = debounceMs;
    pinMode(_pin, INPUT_PULLUP);
    _stable = digitalRead(_pin);
    _lastRaw = _stable;
    _lastChangeMs = millis();
  }

  // Call frequently. Returns true exactly once per press event (LOW edge).
  bool pressed() {
    const bool raw = digitalRead(_pin);
    const uint32_t now = millis();
    if (raw != _lastRaw) {
      _lastRaw = raw;
      _lastChangeMs = now;
    }
    if (now - _lastChangeMs >= _debounceMs && raw != _stable) {
      _stable = raw;
      if (_stable == LOW) return true;
    }
    return false;
  }

  bool isDown() const { return _stable == LOW; }

private:
  uint8_t _pin = 0;
  uint32_t _debounceMs = 25;
  bool _stable = true;
  bool _lastRaw = true;
  uint32_t _lastChangeMs = 0;
};
