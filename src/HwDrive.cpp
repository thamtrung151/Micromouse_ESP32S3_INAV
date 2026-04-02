#include "HwDrive.h"
#include "config.h"

static inline void ledcWriteCompat(int ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR)
  ledcWrite(ch, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void HwDrive::setupPwmPin(int pin, int ch) {
  pinMode(pin, OUTPUT);
  ledcSetup(ch, PWM_FREQ, PWM_RES);
  ledcAttachPin(pin, ch);
  ledcWriteCompat(ch, 0);
}

void HwDrive::writeIN12(int chIN1, int chIN2, int cmd) {
  cmd = constrain(cmd, -PWM_MAX, PWM_MAX);
  if (cmd > 0) {
    ledcWriteCompat(chIN1, (uint32_t)cmd);
    ledcWriteCompat(chIN2, 0);
  } else if (cmd < 0) {
    ledcWriteCompat(chIN1, 0);
    ledcWriteCompat(chIN2, (uint32_t)(-cmd));
  } else {
    ledcWriteCompat(chIN1, 0);
    ledcWriteCompat(chIN2, 0);
  }
}

void HwDrive::begin() {
  // channels cố định
  setupPwmPin(PIN_L_IN1, 0);
  setupPwmPin(PIN_L_IN2, 1);
  setupPwmPin(PIN_R_IN1, 2);
  setupPwmPin(PIN_R_IN2, 3);
  stop();
}

void HwDrive::stop() {
  ledcWriteCompat(0, 0);
  ledcWriteCompat(1, 0);
  ledcWriteCompat(2, 0);
  ledcWriteCompat(3, 0);
}

void HwDrive::brake() {
  ledcWriteCompat(0, PWM_MAX);
  ledcWriteCompat(1, PWM_MAX);
  ledcWriteCompat(2, PWM_MAX);
  ledcWriteCompat(3, PWM_MAX);
}

void HwDrive::setLeft(int cmd) {
  if (MOTOR_INV_LEFT) cmd = -cmd;
  writeIN12(0, 1, cmd);
}

void HwDrive::setRight(int cmd) {
  if (MOTOR_INV_RIGHT) cmd = -cmd;
  writeIN12(2, 3, cmd);
}

void HwDrive::rotateInPlace(int cmd) {
  setLeft(+cmd);
  setRight(-cmd);
}
