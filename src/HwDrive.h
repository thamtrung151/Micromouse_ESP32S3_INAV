#pragma once
#include <Arduino.h>

class HwDrive {
public:
  void begin();
  void stop();

  // cmd: -255..255
  void setLeft(int cmd);
  void setRight(int cmd);
  void rotateInPlace(int cmd); // cmd>0: right turn

private:
  void setupPwmPin(int pin, int ch);
  void writeIN12(int chIN1, int chIN2, int cmd);
};
