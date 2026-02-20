#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>

class Odometry {
public:
  void begin();
  void reset();

  void update(); // gọi mỗi tick

  // distance from start (mm)
  float distMm() const { return _distMm; }

  // encoder-based yaw absolute (yaw10), anchored to _yaw0 when alignment happens
  bool hasYaw() const { return _yawAligned; }
  int16_t encYaw10() const { return _encYaw10Abs; }

  // call once when IMU yaw becomes valid: align encoder yaw to IMU yaw
  void alignYawTo(int16_t yaw0_imu10);

  // raw counts
  int64_t countL() const { return _cL; }
  int64_t countR() const { return _cR; }

private:
  ESP32Encoder _encL, _encR;

  int64_t _cL = 0, _cR = 0;
  int64_t _pL = 0, _pR = 0;

  float _distMm = 0.0f;
  float _encYawRad = 0.0f;

  bool _yawAligned = false;
  int16_t _yaw0_imu10 = 0;
  int16_t _encYaw10Abs = 0;
};
