#include "Odometry.h"
#include "config.h"
#include "AngleUtils.h"

void Odometry::begin() {
  _encL.attachFullQuad(ENC_L_A, ENC_L_B);
  _encR.attachFullQuad(ENC_R_A, ENC_R_B);
  reset();
}

void Odometry::reset() {
  _encL.clearCount();
  _encR.clearCount();
  _cL = _cR = 0;
  _pL = _pR = 0;
  _distMm = 0.0f;
  _encYawRad = 0.0f;
  _yawAligned = false;
  _yaw0_imu10 = 0;
  _encYaw10Abs = 0;
}

void Odometry::alignYawTo(int16_t yaw0_imu10) {
  _yaw0_imu10 = yaw0_imu10;
  _yawAligned = true;
  // cập nhật ngay
  _encYaw10Abs = _yaw0_imu10;
}

void Odometry::update() {
  _cL = _encL.getCount();
  _cR = _encR.getCount();

  int64_t dL = _cL - _pL;
  int64_t dR = _cR - _pR;
  _pL = _cL;
  _pR = _cR;

  float sL = (float)dL * MM_PER_COUNT;
  float sR = (float)dR * MM_PER_COUNT;

  _distMm += 0.5f * (sL + sR);

  // yaw change from diff distance
  float dYawRad = (sL - sR) / WHEELBASE_MM;
  _encYawRad += dYawRad;

  if (_yawAligned) {
    float encYawDeg = _encYawRad * (180.0f / PI_F);
    int32_t y10 = (int32_t)lroundf(encYawDeg * 10.0f) + (int32_t)_yaw0_imu10;
    _encYaw10Abs = wrapYaw10(y10);
  }
}
