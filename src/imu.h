#pragma once
#include <Arduino.h>

class ImuMsp {
public:
  void begin();

  void update();

  void resetHeading();

  bool waitReady(uint32_t timeoutMs = 4000, uint8_t goodFrames = 8);

  bool hasYaw() const { return _valid; }
  int16_t yaw10() const { return _yaw10; }
  uint32_t lastUpdateMs() const { return _lastMs; }

private:
  // MSPv2 helpers
  uint8_t crcDvbS2(const uint8_t *data, uint16_t len);
  uint16_t createMspRequest(uint16_t function, uint8_t *out);
  int readMspV2Frame(uint8_t *buf, uint16_t maxLen, uint32_t timeoutMs);
  int readMspV2FrameSync(uint8_t *buf, uint16_t maxLen, uint32_t timeoutMs);

  bool requestYawOnce(int16_t &yaw10_out);

private:
  HardwareSerial _fc = HardwareSerial(1);
  uint32_t _lastReqMs = 0;

  bool _valid = false;
  // yaw10 trả về theo hệ quy chiếu đã reset
  int16_t _yaw10 = 0;
  // yaw10 thô đọc từ FC (0..3599)
  int16_t _rawYaw10 = 0;
  // offset dùng để đưa yaw hiện tại về 0 (raw - offset)
  int16_t _yawOffset10 = 0;
  bool _haveOffset = false;
  uint32_t _lastMs = 0;
};
