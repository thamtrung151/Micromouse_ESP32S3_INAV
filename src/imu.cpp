#include "imu.h"
#include "config.h"
#include "AngleUtils.h"
#include "rgb.h"

#ifndef FC_BOOT_SETTLE_MS
#define FC_BOOT_SETTLE_MS 2000
#endif

//CRC
uint8_t ImuMsp::crcDvbS2(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
      else crc <<= 1;
    }
  }
  return crc;
}

//MSP
uint16_t ImuMsp::createMspRequest(uint16_t function, uint8_t *out) {
  out[0] = '$';
  out[1] = 'X';
  out[2] = '<';
  out[3] = 0x00;                 // flag
  out[4] = function & 0xFF;
  out[5] = function >> 8;
  out[6] = 0x00;                 // payload size L
  out[7] = 0x00;                 // payload size H
  out[8] = crcDvbS2(&out[3], 5); // CRC over: flag + func(2) + size(2)
  return 9;
}

//MSPv2 frame reader (SYNC + CRC)
// Frame: '$' 'X' '>' flags funcL funcH sizeL sizeH payload... crc
int ImuMsp::readMspV2FrameSync(uint8_t *buf, uint16_t maxLen, uint32_t timeoutMs) {
  uint32_t t0 = millis();

  // 1) Sync to "$X>"
  uint8_t s0 = 0, s1 = 0, s2 = 0;
  while (millis() - t0 < timeoutMs) {
    if (!_fc.available()) { delay(1); continue; }
    s0 = s1; s1 = s2; s2 = (uint8_t)_fc.read();
    if (s0 == '$' && s1 == 'X' && s2 == '>') {
      buf[0] = '$'; buf[1] = 'X'; buf[2] = '>';
      break;
    }
  }
  if (millis() - t0 >= timeoutMs) return 0;

  // 2) Read flags..size => bytes [3..7] (5 bytes)
  uint16_t idx = 3;
  while (idx < 8 && millis() - t0 < timeoutMs) {
    if (_fc.available()) buf[idx++] = (uint8_t)_fc.read();
    else delay(1);
  }
  if (idx < 8) return 0;

  uint16_t payloadSize = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);
  uint16_t totalLen = 8 + payloadSize + 1; // header(8) + payload + crc(1)
  if (totalLen > maxLen) return 0;

  // 3) Read payload + crc
  while (idx < totalLen && millis() - t0 < timeoutMs) {
    if (_fc.available()) buf[idx++] = (uint8_t)_fc.read();
    else delay(1);
  }
  if (idx != totalLen) return 0;

  // 4) CRC check: calc over [flags..payload] => from buf[3], length = totalLen - 4
  uint8_t crcCalc = crcDvbS2(&buf[3], (uint16_t)(totalLen - 4));
  uint8_t crcRx   = buf[totalLen - 1];
  if (crcCalc != crcRx) return 0;

  return (int)totalLen;
}

//Read yaw once
bool ImuMsp::requestYawOnce(int16_t &yaw10_out) {
  uint8_t tx[16];
  uint8_t rx[64];

  const uint16_t txLen = createMspRequest(MSP_ATTITUDE, tx);

  // Flush input BEFORE sending request
  while (_fc.available()) (void)_fc.read();

  _fc.write(tx, txLen);

  // Timeout nên hơi dư một chút lúc FC mới boot
  const int rxLen = readMspV2FrameSync(rx, sizeof(rx), 120);
  if (rxLen <= 0) return false;

  const uint16_t function = (uint16_t)rx[4] | ((uint16_t)rx[5] << 8);
  const uint16_t payloadSize = (uint16_t)rx[6] | ((uint16_t)rx[7] << 8);

  if (function != MSP_ATTITUDE) return false;
  if (payloadSize < 6) return false;

  const uint8_t *payload = &rx[8];

  const int16_t rawYawDeg = (int16_t)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8));
  yaw10_out = wrapYaw10((int32_t)rawYawDeg * 10);

  return true;
}

//Heading reset
void ImuMsp::resetHeading() {
  // Nếu chưa valid, thử đọc một mẫu ngay
  if (!_valid) {
    int16_t y10;
    if (requestYawOnce(y10)) {
      _rawYaw10 = y10;
      _valid = true;
      _lastMs = millis();
    } else {
      return; // không reset được nếu chưa có dữ liệu yaw
    }
  }

  _yawOffset10 = _rawYaw10;
  _haveOffset = true;
  _yaw10 = 0;
}

//Wait FC ready

bool ImuMsp::waitReady(uint32_t timeoutMs, uint8_t goodFrames) {
  const uint32_t start = millis();
  uint32_t firstOkMs = 0;

  uint8_t okStreak = 0;
  uint8_t stableStreak = 0;

  int16_t lastYaw10 = 0;
  bool haveLast = false;

  while (millis() - start < timeoutMs) {
    int16_t y10;
    bool ok = requestYawOnce(y10);

    if (ok) {
      _rawYaw10 = y10;
      _yaw10 = _haveOffset ? wrapYaw10((int32_t)_rawYaw10 - (int32_t)_yawOffset10)
                           : _rawYaw10;
      _valid = true;
      _lastMs = millis();

      if (firstOkMs == 0) {
        firstOkMs = millis();
        okStreak = 0;
        stableStreak = 0;
        haveLast = false;
      }

      okStreak++;

      // stability check: yaw không được nhảy quá lớn giữa các mẫu
      if (!haveLast) {
        haveLast = true;
        lastYaw10 = y10;
        stableStreak = 0;
      } else {
        int16_t d = yawDiff10(y10, lastYaw10); // signed shortest
        lastYaw10 = y10;

        // Cho phép rung nhẹ
        if (abs(d) <= 15) stableStreak++;
        else stableStreak = 0;
      }

      // điều kiện READY:
      if (firstOkMs && (millis() - firstOkMs >= FC_BOOT_SETTLE_MS) &&
          okStreak >= goodFrames && stableStreak >= 6) {
        return true;
      }
    } else {
      okStreak = 0;
      stableStreak = 0;
      haveLast = false;
    }

    // Poll 10Hz
    delay(100);
  }

  return _valid;
}

// ================= begin/update =================
void ImuMsp::begin() {
  _fc.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);


  uint32_t t0 = millis();
  while (millis() - t0 < 80) {
    while (_fc.available()) (void)_fc.read();
    delay(1);
  }

  _valid = false;
  _yaw10 = 0;
  _rawYaw10 = 0;
  _yawOffset10 = 0;
  _haveOffset = false;
  _lastReqMs = 0;
  _lastMs = 0;

  // Chờ FC ready
  (void)waitReady(7000, 8);

  // Quy ước: sau khi boot xong, reset heading một lần
  resetHeading();
  RGB_setFCReady(true);
}

void ImuMsp::update() {
  const uint32_t now = millis();

  // Khi đã chạy bình thường, poll 333Hz
  if (now - _lastReqMs < 3) return;
  _lastReqMs = now;

  int16_t y10;
  if (requestYawOnce(y10)) {
    _rawYaw10 = y10;
    _yaw10 = _haveOffset ? wrapYaw10((int32_t)_rawYaw10 - (int32_t)_yawOffset10)
                         : _rawYaw10;
    _lastMs = now;
    _valid = true;
  }
}
