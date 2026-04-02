#ifndef IR_STANDALONE_TEST

#include <Arduino.h>

#include "config.h"
#include "Motion.h"
#include "Button.h"
#include <rgb.h>
#include "AutoRunner.h"

Motion motion;
Button btn;

AutoRunner autoRunner;

static bool startPending = false;
static uint32_t startDueMs = 0;

enum class AutoStartMode : uint8_t {
  Explore = 0,
  SpeedRun
};

static AutoStartMode pendingAutoStartMode = AutoStartMode::Explore;

enum class FingerStartState : uint8_t {
  Idle = 0,
  Detecting,
  ArmedRelease
};

static FingerStartState fingerStartState = FingerStartState::Idle;
static uint32_t fingerStartDetectMs = 0;

static bool fingerNearStartSensors(Motion& motion) {
  const IrSensors& ir = motion.ir();
  return (ir.ir1_mm() <= FINGER_START_LEFT_MM) &&
         (ir.ir2_mm() <= FINGER_START_FL_MM);
}

static bool fingerStartTriggered(Motion& motion) {
  const bool fingerNear = fingerNearStartSensors(motion);

  switch (fingerStartState) {
    case FingerStartState::Idle:
      if (fingerNear) {
        fingerStartState = FingerStartState::Detecting;
        fingerStartDetectMs = millis();
        RGB_setFingerArming(true);
        RGB_setFingerReady(false);
      }
      return false;

    case FingerStartState::Detecting:
      if (!fingerNear) {
        fingerStartState = FingerStartState::Idle;
        fingerStartDetectMs = 0;
        RGB_setFingerArming(false);
        RGB_setFingerReady(false);
        return false;
      }

      if ((uint32_t)(millis() - fingerStartDetectMs) >= FINGER_START_HOLD_MS) {
        fingerStartState = FingerStartState::ArmedRelease;
        RGB_setFingerArming(false);
        RGB_setFingerReady(true);
      }
      return false;

    case FingerStartState::ArmedRelease:
      if (!fingerNear) {
        fingerStartState = FingerStartState::Idle;
        fingerStartDetectMs = 0;
        RGB_setFingerArming(false);
        RGB_setFingerReady(false);
        return true;
      }
      return false;
  }

  return false;
}

static void fingerStartCancelVisuals() {
  fingerStartState = FingerStartState::Idle;
  fingerStartDetectMs = 0;
  RGB_setFingerArming(false);
  RGB_setFingerReady(false);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  btn.begin(PIN_BTN_STARTSTOP);
  motion.begin();
  motion.setEnabled(false);
  RGB_init();
  autoRunner.begin(motion);
  Serial.println("Micromouse: AUTO mode ready. Finger start explores or speed-runs saved map; button clears saved map.");
  RGB_setFCReady(true);
}

void loop() {
  RGB_loop();

  // Always keep fusion and sensors alive.
  motion.update();
  const bool autoBusy = autoRunner.running();

  if (btn.pressed() && !autoBusy) {
    startPending = false;
    motion.setEnabled(false);
    fingerStartCancelVisuals();
    const bool cleared = autoRunner.clearSavedMap();
    Serial.println(cleared ? "Saved maze cleared." : "Saved maze clear failed.");
  }

  if (!autoBusy && !startPending && fingerStartTriggered(motion)) {
    startPending = true;
    startDueMs = millis() + FINGER_RELEASE_DELAY_MS;
    pendingAutoStartMode = autoRunner.savedMapReady()
                             ? AutoStartMode::SpeedRun
                             : AutoStartMode::Explore;
    motion.setEnabled(false);
  }

  if (startPending && (int32_t)(millis() - startDueMs) >= 0) {
    startPending = false;

    const bool startedOk = (pendingAutoStartMode == AutoStartMode::SpeedRun)
                             ? autoRunner.startSpeedRun()
                             : autoRunner.startExplore();
    if (startedOk) {
      RGB_setStarted(true);
    } else {
      RGB_setStarted(false);
      RGB_setSavedMapReady(autoRunner.savedMapReady());
      Serial.println("AUTO launch rejected.");
    }
  }
  autoRunner.update();

  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last > 50) {
    last = now;

    const auto& ir = motion.ir();
    Serial.printf(
      "run=%d saved=%d pos=(%d,%d) hdg=%u | yaw=%.1f | mm=[%.1f,%.1f,%.1f,%.1f]\n",
      (int)autoRunner.running(), (int)autoRunner.savedMapReady(),
      autoRunner.x(), autoRunner.y(), (unsigned)autoRunner.heading(),
      motion.fusedYaw10()/10.0f,
      ir.ir1_mm(), ir.ir2_mm(), ir.ir3_mm(), ir.ir4_mm()
    );
  }
}

#endif  // !IR_STANDALONE_TEST

void debugPrintIRmm(IrSensors &ir){
  Serial.print(ir.ir1_mm()); Serial.print(",");
  Serial.print(ir.ir2_mm()); Serial.print(",");
  Serial.print(ir.ir3_mm()); Serial.print(",");
  Serial.println(ir.ir4_mm());
}
