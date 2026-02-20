#ifndef IR_STANDALONE_TEST

#include <Arduino.h>

#include "config.h"
#include "Motion.h"
#include "Button.h"
#include <rgb.h>

#if (RUN_MODE == MODE_AUTO)
  #include "AutoRunner.h"
#endif

// ============================
// SCRIPT (kept compatible with your previous firmware)
// ============================
#if (RUN_MODE == MODE_SCRIPT)
static const Step script[] = {
  // Examples:
  // { StepType::DelayMs, 100, 0 },
  // { StepType::MoveCells, 1, 0 },
  // { StepType::TurnDeg, 90, +1 }, // +1 right, -1 left, tinh chỉnh góc bất kì
  // { StepType::DriftDeg, 90, +1 }, //Cua quanh tâm nằm ngoài robot, chọn góc bất kì, +1 right, -1 left
  // { StepType::BackAlign, 80, 0 }, //căn đuôi
  // { StepType::RunDistance, -40, 0 }, // khoảng cách theo mm
  { StepType::RunDistance, 540, 0 },
  

};

#endif

Motion motion;
Button btn;

#if (RUN_MODE == MODE_AUTO)
AutoRunner autoRunner;
#endif

static bool runEnabled = false;
static bool startPending = false;
static uint32_t startDueMs = 0;

void setup() {
  
  Serial.begin(115200);
  delay(200);

  btn.begin(PIN_BTN_STARTSTOP);
  motion.begin();
  motion.setEnabled(false);
  RGB_init();

#if (RUN_MODE == MODE_SCRIPT)
  motion.loadScript(script, sizeof(script) / sizeof(script[0]));
  Serial.println("Micromouse: SCRIPT mode ready. Press button to Start/Stop.");
  RGB_setFCReady(true);

#elif (RUN_MODE == MODE_AUTO)
  autoRunner.begin(motion);
  Serial.println("Micromouse: AUTO mode ready (flood-fill BFS). Press button to Start/Stop.");
  RGB_setFCReady(true);
#else
  #error "Invalid RUN_MODE. Use MODE_SCRIPT or MODE_AUTO."
#endif
}

void loop() {
  
  RGB_loop();
  // Toggle start/stop on button press
  if (btn.pressed()) {
    // If we are stopped -> arm a delayed start.
    // If we are running OR start is pending -> stop/cancel immediately.
    if (!runEnabled && !startPending) {
      startPending = true;
      startDueMs = millis() + START_DELAY_MS;
      // keep motion disabled during the delay window
      motion.setEnabled(false);
#if (RUN_MODE == MODE_AUTO)
      autoRunner.stop();
#endif
    } else {
      // Stop / cancel
      runEnabled = false;
      startPending = false;
      motion.setEnabled(false);
#if (RUN_MODE == MODE_AUTO)
      autoRunner.stop();
#endif
    }
  }

  // If Start was requested, enable motion after the requested delay.
  if (startPending && (int32_t)(millis() - startDueMs) >= 0) {
    startPending = false;
    runEnabled = true;
    motion.setEnabled(true);
    
    RGB_setStarted(true);

#if (RUN_MODE == MODE_AUTO)
    autoRunner.start();
#endif
  }

  // Always keep the motion system running its sensor fusion.
  motion.update();


#if (RUN_MODE == MODE_SCRIPT)
  static bool scriptDoneNotified = false;

  if (motion.done() && !scriptDoneNotified) {
    RGB_setArrived(true);        // <<< ĐẶT CHÍNH XÁC Ở ĐÂY
    scriptDoneNotified = true;   // đảm bảo chỉ gọi 1 lần
  }
#endif



#if (RUN_MODE == MODE_AUTO)
  // High-level planner
  autoRunner.update();
#endif

  // Light debug logging
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last > 100) {
    last = now;

    const auto& ir = motion.ir();

#if (RUN_MODE == MODE_SCRIPT)
    Serial.printf(
      "en=%d done=%d | yaw=%.1f deg | dist=%.1f mm | ir=[%u,%u,%u,%u]\n",
      (int)motion.enabled(), (int)motion.done(), motion.fusedYaw10()/10.0f,
      motion.distMm(), ir.ir1(), ir.ir2(), ir.ir3(), ir.ir4()
    );
#else
    Serial.printf(
      "en=%d run=%d goal=%d | pos=(%d,%d) hdg=%u | yaw=%.1f | ir=[%u,%u,%u,%u]\n",
      (int)motion.enabled(), (int)autoRunner.running(), (int)autoRunner.reachedGoal(),
      autoRunner.x(), autoRunner.y(), (unsigned)autoRunner.heading(),
      motion.fusedYaw10()/10.0f, ir.ir1(), ir.ir2(), ir.ir3(), ir.ir4()
    );
#endif
  }
}

#endif // !IR_STANDALONE_TEST
