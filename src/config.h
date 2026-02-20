#pragma once
#include <Arduino.h>

// BUILD MODE SELECTION

// Choose ONE at compile time.
// - MODE_SCRIPT: run the existing scripted motion demo.
// - MODE_AUTO:   run a basic micromouse flood-fill solver.


#define MODE_SCRIPT 1
#define MODE_AUTO   2

#ifndef RUN_MODE
  #define RUN_MODE MODE_SCRIPT
#endif

// AUTO TURN PRIMITIVE
// AutoRunner generates motion as a short script (turn + MoveCells).
// - AUTO_TURN_INPLACE: StepType::TurnDeg (stop-and-turn)
// - AUTO_TURN_DRIFT:   StepType::DriftDeg (moving arc, blends into next Move)

#define AUTO_TURN_INPLACE 0
#define AUTO_TURN_DRIFT   1

#ifndef AUTO_TURN_STYLE
  #define AUTO_TURN_STYLE AUTO_TURN_INPLACE
#endif

// BOARD / PINS

// Encoders (ESP32Encoder full-quad)
static constexpr int ENC_L_A = 7;
static constexpr int ENC_L_B = 6;
static constexpr int ENC_R_A = 8;
static constexpr int ENC_R_B = 9;

// Motors (TA6586 IN1/IN2)
static constexpr int PIN_L_IN1 = 11;
static constexpr int PIN_L_IN2 = 10;
static constexpr int PIN_R_IN1 = 12;
static constexpr int PIN_R_IN2 = 13;

static constexpr bool MOTOR_INV_LEFT  = false;
static constexpr bool MOTOR_INV_RIGHT = false;

// PWM
static constexpr int PWM_FREQ = 1000;   // Hz
static constexpr int PWM_RES  = 10;      // bits
static constexpr int PWM_MAX  = (1 << PWM_RES) - 1; // 255
// UART to FC (MSP v2)

static constexpr uint32_t UART_BAUD   = 115200;
static constexpr int UART_TX_PIN = 14;
static constexpr int UART_RX_PIN = 15;

// MSP function
static constexpr uint16_t MSP_ATTITUDE = 108;

// USER INPUT
// Start/Stop toggle button (active LOW). User stated IO11 pulled down to GND when pressed.
static constexpr int PIN_BTN_STARTSTOP = 35;
static constexpr uint32_t START_DELAY_MS = 700;

//RGB LED (WS2812)
// Chỉnh các giá trị theo phần cứng của bạn
#define RGB_LED_PIN            21        // IO21 như bạn yêu cầu
#define RGB_LED_COUNT          1         // Số LED WS2812 (đổi nếu cần)

// Định nghĩa 4 màu
#define RGB_COLOR1_R           0
#define RGB_COLOR1_G           255
#define RGB_COLOR1_B           0    // ví dụ: xanh lá

#define RGB_COLOR2_R           0
#define RGB_COLOR2_G           0
#define RGB_COLOR2_B           255  // ví dụ: xanh dương

#define RGB_COLOR3_R           255
#define RGB_COLOR3_G           0
#define RGB_COLOR3_B           0    // ví dụ: đỏ

#define RGB_COLOR4_R           255
#define RGB_COLOR4_G           255
#define RGB_COLOR4_B           0    // ví dụ: vàng

// Rainbow behaviour
//  - RGB_RAINBOW_SMOOTH = 1 -> chuyển mượt (hue wheel)
//  - RGB_RAINBOW_SMOOTH = 0 -> chuyển theo bước 7 màu (nhảy)
#define RGB_RAINBOW_SMOOTH     1
#define RGB_RAINBOW_STEP_MS    150      // ms giữa các bước trong chế độ 'step'
#define RGB_RAINBOW_SPEED_MS   20       // ms per hue increment trong chế độ 'smooth'

// Brightness (0.0f .. 1.0f)
#define RGB_BRIGHTNESS         0.6f


// IR SENSORS (ADC)
// Photosensor modules with 10k pull-up to 3V3 (inverted):
//   - Wall present   => ADC small (your typical: 350-400 @ 13-bit)
//   - No wall/open   => ADC large (your typical: 4000-7000 @ 13-bit)
//
// IR1..IR4 from left to right. IR2/IR3 look forward (slightly angled).
static constexpr int IR1_PIN = 1;   // left side
static constexpr int IR2_PIN = 2;   // front-left
static constexpr int IR3_PIN = 4;   // front-right
static constexpr int IR4_PIN = 5;   // right side

static constexpr int IR_ADC_BITS = 13; // 0..8191

// Wall detection thresholds. Because the sensor is inverted, a wall is detected when ADC < TH.
static constexpr uint16_t IR_SIDE_WALL_TH  = 8000; // ir1/ir4 < th => side wall
static constexpr uint16_t IR_FRONT_WALL_TH =7500; // ir2/ir3 < th => front wall

// IR low-pass filter time constant (seconds). Lower = more responsive, higher = smoother.
static constexpr float IR_LP_TAU_S = 0.020f;

// ============================
// WALL CORRECTION (CENTERING)
// ============================
// Enable yaw bias based on side IR during straight motion.
static constexpr bool WALL_CORR_ENABLE = true;

// Apply wall correction only when the straight-motion PWM is above this (avoid noise at low speed).
static constexpr uint16_t WALL_CORR_MIN_PWM = 80;

// For single-wall centering, keep the side sensor near this value when in corridor.
static constexpr uint16_t IR_SIDE_SETPOINT = 6000;

// Gains in yaw10 per ADC count.
// - BOTH walls: corr ~ k*(IR4-IR1)
// - ONE wall:   corr ~ k*(setpoint-IRx) with sign depending on side
static constexpr float WALL_BOTH_K_YAW10_PER_COUNT = 0.03f;
static constexpr float WALL_ONE_K_YAW10_PER_COUNT  =0.03f;

// Clamp wall correction (yaw10 = 0.1 deg)
static constexpr int16_t WALL_CORR_MAX_YAW10 = 40; // 6.0 deg


// CONTROL RATE
static constexpr uint32_t CONTROL_PERIOD_MS = 2;   // 500 Hz

// MATH / UNITS
static constexpr float  PI_F     = 3.1415926f;

// yaw10 = yaw * 10 (0.1 deg). wrap = 0..3599
static constexpr int16_t YAW_WRAP = 3600;

// ROBOT GEOMETRY
static constexpr float WHEEL_DIAMETER_MM     = 27.4f;
static constexpr float COUNTS_PER_OUTPUT_REV = 217.77777777f;
static constexpr float WHEELBASE_MM          = 51.7f;

static constexpr float MM_PER_COUNT = (PI_F * WHEEL_DIAMETER_MM) / COUNTS_PER_OUTPUT_REV;

// Cell size
static constexpr float CELL_MM = 176.0f;

// FUSION SENSORs
static constexpr float FUSION_ALPHA_IMU = 0.92f;

// TURN TUNING
static constexpr float TURN_KP = 0.82f;
static constexpr float TURN_KI = 0.0f;
static constexpr float TURN_KD = 0.042f;

static constexpr int   TURN_MAX_PWM = 250;
static constexpr int   TURN_MIN_PWM = 70;
static constexpr int16_t TURN_SLOW_ZONE_YAW10 = 500;
static constexpr int16_t TURN_TOL_YAW10       = 20;
static constexpr int   TURN_STABLE_CYCLES     = 20;

// FORWARD / HEADING
static constexpr int   FWD_CRUISE_PWM = 200;
static constexpr int   FWD_MIN_PWM    = 55;
static constexpr float FWD_SLOW_ZONE_MM = 140.0f;

static constexpr float HEADING_KP = 0.55f;
static constexpr float HEADING_KD = 0.06f;

static constexpr uint32_t POST_STEP_SETTLE_MS = 30;

// MOTION (các biến Motion.cpp hiện tại ĐANG DÙNG)
static constexpr float MOVE_V_MAX = 500.0f;
static constexpr float MOVE_A_ACC = 400.0f;      // mm/s^2
static constexpr float MOVE_A_DEC = 500.0f;      // mm/s^2

static constexpr float BRAKE_MARGIN_BASE_MM = 25.0f;
static constexpr float BRAKE_LAG_S          = 0.16f;
static constexpr float CREEP_ZONE_MM        = 30.0f;
static constexpr float V_CREEP_MM_S         = 70.0f;
static constexpr float POS_TOL_MM      = 3.5f;
static constexpr float V_STOP_MM_S     = 40.0f;

static constexpr float SPEED_TAU_S = 0.030f;

static constexpr int      PWM_KICK_MIN   = 70;
static constexpr int      PWM_RUN_MIN    = FWD_MIN_PWM;
static constexpr uint32_t KICK_MS        = 100;
static constexpr float    V_KICK_THRESH  = 80.0f;

static constexpr float SPEED_KP    = 0.55f;
static constexpr float SPEED_KI    = 0.050f;
static constexpr float SPEED_I_LIM = 120.0f;
static constexpr float SPEED_U_LIM = 220.0f;

static constexpr int MOVE_PWM_MAX      = 400; //Giới hạn cho V MAX, ảnh hưởng cả gia tốc
static constexpr int PWM_SLEW_PER_SEC  = 600;

static constexpr int HEADING_CORR_LIM  = 90;

// DriftDeg
static constexpr float DRIFT_ARC_MM_PER_90 = 180.0f;   // mm travelled during a 90deg drift

// Drift completion tolerances
static constexpr float  DRIFT_POS_TOL_MM     = 6.0f;   // mm
static constexpr int16_t DRIFT_YAW_TOL_YAW10 = 20;     

// Drift speed cap and heading gains
static constexpr float DRIFT_V_MAX = 450.0f;           // mm/s
static constexpr float DRIFT_HEADING_KP = 0.70f;
static constexpr float DRIFT_HEADING_KD = 0.040f;
static constexpr int   DRIFT_CORR_LIM   = 55;          // PWM

// BackAlign
static constexpr float BACK_ALIGN_MAX_MM    = 120.0f;
static constexpr int   BACK_ALIGN_PWM       = 55;      
static constexpr float BACK_STALL_V_MM_S    = 10.0f;   
static constexpr uint32_t BACK_STALL_MS     = 120;    
static constexpr float BACK_MIN_EXTRA_MM    = 1.0f;  

static constexpr float BACK_HEADING_KP = 0.60f;
static constexpr float BACK_HEADING_KD = 0.030f;
static constexpr int   BACK_CORR_LIM   = 35;