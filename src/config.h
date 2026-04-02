#pragma once
#include <Arduino.h>

// CONTINUOUS AUTO
static constexpr float AUTO_CELL_MM = 176.0f;

static constexpr int   AUTO_STRAIGHT_PWM = 52;
static constexpr bool  AUTO_SPEED_PI_ENABLE = true;
static constexpr float AUTO_SPEED_TARGET_MM_S = 250.0f;
static constexpr float AUTO_SPEED_PI_ARM_MM = 00.0f;   // let the robot spool up open-loop
static constexpr float AUTO_SPEED_KP = 0.33f;           // pwm per (mm/s)
static constexpr float AUTO_SPEED_KI = 0.0023f;          // pwm per mm
static constexpr float AUTO_SPEED_I_LIM = 1200.0f;      // integral clamp in speed-error*second domain
static constexpr int   AUTO_SPEED_TRIM_LIM = 15;        // max +/- PWM trim from PI
static constexpr float AUTO_HEADING_KP = 0.345f;
static constexpr float AUTO_HEADING_KD = 0.0242f;
static constexpr int   AUTO_HEADING_CORR_LIM = 100;

static constexpr bool  AUTO_IR_GATE_ENABLE = false;

// Turn / goal braking.
static constexpr uint32_t AUTO_TURN_BRAKE_MS = 150;
static constexpr uint32_t AUTO_GOAL_BRAKE_MS = 0;

// Speed-run soft pre-turn (curve into the corner instead of stop+in-place 90).
// The robot begins curving before the turning center: trigger = center - LEAD_MM.
static constexpr bool  SPEEDRUN_PRETURN_ENABLE = false;
static constexpr float SPEEDRUN_PRETURN_LEAD_MM = 40.0f; // keep for map lookahead arming only
static constexpr float SPEEDRUN_PRETURN_FRONT_TRIGGER_MM = 130.0f;
static constexpr float SPEEDRUN_PRETURN_ARC_MM = 30.0f;
static constexpr int16_t SPEEDRUN_PRETURN_YAW_DEG = 90;
static constexpr int   SPEEDRUN_PRETURN_OUTER_PWM = 60;
static constexpr int   SPEEDRUN_PRETURN_INNER_PWM = 0;
static constexpr int   SPEEDRUN_PRETURN_CORR_LIM = 25;
static constexpr int16_t SPEEDRUN_PRETURN_YAW_TOL_YAW10 = 40;
static constexpr float SPEEDRUN_PRETURN_FINISH_EXTRA_MM = 10.0f;
static constexpr float SPEEDRUN_PRETURN_TRIGGER_TOL_MM = 3.0f;

// Front-wall braking in continuous AUTO.
static constexpr float AUTO_FRONT_BRAKE_ARM_FROM_CENTER_MM = 120.0f;
static constexpr float AUTO_FRONT_BRAKE_TH_MM = 100.0f;
static constexpr uint8_t AUTO_FRONT_BRAKE_CONFIRM_CYCLES = 1;
static constexpr uint32_t AUTO_FRONT_BRAKE_MS = 50;

// Physical re-anchor by encoder only. (căn tường sau)
static constexpr float AUTO_REAR_REANCHOR_BACK_MM = 109.0f; //105
static constexpr float AUTO_REAR_REANCHOR_TO_CENTER_MM = 59.0f;  //59

// Legacy rear-align constants kept only so unused Motion legacy code still builds.
static constexpr int   AUTO_REAR_ALIGN_PWM = 70;
static constexpr float AUTO_REAR_ALIGN_STALL_ARM_MM = 0.0f;
static constexpr float AUTO_REAR_ALIGN_STALL_V_MM_S = 10.0f;
static constexpr uint32_t AUTO_REAR_ALIGN_STALL_MS = 150;
static constexpr float AUTO_REAR_ALIGN_HEADING_KP = 0.33f;
static constexpr float AUTO_REAR_ALIGN_HEADING_KD = 0.030f;
static constexpr int   AUTO_REAR_ALIGN_CORR_LIM = 35;

// Encoders
static constexpr int ENC_L_A = 7;
static constexpr int ENC_L_B = 6;
static constexpr int ENC_R_A = 8;
static constexpr int ENC_R_B = 9;

// Motors
static constexpr int PIN_L_IN1 = 11;
static constexpr int PIN_L_IN2 = 10;
static constexpr int PIN_R_IN1 = 12;
static constexpr int PIN_R_IN2 = 13;

static constexpr bool MOTOR_INV_LEFT  = false;
static constexpr bool MOTOR_INV_RIGHT = false;

// PWM
static constexpr int PWM_FREQ = 1000;  
static constexpr int PWM_RES  = 10;      
static constexpr int PWM_MAX  = (1 << PWM_RES) - 1;
// MSPv2 Protocol

static constexpr uint32_t UART_BAUD   = 115200;
static constexpr int UART_TX_PIN = 14;
static constexpr int UART_RX_PIN = 15;

// MSP function
static constexpr uint16_t MSP_ATTITUDE = 108;

// Input
static constexpr int PIN_BTN_STARTSTOP = 35;
static constexpr uint32_t START_DELAY_MS = 700;

// Start function
static constexpr float FINGER_START_LEFT_MM = 45.0f; 
static constexpr float FINGER_START_FL_MM   = 60.0f;
static constexpr uint32_t FINGER_START_HOLD_MS = 200; 
static constexpr uint32_t FINGER_RELEASE_DELAY_MS = 700;


//WS2812 //hi
#define RGB_LED_PIN            21     
#define RGB_LED_COUNT          1    

// Define 4 colors
#define RGB_COLOR1_R           0
#define RGB_COLOR1_G           255
#define RGB_COLOR1_B           0  

#define RGB_COLOR2_R           0
#define RGB_COLOR2_G           0
#define RGB_COLOR2_B           255 

#define RGB_COLOR3_R           255
#define RGB_COLOR3_G           0
#define RGB_COLOR3_B           0 

#define RGB_COLOR4_R           255
#define RGB_COLOR4_G           255
#define RGB_COLOR4_B           0

// Rainbow behaviour
//  RGB_RAINBOW_SMOOTH = 1 -> chuyển mượt
//  RGB_RAINBOW_SMOOTH = 0 -> chuyển theo bước 7 màu
#define RGB_RAINBOW_SMOOTH     1
#define RGB_RAINBOW_STEP_MS    100 
#define RGB_RAINBOW_SPEED_MS   10  

#define RGB_LED_BRIGHTNESS     0.3f


// IR SENSORS
// IR1..IR4 from left to right
static constexpr int IR1_PIN = 2;  
static constexpr int IR2_PIN = 1;   
static constexpr int IR3_PIN = 5;  
static constexpr int IR4_PIN = 4; 

static constexpr int IR_ADC_BITS = 13; 

// Wall detection thresholds
static constexpr uint16_t IR_SIDE_WALL_TH  = 8000;
static constexpr uint16_t IR_FRONT_WALL_TH =8000;

// IR LPF time constant (seconds). Lower = more responsive, higher = smoother.
static constexpr float IR_LP_TAU_S = 0.005f;

// IR switch
#define PIN_IR_FET            18 

// IR LED power
#define IR_POWER              1.0f 

// IR carrier frequency
#define IR_CARRIER_FREQ_HZ    38000
#define IR_ENABLE_DELAY_MS    0

// WALL CORRECTION (slow heading-bias layer, no continuous PID)
// Motion::tickAutoStraight() remains the fast loop.
// AutoRunner updates a slow heading bias from IR and adds it on top of the
// straight base heading. This is safer for narrow-angle side IR sensors.
static constexpr bool WALL_CORR_ENABLE = true;

// --- Distance based wall detection (mm) ---
static constexpr float IR_SIDE_WALL_TH_MM  = 110.0f;
static constexpr float IR_FRONT_WALL_TH_MM = 130.0f;

// Desired side-wall distance measured from robot center.
static constexpr float IR_SIDE_SETPOINT_MM = 75.0f;
static constexpr float IR_LEFT_SETPOINT_MM  = IR_SIDE_SETPOINT_MM;
static constexpr float IR_RIGHT_SETPOINT_MM = IR_SIDE_SETPOINT_MM;

// Optional systematic bias for dual-wall error = left_mm - right_mm - bias.
static constexpr float WALL_BOTH_DIFF_BIAS_MM = 5.0f;

// Slow wall-bias controller tuning.
static constexpr uint32_t WALL_CORR_UPDATE_MS = 25;
static constexpr float WALL_CORR_PHASE_START_MM = 20.0f;
static constexpr float WALL_CORR_PHASE_END_MM   = 160.0f;

// Deg/mm gains used to build a heading bias command.
static constexpr float WALL_CORR_DUAL_GAIN_DEG_PER_MM   = 0.25f;
static constexpr float WALL_CORR_SINGLE_GAIN_DEG_PER_MM = 0.25f;

// Limit the commanded and applied heading bias.
static constexpr int16_t WALL_CORR_BIAS_MAX_DEG = 11;
static constexpr float WALL_CORR_SLEW_DEG_PER_S = 60.0f;

// When outside the valid window or walls disappear, bias decays back toward zero.
static constexpr float WALL_CORR_DECAY_DEG_PER_S = 15.0f;

// Keep only side distances in a trusted range when forming the wall observation.
static constexpr float WALL_CORR_VALID_MIN_MM = 37.5f;
static constexpr float WALL_CORR_VALID_MAX_MM = 130.0f;


// CONTROL RATE
static constexpr uint32_t CONTROL_PERIOD_US = 2000; // 500 Hz
static constexpr uint32_t CONTROL_PERIOD_MS = CONTROL_PERIOD_US / 1000;

// MATH / UNITS
static constexpr float  PI_F     = 3.1415926f;
static constexpr int16_t YAW_WRAP = 3600;

// ROBOT GEOMETRY
static constexpr float WHEEL_DIAMETER_MM     = 27.8f;
static constexpr float COUNTS_PER_OUTPUT_REV = 217.77777777f;
static constexpr float WHEELBASE_MM          = 51.7f;

static constexpr float MM_PER_COUNT = (PI_F * WHEEL_DIAMETER_MM) / COUNTS_PER_OUTPUT_REV;

// FUSION SENSORs
static constexpr float FUSION_ALPHA_IMU = 1.00f;

// TURN TUNING
static constexpr float TURN_KP = 0.88f;
static constexpr float TURN_KI = 0.0005f;
static constexpr float TURN_KD = 0.050f;

static constexpr int   TURN_MAX_PWM = 120;
static constexpr int   TURN_MIN_PWM = 75;
static constexpr int16_t TURN_SLOW_ZONE_YAW10 = 300;
static constexpr int16_t TURN_TOL_YAW10       = 10;
static constexpr int   TURN_STABLE_CYCLES     = 5;

// FORWARD / HEADINGSS
static constexpr int   FWD_CRUISE_PWM = 80;
static constexpr int   FWD_MIN_PWM    = 40;

static constexpr float HEADING_KP = 0.30f;
static constexpr float HEADING_KD = 0.03f;

// MOTION
static constexpr float MOVE_V_MAX = 500.0f;
static constexpr float MOVE_A_ACC = 400.0f;     
static constexpr float MOVE_A_DEC = 500.0f;  

static constexpr float BRAKE_MARGIN_BASE_MM = 25.0f;
static constexpr float BRAKE_LAG_S          = 0.16f;
static constexpr float CREEP_ZONE_MM        = 30.0f;
static constexpr float V_CREEP_MM_S         = 70.0f;
static constexpr float POS_TOL_MM      = 3.5f;
static constexpr float V_STOP_MM_S     = 40.0f;

static constexpr float SPEED_TAU_S = 0.030f;

static constexpr int      PWM_KICK_MIN   = 80;
static constexpr int      PWM_RUN_MIN    = FWD_MIN_PWM;
static constexpr uint32_t KICK_MS        = 100;
static constexpr float    V_KICK_THRESH  = 80.0f;

static constexpr float SPEED_KP    = 0.55f;
static constexpr float SPEED_KI    = 0.050f;
static constexpr float SPEED_I_LIM = 120.0f;
static constexpr float SPEED_U_LIM = 220.0f;

static constexpr int MOVE_PWM_MAX      = 400;
static constexpr int PWM_SLEW_PER_SEC  = 600;

static constexpr int HEADING_CORR_LIM  = 90;
