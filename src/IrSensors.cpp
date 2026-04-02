#include "IrSensors.h"

static constexpr uint8_t IR_PWM_CH = 7;
static bool ir_on = false;

// Lookup tables (ADC -> mm) for each IR sensor
static const uint16_t LUT1_adc[] = {592,1030,2255,3320,4125,4760,5310,5740,6060,6350,6590,6820,7005,7175,7325,7455,7570,7680,7780,7920};
static const uint16_t LUT1_mm[]  = {10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105};
static const uint16_t LUT2_adc[] = {477,567,1630,3022,4010,4745,5300,5725,6050,6320,6555,6760,6930,7090,7235,7375,7470,7630,7746,7958};
static const uint16_t LUT2_mm[]  = {10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105};
static const uint16_t LUT3_adc[] = {471,564,1550,2840,3815,4530,5030,5455,5805,6100,6350,6570,6765,6930,7080,7220,7340,7465,7575,7668};
static const uint16_t LUT3_mm[]  = {10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105};
static const uint16_t LUT4_adc[] = {470,770,2300,3440,4240,4920,5340,5710,6040,6310,6550,6730,6910,7080,7230,7345,7450,7600,7650,7720};
static const uint16_t LUT4_mm[]  = {10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100,105};

static inline float interp_adc_to_mm(const uint16_t* adc_table, const uint16_t* mm_table, size_t n, uint16_t adc) {
  if (adc <= adc_table[0]) return (float)mm_table[0];
  size_t i = 1;
  for (; i < n; ++i) {
    if (adc <= adc_table[i]) break;
  }
  if (i == n) return (float)mm_table[n - 1];
  float x0 = (float)adc_table[i - 1], x1 = (float)adc_table[i];
  float y0 = (float)mm_table[i - 1], y1 = (float)mm_table[i];
  if (x1 == x0) return y0;
  float t = ((float)adc - x0) / (x1 - x0);
  return y0 + t * (y1 - y0);
}

void IrSensors::begin() {
  // Setup IR FET PWM
  ledcSetup(IR_PWM_CH, IR_CARRIER_FREQ_HZ, 10); // 10-bit resolution
  ledcAttachPin(PIN_IR_FET, IR_PWM_CH);

  analogReadResolution(IR_ADC_BITS);
  pinMode(IR1_PIN, INPUT);
  pinMode(IR2_PIN, INPUT);
  pinMode(IR3_PIN, INPUT);
  pinMode(IR4_PIN, INPUT);

  // Start with IR ON by default; AUTO mode may gate it later.
  enableIR(true);

  // initialize with first read
  _ir1 = (uint16_t)analogRead(IR1_PIN);
  _ir2 = (uint16_t)analogRead(IR2_PIN);
  _ir3 = (uint16_t)analogRead(IR3_PIN);
  _ir4 = (uint16_t)analogRead(IR4_PIN);
}

void IrSensors::update(float dt) {
  if (dt <= 0.0f) return;
  const float alpha = dt / (IR_LP_TAU_S + dt);

  const uint16_t r1 = (uint16_t)analogRead(IR1_PIN);
  const uint16_t r2 = (uint16_t)analogRead(IR2_PIN);
  const uint16_t r3 = (uint16_t)analogRead(IR3_PIN);
  const uint16_t r4 = (uint16_t)analogRead(IR4_PIN);

  // IIR: y += a*(x-y)
  _ir1 = clampU16((int)lroundf((float)_ir1 + alpha * ((float)r1 - (float)_ir1)));
  _ir2 = clampU16((int)lroundf((float)_ir2 + alpha * ((float)r2 - (float)_ir2)));
  _ir3 = clampU16((int)lroundf((float)_ir3 + alpha * ((float)r3 - (float)_ir3)));
  _ir4 = clampU16((int)lroundf((float)_ir4 + alpha * ((float)r4 - (float)_ir4)));
}

float IrSensors::ir1_mm() const {
  const size_t n = sizeof(LUT1_adc) / sizeof(LUT1_adc[0]);
  float mm = interp_adc_to_mm(LUT1_adc, LUT1_mm, n, _ir1);
  return mm + 27.5f;
}
float IrSensors::ir2_mm() const {
  const size_t n = sizeof(LUT2_adc) / sizeof(LUT2_adc[0]);
  float mm = interp_adc_to_mm(LUT2_adc, LUT2_mm, n, _ir2);
  return mm + 44.0f;
}
float IrSensors::ir3_mm() const {
  const size_t n = sizeof(LUT3_adc) / sizeof(LUT3_adc[0]);
  float mm = interp_adc_to_mm(LUT3_adc, LUT3_mm, n, _ir3);
  return mm + 44.0f;
}
float IrSensors::ir4_mm() const {
  const size_t n = sizeof(LUT4_adc) / sizeof(LUT4_adc[0]);
  float mm = interp_adc_to_mm(LUT4_adc, LUT4_mm, n, _ir4);
  return mm + 27.5f;
}

bool IrSensors::leftWall() const  { return ir1_mm() < (float)IR_SIDE_WALL_TH_MM; }
bool IrSensors::rightWall() const { return ir4_mm() < (float)IR_SIDE_WALL_TH_MM; }
bool IrSensors::frontWall() const { return frontWallAt((float)IR_FRONT_WALL_TH_MM); }
bool IrSensors::frontWallAt(float thMm) const {
  return (ir2_mm() < thMm) && (ir3_mm() < thMm);
}

IrSensors::WallObservation IrSensors::observeWallError() const {
  WallObservation out;
  if (!WALL_CORR_ENABLE) return out;

  const float left_mm  = ir1_mm();
  const float right_mm = ir4_mm();
  const bool lw = leftWall();
  const bool rw = rightWall();

  const bool leftTrusted  = lw && (left_mm  >= WALL_CORR_VALID_MIN_MM) && (left_mm  <= WALL_CORR_VALID_MAX_MM);
  const bool rightTrusted = rw && (right_mm >= WALL_CORR_VALID_MIN_MM) && (right_mm <= WALL_CORR_VALID_MAX_MM);

  if (leftTrusted && rightTrusted) {
    out.valid = true;
    out.dualWall = true;
    // Positive error means left sensor sees farther than right sensor => robot is biased right.
    out.errorMm = (left_mm - right_mm) - WALL_BOTH_DIFF_BIAS_MM;
    return out;
  }

  if (leftTrusted) {
    out.valid = true;
    out.dualWall = false;
    out.errorMm = left_mm - IR_LEFT_SETPOINT_MM;
    return out;
  }

  if (rightTrusted) {
    out.valid = true;
    out.dualWall = false;
    out.errorMm = IR_RIGHT_SETPOINT_MM - right_mm;
    return out;
  }

  return out;
}

IrSensors::WallCorrectionSample IrSensors::captureWallCorrection() const {
  WallCorrectionSample out;
  const WallObservation obs = observeWallError();
  if (!obs.valid) return out;

  out.valid = true;
  out.dualWall = obs.dualWall;
  out.errorMm = obs.errorMm;

  const float gain = obs.dualWall ? WALL_CORR_DUAL_GAIN_DEG_PER_MM
                                  : WALL_CORR_SINGLE_GAIN_DEG_PER_MM;
  int deltaDeg = (int)lroundf(gain * obs.errorMm);
  deltaDeg = constrain(deltaDeg, -WALL_CORR_BIAS_MAX_DEG, WALL_CORR_BIAS_MAX_DEG);
  out.headingDeltaDeg = (int16_t)deltaDeg;
  out.restoreNextCell = false;
  return out;
}

int16_t IrSensors::wallYawBias10(uint16_t straightPwmCmd, float dt) const {
  (void)straightPwmCmd;
  (void)dt;
  return 0;
}

void IrSensors::resetWallController() const {
}

void IrSensors::enableIR(bool enable) {
  if (enable && !ir_on) {
    uint32_t duty = (uint32_t)(IR_POWER * 1023.0f);
    ledcWrite(IR_PWM_CH, duty);
    ir_on = true;
  }
  else if (!enable && ir_on) {
    ledcWrite(IR_PWM_CH, 0);
    ir_on = false;
  }
}

bool IrSensors::irEnabled() const {
  return ir_on;
}
