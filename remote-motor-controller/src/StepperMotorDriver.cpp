/**
 * StepperMotorDriver.cpp
 * L6470ステッピングモータドライバ実装
 * 
 * @author SDDL Project
 * @date 2026/01/18
 */

#include "StepperMotorDriver.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

StepperMotorDriver::StepperMotorDriver() 
  : motor_(STEPPER_PIN_CS),
    currentSpeed_(0.0f),
    targetSpeed_(0.0f),
    currentStepMode_(STEP_128),
    isRunning_(false) {
}

StepperMotorDriver::~StepperMotorDriver() {
  if (isRunning_) {
    softStop();
  }
}

// ============================================================================
// 公開メソッド
// ============================================================================

bool StepperMotorDriver::init() {
  Serial.println("[StepperMotor] Initializing L6470...");
  Serial.printf("[StepperMotor] Pin Config - CS:%d SCK:%d MOSI:%d MISO:%d RST:%d BUSY:%d\n",
                STEPPER_PIN_CS, STEPPER_PIN_SPI_SCK, STEPPER_PIN_SPI_MOSI, 
                STEPPER_PIN_SPI_MISO, STEPPER_PIN_RESET, STEPPER_PIN_BUSY);
  
  // ピン設定
  Serial.println("[StepperMotor] Setting pins...");
  motor_.set_pins(STEPPER_PIN_SPI_SCK, STEPPER_PIN_SPI_MOSI, 
                  STEPPER_PIN_SPI_MISO, STEPPER_PIN_RESET, STEPPER_PIN_BUSY);
  Serial.println("[StepperMotor] Pins set successfully");
  
  // 初期化
  Serial.println("[StepperMotor] Calling motor.init()...");
  motor_.init();
  Serial.println("[StepperMotor] motor.init() completed");
  delay(100);
  
  // L6470パラメータ設定
  Serial.println("[StepperMotor] Configuring parameters...");
  motor_.setMicroSteps(8);  // 固定で8分割
  Serial.println("[StepperMotor]   - MicroSteps: 8 (fixed)");
  
  motor_.setAcc(MOTOR_ACCELERATION);
  Serial.printf("[StepperMotor]   - Acceleration: %d\n", MOTOR_ACCELERATION);
  
  motor_.setMaxSpeed(MOTOR_MAX_SPEED);
  Serial.printf("[StepperMotor]   - MaxSpeed: %d\n", MOTOR_MAX_SPEED);
  
  motor_.setMinSpeed(MOTOR_MIN_SPEED);
  Serial.printf("[StepperMotor]   - MinSpeed: %d\n", MOTOR_MIN_SPEED);
  
  motor_.setThresholdSpeed(MOTOR_THRESHOLD_SPEED);
  Serial.printf("[StepperMotor]   - ThresholdSpeed: %d\n", MOTOR_THRESHOLD_SPEED);
  
  motor_.setOverCurrent(OCD_THRESHOLD);
  Serial.printf("[StepperMotor]   - OverCurrent: %d mA\n", OCD_THRESHOLD);
  
  motor_.setStallCurrent(STALL_CURRENT);
  Serial.printf("[StepperMotor]   - StallCurrent: %d mA\n", STALL_CURRENT);
  
  // KVAL設定
  Serial.println("[StepperMotor]   - Setting KVAL parameters...");
  motor_.SetParam(0x09, KVAL_PARAM);  // KVAL_HOLD
  motor_.SetParam(0x0A, KVAL_PARAM);  // KVAL_RUN
  motor_.SetParam(0x0B, KVAL_PARAM);  // KVAL_ACC
  motor_.SetParam(0x0C, KVAL_PARAM);  // KVAL_DEC
  Serial.printf("[StepperMotor]   - KVAL: 0x%02X\n", KVAL_PARAM);
  
  Serial.println("[StepperMotor] L6470 initialized successfully");
  
  currentStepMode_ = STEP_8;
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
  
  return true;
}

bool StepperMotorDriver::setSpeed(float speed, StepMode& currentStepMode) {
  Serial.printf("[StepperMotor] setSpeed called: %.2f step/s\n", speed);
  
  targetSpeed_ = speed;
  currentStepMode = currentStepMode_;
  
  // 速度設定
  if (abs(speed) < 1.0f) {
    Serial.println("[StepperMotor] Stopping motor (speed < 1.0)");
    motor_.softStop();
    isRunning_ = false;
    currentSpeed_ = 0.0f;
  } else {
    float absSpeed = abs(speed);
    
    if (speed > 0) {
      Serial.printf("[StepperMotor] Running forward at %.2f step/s\n", absSpeed);
      motor_.run(1, absSpeed);  // dSPIN_FWD = 1
    } else {
      Serial.printf("[StepperMotor] Running reverse at %.2f step/s\n", absSpeed);
      motor_.run(0, absSpeed);  // dSPIN_REV = 0
    }
    
    isRunning_ = true;
    currentSpeed_ = speed;
  }
  
  Serial.printf("[StepperMotor] Motor state - Running: %d, Current: %.2f, Target: %.2f\n",
                isRunning_, currentSpeed_, targetSpeed_);
  
  return true;
}

void StepperMotorDriver::softStop() {
  Serial.println("[StepperMotor] Soft stop");
  motor_.softStop();
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
}

void StepperMotorDriver::hardStop() {
  Serial.println("[StepperMotor] HARD STOP (Emergency)");
  motor_.hardStop();
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
}

bool StepperMotorDriver::isBusy() {
  return motor_.isBusy();
}

float StepperMotorDriver::getCurrentSpeed() {
  unsigned long speedReg = motor_.GetParam(0x04);
  float actualSpeed = (float)speedReg / 67.106f;
  
  unsigned int status = motor_.getStatus();
  bool isForward = (status & 0x0010) != 0;
  
  if (actualSpeed < 1.0f) {
    return 0.0f;
  }
  
  return isForward ? actualSpeed : -actualSpeed;
}

bool StepperMotorDriver::isRunning() const {
  return isRunning_;
}

StepMode StepperMotorDriver::getStepMode() const {
  return currentStepMode_;
}

// ============================================================================
// プライベートメソッド
// ============================================================================

void StepperMotorDriver::optimizeStepMode(float speed) {
  float absSpeed = abs(speed);
  StepMode newMode = currentStepMode_;
  
  if (absSpeed < SPEED_THRESHOLD_LOW) {
    newMode = STEP_128;
  } else if (absSpeed < SPEED_THRESHOLD_HIGH) {
    newMode = STEP_32;
  } else {
    newMode = STEP_8;
  }
  
  if (newMode != currentStepMode_) {
    Serial.printf("[StepperMotor] Step mode change: %d -> %d (speed: %.2f)\n", 
                  currentStepMode_, newMode, speed);
    changeStepMode(newMode);
  }
}

void StepperMotorDriver::changeStepMode(StepMode newMode) {
  Serial.println("[StepperMotor] Stopping for mode change...");
  motor_.softStop();
  
  while (motor_.isBusy()) {
    delay(10);
  }
  Serial.println("[StepperMotor] Motor stopped");
  
  Serial.printf("[StepperMotor] Setting micro steps to %d\n", newMode);
  motor_.setMicroSteps(newMode);
  Serial.println("[StepperMotor] Mode change completed");
  
  currentStepMode_ = newMode;
}
