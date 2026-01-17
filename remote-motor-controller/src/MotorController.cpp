/**
 * MotorController.cpp
 * L6470ステッピングモータドライバ制御クラス実装
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#include "MotorController.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

MotorController::MotorController() 
  : motor_(PIN_CS),
    currentSpeed_(0.0f),
    targetSpeed_(0.0f),
    currentStepMode_(STEP_128),
    isRunning_(false) {
}

MotorController::~MotorController() {
  // モータを停止
  if (isRunning_) {
    softStop();
  }
}

// ============================================================================
// 公開メソッド
// ============================================================================

bool MotorController::init() {
  Serial.println("[MotorController] Initializing L6470...");
  Serial.printf("[MotorController] Pin Config - CS:%d SCK:%d MOSI:%d MISO:%d RST:%d BUSY:%d\n",
                PIN_CS, PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_RESET, PIN_BUSY);
  
  // ピン設定（L6470ライブラリのソフトウェアSPI使用）
  Serial.println("[MotorController] Setting pins...");
  motor_.set_pins(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_RESET, PIN_BUSY);
  Serial.println("[MotorController] Pins set successfully");
  
  // 初期化
  Serial.println("[MotorController] Calling motor.init()...");
  motor_.init();
  Serial.println("[MotorController] motor.init() completed");
  delay(100);
  
  // L6470パラメータ設定（仕様書準拠）
  Serial.println("[MotorController] Configuring parameters...");
  motor_.setMicroSteps(128);  // 初期は128分割
  Serial.println("[MotorController]   - MicroSteps: 128");
  
  motor_.setAcc(100);         // 加速度
  Serial.println("[MotorController]   - Acceleration: 100");
  
  motor_.setMaxSpeed(800);    // 最大速度
  Serial.println("[MotorController]   - MaxSpeed: 800");
  
  motor_.setMinSpeed(1);      // 最小速度
  Serial.println("[MotorController]   - MinSpeed: 1");
  
  motor_.setThresholdSpeed(1000);  // 閾値速度
  Serial.println("[MotorController]   - ThresholdSpeed: 1000");
  
  motor_.setOverCurrent(OCD_THRESHOLD);  // 過電流検出
  Serial.printf("[MotorController]   - OverCurrent: %d mA\n", OCD_THRESHOLD);
  
  motor_.setStallCurrent(STALL_CURRENT); // ストール電流
  Serial.printf("[MotorController]   - StallCurrent: %d mA\n", STALL_CURRENT);
  
  // KVAL設定（モータ駆動電圧）
  Serial.println("[MotorController]   - Setting KVAL parameters...");
  motor_.SetParam(0x09, KVAL_PARAM);  // KVAL_HOLD
  motor_.SetParam(0x0A, KVAL_PARAM);  // KVAL_RUN
  motor_.SetParam(0x0B, KVAL_PARAM);  // KVAL_ACC
  motor_.SetParam(0x0C, KVAL_PARAM);  // KVAL_DEC
  Serial.printf("[MotorController]   - KVAL: 0x%02X\n", KVAL_PARAM);
  
  Serial.println("[MotorController] L6470 initialized successfully");
  
  currentStepMode_ = STEP_128;
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
  
  return true;
}

bool MotorController::setSpeed(float speed, StepMode& currentStepMode) {
  Serial.printf("[MotorController] setSpeed called: %.2f step/s\n", speed);
  
  targetSpeed_ = speed;
  
  // 動的マイクロステップ最適化
  optimizeStepMode(speed);
  
  // 外部に現在のステップモードを反映
  currentStepMode = currentStepMode_;
  
  // 速度設定
  if (abs(speed) < 1.0f) {
    // 停止
    Serial.println("[MotorController] Stopping motor (speed < 1.0)");
    motor_.softStop();
    isRunning_ = false;
    currentSpeed_ = 0.0f;
  } else {
    // L6470ライブラリのrun関数: run(dir, speed)
    // dir: dSPIN_FWD(1)=正転, dSPIN_REV(0)=逆転
    // speed: float値をstep/sで指定（ライブラリ内部でSpdCalc()により変換される）
    float absSpeed = abs(speed);
    
    if (speed > 0) {
      Serial.printf("[MotorController] Running forward at %.2f step/s\n", absSpeed);
      motor_.run(1, absSpeed);  // dSPIN_FWD = 1
    } else {
      Serial.printf("[MotorController] Running reverse at %.2f step/s\n", absSpeed);
      motor_.run(0, absSpeed);  // dSPIN_REV = 0
    }
    
    isRunning_ = true;
    currentSpeed_ = speed;
  }
  
  Serial.printf("[MotorController] Motor state - Running: %d, Current: %.2f, Target: %.2f\n",
                isRunning_, currentSpeed_, targetSpeed_);
  
  return true;
}

void MotorController::softStop() {
  Serial.println("[MotorController] Soft stop");
  motor_.softStop();
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
}

void MotorController::hardStop() {
  Serial.println("[MotorController] HARD STOP (Emergency)");
  motor_.hardStop();
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
}

bool MotorController::isBusy() {
  return motor_.isBusy();
}

float MotorController::getCurrentSpeed() const {
  return currentSpeed_;
}

bool MotorController::isRunning() const {
  return isRunning_;
}

StepMode MotorController::getStepMode() const {
  return currentStepMode_;
}

// ============================================================================
// プライベートメソッド
// ============================================================================

void MotorController::optimizeStepMode(float speed) {
  float absSpeed = abs(speed);
  StepMode newMode = currentStepMode_;
  
  // 速度域に応じて最適なマイクロステップを決定
  if (absSpeed < SPEED_THRESHOLD_LOW) {
    newMode = STEP_128;  // 低速・静音
  } else if (absSpeed < SPEED_THRESHOLD_HIGH) {
    newMode = STEP_32;   // 中速
  } else {
    newMode = STEP_8;    // 高速・高トルク
  }
  
  // モード変更が必要な場合
  if (newMode != currentStepMode_) {
    Serial.printf("[MotorController] Step mode change: %d -> %d (speed: %.2f)\n", 
                  currentStepMode_, newMode, speed);
    changeStepMode(newMode);
  }
}

void MotorController::changeStepMode(StepMode newMode) {
  // モータを一時停止
  Serial.println("[MotorController] Stopping for mode change...");
  motor_.softStop();
  
  // モータが停止するまで待機
  while (motor_.isBusy()) {
    delay(10);
  }
  Serial.println("[MotorController] Motor stopped");
  
  // ステップモード変更
  Serial.printf("[MotorController] Setting micro steps to %d\n", newMode);
  motor_.setMicroSteps(newMode);
  Serial.println("[MotorController] Mode change completed");
  
  currentStepMode_ = newMode;
}
