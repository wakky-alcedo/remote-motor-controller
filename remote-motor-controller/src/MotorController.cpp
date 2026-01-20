/**
 * MotorController.cpp
 * モーター制御統合クラス実装（ドライバー抽象化層）
 * 
 * @author SDDL Project
 * @date 2026/01/18
 */

#include "MotorController.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

MotorController::MotorController() : driver_(nullptr) {
  // config.hの設定に基づいて適切なドライバーを生成
#ifdef MOTOR_TYPE_DC
  Serial.println("[MotorController] Creating DC Motor Driver");
  driver_ = new DCMotorDriver();
#else
  Serial.println("[MotorController] Creating Stepper Motor Driver");
  driver_ = new StepperMotorDriver();
#endif
}

MotorController::~MotorController() {
  if (driver_ != nullptr) {
    delete driver_;
    driver_ = nullptr;
  }
}

// ============================================================================
// 公開メソッド（ドライバーへ委譲）
// ============================================================================

bool MotorController::init() {
  if (driver_ == nullptr) {
    Serial.println("[MotorController] ERROR: Driver is null");
    return false;
  }
  return driver_->init();
}

bool MotorController::setSpeed(float speed, StepMode& currentStepMode) {
  if (driver_ == nullptr) {
    Serial.println("[MotorController] ERROR: Driver is null");
    return false;
  }
  return driver_->setSpeed(speed, currentStepMode);
}

void MotorController::softStop() {
  if (driver_ != nullptr) {
    driver_->softStop();
  }
}

void MotorController::hardStop() {
  if (driver_ != nullptr) {
    driver_->hardStop();
  }
}

bool MotorController::isBusy() {
  if (driver_ == nullptr) {
    return false;
  }
  return driver_->isBusy();
}

float MotorController::getCurrentSpeed() {
  if (driver_ == nullptr) {
    return 0.0f;
  }
  return driver_->getCurrentSpeed();
}

bool MotorController::isRunning() const {
  if (driver_ == nullptr) {
    return false;
  }
  return driver_->isRunning();
}

StepMode MotorController::getStepMode() const {
  if (driver_ == nullptr) {
     return STEP_FULL;
  }
  return driver_->getStepMode();
}

float MotorController::getCurrentDuty() const {
  if (driver_ == nullptr) {
    return 0.0f;
  }
  return driver_->getCurrentDuty();
}

void MotorController::updatePID() {
  // DCモーターのPID制御更新（タイマー割り込みで自動実行されるため、ここでは何もしない）
  // この関数は後方互換性のために残されています
}
