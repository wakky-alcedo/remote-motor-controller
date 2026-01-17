/**
 * StepperMotorDriver.h
 * L6470ステッピングモータドライバ実装
 * 
 * @author SDDL Project
 * @date 2026/01/18
 */

#ifndef STEPPER_MOTOR_DRIVER_H
#define STEPPER_MOTOR_DRIVER_H

#include <Arduino.h>
#include <L6470.h>
#include "IMotorDriver.h"
#include "config.h"

// ============================================================================
// ピン定義 (XIAO ESP32C3用)
// ============================================================================
#define STEPPER_PIN_SPI_SCK   D8   // SCK
#define STEPPER_PIN_SPI_MISO  D9   // MISO
#define STEPPER_PIN_SPI_MOSI  D10  // MOSI
#define STEPPER_PIN_CS        D7   // Chip Select
#define STEPPER_PIN_BUSY      D6   // BUSY Signal
#ifndef STEPPER_PIN_RESET
#define STEPPER_PIN_RESET     D5   // RESET Signal
#endif

// ============================================================================
// L6470パラメータ設定
// ============================================================================
#define STALL_CURRENT 3000 // ストール電流 (mA)

/**
 * L6470ステッピングモータードライバー実装クラス
 */
class StepperMotorDriver : public IMotorDriver {
public:
  StepperMotorDriver();
  virtual ~StepperMotorDriver();
  
  // IMotorDriverインターフェース実装
  virtual bool init() override;
  virtual bool setSpeed(float speed, StepMode& currentStepMode) override;
  virtual void softStop() override;
  virtual void hardStop() override;
  virtual bool isBusy() override;
  virtual float getCurrentSpeed() override;
  virtual bool isRunning() const override;
  virtual StepMode getStepMode() const override;

private:
  L6470 motor_;                // L6470ドライバインスタンス
  float currentSpeed_;         // 現在速度 (step/s)
  float targetSpeed_;          // 目標速度 (step/s)
  StepMode currentStepMode_;   // 現在のマイクロステップ設定
  bool isRunning_;             // モータ動作中フラグ
  
  void optimizeStepMode(float speed);
  void changeStepMode(StepMode newMode);
};

#endif // STEPPER_MOTOR_DRIVER_H
