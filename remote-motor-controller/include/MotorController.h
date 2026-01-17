/**
 * MotorController.h
 * L6470ステッピングモータドライバ制御クラス
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <L6470.h>
#include "SystemState.h"

// ============================================================================
// ピン定義 (XIAO ESP32C3用)
// ============================================================================
#define PIN_SPI_SCK   8   // SCK
#define PIN_SPI_MISO  9   // MISO
#define PIN_SPI_MOSI  10  // MOSI
#define PIN_CS        7   // Chip Select
#define PIN_BUSY      6   // BUSY Signal
#define PIN_RESET     5   // RESET Signal

// ============================================================================
// L6470パラメータ設定
// ============================================================================
#define KVAL_PARAM 0x29    // KVAL_HOLD/RUN/ACC/DEC
#define OCD_THRESHOLD 6000 // 過電流検出 (mA)
#define STALL_CURRENT 3000 // ストール電流 (mA)

// 速度閾値（step/s）
#define SPEED_THRESHOLD_LOW  500    // 128→32分割への遷移点
#define SPEED_THRESHOLD_HIGH 2000   // 32→8分割への遷移点

// ============================================================================
// MotorController クラス
// ============================================================================
class MotorController {
public:
  /**
   * コンストラクタ
   */
  MotorController();
  
  /**
   * デストラクタ
   */
  ~MotorController();
  
  /**
   * モータドライバの初期化
   * @return 成功時true
   */
  bool init();
  
  /**
   * モータ速度を設定
   * @param speed 目標速度 (step/s、負値で逆転)
   * @param currentStepMode 現在のステップモード（参照渡しで更新）
   * @return 成功時true
   */
  bool setSpeed(float speed, StepMode& currentStepMode);
  
  /**
   * モータを停止（ソフトストップ）
   */
  void softStop();
  
  /**
   * モータを緊急停止（ハードストップ）
   */
  void hardStop();
  
  /**
   * モータがビジー状態かチェック
   * @return ビジー状態の場合true
   */
  bool isBusy();
  
  /**
   * 現在の速度を取得
   * @return 現在速度 (step/s)
   */
  float getCurrentSpeed() const;
  
  /**
   * モータの動作状態を取得
   * @return 動作中の場合true
   */
  bool isRunning() const;
  
  /**
   * ステップモードを取得
   * @return 現在のステップモード
   */
  StepMode getStepMode() const;
  
private:
  L6470 motor_;                // L6470ドライバインスタンス
  float currentSpeed_;         // 現在速度 (step/s)
  float targetSpeed_;          // 目標速度 (step/s)
  StepMode currentStepMode_;   // 現在のマイクロステップ設定
  bool isRunning_;             // モータ動作中フラグ
  
  /**
   * 動的マイクロステップ最適化
   * 速度域に応じて最適なマイクロステップ設定に自動遷移
   * @param speed 目標速度
   */
  void optimizeStepMode(float speed);
  
  /**
   * マイクロステップモードを変更
   * @param newMode 新しいステップモード
   */
  void changeStepMode(StepMode newMode);
};

#endif // MOTOR_CONTROLLER_H
