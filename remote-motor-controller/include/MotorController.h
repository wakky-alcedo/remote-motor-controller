/**
 * MotorController.h
 * モーター制御統合クラス（ドライバー抽象化層）
 * 
 * DCモーターとステッピングモーター間の切り替えをconfig.hで実現
 * 
 * @author SDDL Project
 * @date 2026/01/18
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "SystemState.h"
#include "IMotorDriver.h"

// config.hの設定に基づいて適切なドライバーをインクルード
#ifdef MOTOR_TYPE_DC
  #include "DCMotorDriver.h"
#else
  #include "StepperMotorDriver.h"
#endif

// ============================================================================
// MotorController クラス（ファサードパターン）
// ============================================================================
/**
 * モーター制御統合クラス
 * 
 * IMotorDriverインターフェースを使用し、config.hの設定に基づいて
 * DCモーターまたはステッピングモーターを自動的に選択します。
 * 
 * 使用例:
 *   MotorController motor;
 *   motor.init();
 *   motor.setSpeed(50.0, stepMode);  // DCモード: 50%、ステッパー: 50 step/s
 */
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
   * @param speed 目標速度（DCモード: %, ステッパーモード: step/s、負値で逆転）
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
   * @return 現在速度
   */
  float getCurrentSpeed();
  
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
  IMotorDriver* driver_;  // モータードライバーインターフェース（ポリモーフィズム）
};

#endif // MOTOR_CONTROLLER_H
