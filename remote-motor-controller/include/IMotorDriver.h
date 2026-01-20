/**
 * IMotorDriver.h
 * モータードライバー抽象インターフェース
 * 
 * DCモーターとステッピングモーター間の切り替えを容易にするための抽象化層
 * 
 * @author SDDL Project
 * @date 2026/01/18
 */

#ifndef I_MOTOR_DRIVER_H
#define I_MOTOR_DRIVER_H

#include <Arduino.h>
#include "SystemState.h"

/**
 * モータードライバー抽象インターフェース
 * 
 * このインターフェースを実装することで、異なるモータータイプを
 * 統一的に扱うことができます。
 */
class IMotorDriver {
public:
  virtual ~IMotorDriver() {}
  
  /**
   * モータードライバーの初期化
   * @return 成功時true
   */
  virtual bool init() = 0;
  
  /**
   * モータ速度を設定
   * @param speed 目標速度（単位はドライバー依存、負値で逆転）
   * @param currentStepMode ステップモード（ステッピングモーターのみ使用）
   * @return 成功時true
   */
  virtual bool setSpeed(float speed, StepMode& currentStepMode) = 0;
  
  /**
   * モータを停止（ソフトストップ）
   */
  virtual void softStop() = 0;
  
  /**
   * モータを緊急停止（ハードストップ）
   */
  virtual void hardStop() = 0;
  
  /**
   * モータがビジー状態かチェック
   * @return ビジー状態の場合true
   */
  virtual bool isBusy() = 0;
  
  /**
   * 現在の速度を取得
   * @return 現在速度
   */
  virtual float getCurrentSpeed() = 0;
  
  /**
   * モータの動作状態を取得
   * @return 動作中の場合true
   */
  virtual bool isRunning() const = 0;
  
  /**
   * ステップモードを取得（ステッピングモーターのみ有効）
   * @return 現在のステップモード
   */
  virtual StepMode getStepMode() const = 0;
  
  /**
   * 現在のPWMデューティ比を取得（DCモーターのみ有効）
   * @return デューティ比 (0-100%)
   */
  virtual float getCurrentDuty() const = 0;
};

#endif // I_MOTOR_DRIVER_H
