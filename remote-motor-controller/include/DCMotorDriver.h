/**
 * DCMotorDriver.h
 * DCモータードライバ実装（PWM制御）
 * 
 * @author SDDL Project
 * @date 2026/01/18
 */

#ifndef DC_MOTOR_DRIVER_H
#define DC_MOTOR_DRIVER_H

#include <Arduino.h>
#include "IMotorDriver.h"
#include "config.h"

// ============================================================================
// ピン定義 (XIAO ESP32C3用)
// ============================================================================
#define DC_PIN_PWM_A      D8   // PWM出力A (モーター正転制御)
#define DC_PIN_PWM_B      D9   // PWM出力B (モーター逆転制御)
#define DC_PIN_ENABLE     D10  // イネーブル信号（オプション）
#define ENCODER_PIN       D4   // エンコーダ入力 (オプション)

// ============================================================================
// PWMパラメータ
// ============================================================================
#define DC_PWM_FREQ       20000  // PWM周波数 (Hz) - 可聴域外
#define DC_PWM_RESOLUTION 8      // PWM分解能 (bit) - 0〜255
#define DC_PWM_CHANNEL_A  0      // PWMチャンネルA
#define DC_PWM_CHANNEL_B  1      // PWMチャンネルB

/**
 * DCモータードライバー実装クラス
 * 
 * 標準的なHブリッジドライバ（L298N, TB6612等）に対応
 * PWMデューティ比で速度制御、2ピン方式で方向制御
 */
class DCMotorDriver : public IMotorDriver {
public:
  DCMotorDriver();
  virtual ~DCMotorDriver();
  
  // IMotorDriverインターフェース実装
  virtual bool init() override;
  virtual bool setSpeed(float speed, StepMode& currentStepMode) override;
  virtual void softStop() override;
  virtual void hardStop() override;
  virtual bool isBusy() override;
  virtual float getCurrentSpeed() override;
  virtual bool isRunning() const override;
  virtual StepMode getStepMode() const override;
  
  /**
   * PID制御更新（定期的に呼び出す必要あり）
   */
  void updatePID();
  
  /**
   * 現在のRPMを取得
   * @return RPM値
   */
  float getCurrentRPM() const;
  
  /**
   * 現在のPWMデューティ比を取得
   * @return デューティ比 (0-100%)
   */
  float getCurrentDuty() const;
  
  /**
   * エンコーダー割り込みハンドラ（静的メソッド）
   */
  static void IRAM_ATTR encoderISR();
  
  /**
   * タイマー割り込みハンドラ（静的メソッド）
   * 1kHz（1ms）周期でRPM計算とPID制御を実行
   */
  static void IRAM_ATTR timerISR();

private:
  float currentSpeed_;         // 現在速度 (-100.0 〜 +100.0 %)
  float targetSpeed_;          // 目標速度
  float targetRPM_;            // 目標RPM
  bool isRunning_;             // モータ動作中フラグ
  volatile float currentDuty_; // 現在のPWMデューティ比 (0-100%)
  
  // ============================================================================
  // M/T法エンコーダー計測
  // ============================================================================
  // M法変数（パルスカウント）
  static volatile unsigned long encoderCount_;  // 累積エンコーダーパルスカウント
  volatile unsigned long lastEncoderCount_;     // 前回計測時のカウント
  
  // T法変数（周期計測）
  static volatile unsigned long lastEdgeTime_;  // 最新エッジの時刻 (micros)
  static volatile unsigned long edgeInterval_;  // 最新のエッジ間隔 (μs)
  static volatile unsigned long firstEdgeTime_; // 計測開始時の最初のエッジ時刻
  static volatile unsigned long edgeCountInPeriod_; // 計測期間内のエッジ数
  
  // RPM計算関連
  volatile unsigned long controlCycleCount_;    // 制御周期カウンタ（1ms毎にインクリメント）
  volatile unsigned long rpmCalcCycleCount_;    // RPM計算用サイクルカウンタ
  volatile float currentRPM_;                   // 現在のRPM（M/T法計算後）
  
  // タイマー関連
  static hw_timer_t* timer_;                    // ハードウェアタイマー
  
  // PID制御関連（速度型PID）
  float pidLastError_;         // PID前回誤差 e(k-1)
  float pidLastError2_;        // PID前々回誤差 e(k-2)（微分計算用）
  float pidLastOutput_;        // 前回のPID出力値
  unsigned long lastPIDTime_;  // 前回のPID更新時刻
  bool pidEnabled_;            // PID制御有効フラグ
  
  /**
   * PWMデューティ比を設定
   * @param speed 速度 (-100.0 〜 +100.0 %)
   */
  void setPWM(float speed);
  
  /**
   * 速度をPWMデューティ比に変換
   * @param speed 速度（%）
   * @return PWMデューティ比 (0-255)
   */
  uint8_t speedToDuty(float speed);
  
  /**
   * RPMを計算
   */
  void calculateRPM();
  
  /**
   * PID制御でPWM出力を計算
   * @return PWM出力値（%）
   */
  float calculatePID();
  
  // 静的インスタンスポインタ（割り込みハンドラ用）
  static DCMotorDriver* instance_;
};

#endif // DC_MOTOR_DRIVER_H
