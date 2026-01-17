/**
 * SystemState.h
 * システム全体の状態管理
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>

// ============================================================================
// 制御モード定義
// ============================================================================
enum ControlMode {
  MODE_INTERNAL,  // 手動制御（Web UI経由）
  MODE_EXTERNAL   // PC連携（UDP経由）
};

// ============================================================================
// マイクロステップ定義
// ============================================================================
enum StepMode {
  STEP_128 = 128,  // 低速・静音
  STEP_32  = 32,   // 中速
  STEP_8   = 8,    // 高速・高トルク
  STEP_FULL = 1    // フルステップ
};

// ============================================================================
// システム状態構造体
// ============================================================================
struct SystemState {
  // 制御モード
  ControlMode mode;
  
  // モータ速度情報
  float targetSpeed;      // 目標速度 (step/s)
  float currentSpeed;     // 現在速度 (step/s)
  StepMode stepMode;      // 現在のマイクロステップ設定
  
  // モータ状態フラグ
  bool motorRunning;      // モータ動作中フラグ
  bool emergencyStop;     // 緊急停止フラグ
  
  // タイムスタンプ
  unsigned long lastUdpTime;  // 最後のUDP受信時刻
  
  // コンストラクタ（初期化）
  SystemState() :
    mode(MODE_INTERNAL),
    targetSpeed(0.0f),
    currentSpeed(0.0f),
    stepMode(STEP_128),
    motorRunning(false),
    emergencyStop(false),
    lastUdpTime(0) {}
  
  // 状態リセット
  void reset() {
    mode = MODE_INTERNAL;
    targetSpeed = 0.0f;
    currentSpeed = 0.0f;
    stepMode = STEP_128;
    motorRunning = false;
    emergencyStop = false;
    lastUdpTime = 0;
  }
  
  // 緊急停止状態をクリア
  void clearEmergencyStop() {
    emergencyStop = false;
  }
  
  // モード文字列取得
  const char* getModeString() const {
    return (mode == MODE_INTERNAL) ? "INTERNAL" : "EXTERNAL";
  }
};

#endif // SYSTEM_STATE_H
