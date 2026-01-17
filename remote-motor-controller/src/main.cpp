/**
 * EtherSpin-ESP: ステッパーモータWebコントローラ
 * 
 * PC上の計算モデル（Python/MATLAB）やWebブラウザから、
 * Wi-Fi経由でステッピングモータを直接ドライブするための高精度制御システム
 * 
 * @author SDDL Project
 * @date 2026/01/17 (Refactored)
 */

#include <Arduino.h>
#include "SystemController.h"

// ============================================================================
// グローバルインスタンス
// ============================================================================
SystemController systemController;

// ============================================================================
// Setup & Loop
// ============================================================================

void setup() {
  // シリアル初期化
  Serial.begin(115200);
  delay(1000);
  
  // システムコントローラ初期化
  if (!systemController.init()) {
    Serial.println("[Main] System initialization failed!");
    while (1) {
      delay(1000);
    }
  }
  
  Serial.println("[Main] System initialization complete");
}

void loop() {
  // システムコントローラのメインループ処理
  systemController.update();
}
