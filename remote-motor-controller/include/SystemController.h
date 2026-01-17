/**
 * SystemController.h
 * システム全体の統合制御クラス
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#ifndef SYSTEM_CONTROLLER_H
#define SYSTEM_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "SystemState.h"
#include "MotorController.h"
#include "NetworkManager.h"
#include "UDPController.h"
#include "WebInterface.h"

// ============================================================================
// WiFi設定（config.hから読み込み）
// ============================================================================
// WIFI_SSID, WIFI_PASSWORD, MDNS_HOSTNAME, UDP_PORT, WATCHDOG_TIMEOUT_MS
// はconfig.hで定義されています

// アクセスポイント設定
#define AP_SSID "EtherSpin-ESP"
#define AP_PASSWORD "12345678"
#define AP_IP_ADDR 192, 168, 4, 1

// 状態ブロードキャスト間隔
#define BROADCAST_INTERVAL_MS 500

// ============================================================================
// SystemController クラス
// ============================================================================
class SystemController {
public:
  /**
   * コンストラクタ
   */
  SystemController();
  
  /**
   * デストラクタ
   */
  ~SystemController();
  
  /**
   * システム全体を初期化
   * @return 成功時true
   */
  bool init();
  
  /**
   * メインループ処理
   * UDPパケット受信、Webコマンド処理、ウォッチドッグチェックを実行
   */
  void update();
  
  /**
   * システム状態を取得
   * @return システム状態への参照
   */
  SystemState& getState();
  
  /**
   * システム情報を出力
   */
  void printSystemInfo();
  
private:
  // コンポーネント
  MotorController motor_;
  NetworkManager network_;
  UDPController udp_;
  WebInterface web_;
  
  // システム状態
  SystemState state_;
  
  // タイムスタンプ
  unsigned long lastBroadcastTime_;
  unsigned long lastStatusLogTime_;
  
  /**
   * UDPコマンドを処理
   */
  void handleUDPCommand();
  
  /**
   * Webコマンドを処理
   */
  void handleWebCommand();
  
  /**
   * ウォッチドッグタイマーをチェック
   */
  void checkWatchdog();
  
  /**
   * システム状態をブロードキャスト
   */
  void broadcastState();
  
  /**
   * 定期的な状態ログ出力
   */
  void logStatus();
  
  /**
   * モータ速度を設定（内部モード用）
   * @param speed 目標速度
   */
  void setMotorSpeedInternal(float speed);
  
  /**
   * モータ速度を設定（外部モード用）
   * @param speed 目標速度
   */
  void setMotorSpeedExternal(float speed);
};

#endif // SYSTEM_CONTROLLER_H
