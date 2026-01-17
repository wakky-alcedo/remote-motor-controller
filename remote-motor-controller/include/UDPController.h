/**
 * UDPController.h
 * UDP通信とウォッチドッグ管理クラス
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#ifndef UDP_CONTROLLER_H
#define UDP_CONTROLLER_H

#include <Arduino.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// ============================================================================
// デフォルト設定
// ============================================================================
#define DEFAULT_UDP_PORT 8888
#define DEFAULT_WATCHDOG_TIMEOUT_MS 1000  // 1秒

// ============================================================================
// UDPController クラス
// ============================================================================
class UDPController {
public:
  /**
   * コンストラクタ
   * @param watchdogTimeout ウォッチドッグタイムアウト時間 (ms)
   */
  UDPController(uint32_t watchdogTimeout = DEFAULT_WATCHDOG_TIMEOUT_MS);
  
  /**
   * デストラクタ
   */
  ~UDPController();
  
  /**
   * UDPサーバーを開始
   * @param port ポート番号
   * @return 成功時true
   */
  bool begin(uint16_t port = DEFAULT_UDP_PORT);
  
  /**
   * UDPパケットを受信して解析
   * @param speed 受信した速度指令を格納する変数（参照渡し）
   * @return パケット受信成功時true
   */
  bool receivePacket(float& speed);
  
  /**
   * ウォッチドッグタイマーを更新
   */
  void updateWatchdog();
  
  /**
   * ウォッチドッグタイムアウトをチェック
   * @return タイムアウト時true
   */
  bool isWatchdogTimeout() const;
  
  /**
   * 最後のパケット受信時刻を取得
   * @return 受信時刻 (millis)
   */
  unsigned long getLastPacketTime() const;
  
  /**
   * ウォッチドッグタイムアウト時間を設定
   * @param timeout タイムアウト時間 (ms)
   */
  void setWatchdogTimeout(uint32_t timeout);
  
  /**
   * ウォッチドッグタイムアウト時間を取得
   * @return タイムアウト時間 (ms)
   */
  uint32_t getWatchdogTimeout() const;
  
  /**
   * リスニングポートを取得
   * @return ポート番号
   */
  uint16_t getPort() const;
  
private:
  WiFiUDP udp_;                      // UDPインスタンス
  uint16_t port_;                    // リスニングポート
  uint32_t watchdogTimeout_;         // ウォッチドッグタイムアウト (ms)
  unsigned long lastPacketTime_;     // 最後のパケット受信時刻
  
  /**
   * JSONパケットを解析
   * @param json JSON文字列
   * @param speed 解析した速度値を格納
   * @return 解析成功時true
   */
  bool parseJsonPacket(const char* json, float& speed);
};

#endif // UDP_CONTROLLER_H
