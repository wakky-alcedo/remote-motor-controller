/**
 * NetworkManager.h
 * WiFi接続とネットワーク管理クラス
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

// ============================================================================
// デフォルトネットワーク設定
// ============================================================================
#define DEFAULT_MDNS_HOSTNAME "motor"
#define DEFAULT_AP_SSID "EtherSpin-ESP"
#define DEFAULT_AP_PASSWORD " "  // 最低8文字必要

// ============================================================================
// NetworkManager クラス
// ============================================================================
class NetworkManager {
public:
  /**
   * コンストラクタ
   */
  NetworkManager();
  
  /**
   * デストラクタ
   */
  ~NetworkManager();
  
  /**
   * WiFi接続を開始
   * @param ssid WiFi SSID
   * @param password WiFiパスワード
   * @param maxRetries 最大再試行回数（デフォルト: 30）
   * @return 接続成功時true
   */
  bool connectWiFi(const char* ssid, const char* password, int maxRetries = 30);
  
  /**
   * アクセスポイントモードで起動
   * @param ssid AP SSID
   * @param password APパスワード
   * @param ip IPアドレス
   * @param gateway ゲートウェイ
   * @param subnet サブネットマスク
   * @return 起動成功時true
   */
  bool startAccessPoint(const char* ssid, const char* password,
                        IPAddress ip, IPAddress gateway, IPAddress subnet);
  
  /**
   * mDNSレスポンダを開始
   * @param hostname ホスト名（例: "motor" -> "motor.local"）
   * @return 起動成功時true
   */
  bool startMDNS(const char* hostname);
  
  /**
   * WiFi接続状態を確認
   * @return 接続中の場合true
   */
  bool isConnected() const;
  
  /**
   * アクセスポイントモードかチェック
   * @return APモードの場合true
   */
  bool isAPMode() const;
  
  /**
   * IPアドレスを取得
   * @return IPアドレス文字列
   */
  String getIPAddress() const;
  
  /**
   * 信号強度を取得（STAモードのみ）
   * @return RSSI値 (dBm)
   */
  int getRSSI() const;
  
  /**
   * ネットワーク情報をシリアル出力
   */
  void printNetworkInfo() const;
  
private:
  String ssid_;           // 接続先SSID
  String password_;       // 接続パスワード
  String mdnsHostname_;   // mDNSホスト名
  bool isAPMode_;         // APモードフラグ
  
  /**
   * WiFi接続をトライ
   * @param maxRetries 最大再試行回数
   * @return 接続成功時true
   */
  bool tryConnect(int maxRetries);
};

#endif // NETWORK_MANAGER_H
