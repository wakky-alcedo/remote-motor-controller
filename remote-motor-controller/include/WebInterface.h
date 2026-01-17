/**
 * WebInterface.h
 * Webサーバー・WebSocket管理クラス
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "SystemState.h"

// ============================================================================
// Webコマンド定義
// ============================================================================
enum WebCommand {
  CMD_NONE,
  CMD_SET_SPEED,
  CMD_STOP,
  CMD_EMERGENCY_STOP,
  CMD_RESET,
  CMD_SET_MODE
};

// ============================================================================
// Webコマンド構造体
// ============================================================================
struct WebCommandData {
  WebCommand command;
  float value;
  String stringValue;
  
  WebCommandData() : command(CMD_NONE), value(0.0f), stringValue("") {}
};

// ============================================================================
// WebInterface クラス
// ============================================================================
class WebInterface {
public:
  /**
   * コンストラクタ
   * @param port サーバーポート（デフォルト: 80）
   */
  WebInterface(uint16_t port = 80);
  
  /**
   * デストラクタ
   */
  ~WebInterface();
  
  /**
   * Webサーバーを初期化
   * @return 成功時true
   */
  bool init();
  
  /**
   * システム状態をWebSocketでブロードキャスト
   * @param state システム状態
   */
  void broadcastState(const SystemState& state);
  
  /**
   * WebSocketクライアントをクリーンアップ
   */
  void cleanupClients();
  
  /**
   * 接続中のWebSocketクライアント数を取得
   * @return クライアント数
   */
  size_t getClientCount() const;
  
  /**
   * Webコマンドをチェック（ポーリング用）
   * @param commandData コマンドデータを格納する構造体（参照渡し）
   * @return コマンドがある場合true
   */
  bool hasCommand(WebCommandData& commandData);
  
  /**
   * WebSocketイベントハンドラを設定するための内部関数
   * このメソッドはinitから呼ばれる
   */
  void setupWebSocketHandler();
  
private:
  AsyncWebServer server_;     // Webサーバーインスタンス
  AsyncWebSocket ws_;         // WebSocketインスタンス
  uint16_t port_;             // サーバーポート
  
  WebCommandData pendingCommand_;  // 保留中のコマンド
  bool hasNewCommand_;             // 新しいコマンドがあるか
  
  /**
   * WebSocketイベントハンドラ
   */
  void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                        AwsEventType type, void *arg, uint8_t *data, size_t len);
  
  /**
   * WebSocketメッセージを処理
   * @param data 受信データ
   * @param len データ長
   */
  void handleWebSocketMessage(uint8_t *data, size_t len);
  
  /**
   * HTMLページを生成
   * @return HTML文字列
   */
  const char* getIndexHTML();
  
  /**
   * ステータスAPIのJSONレスポンスを生成
   * @param state システム状態
   * @return JSON文字列
   */
  String generateStatusJson(const SystemState& state);
};

#endif // WEB_INTERFACE_H
