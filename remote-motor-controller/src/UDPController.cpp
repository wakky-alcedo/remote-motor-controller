/**
 * UDPController.cpp
 * UDP通信とウォッチドッグ管理クラス実装
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#include "UDPController.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

UDPController::UDPController(uint32_t watchdogTimeout)
  : port_(0),
    watchdogTimeout_(watchdogTimeout),
    lastPacketTime_(0) {
}

UDPController::~UDPController() {
  udp_.stop();
}

// ============================================================================
// 公開メソッド
// ============================================================================

bool UDPController::begin(uint16_t port) {
  Serial.println("[UDPController] Starting UDP server...");
  
  port_ = port;
  
  if (udp_.begin(port)) {
    Serial.printf("[UDPController] UDP server started on port %d\n", port);
    lastPacketTime_ = millis();
    return true;
  } else {
    Serial.println("[UDPController] Failed to start UDP server");
    return false;
  }
}

bool UDPController::receivePacket(float& speed) {
  int packetSize = udp_.parsePacket();
  
  if (packetSize > 0) {
    char incomingPacket[256];
    int len = udp_.read(incomingPacket, sizeof(incomingPacket) - 1);
    
    if (len > 0) {
      incomingPacket[len] = 0;  // Null終端
      
      // JSONパケット解析
      if (parseJsonPacket(incomingPacket, speed)) {
        // ウォッチドッグ更新
        updateWatchdog();
        
        Serial.printf("[UDPController] Received speed command: %.2f step/s\n", speed);
        return true;
      } else {
        Serial.println("[UDPController] Failed to parse JSON packet");
        return false;
      }
    }
  }
  
  return false;
}

void UDPController::updateWatchdog() {
  lastPacketTime_ = millis();
}

bool UDPController::isWatchdogTimeout() const {
  unsigned long elapsed = millis() - lastPacketTime_;
  return (elapsed > watchdogTimeout_);
}

unsigned long UDPController::getLastPacketTime() const {
  return lastPacketTime_;
}

void UDPController::setWatchdogTimeout(uint32_t timeout) {
  watchdogTimeout_ = timeout;
  Serial.printf("[UDPController] Watchdog timeout set to %d ms\n", timeout);
}

uint32_t UDPController::getWatchdogTimeout() const {
  return watchdogTimeout_;
}

uint16_t UDPController::getPort() const {
  return port_;
}

// ============================================================================
// プライベートメソッド
// ============================================================================

bool UDPController::parseJsonPacket(const char* json, float& speed) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);
  
  if (error) {
    Serial.printf("[UDPController] JSON parse error: %s\n", error.c_str());
    return false;
  }
  
  // 速度指令を取得
  if (doc.containsKey("v")) {
    speed = doc["v"];
    return true;
  } else {
    Serial.println("[UDPController] JSON packet does not contain 'v' field");
    return false;
  }
}
