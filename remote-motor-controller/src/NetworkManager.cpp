/**
 * NetworkManager.cpp
 * WiFi接続とネットワーク管理クラス実装
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#include "NetworkManager.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

NetworkManager::NetworkManager() 
  : ssid_(""),
    password_(""),
    mdnsHostname_(""),
    isAPMode_(false) {
}

NetworkManager::~NetworkManager() {
  // WiFi切断
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
}

// ============================================================================
// 公開メソッド
// ============================================================================

bool NetworkManager::connectWiFi(const char* ssid, const char* password, int maxRetries) {
  Serial.println("[NetworkManager] Connecting to WiFi...");
  Serial.printf("[NetworkManager] SSID: %s\n", ssid);
  
  ssid_ = String(ssid);
  password_ = String(password);
  isAPMode_ = false;
  
  // WiFiモードをSTAに設定
  Serial.println("[NetworkManager] Setting WiFi mode to STA...");
  WiFi.mode(WIFI_STA);
  
  // 接続開始
  Serial.println("[NetworkManager] Starting connection...");
  WiFi.begin(ssid, password);
  
  // 接続を試行
  if (tryConnect(maxRetries)) {
    printNetworkInfo();
    return true;
  }
  
  return false;
}

bool NetworkManager::startAccessPoint(const char* ssid, const char* password,
                                      IPAddress ip, IPAddress gateway, IPAddress subnet) {
  Serial.println("\n[NetworkManager] Starting Access Point mode...");
  
  ssid_ = String(ssid);
  password_ = String(password);
  isAPMode_ = true;
  
  // APモードに設定
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ip, gateway, subnet);
  
  // AP起動
  if (WiFi.softAP(ssid, password)) {
    Serial.println("[NetworkManager] Access Point started successfully!");
    Serial.printf("[NetworkManager] AP SSID: %s\n", ssid);
    Serial.printf("[NetworkManager] AP Password: %s\n", password);
    Serial.printf("[NetworkManager] AP IP Address: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.println("[NetworkManager] Connect to this WiFi network to access the web interface");
    return true;
  } else {
    Serial.println("[NetworkManager] Failed to start Access Point!");
    return false;
  }
}

bool NetworkManager::startMDNS(const char* hostname) {
  Serial.println("[NetworkManager] Starting mDNS responder...");
  
  mdnsHostname_ = String(hostname);
  
  if (MDNS.begin(hostname)) {
    Serial.printf("[NetworkManager] mDNS responder started: http://%s.local\n", hostname);
    MDNS.addService("http", "tcp", 80);
    Serial.println("[NetworkManager] HTTP service registered");
    return true;
  } else {
    Serial.println("[NetworkManager] Failed to start mDNS responder");
    return false;
  }
}

bool NetworkManager::isConnected() const {
  return (WiFi.status() == WL_CONNECTED);
}

bool NetworkManager::isAPMode() const {
  return isAPMode_;
}

String NetworkManager::getIPAddress() const {
  if (isAPMode_) {
    return WiFi.softAPIP().toString();
  } else {
    return WiFi.localIP().toString();
  }
}

int NetworkManager::getRSSI() const {
  if (isAPMode_) {
    return 0;  // APモードではRSSI取得不可
  }
  return WiFi.RSSI();
}

void NetworkManager::printNetworkInfo() const {
  Serial.println("\n[NetworkManager] ===== Network Information =====");
  
  if (isAPMode_) {
    Serial.println("[NetworkManager] Mode: Access Point (AP)");
    Serial.printf("[NetworkManager] SSID: %s\n", ssid_.c_str());
    Serial.printf("[NetworkManager] IP Address: %s\n", WiFi.softAPIP().toString().c_str());
  } else {
    Serial.println("[NetworkManager] Mode: Station (STA)");
    Serial.printf("[NetworkManager] SSID: %s\n", ssid_.c_str());
    Serial.printf("[NetworkManager] IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[NetworkManager] Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("[NetworkManager] Subnet: %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("[NetworkManager] DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("[NetworkManager] Signal Strength (RSSI): %d dBm\n", WiFi.RSSI());
  }
  
  if (!mdnsHostname_.isEmpty()) {
    Serial.printf("[NetworkManager] mDNS: http://%s.local\n", mdnsHostname_.c_str());
  }
  
  Serial.println("[NetworkManager] ================================\n");
}

// ============================================================================
// プライベートメソッド
// ============================================================================

bool NetworkManager::tryConnect(int maxRetries) {
  int attempts = 0;
  
  while (WiFi.status() != WL_CONNECTED && attempts < maxRetries) {
    delay(500);
    Serial.print(".");
    attempts++;
    
    if (attempts % 10 == 0) {
      Serial.printf(" [%d/%d]\n", attempts, maxRetries);
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[NetworkManager] Connected!");
    return true;
  } else {
    Serial.println("\n[NetworkManager] Connection failed!");
    Serial.printf("[NetworkManager] Final status code: %d\n", WiFi.status());
    Serial.println("[NetworkManager] Possible reasons:");
    Serial.println("[NetworkManager]   - Wrong SSID or password");
    Serial.println("[NetworkManager]   - Router out of range");
    Serial.println("[NetworkManager]   - Router authentication issues");
    return false;
  }
}
