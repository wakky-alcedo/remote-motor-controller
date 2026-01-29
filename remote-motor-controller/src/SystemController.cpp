/**
 * SystemController.cpp
 * システム全体の統合制御クラス実装
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#include "SystemController.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

SystemController::SystemController()
  : motor_(),
    network_(),
    udp_(WATCHDOG_TIMEOUT_MS),
    web_(80),
    dataLogger_(),
    state_(),
    lastBroadcastTime_(0),
    lastStatusLogTime_(0),
    lastRecordingBroadcastTime_(0) {
}

SystemController::~SystemController() {
  // 各コンポーネントのデストラクタが自動的に呼ばれる
}

// ============================================================================
// 公開メソッド
// ============================================================================

bool SystemController::init() {
  Serial.println("\n\n===========================================");
  Serial.println("EtherSpin-ESP: Stepper Motor Web Controller");
  Serial.println("===========================================\n");
  Serial.println("[SystemController] Starting initialization sequence...");
  
  // システム情報出力
  printSystemInfo();
  
  // システム状態初期化
  Serial.println("[SystemController] Initializing system state...");
  state_.reset();
  Serial.println("[SystemController] System state initialized");
  
  // ステップ1: モータドライバ初期化
  Serial.println("\n[SystemController] Step 1/4: Initializing motor driver...");
  if (!motor_.init()) {
    Serial.println("[SystemController] ERROR: Motor initialization failed!");
    return false;
  }
  Serial.println("[SystemController] Motor driver initialization complete\n");
  
  // ステップ2: WiFi接続
  Serial.println("[SystemController] Step 2/4: Connecting to WiFi...");
  if (!network_.connectWiFi(WIFI_SSID, WIFI_PASSWORD)) {
    // WiFi接続失敗 -> APモードで起動
    Serial.println("[SystemController] WiFi connection failed, starting AP mode...");
    IPAddress apIP(AP_IP_ADDR);
    IPAddress apGateway(AP_IP_ADDR);
    IPAddress apSubnet(255, 255, 255, 0);
    
    if (!network_.startAccessPoint(AP_SSID, AP_PASSWORD, apIP, apGateway, apSubnet)) {
      Serial.println("[SystemController] ERROR: Failed to start AP mode!");
      return false;
    }
  }
  
  // mDNS開始
  network_.startMDNS(MDNS_HOSTNAME);
  Serial.println("[SystemController] WiFi initialization complete\n");
  
  // ステップ3: UDPサーバー開始
  Serial.println("[SystemController] Step 3/4: Starting UDP server...");
  if (!udp_.begin(UDP_PORT)) {
    Serial.println("[SystemController] ERROR: UDP initialization failed!");
    return false;
  }
  Serial.println("[SystemController] UDP server started\n");
  
  // ステップ4: Webサーバー起動
  Serial.println("[SystemController] Step 4/4: Starting web server...");
  if (!web_.init()) {
    Serial.println("[SystemController] ERROR: Web server initialization failed!");
    return false;
  }
  
  // DataLoggerをWebInterfaceに設定
  web_.setDataLogger(&dataLogger_);
  Serial.println("[SystemController] Web server initialization complete\n");
  
  // 初期化完了メッセージ
  Serial.println("\n=== System Ready ===");
  Serial.println("Access via:");
  if (network_.isConnected()) {
    Serial.printf("  - http://%s\n", network_.getIPAddress().c_str());
    Serial.printf("  - http://%s.local\n", MDNS_HOSTNAME);
    Serial.printf("  - UDP: Port %d\n", UDP_PORT);
  } else if (network_.isAPMode()) {
    Serial.printf("  - Connect to WiFi: %s\n", AP_SSID);
    Serial.printf("  - Password: %s\n", AP_PASSWORD);
    Serial.printf("  - Then open: http://%s\n", network_.getIPAddress().c_str());
    Serial.printf("  - Or: http://%s.local\n", MDNS_HOSTNAME);
  }
  Serial.println("======================\n");
  
  // タイムスタンプ初期化
  lastBroadcastTime_ = millis();
  lastStatusLogTime_ = millis();
  
  return true;
}

void SystemController::update() {
  // UDPコマンド処理
  handleUDPCommand();
  
  // Webコマンド処理
  handleWebCommand();
  
  // ウォッチドッグチェック
  checkWatchdog();
  
  // PID制御更新（DCモーターの場合のみ）
  motor_.updatePID();
  
  // データロガー記録（収録中の場合）
  if (dataLogger_.isRecording()) {
    state_.currentSpeed = motor_.getCurrentSpeed();
    dataLogger_.record(state_.currentSpeed, state_.targetSpeed);
  }
  
  // 状態ブロードキャスト（500msごと、クライアント接続時のみ）
  unsigned long now = millis();
  if (now - lastBroadcastTime_ > BROADCAST_INTERVAL_MS) {
    broadcastState();
    lastBroadcastTime_ = now;
  }
  
  // 収録状態ブロードキャスト（500msごと）
  if (now - lastRecordingBroadcastTime_ > 500) {
    if (web_.getClientCount() > 0) {
      web_.broadcastRecordingState(
        dataLogger_.isRecording(),
        dataLogger_.getRecordCount(),
        dataLogger_.getRecordDuration()
      );
    }
    lastRecordingBroadcastTime_ = now;
  }
  
  // 定期的な状態ログ出力（10秒ごと）
  if (now - lastStatusLogTime_ > 10000) {
    logStatus();
    lastStatusLogTime_ = now;
  }
  
  // 少し待機
  delay(10);
}

SystemState& SystemController::getState() {
  return state_;
}

void SystemController::printSystemInfo() {
  Serial.printf("[SystemController] ESP32 Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("[SystemController] Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("[SystemController] CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("[SystemController] Free Heap: %d bytes\n", ESP.getFreeHeap());
}

// ============================================================================
// プライベートメソッド
// ============================================================================

void SystemController::handleUDPCommand() {
  float speed;
  if (udp_.receivePacket(speed)) {
    // External Modeに自動遷移
    if (state_.mode != MODE_EXTERNAL) {
      Serial.println("[SystemController] Switching to EXTERNAL mode");
      state_.mode = MODE_EXTERNAL;
    }
    
    // 速度設定
    setMotorSpeedExternal(speed);
  }
}

void SystemController::handleWebCommand() {
  WebCommandData cmd;
  if (web_.hasCommand(cmd)) {
    switch (cmd.command) {
      case CMD_SET_SPEED:
        state_.mode = MODE_INTERNAL;
        setMotorSpeedInternal(cmd.value);
        break;
        
      case CMD_STOP:
        setMotorSpeedInternal(0);
        break;
        
      case CMD_EMERGENCY_STOP:
        Serial.println("[SystemController] EMERGENCY STOP!");
        motor_.hardStop();
        state_.emergencyStop = true;
        state_.motorRunning = false;
        state_.currentSpeed = 0;
        state_.targetSpeed = 0;
        break;
        
      case CMD_RESET:
        Serial.println("[SystemController] Reset emergency stop");
        state_.clearEmergencyStop();
        break;
        
      case CMD_SET_MODE:
        if (cmd.stringValue == "external") {
          state_.mode = MODE_EXTERNAL;
          Serial.println("[SystemController] Mode set to EXTERNAL");
        } else {
          state_.mode = MODE_INTERNAL;
          Serial.println("[SystemController] Mode set to INTERNAL");
        }
        break;
      
      case CMD_START_RECORDING:
        Serial.println("[SystemController] Starting data recording...");
        dataLogger_.startRecording();
        break;
      
      case CMD_STOP_RECORDING:
        Serial.println("[SystemController] Stopping data recording...");
        dataLogger_.stopRecording();
        break;
        
      default:
        break;
    }
    
    // 状態を即座にブロードキャスト
    broadcastState();
  }
}

void SystemController::checkWatchdog() {
  if (state_.mode == MODE_EXTERNAL) {
    if (udp_.isWatchdogTimeout() && state_.motorRunning) {
      Serial.println("[SystemController] Watchdog timeout! Stopping motor...");
      motor_.softStop();
      state_.motorRunning = false;
      state_.currentSpeed = 0;
      
      // INTERNAL modeに戻る
      state_.mode = MODE_INTERNAL;
    }
  }
}

void SystemController::broadcastState() {
  web_.cleanupClients();
  
  // クライアントが接続されている場合のみブロードキャスト
  if (web_.getClientCount() > 0) {
    // 最新の状態を取得
    state_.currentSpeed = motor_.getCurrentSpeed();
    state_.motorRunning = motor_.isRunning();
    state_.stepMode = motor_.getStepMode();
    state_.pwmDuty = motor_.getCurrentDuty();  // DCモーターの場合はduty比、ステッピングは0
    
    // デバッグ: 取得した速度を表示
    Serial.printf("[SystemController] Broadcasting - Current: %.2f, Target: %.2f, Running: %d\n",
                  state_.currentSpeed, state_.targetSpeed, state_.motorRunning);
    
    // ブロードキャスト
    web_.broadcastState(state_);
  }
}

void SystemController::logStatus() {
  Serial.printf("[SystemController] Status - Mode: %s, Speed: %.2f, Running: %d, Free Heap: %d\n",
                state_.getModeString(),
                state_.currentSpeed,
                state_.motorRunning,
                ESP.getFreeHeap());
}

void SystemController::setMotorSpeedInternal(float speed) {
  Serial.printf("[SystemController] Internal mode - Set speed: %.2f\n", speed);
  motor_.setSpeed(speed, state_.stepMode);
  state_.targetSpeed = speed;
  
  // モーター設定後に状態を更新
  state_.currentSpeed = motor_.getCurrentSpeed();
  state_.motorRunning = motor_.isRunning();
  
  Serial.printf("[SystemController] State updated - Current: %.2f, Target: %.2f, Running: %d\n",
                state_.currentSpeed, state_.targetSpeed, state_.motorRunning);
}

void SystemController::setMotorSpeedExternal(float speed) {
  Serial.printf("[SystemController] External mode - Set speed: %.2f\n", speed);
  motor_.setSpeed(speed, state_.stepMode);
  state_.targetSpeed = speed;
  
  // モーター設定後に状態を更新
  state_.currentSpeed = motor_.getCurrentSpeed();
  state_.motorRunning = motor_.isRunning();
  state_.lastUdpTime = millis();
  
  Serial.printf("[SystemController] State updated - Current: %.2f, Target: %.2f, Running: %d\n",
                state_.currentSpeed, state_.targetSpeed, state_.motorRunning);
}
