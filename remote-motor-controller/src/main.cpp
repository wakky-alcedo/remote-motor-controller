/**
 * EtherSpin-ESP: ステッパーモータWebコントローラ
 * 
 * PC上の計算モデル（Python/MATLAB）やWebブラウザから、
 * Wi-Fi経由でステッピングモータを直接ドライブするための高精度制御システム
 * 
 * @author SDDL Project
 * @date 2026/01/07
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <L6470.h>
#include <WiFiUdp.h>

// ============================================================================
// ピン定義 (XIAO ESP32C3用に調整)
// ============================================================================
#define PIN_SPI_SCK   8   // SCK
#define PIN_SPI_MISO  9   // MISO
#define PIN_SPI_MOSI  10  // MOSI
#define PIN_CS        7   // Chip Select
#define PIN_BUSY      6   // BUSY Signal
#define PIN_RESET     5   // RESET Signal

// ============================================================================
// Wi-Fi設定（環境に合わせて変更してください）
// ============================================================================
const char* WIFI_SSID = "SDDLnet";
const char* WIFI_PASSWORD = "smallbear";
const char* MDNS_HOSTNAME = "motor";  // motor.local でアクセス可能

// アクセスポイント設定（WiFi接続失敗時に使用）
const char* AP_SSID = "EtherSpin-ESP";
const char* AP_PASSWORD = " ";  // 最低8文字必要
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GATEWAY(192, 168, 4, 1);
IPAddress AP_SUBNET(255, 255, 255, 0);

// ============================================================================
// UDP設定
// ============================================================================
#define UDP_PORT 8888
WiFiUDP udp;

// ============================================================================
// L6470ドライバ設定
// ============================================================================
L6470 motor(PIN_CS);

// 電気的パラメータ（仕様書準拠）
#define KVAL_PARAM 0x29    // KVAL_HOLD/RUN/ACC/DEC
#define OCD_THRESHOLD 6000 // 過電流検出 (mA)
#define STALL_CURRENT 3000 // ストール電流 (mA)

// マイクロステップ設定
enum StepMode {
  STEP_128 = 128,  // 低速・静音
  STEP_32  = 32,   // 中速
  STEP_8   = 8,    // 高速・高トルク
  STEP_FULL = 1    // フルステップ
};

// 速度閾値（step/s）
#define SPEED_THRESHOLD_LOW  500    // 128→32分割への遷移点
#define SPEED_THRESHOLD_HIGH 2000   // 32→8分割への遷移点

// ============================================================================
// Webサーバー
// ============================================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // WebSocket for real-time updates

// ============================================================================
// システム状態
// ============================================================================
enum ControlMode {
  MODE_INTERNAL,  // 手動制御（Web UI経由）
  MODE_EXTERNAL   // PC連携（UDP経由）
};

struct SystemState {
  ControlMode mode;
  float targetSpeed;      // 目標速度 (step/s)
  float currentSpeed;     // 現在速度 (step/s)
  StepMode stepMode;      // 現在のマイクロステップ設定
  unsigned long lastUdpTime;  // 最後のUDP受信時刻
  bool motorRunning;
  bool emergencyStop;
} state;

// ============================================================================
// ウォッチドッグ設定
// ============================================================================
#define WATCHDOG_TIMEOUT_MS 1000  // 1秒

// ============================================================================
// L6470制御関数
// ============================================================================

/**
 * L6470の初期化と電気的パラメータの設定
 */
void initMotor() {
  Serial.println("[Motor] Initializing L6470...");
  Serial.printf("[Motor] Pin Config - CS:%d SCK:%d MOSI:%d MISO:%d RST:%d BUSY:%d\n",
                PIN_CS, PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_RESET, PIN_BUSY);
  
  // ピン設定（L6470ライブラリのソフトウェアSPI使用）
  Serial.println("[Motor] Setting pins...");
  motor.set_pins(PIN_SPI_SCK, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_RESET, PIN_BUSY);
  Serial.println("[Motor] Pins set successfully");
  
  // 初期化
  Serial.println("[Motor] Calling motor.init()...");
  motor.init();
  Serial.println("[Motor] motor.init() completed");
  delay(100);
  
  // L6470パラメータ設定（仕様書準拠）
  Serial.println("[Motor] Configuring parameters...");
  motor.setMicroSteps(128);  // 初期は128分割
  Serial.println("[Motor]   - MicroSteps: 128");
  motor.setAcc(100);         // 加速度
  Serial.println("[Motor]   - Acceleration: 100");
  motor.setMaxSpeed(800);    // 最大速度
  Serial.println("[Motor]   - MaxSpeed: 800");
  motor.setMinSpeed(1);      // 最小速度
  Serial.println("[Motor]   - MinSpeed: 1");
  motor.setThresholdSpeed(1000);  // 閾値速度
  Serial.println("[Motor]   - ThresholdSpeed: 1000");
  motor.setOverCurrent(OCD_THRESHOLD);  // 過電流検出
  Serial.printf("[Motor]   - OverCurrent: %d mA\n", OCD_THRESHOLD);
  motor.setStallCurrent(STALL_CURRENT); // ストール電流
  Serial.printf("[Motor]   - StallCurrent: %d mA\n", STALL_CURRENT);
  
  Serial.println("[Motor] L6470 initialized successfully");
  
  state.stepMode = STEP_128;
  state.motorRunning = false;
}

/**
 * 動的マイクロステップ最適化
 * 速度域に応じて最適なマイクロステップ設定に自動遷移
 */
void optimizeStepMode(float speed) {
  float absSpeed = abs(speed);
  StepMode newMode = state.stepMode;
  
  if (absSpeed < SPEED_THRESHOLD_LOW) {
    newMode = STEP_128;  // 低速・静音
  } else if (absSpeed < SPEED_THRESHOLD_HIGH) {
    newMode = STEP_32;   // 中速
  } else {
    newMode = STEP_8;    // 高速・高トルク
  }
  
  // モード変更が必要な場合
  if (newMode != state.stepMode) {
    Serial.printf("[Motor] Step mode change: %d -> %d (speed: %.2f)\n", state.stepMode, newMode, speed);
    
    // モータを一時停止
    Serial.println("[Motor] Stopping for mode change...");
    motor.softStop();
    while (motor.isBusy()) delay(10);
    Serial.println("[Motor] Motor stopped");
    
    // ステップモード変更
    Serial.printf("[Motor] Setting micro steps to %d\n", newMode);
    motor.setMicroSteps(newMode);
    Serial.println("[Motor] Mode change completed");
    
    state.stepMode = newMode;
  }
}

/**
 * モータ速度を設定（step/s）
 */
void setMotorSpeed(float speed) {
  Serial.printf("[Motor] setMotorSpeed called: %.2f step/s\n", speed);
  
  // 動的マイクロステップ最適化
  optimizeStepMode(speed);
  
  // 速度設定
  if (abs(speed) < 1.0) {
    // 停止
    Serial.println("[Motor] Stopping motor (speed < 1.0)");
    motor.softStop();
    state.motorRunning = false;
    state.currentSpeed = 0;
  } else {
    // 速度をL6470フォーマットに変換
    long speedValue = abs(speed);
    
    // L6470ライブラリのrun関数: run(dir, speed)
    // dir: 1=正転, 0=逆転
    if (speed > 0) {
      Serial.printf("[Motor] Running forward at %ld step/s\n", speedValue);
      motor.run(1, speedValue);
    } else {
      Serial.printf("[Motor] Running reverse at %ld step/s\n", speedValue);
      motor.run(0, speedValue);
    }
    
    state.motorRunning = true;
    state.currentSpeed = speed;
  }
  
  state.targetSpeed = speed;
  Serial.printf("[Motor] Motor state - Running: %d, Current: %.2f, Target: %.2f\n",
                state.motorRunning, state.currentSpeed, state.targetSpeed);
}

/**
 * 緊急停止（HardStop）
 */
void emergencyStop() {
  Serial.println("[Motor] EMERGENCY STOP!");
  motor.hardStop();
  state.emergencyStop = true;
  state.motorRunning = false;
  state.currentSpeed = 0;
  state.targetSpeed = 0;
}

// ============================================================================
// UDP通信処理
// ============================================================================

/**
 * UDPパケットを処理
 */
void handleUdpPacket() {
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char incomingPacket[256];
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    
    // JSON解析
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, incomingPacket);
    
    if (error) {
      Serial.printf("[UDP] JSON parse error: %s\n", error.c_str());
      return;
    }
    
    // 速度指令を取得
    if (doc.containsKey("v")) {
      float speed = doc["v"];
      
      // External Modeに自動遷移
      if (state.mode != MODE_EXTERNAL) {
        Serial.println("[UDP] Switching to EXTERNAL mode");
        state.mode = MODE_EXTERNAL;
      }
      
      // 速度設定
      setMotorSpeed(speed);
      
      // Watchdog更新
      state.lastUdpTime = millis();
      
      Serial.printf("[UDP] Speed command: %.2f step/s\n", speed);
    }
  }
}

/**
 * ウォッチドッグチェック
 * 指定時間内にUDPパケットが届かない場合、自動停止
 */
void checkWatchdog() {
  if (state.mode == MODE_EXTERNAL) {
    unsigned long elapsed = millis() - state.lastUdpTime;
    
    if (elapsed > WATCHDOG_TIMEOUT_MS && state.motorRunning) {
      Serial.println("[Watchdog] Timeout! Stopping motor...");
      motor.softStop();
      state.motorRunning = false;
      state.currentSpeed = 0;
      
      // INTERNAL modeに戻る
      state.mode = MODE_INTERNAL;
    }
  }
}

// ============================================================================
// Webサーバー処理
// ============================================================================

/**
 * WebSocket経由でシステム状態を送信
 */
void broadcastSystemState() {
  JsonDocument doc;
  doc["mode"] = (state.mode == MODE_INTERNAL) ? "internal" : "external";
  doc["speed"] = state.currentSpeed;
  doc["target"] = state.targetSpeed;
  doc["running"] = state.motorRunning;
  doc["emergency"] = state.emergencyStop;
  doc["stepMode"] = state.stepMode;
  
  String output;
  serializeJson(doc, output);
  ws.textAll(output);
}

/**
 * WebSocketイベントハンドラ
 */
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                      AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WebSocket] Client #%u connected\n", client->id());
    // 接続時に現在の状態を送信
    broadcastSystemState();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WebSocket] Client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      data[len] = 0;
      
      // JSON解析
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, (char*)data);
      
      if (!error) {
        // コマンド処理
        if (doc.containsKey("cmd")) {
          String cmd = doc["cmd"].as<String>();
          
          if (cmd == "setSpeed" && doc.containsKey("value")) {
            float speed = doc["value"];
            state.mode = MODE_INTERNAL;
            setMotorSpeed(speed);
            Serial.printf("[WebSocket] Set speed: %.2f\n", speed);
          } else if (cmd == "stop") {
            setMotorSpeed(0);
            Serial.println("[WebSocket] Stop");
          } else if (cmd == "emergencyStop") {
            emergencyStop();
            Serial.println("[WebSocket] Emergency Stop");
          } else if (cmd == "reset") {
            state.emergencyStop = false;
            Serial.println("[WebSocket] Reset emergency stop");
          } else if (cmd == "setMode" && doc.containsKey("value")) {
            String modeStr = doc["value"].as<String>();
            state.mode = (modeStr == "external") ? MODE_EXTERNAL : MODE_INTERNAL;
            Serial.printf("[WebSocket] Set mode: %s\n", modeStr.c_str());
          }
          
          broadcastSystemState();
        }
      }
    }
  }
}

/**
 * HTMLページを生成
 */
const char* getIndexHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html lang="ja">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>EtherSpin-ESP Controller</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
            background: white;
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
        }
        h1 {
            color: #667eea;
            text-align: center;
            margin-bottom: 10px;
            font-size: 2em;
        }
        .subtitle {
            text-align: center;
            color: #666;
            margin-bottom: 30px;
        }
        .status-card {
            background: #f8f9fa;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        .status-item {
            text-align: center;
        }
        .status-label {
            font-size: 0.85em;
            color: #666;
            margin-bottom: 5px;
        }
        .status-value {
            font-size: 1.5em;
            font-weight: bold;
            color: #333;
        }
        .control-section {
            margin: 20px 0;
        }
        .control-section h2 {
            color: #333;
            font-size: 1.3em;
            margin-bottom: 15px;
            padding-bottom: 10px;
            border-bottom: 2px solid #667eea;
        }
        .slider-container {
            margin: 20px 0;
        }
        .slider {
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: #ddd;
            outline: none;
            margin: 10px 0;
        }
        .slider::-webkit-slider-thumb {
            appearance: none;
            width: 24px;
            height: 24px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
        }
        .btn {
            padding: 12px 24px;
            border: none;
            border-radius: 8px;
            font-size: 1em;
            cursor: pointer;
            transition: all 0.3s;
            font-weight: 600;
            margin: 5px;
        }
        .btn:hover {
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.2);
        }
        .btn-primary {
            background: #667eea;
            color: white;
        }
        .btn-danger {
            background: #e74c3c;
            color: white;
            font-size: 1.2em;
            padding: 15px 30px;
        }
        .btn-success {
            background: #27ae60;
            color: white;
        }
        .mode-switch {
            display: flex;
            gap: 10px;
            justify-content: center;
            margin: 20px 0;
        }
        .mode-btn {
            flex: 1;
            padding: 15px;
            border: 2px solid #667eea;
            background: white;
            color: #667eea;
            border-radius: 8px;
            cursor: pointer;
            font-weight: 600;
            transition: all 0.3s;
        }
        .mode-btn.active {
            background: #667eea;
            color: white;
        }
        .emergency-section {
            text-align: center;
            margin-top: 30px;
            padding-top: 30px;
            border-top: 2px solid #eee;
        }
        .connection-status {
            display: inline-block;
            padding: 5px 15px;
            border-radius: 20px;
            font-size: 0.85em;
            margin-bottom: 20px;
        }
        .connection-status.connected {
            background: #d4edda;
            color: #155724;
        }
        .connection-status.disconnected {
            background: #f8d7da;
            color: #721c24;
        }
        #speedChart {
            margin-top: 20px;
            height: 200px;
            background: #f8f9fa;
            border-radius: 10px;
            padding: 15px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🔄 EtherSpin-ESP</h1>
        <p class="subtitle">ステッパーモータWebコントローラ</p>
        
        <div style="text-align: center;">
            <span class="connection-status" id="connectionStatus">接続中...</span>
        </div>

        <div class="status-card">
            <h3 style="margin-bottom: 15px;">📊 リアルタイムステータス</h3>
            <div class="status-grid">
                <div class="status-item">
                    <div class="status-label">現在速度</div>
                    <div class="status-value" id="currentSpeed">0</div>
                    <div style="font-size: 0.8em; color: #999;">step/s</div>
                </div>
                <div class="status-item">
                    <div class="status-label">目標速度</div>
                    <div class="status-value" id="targetSpeed">0</div>
                    <div style="font-size: 0.8em; color: #999;">step/s</div>
                </div>
                <div class="status-item">
                    <div class="status-label">制御モード</div>
                    <div class="status-value" id="modeStatus">Internal</div>
                </div>
                <div class="status-item">
                    <div class="status-label">モータ状態</div>
                    <div class="status-value" id="motorStatus">停止</div>
                </div>
            </div>
        </div>

        <div class="control-section">
            <h2>⚙️ 制御設定</h2>
            
            <div class="mode-switch">
                <button class="mode-btn active" onclick="setMode('internal')" id="btnInternal">
                    Internal Mode<br><small>手動制御</small>
                </button>
                <button class="mode-btn" onclick="setMode('external')" id="btnExternal">
                    External Mode<br><small>PC連携</small>
                </button>
            </div>

            <div class="slider-container">
                <label style="display: block; margin-bottom: 10px; font-weight: 600;">
                    速度調整: <span id="speedValue">0</span> step/s
                </label>
                <input type="range" min="-3000" max="3000" value="0" class="slider" 
                       id="speedSlider" oninput="updateSpeedDisplay(this.value)">
                <div style="display: flex; justify-content: space-between; font-size: 0.85em; color: #666;">
                    <span>-3000</span>
                    <span>0</span>
                    <span>3000</span>
                </div>
            </div>

            <div style="text-align: center; margin-top: 20px;">
                <button class="btn btn-primary" onclick="applySpeed()">速度を適用</button>
                <button class="btn btn-success" onclick="stopMotor()">停止</button>
            </div>
        </div>

        <div class="emergency-section">
            <h2 style="color: #e74c3c; margin-bottom: 20px;">🚨 緊急停止</h2>
            <button class="btn btn-danger" onclick="emergencyStop()">緊急停止 (EMG STOP)</button>
            <p style="margin-top: 10px; color: #666; font-size: 0.9em;">
                緊急停止後は、リセットボタンでシステムを再起動してください
            </p>
            <button class="btn btn-primary" onclick="resetEmergency()" style="margin-top: 10px;">
                リセット
            </button>
        </div>
    </div>

    <script>
        let ws;
        let reconnectInterval;

        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + '/ws');
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                document.getElementById('connectionStatus').textContent = '接続済み';
                document.getElementById('connectionStatus').className = 'connection-status connected';
                if (reconnectInterval) {
                    clearInterval(reconnectInterval);
                    reconnectInterval = null;
                }
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                document.getElementById('connectionStatus').textContent = '切断';
                document.getElementById('connectionStatus').className = 'connection-status disconnected';
                
                if (!reconnectInterval) {
                    reconnectInterval = setInterval(connectWebSocket, 3000);
                }
            };
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                updateStatus(data);
            };
        }

        function updateStatus(data) {
            document.getElementById('currentSpeed').textContent = data.speed.toFixed(1);
            document.getElementById('targetSpeed').textContent = data.target.toFixed(1);
            document.getElementById('modeStatus').textContent = 
                data.mode === 'internal' ? 'Internal' : 'External';
            document.getElementById('motorStatus').textContent = 
                data.running ? '動作中' : '停止';
            
            // Update mode buttons
            document.getElementById('btnInternal').classList.toggle('active', data.mode === 'internal');
            document.getElementById('btnExternal').classList.toggle('active', data.mode === 'external');
        }

        function sendCommand(cmd, value = null) {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const msg = { cmd: cmd };
                if (value !== null) msg.value = value;
                ws.send(JSON.stringify(msg));
            }
        }

        function updateSpeedDisplay(value) {
            document.getElementById('speedValue').textContent = value;
        }

        function applySpeed() {
            const speed = parseFloat(document.getElementById('speedSlider').value);
            sendCommand('setSpeed', speed);
        }

        function stopMotor() {
            sendCommand('stop');
            document.getElementById('speedSlider').value = 0;
            document.getElementById('speedValue').textContent = '0';
        }

        function emergencyStop() {
            if (confirm('緊急停止を実行しますか？')) {
                sendCommand('emergencyStop');
            }
        }

        function resetEmergency() {
            sendCommand('reset');
        }

        function setMode(mode) {
            sendCommand('setMode', mode);
        }

        // Initialize
        connectWebSocket();
    </script>
</body>
</html>
)rawliteral";
}

/**
 * Webサーバー初期化
 */
void initWebServer() {
  Serial.println("[Web] Initializing web server...");
  
  // WebSocketハンドラ
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  // トップページ
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", getIndexHTML());
  });
  
  // ステータスAPI
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument doc;
    doc["mode"] = (state.mode == MODE_INTERNAL) ? "internal" : "external";
    doc["speed"] = state.currentSpeed;
    doc["target"] = state.targetSpeed;
    doc["running"] = state.motorRunning;
    doc["emergency"] = state.emergencyStop;
    
    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
  });
  
  server.begin();
  Serial.println("[Web] Web server started");
}

// ============================================================================
// Wi-Fi接続
// ============================================================================

void initWiFi() {
  Serial.println("[WiFi] Connecting to WiFi...");
  Serial.printf("[WiFi] SSID: %s\n", WIFI_SSID);
  Serial.printf("[WiFi] Setting WiFi mode to STA...\n");
  
  WiFi.mode(WIFI_STA);
  Serial.println("[WiFi] Starting connection...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts % 10 == 0) {
      Serial.printf(" [%d/30]\n", attempts);
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.printf("[WiFi] IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("[WiFi] Subnet: %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("[WiFi] DNS: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("[WiFi] Signal Strength (RSSI): %d dBm\n", WiFi.RSSI());
    
    // mDNS設定
    Serial.println("[mDNS] Starting mDNS responder...");
    if (MDNS.begin(MDNS_HOSTNAME)) {
      Serial.printf("[mDNS] Responder started: http://%s.local\n", MDNS_HOSTNAME);
      MDNS.addService("http", "tcp", 80);
      Serial.println("[mDNS] HTTP service registered");
    } else {
      Serial.println("[mDNS] Failed to start mDNS responder");
    }
  } else {
    Serial.println("\n[WiFi] Connection failed!");
    Serial.printf("[WiFi] Final status code: %d\n", WiFi.status());
    Serial.println("[WiFi] Possible reasons:");
    Serial.println("[WiFi]   - Wrong SSID or password");
    Serial.println("[WiFi]   - Router out of range");
    Serial.println("[WiFi]   - Router authentication issues");
    
    // アクセスポイントモードで起動
    Serial.println("\n[WiFi] Starting Access Point mode...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
    
    if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
      Serial.println("[WiFi] Access Point started successfully!");
      Serial.printf("[WiFi] AP SSID: %s\n", AP_SSID);
      Serial.printf("[WiFi] AP Password: %s\n", AP_PASSWORD);
      Serial.printf("[WiFi] AP IP Address: %s\n", WiFi.softAPIP().toString().c_str());
      Serial.println("[WiFi] Connect to this WiFi network to access the web interface");
      
      // mDNS設定
      if (MDNS.begin(MDNS_HOSTNAME)) {
        Serial.printf("[mDNS] Responder started: http://%s.local\n", MDNS_HOSTNAME);
        MDNS.addService("http", "tcp", 80);
      }
    } else {
      Serial.println("[WiFi] Failed to start Access Point!");
    }
  }
}

// ============================================================================
// Setup & Loop
// ============================================================================

void setup() {
  // シリアル初期化
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n===========================================");
  Serial.println("EtherSpin-ESP: Stepper Motor Web Controller");
  Serial.println("===========================================\n");
  Serial.println("[System] Starting initialization sequence...");
  Serial.printf("[System] ESP32 Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("[System] Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("[System] CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("[System] Free Heap: %d bytes\n", ESP.getFreeHeap());
  
  // システム状態初期化
  Serial.println("[System] Initializing system state...");
  state.mode = MODE_INTERNAL;
  state.targetSpeed = 0;
  state.currentSpeed = 0;
  state.stepMode = STEP_128;
  state.lastUdpTime = millis();
  state.motorRunning = false;
  state.emergencyStop = false;
  Serial.println("[System] System state initialized");
  
  // L6470初期化
  Serial.println("[System] Step 1/4: Initializing L6470 motor driver...");
  initMotor();
  Serial.println("[System] Motor driver initialization complete\n");
  
  // Wi-Fi接続
  Serial.println("[System] Step 2/4: Connecting to WiFi...");
  initWiFi();
  Serial.println("[System] WiFi initialization complete\n");
  
  // UDP開始
  Serial.println("[System] Step 3/4: Starting UDP server...");
  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);
  Serial.println("[System] UDP server started\n");
  
  // Webサーバー起動
  Serial.println("[System] Step 4/4: Starting web server...");
  initWebServer();
  Serial.println("[System] Web server initialization complete\n");
  
  Serial.println("\n=== System Ready ===");
  Serial.println("Access via:");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("  - http://%s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  - http://%s.local\n", MDNS_HOSTNAME);
    Serial.printf("  - UDP: Port %d\n", UDP_PORT);
  } else if (WiFi.getMode() == WIFI_AP) {
    Serial.printf("  - Connect to WiFi: %s\n", AP_SSID);
    Serial.printf("  - Password: %s\n", AP_PASSWORD);
    Serial.printf("  - Then open: http://%s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("  - Or: http://%s.local\n", MDNS_HOSTNAME);
  } else {
    Serial.println("  - No network connection");
  }
  Serial.println("======================\n");
}

void loop() {
  // 起動確認用（最初の1回のみ）
  static bool firstLoop = true;
  if (firstLoop) {
    Serial.println("[Loop] Entering main loop");
    firstLoop = false;
  }
  
  // UDP通信処理
  handleUdpPacket();
  
  // ウォッチドッグチェック
  checkWatchdog();
  
  // WebSocket状態ブロードキャスト（500msごと、クライアント接続時のみ）
  static unsigned long lastBroadcast = 0;
  if (millis() - lastBroadcast > 500) {
    ws.cleanupClients();
    // クライアントが接続されている場合のみブロードキャスト
    if (ws.count() > 0) {
      broadcastSystemState();
    }
    lastBroadcast = millis();
  }
  
  // 定期的な状態ログ出力（10秒ごと）
  static unsigned long lastStatusLog = 0;
  if (millis() - lastStatusLog > 10000) {
    Serial.printf("[Status] Mode: %s, Speed: %.2f, Running: %d, Free Heap: %d\n",
                  state.mode == MODE_INTERNAL ? "INT" : "EXT",
                  state.currentSpeed,
                  state.motorRunning,
                  ESP.getFreeHeap());
    lastStatusLog = millis();
  }
  
  // 少し待機
  delay(10);
}