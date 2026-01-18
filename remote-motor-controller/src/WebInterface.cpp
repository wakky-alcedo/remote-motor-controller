/**
 * WebInterface.cpp
 * Webサーバー・WebSocket管理クラス実装
 * 
 * @author SDDL Project
 * @date 2026/01/17
 */

#include "WebInterface.h"
#include "config.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

WebInterface::WebInterface(uint16_t port)
  : server_(port),
    ws_("/ws"),
    port_(port),
    hasNewCommand_(false) {
}

WebInterface::~WebInterface() {
  // サーバーの停止処理はAsyncWebServerが自動的に行う
}

// ============================================================================
// 公開メソッド
// ============================================================================

bool WebInterface::init() {
  Serial.println("[WebInterface] Initializing web server...");
  
  // WebSocketハンドラ設定
  setupWebSocketHandler();
  server_.addHandler(&ws_);
  
  // トップページ
  server_.on("/", HTTP_GET, [this](AsyncWebServerRequest *request) {
    request->send(200, "text/html", getIndexHTML());
  });
  
  // ステータスAPI（使用しないが互換性のため残す）
  server_.on("/api/status", HTTP_GET, [this](AsyncWebServerRequest *request) {
    // ダミーの状態を返す（実際の状態は別途管理）
    request->send(200, "application/json", "{\"status\":\"ok\"}");
  });
  
  // サーバー開始
  server_.begin();
  Serial.printf("[WebInterface] Web server started on port %d\n", port_);
  
  return true;
}

void WebInterface::broadcastState(const SystemState& state) {
  JsonDocument doc;
  doc["mode"] = (state.mode == MODE_INTERNAL) ? "internal" : "external";
  doc["speed"] = state.currentSpeed;
  doc["target"] = state.targetSpeed;
  doc["running"] = state.motorRunning;
  doc["emergency"] = state.emergencyStop;
  doc["stepMode"] = state.stepMode;
  
  String output;
  serializeJson(doc, output);
  
  // デバッグ: ブロードキャストする内容を表示
  Serial.printf("[WebInterface] Broadcasting: %s\n", output.c_str());
  
  ws_.textAll(output);
}

void WebInterface::cleanupClients() {
  ws_.cleanupClients();
}

size_t WebInterface::getClientCount() const {
  return ws_.count();
}

bool WebInterface::hasCommand(WebCommandData& commandData) {
  if (hasNewCommand_) {
    commandData = pendingCommand_;
    hasNewCommand_ = false;
    return true;
  }
  return false;
}

void WebInterface::setupWebSocketHandler() {
  // WebSocketイベントハンドラを設定
  // Lambda関数を使用してメンバ関数を呼び出す
  ws_.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client,
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
    this->onWebSocketEvent(server, client, type, arg, data, len);
  });
}

// ============================================================================
// プライベートメソッド
// ============================================================================

void WebInterface::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                                    AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WebInterface] WebSocket client #%u connected\n", client->id());
    // 接続時は状態をブロードキャストする（SystemControllerが行う）
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WebInterface] WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len) {
      handleWebSocketMessage(data, len);
    }
  }
}

void WebInterface::handleWebSocketMessage(uint8_t *data, size_t len) {
  // Null終端を追加
  data[len] = 0;
  
  // JSON解析
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, (char*)data);
  
  if (error) {
    Serial.printf("[WebInterface] JSON parse error: %s\n", error.c_str());
    return;
  }
  
  // コマンド処理
  if (doc.containsKey("cmd")) {
    String cmd = doc["cmd"].as<String>();
    
    WebCommandData commandData;
    
    if (cmd == "setSpeed" && doc.containsKey("value")) {
      commandData.command = CMD_SET_SPEED;
      commandData.value = doc["value"];
      Serial.printf("[WebInterface] Command: setSpeed(%.2f)\n", commandData.value);
    } else if (cmd == "stop") {
      commandData.command = CMD_STOP;
      Serial.println("[WebInterface] Command: stop");
    } else if (cmd == "emergencyStop") {
      commandData.command = CMD_EMERGENCY_STOP;
      Serial.println("[WebInterface] Command: emergencyStop");
    } else if (cmd == "reset") {
      commandData.command = CMD_RESET;
      Serial.println("[WebInterface] Command: reset");
    } else if (cmd == "setMode" && doc.containsKey("value")) {
      commandData.command = CMD_SET_MODE;
      commandData.stringValue = doc["value"].as<String>();
      Serial.printf("[WebInterface] Command: setMode(%s)\n", commandData.stringValue.c_str());
    }
    
    // コマンドを保存
    if (commandData.command != CMD_NONE) {
      pendingCommand_ = commandData;
      hasNewCommand_ = true;
    }
  }
}

const char* WebInterface::getIndexHTML() {
  // MOTOR_MAX_SPEEDに基づいて動的にHTMLを生成
  static String html;
  html = R"rawliteral(
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
                    <div style="font-size: 0.8em; color: #999;" id="currentSpeedUnit">)rawliteral";
  #ifdef MOTOR_TYPE_DC
  html += R"rawliteral(RPM)rawliteral";
  #else
  html += R"rawliteral(step/s)rawliteral";
  #endif
  html += R"rawliteral(</div>
                </div>
                <div class="status-item">
                    <div class="status-label">目標速度</div>
                    <div class="status-value" id="targetSpeed">0</div>
                    <div style="font-size: 0.8em; color: #999;" id="targetSpeedUnit">)rawliteral";
  #ifdef MOTOR_TYPE_DC
  html += R"rawliteral(RPM)rawliteral";
  #else
  html += R"rawliteral(step/s)rawliteral";
  #endif
  html += R"rawliteral(</div>
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
                    速度調整: <span id="speedValue">0</span> <span id="speedSliderUnit">)rawliteral";
  #ifdef MOTOR_TYPE_DC
  html += R"rawliteral(RPM)rawliteral";
  #else
  html += R"rawliteral(step/s)rawliteral";
  #endif
  html += R"rawliteral(</span>
                </label>
                <input type="range" min=")rawliteral";
  #ifdef MOTOR_TYPE_DC
  html += String(-DC_MOTOR_MAX_RPM);
  #else
  html += String(-MOTOR_MAX_SPEED);
  #endif
  html += R"rawliteral(" max=")rawliteral";
  #ifdef MOTOR_TYPE_DC
  html += String(DC_MOTOR_MAX_RPM);
  #else
  html += String(MOTOR_MAX_SPEED);
  #endif
  html += R"rawliteral(" value="0" class="slider" 
                       id="speedSlider" oninput="updateSpeedDisplay(this.value)">
                <div style="display: flex; justify-content: space-between; font-size: 0.85em; color: #666;">
                    <span>)rawliteral";
  #ifdef MOTOR_TYPE_DC
  html += String(-DC_MOTOR_MAX_RPM);
  #else
  html += String(-MOTOR_MAX_SPEED);
  #endif
  html += R"rawliteral(</span>
                    <span>0</span>
                    <span>)rawliteral";
  #ifdef MOTOR_TYPE_DC
  html += String(DC_MOTOR_MAX_RPM);
  #else
  html += String(MOTOR_MAX_SPEED);
  #endif
  html += R"rawliteral(</span>
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
  
  return html.c_str();
}
