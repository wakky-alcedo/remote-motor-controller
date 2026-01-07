# EtherSpin-ESP: ステッパーモータWebコントローラ

PC上の計算モデル（Python/MATLAB）やWebブラウザから、Wi-Fi経由でステッピングモータを直接ドライブするための高精度制御システム。

## 🌟 特徴

- **ワイヤレス・ストリーミング制御**: PC側で計算した速度プロファイルをUDPでストリーミング
- **L6470 自律パルス生成**: Wi-Fi通信負荷によるパルスの乱れを物理的に排除
- **ハイブリッド・インターフェース**: UDP（リアルタイム制御）+ Webブラウザ（設定・モニタリング）
- **動的マイクロステップ最適化**: 速度域に応じて自動的に最適なステップモードに遷移
- **フェイルセーフ・ウォッチドッグ**: 通信途絶時の自動停止機能

## 📋 必要なハードウェア

- **マイコン**: Seeed XIAO ESP32C3（または互換ESP32ボード）
- **モータドライバ**: L6470（SPI接続）
- **ステッピングモータ**: L6470対応のもの
- **電源**: モータ用電源（12V/24V推奨）

## 🔌 配線図

| ESP32C3 | L6470 | 機能 |
|---------|-------|------|
| GPIO 8  | SCK   | SPI Clock |
| GPIO 9  | MISO  | SPI MISO |
| GPIO 10 | MOSI  | SPI MOSI |
| GPIO 7  | CS    | Chip Select |
| GPIO 6  | BUSY  | Busy Signal |
| GPIO 5  | RESET | Reset Signal |

## 🚀 セットアップ手順

### 1. 開発環境の準備

PlatformIOをインストール（VS Code拡張機能推奨）:
```bash
# VS Codeの拡張機能から "PlatformIO IDE" をインストール
```

### 2. WiFi設定の編集

`include/config.h` または `src/main.cpp` のWi-Fi設定を編集:
```cpp
const char* WIFI_SSID = "your_wifi_ssid";        // あなたのWi-Fi SSID
const char* WIFI_PASSWORD = "your_wifi_password"; // あなたのWi-Fiパスワード
```

### 3. ビルドと書き込み

```bash
# PlatformIO CLIを使用する場合
cd remote-motor-control
pio run --target upload

# VS Codeを使用する場合
# 1. PlatformIOアイコンをクリック
# 2. "Upload" をクリック
```

### 4. シリアルモニタで確認

```bash
# PlatformIO CLIの場合
pio device monitor

# VS Codeの場合
# "Serial Monitor" をクリック
```

起動時に以下のような情報が表示されます:
```
[WiFi] IP Address: 192.168.1.100
[mDNS] Responder started: http://motor.local
[UDP] Listening on port 8888
```

## 🌐 使用方法

### Webブラウザから制御

1. ブラウザで `http://motor.local` または `http://[ESP32のIPアドレス]` にアクセス
2. ダッシュボードで現在の状態を確認
3. スライダーで速度を調整して「速度を適用」をクリック
4. 緊急停止が必要な場合は「緊急停止」ボタンをクリック

### Pythonから制御

サンプルコードを実行:

```bash
# 1. 定速回転
python udp_client.py 1

# 2. 台形加速プロファイル
python udp_client.py 2

# 3. サイン波速度プロファイル
python udp_client.py 3

# 4. S字カーブ加速
python udp_client.py 4
```

### カスタムPythonスクリプト

```python
import socket
import json
import time

# モータコントローラ初期化
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
esp32_ip = "motor.local"  # またはIPアドレス
udp_port = 8888

# 速度を設定（step/s）
def set_speed(speed):
    message = json.dumps({"v": float(speed)})
    sock.sendto(message.encode(), (esp32_ip, udp_port))

# 例: 1000 step/sで3秒間回転
set_speed(1000)
time.sleep(3)
set_speed(0)  # 停止
```

## 📡 UDP通信プロトコル

### 速度指令
```json
{"v": 1500.0}
```
- `v`: 速度（step/s）
  - 正の値: 正回転
  - 負の値: 逆回転
  - 0: 停止

### 推奨更新頻度
- 20ms〜100ms（10Hz〜50Hz）

### ウォッチドッグ
- UDP通信が1秒以上途絶えると自動的にモータが停止します
- 継続的に速度指令を送信することでモータが動作し続けます

## ⚙️ パラメータ調整

### L6470電気的パラメータ

`src/main.cpp` で調整可能:

```cpp
#define KVAL_PARAM 0x29        // KVAL設定（トルク調整）
#define OCD_THRESHOLD 0x0F     // 過電流検出閾値
```

### 動的マイクロステップ最適化

速度閾値の調整:

```cpp
#define SPEED_THRESHOLD_LOW  500    // 128→32分割への遷移点
#define SPEED_THRESHOLD_HIGH 2000   // 32→8分割への遷移点
```

### ウォッチドッグタイムアウト

```cpp
#define WATCHDOG_TIMEOUT_MS 1000  // タイムアウト時間（ミリ秒）
```

## 🐛 トラブルシューティング

### WiFiに接続できない
- SSIDとパスワードが正しいか確認
- 2.4GHz帯のWi-Fiを使用しているか確認（ESP32は5GHz非対応）

### モータが動かない
- L6470の配線を確認
- 電源が供給されているか確認
- シリアルモニタでエラーメッセージを確認

### motor.localにアクセスできない
- ルーターがmDNSをサポートしているか確認
- 代わりにIPアドレスで直接アクセス
- Windowsの場合はBonjour Print Servicesのインストールが必要な場合があります

### UDP通信が届かない
- ファイアウォールでUDPポート8888が許可されているか確認
- ESP32とPCが同じネットワークに接続されているか確認

## 📚 API リファレンス

### Web UI エンドポイント

- `GET /` - メインダッシュボード
- `GET /api/status` - システム状態をJSONで取得
- `WebSocket /ws` - リアルタイム双方向通信

### WebSocketメッセージ

#### クライアント → サーバー
```json
{"cmd": "setSpeed", "value": 1000}    // 速度設定
{"cmd": "stop"}                        // 停止
{"cmd": "emergencyStop"}              // 緊急停止
{"cmd": "reset"}                      // 緊急停止リセット
{"cmd": "setMode", "value": "external"} // モード切替
```

#### サーバー → クライアント
```json
{
  "mode": "internal",
  "speed": 1000.0,
  "target": 1000.0,
  "running": true,
  "emergency": false,
  "stepMode": 0
}
```

## 📄 ライセンス

このプロジェクトはMITライセンスの下で公開されています。

## 👥 開発者

SDDL Project

## 📝 更新履歴

- **2026/01/07**: 初回リリース
  - L6470制御機能実装
  - UDP通信機能実装
  - Webダッシュボード実装
  - ウォッチドッグ機能実装
  - 動的マイクロステップ最適化実装
