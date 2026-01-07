# セットアップガイド

## 📦 必要なもの

### ハードウェア
- Seeed XIAO ESP32C3
- L6470 ステッピングモータドライバ
- ステッピングモータ（バイポーラ型）
- 電源アダプタ（モータ用：12V-48V、1A以上推奨）
- ジャンパーワイヤー
- ブレッドボード（オプション）

### ソフトウェア
- Visual Studio Code
- PlatformIO IDE拡張機能
- Python 3.7以上（Python制御を使用する場合）
- MATLAB R2020b以上（MATLAB制御を使用する場合）

## 🔧 ハードウェアセットアップ

### 1. ESP32C3とL6470の接続

```
[XIAO ESP32C3]          [L6470]
GPIO 8  (SCK)   ----->  SCK
GPIO 9  (MISO)  <-----  SDO (MISO)
GPIO 10 (MOSI)  ----->  SDI (MOSI)
GPIO 7  (CS)    ----->  CS
GPIO 6  (BUSY)  <-----  BUSY
GPIO 5  (RESET) ----->  STBY/RESET

3.3V            ----->  VDD
GND             ----->  GND
```

⚠️ **重要**: 
- L6470のVDDは3.3Vに接続（ロジック電源）
- モータ用電源は別途L6470のVSとGNDに接続
- 電源は必ずGNDを共通にしてください

### 2. モータの接続

L6470の出力端子にステッピングモータを接続:
```
L6470 OUT1A/B -> モータ Coil A
L6470 OUT2A/B -> モータ Coil B
```

### 3. 電源接続

```
[電源アダプタ]
  (+) ----->  L6470 VS (モータ用電源)
  (-) ----->  GND (共通GND)
```

推奨電源電圧:
- 12V: 小型モータ、低トルク用
- 24V: 中型モータ、標準トルク用（推奨）
- 48V: 大型モータ、高速・高トルク用

## 💻 ソフトウェアセットアップ

### 1. PlatformIOのインストール

1. Visual Studio Codeを起動
2. 拡張機能アイコンをクリック
3. "PlatformIO IDE" を検索
4. "Install" をクリック
5. VS Codeを再起動

### 2. プロジェクトを開く

```bash
# プロジェクトフォルダを開く
code remote-motor-control
```

または、VS CodeのFile > Open Folderから `remote-motor-control` フォルダを開く

### 3. Wi-Fi設定

`src/main.cpp` の以下の部分を編集:

```cpp
// Wi-Fi設定（環境に合わせて変更してください）
const char* WIFI_SSID = "your_wifi_ssid";        // ← あなたのWi-Fi SSID
const char* WIFI_PASSWORD = "your_wifi_password"; // ← あなたのWi-Fiパスワード
```

### 4. ビルドとアップロード

#### 方法1: VS Code（推奨）

1. PlatformIOのアイコンをクリック（左サイドバー）
2. "seeed_xiao_esp32c3" > "General" > "Upload" をクリック
3. アップロードが完了するまで待つ

#### 方法2: コマンドライン

```bash
cd remote-motor-control
pio run --target upload
```

### 5. 動作確認

1. シリアルモニタを開く（PlatformIO > Monitor）
2. 以下のようなメッセージが表示されることを確認:

```
===========================================
EtherSpin-ESP: Stepper Motor Web Controller
===========================================

[Motor] Initializing L6470...
[Motor] L6470 initialized successfully
[WiFi] Connecting to WiFi...
[WiFi] Connected!
[WiFi] IP Address: 192.168.1.100
[mDNS] Responder started: http://motor.local
[UDP] Listening on port 8888
[Web] Web server started

=== System Ready ===
```

## 🌐 接続テスト

### Webブラウザからのテスト

1. ブラウザを開く
2. `http://motor.local` にアクセス
   - うまくいかない場合は、シリアルモニタに表示されたIPアドレス（例: `http://192.168.1.100`）を使用
3. ダッシュボードが表示されることを確認
4. スライダーを動かして「速度を適用」をクリック
5. モータが回転することを確認

### Pythonからのテスト

```bash
# Python環境の準備（初回のみ）
python -m pip install --upgrade pip

# テスト実行
cd remote-motor-control
python udp_client.py 1
```

モータが500 step/sで5秒間回転し、停止すれば成功です。

### MATLABからのテスト

1. MATLABを起動
2. `remote-motor-control` フォルダに移動
3. `udp_client.m` を開く
4. 先頭の設定セクションを実行（Ctrl+Enter）
5. "例1: 定速回転" セクションを実行
6. モータが回転することを確認

## 🔍 トラブルシューティング

### WiFiに接続できない

**症状**: シリアルモニタに "WiFi Connection failed!" と表示される

**解決方法**:
1. SSIDとパスワードが正しいか確認
2. 2.4GHz帯のWi-Fiを使用しているか確認（5GHzは非対応）
3. Wi-FiのSSIDが隠蔽されていないか確認
4. ルーターのファイアウォール設定を確認

### モータが動かない

**症状**: Webから制御しても音がしない、動かない

**解決方法**:
1. モータ用電源が供給されているか確認（テスターで電圧測定）
2. L6470の配線を再確認（特にSPIピン）
3. シリアルモニタでエラーメッセージを確認
4. L6470のLEDが点灯しているか確認
5. モータが固定されておらず、自由に回転できるか確認

### L6470が認識されない

**症状**: "[Motor] L6470 initialization failed" と表示される

**解決方法**:
1. SPI配線（SCK、MISO、MOSI、CS）を確認
2. L6470の電源（VDD=3.3V）を確認
3. RESETピンの配線を確認
4. L6470の初期不良の可能性を確認

### motor.localにアクセスできない

**症状**: ブラウザで "このサイトにアクセスできません" と表示

**解決方法**:
1. ESP32とPCが同じネットワークに接続されているか確認
2. シリアルモニタからIPアドレスを確認し、直接アクセス
3. Windowsの場合、Bonjour Print Servicesをインストール
   - https://support.apple.com/kb/DL999
4. ファイアウォールでHTTP（ポート80）が許可されているか確認

### UDP通信が届かない

**症状**: Pythonスクリプトを実行してもモータが動かない

**解決方法**:
1. ESP32とPCが同じネットワークに接続されているか確認
2. ファイアウォールでUDPポート8888が許可されているか確認
3. Pythonスクリプトで正しいIPアドレスを使用しているか確認
4. シリアルモニタで "[UDP] Speed command" メッセージが表示されるか確認

### モータが途中で止まる

**症状**: 数秒動いた後、自動的に停止する

**原因**: ウォッチドッグ機能が作動

**解決方法**:
1. UDP通信を継続的に送信する（最低でも1秒に1回）
2. サンプルコードの更新間隔（`dt`）を短くする
3. ウォッチドッグタイムアウトを延長（`WATCHDOG_TIMEOUT_MS`を変更）

## 📊 パフォーマンス最適化

### 通信遅延の最小化

```python
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
```

### 高頻度更新

推奨更新頻度:
- 低速（<500 step/s）: 50ms（20Hz）
- 中速（500-2000 step/s）: 20ms（50Hz）
- 高速（>2000 step/s）: 10ms（100Hz）

### マイクロステップ設定の調整

```cpp
#define SPEED_THRESHOLD_LOW  500    // より静かな動作が必要な場合は上げる
#define SPEED_THRESHOLD_HIGH 2000   // より高速が必要な場合は下げる
```

## 🔐 セキュリティに関する注意

⚠️ **重要**: このシステムは産業用途を想定していません。

- パスワード認証は実装されていません
- 同一ネットワーク上の誰でも制御可能です
- 重要な用途には使用しないでください
- 安全装置（リミットスイッチ等）を別途設置してください

## 📖 次のステップ

1. [README.md](README.md) で全体像を把握
2. [SPECIFICATION.md](../SPECIFICATION.md) で技術仕様を確認
3. サンプルコード（`udp_client.py`, `udp_client.m`）を参考に独自の制御プログラムを作成
4. 実際の用途に合わせてパラメータをチューニング

## 💡 ヒント

- 初めてのテストでは低速（100-500 step/s）から始めましょう
- 加速・減速プロファイルを使うことで、脱調を防げます
- L6470のKVAL設定を調整することでトルクを最適化できます
- モータが発熱する場合は、電流制限（OCD_THRESHOLD）を下げてください

---

質問やトラブルがある場合は、GitHubのIssuesで報告してください。
