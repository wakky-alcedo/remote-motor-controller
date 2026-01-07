# EtherSpin-ESP

**PC上の計算モデル（Python/MATLAB）やWebブラウザから、Wi-Fi経由でステッピングモータを直接ドライブするための高精度制御システム**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange.svg)](https://platformio.org/)
[![ESP32](https://img.shields.io/badge/ESP32-Compatible-blue.svg)](https://www.espressif.com/en/products/socs/esp32)

## 🎯 プロジェクトの目的

従来のマイコン制御では、加速曲線を変更するたびにファームウェアの書き換えやビルドが必要でした。**EtherSpin-ESP**は、マイコンを単なるパルス生成エンジン（透過的なブリッジ）として扱い、制御ロジックのすべてをPC側に持たせることで、**開発効率を飛躍的に向上**させます。

### 何ができるようになるのか

- 🧮 **数式をそのまま物理動作へ**: Pythonで計算した二次関数、サイン波、S字カーブなどの速度プロファイルを、そのままリアルタイムにモータへ反映
- 🔄 **ノンストップ調整**: モータを回しながら、PC側のコードを一行書き換えるだけで、次の瞬間の挙動を変化
- 💻 **分散処理**: 複雑な軌道計算はパワフルなPCで行い、ESP32は通信とパルス出力のみに専念

## ✨ 主な特徴

- **ワイヤレス・ストリーミング制御**: PC側で計算した速度プロファイルをUDP経由でストリーミング
- **L6470 自律パルス生成**: Wi-Fi通信負荷によるパルスの乱れを物理的に排除
- **ハイブリッド・インターフェース**: UDP（リアルタイム制御）+ Webブラウザ（設定・モニタリング）
- **動的マイクロステップ最適化**: 速度域に応じて自動的に最適なステップモードに遷移
- **フェイルセーフ・ウォッチドッグ**: 通信途絶時の自動停止機能

## 📸 スクリーンショット

### Webダッシュボード
モダンで直感的なUIでモータの状態をリアルタイム監視・制御

### 速度プロファイル例
PythonやMATLABから複雑な速度プロファイルを実行

## 🚀 クイックスタート

### 必要なもの

- Seeed XIAO ESP32C3（または互換ESP32ボード）
- L6470 ステッピングモータドライバ
- ステッピングモータ
- PlatformIO IDE

### セットアップ（3ステップ）

```bash
# 1. リポジトリをクローン
git clone https://github.com/yourusername/remote-motor-control.git
cd remote-motor-control/remote-motor-control

# 2. Wi-Fi設定を編集（src/main.cppまたはinclude/config.h）
# WIFI_SSIDとWIFI_PASSWORDを設定

# 3. ビルド＆アップロード
pio run --target upload
```

### 使用例

#### Pythonから制御

```python
import socket
import json
import time
import math

# UDP接続
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
esp32_ip = "motor.local"

# サイン波で振動
for t in range(200):
    speed = 1500 * math.sin(2 * math.pi * 0.2 * t / 10)
    message = json.dumps({"v": speed})
    sock.sendto(message.encode(), (esp32_ip, 8888))
    time.sleep(0.02)
```

#### Webブラウザから制御

1. ブラウザで `http://motor.local` にアクセス
2. スライダーで速度を調整
3. リアルタイムで状態を監視

## 📚 ドキュメント

プロジェクト内に詳細なドキュメントを用意しています：

- 📋 [**SPECIFICATION.md**](SPECIFICATION.md) - 技術仕様書
- 🔧 [**remote-motor-control/SETUP.md**](remote-motor-control/SETUP.md) - 詳細なセットアップガイド
- 🏗️ [**remote-motor-control/ARCHITECTURE.md**](remote-motor-control/ARCHITECTURE.md) - システムアーキテクチャ
- 📖 [**remote-motor-control/README.md**](remote-motor-control/README.md) - 使用方法とAPI

## 🔌 ハードウェア構成

```
[XIAO ESP32C3]          [L6470]          [Motor]
GPIO 8  (SCK)   -----> SCK
GPIO 9  (MISO)  <----- MISO
GPIO 10 (MOSI)  -----> MOSI
GPIO 7  (CS)    -----> CS              
GPIO 6  (BUSY)  <----- BUSY
GPIO 5  (RESET) -----> RESET
                       OUT1A/B  -----> Coil A
                       OUT2A/B  -----> Coil B
```

## 🌐 通信プロトコル

### UDP速度指令（Port 8888）

```json
{"v": 1500.0}
```

- `v`: 速度 (step/s)
  - 正の値: 正回転
  - 負の値: 逆回転
  - 0: 停止

### WebSocket API

リアルタイムな双方向通信でシステム状態を取得・制御

## 📊 サンプルコード

### Python

- `udp_client.py` - 5つの実用例を含む完全なサンプル
  1. 定速回転
  2. 台形加速プロファイル
  3. サイン波速度プロファイル
  4. S字カーブ加速
  5. インタラクティブ制御

### MATLAB

- `udp_client.m` - 6つの実用例
  1. 定速回転
  2. 台形加速プロファイル
  3. サイン波速度プロファイル
  4. S字カーブ加速
  5. 二次関数速度プロファイル
  6. 往復運動

## 🛠️ 技術スタック

- **マイコン**: ESP32 (Seeed XIAO ESP32C3)
- **フレームワーク**: Arduino / PlatformIO
- **ドライバIC**: L6470 (SPI通信)
- **通信**: WiFi (UDP + HTTP + WebSocket)
- **プロトコル**: mDNS, JSON
- **ライブラリ**:
  - ArduinoJson
  - ESPAsyncWebServer
  - AutoDriver (L6470)

## 🔍 トラブルシューティング

よくある問題と解決方法は [SETUP.md](remote-motor-control/SETUP.md#-トラブルシューティング) を参照してください。

### WiFiに接続できない
- 2.4GHz帯のWi-Fiを使用していますか？（5GHzは非対応）
- SSIDとパスワードが正しいですか？

### モータが動かない
- 電源は供給されていますか？
- 配線を再確認してください
- シリアルモニタでエラーを確認してください

## 🤝 コントリビューション

バグ報告、機能リクエスト、プルリクエストを歓迎します！

1. このリポジトリをフォーク
2. 機能ブランチを作成 (`git checkout -b feature/amazing-feature`)
3. 変更をコミット (`git commit -m 'Add amazing feature'`)
4. ブランチにプッシュ (`git push origin feature/amazing-feature`)
5. プルリクエストを作成

## 📄 ライセンス

このプロジェクトは [MIT License](remote-motor-control/LICENSE) の下で公開されています。

## 👥 開発者

**SDDL Project**

## 🙏 謝辞

- ESP32コミュニティ
- PlatformIOチーム
- L6470ライブラリ開発者

## 📝 更新履歴

### v1.0.0 (2026/01/07)

初回リリース

- ✅ L6470制御機能実装
- ✅ UDP通信機能実装
- ✅ Webダッシュボード実装
- ✅ ウォッチドッグ機能実装
- ✅ 動的マイクロステップ最適化実装
- ✅ Python/MATLABサンプルコード
- ✅ 包括的なドキュメント

---

⭐ このプロジェクトが役に立ったら、スターをつけてください！
