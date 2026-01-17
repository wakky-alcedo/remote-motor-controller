# モータードライバー抽象化設計

## 概要

このプロジェクトは、**DCモーター**と**ステッピングモーター**間で容易に切り替え可能な設計を採用しています。`config.h`でモータータイプを選択するだけで、コンパイル時に適切なドライバーが自動的に選択されます。

## アーキテクチャ

```
┌─────────────────────────────────────────┐
│        MotorController (Facade)         │  ← 既存コードとの互換性を維持
│  - init()                               │
│  - setSpeed()                           │
│  - softStop() / hardStop()              │
│  - getCurrentSpeed()                    │
└──────────────┬──────────────────────────┘
               │ IMotorDriver* (ポリモーフィズム)
               ├──────────────┬──────────────┐
         ┌─────▼─────┐  ┌────▼──────┐
         │  DCMotor  │  │  Stepper  │
         │  Driver   │  │  Motor    │
         │  (PWM)    │  │  Driver   │
         │           │  │  (L6470)  │
         └───────────┘  └───────────┘
```

### 主要コンポーネント

| ファイル | 説明 |
|---------|------|
| `IMotorDriver.h` | モータードライバー抽象インターフェース |
| `DCMotorDriver.h/.cpp` | DCモーター実装（PWM制御） |
| `StepperMotorDriver.h/.cpp` | ステッピングモーター実装（L6470） |
| `MotorController.h/.cpp` | 統合制御クラス（ファサード） |
| `config.h` | モータータイプ選択設定 |

## モータータイプの切り替え方法

### 1. DCモーターを使用する場合

`include/config.h`を以下のように編集：

```cpp
// ============================================================================
// モータータイプ選択
// ============================================================================
#define MOTOR_TYPE_DC       // DCモーターを使用
// #define MOTOR_TYPE_STEPPER  // ステッピングモーターを使用（コメントアウト）
```

### 2. ステッピングモーターを使用する場合

`include/config.h`を以下のように編集：

```cpp
// ============================================================================
// モータータイプ選択
// ============================================================================
// #define MOTOR_TYPE_DC       // DCモーターを使用（コメントアウト）
#define MOTOR_TYPE_STEPPER  // ステッピングモーターを使用
```

### 3. ビルド＆アップロード

設定変更後、通常通りビルド・アップロードを実行：

```bash
pio run --target upload
```

## ピン配置

### DCモーター（Hブリッジドライバー: L298N, TB6612等）

| ESP32C3 Pin | 機能 | 接続先 |
|-------------|------|--------|
| D8 | PWM_A | モータードライバー IN1/AIN1 |
| D9 | PWM_B | モータードライバー IN2/AIN2 |
| D10 | ENABLE | モータードライバー EN（常時HIGH可） |

**制御方式:**
- 正転: PWM_A = デューティ比, PWM_B = 0
- 逆転: PWM_A = 0, PWM_B = デューティ比
- 停止: PWM_A = 0, PWM_B = 0
- ブレーキ: PWM_A = 255, PWM_B = 255

### ステッピングモーター（L6470ドライバー）

| ESP32C3 Pin | 機能 | 接続先 |
|-------------|------|--------|
| D8 | SPI SCK | L6470 CK |
| D9 | SPI MISO | L6470 SDO |
| D10 | SPI MOSI | L6470 SDI |
| D7 | CS | L6470 CS |
| D6 | BUSY | L6470 BUSY |
| D5 | RESET | L6470 STBY/RESET |

## 速度設定の違い

### DCモーター
- **単位:** パーセント (%)
- **範囲:** -100.0 〜 +100.0
- **例:** `setSpeed(50.0)` → 50%出力で正転

### ステッピングモーター
- **単位:** step/s
- **範囲:** -3000.0 〜 +3000.0 (config.hのMOTOR_MAX_SPEEDに依存)
- **例:** `setSpeed(1500.0)` → 1500 step/sで正転

> **互換性:** DCモーターモードでは、大きな値（例: 3000）を渡すと自動的に100%にスケーリングされます。

## 既存コードの変更不要

`MotorController`クラスの公開APIは変更されていないため、既存の呼び出しコードは**そのまま動作**します：

```cpp
MotorController motor;
StepMode stepMode = STEP_1;

// 初期化
motor.init();

// 速度設定
motor.setSpeed(50.0, stepMode);  // DCモード: 50%, ステッパー: 50 step/s

// 停止
motor.softStop();

// 緊急停止
motor.hardStop();

// 状態取得
bool running = motor.isRunning();
float currentSpeed = motor.getCurrentSpeed();
```

## 設計の利点

### 1. **簡単な切り替え**
`config.h`の1行を変更するだけでモータータイプを切り替え可能

### 2. **既存コードの互換性**
`MotorController`の公開APIは不変。既存のシステムコントローラー、UDP、Webインターフェースは変更不要

### 3. **拡張性**
新しいモータータイプ（例: サーボモーター、BLDCモーター）も`IMotorDriver`を実装すれば追加可能

### 4. **型安全性**
コンパイル時に適切なドライバーが選択され、ランタイムエラーを防止

### 5. **テスト容易性**
各ドライバーを独立してテスト可能

## カスタマイズ

### DCモーターの調整

`DCMotorDriver.cpp`の`speedToDuty()`関数で最小デューティ比を調整可能：

```cpp
uint8_t DCMotorDriver::speedToDuty(float speed) {
  uint8_t duty = (uint8_t)((speed / 100.0f) * 255.0f);
  
  // 最小デューティ比の設定（モーターの特性に合わせて調整）
  if (duty > 0 && duty < 30) {
    duty = 30;  // 例: 12% 最小出力
  }
  
  return constrain(duty, 0, 255);
}
```

### ステッピングモーターの調整

`config.h`でL6470のパラメータを調整：

```cpp
#define KVAL_PARAM 0xC0              // 駆動電圧 (75%)
#define MOTOR_MAX_SPEED 3000         // 最大速度 (step/s)
#define MOTOR_ACCELERATION 50        // 加速度 (step/s²)
#define OCD_THRESHOLD 0x0F           // 過電流検出 (6.0A)
```

## トラブルシューティング

### DCモーターが動かない
1. ピン接続を確認
2. `speedToDuty()`の最小デューティ比を調整
3. モータードライバーの電源供給を確認

### ステッピングモーターが脱調する
1. `KVAL_PARAM`を増加（例: 0xC0 → 0xFF）
2. `MOTOR_ACCELERATION`を減少（例: 50 → 20）
3. 速度を下げる

### コンパイルエラー
- `config.h`で`MOTOR_TYPE_DC`または`MOTOR_TYPE_STEPPER`のいずれか**1つだけ**を定義
- L6470ライブラリがインストールされているか確認（ステッピングモーターモード時）

## ライセンス

SDDL Project © 2026
