/**
 * DCMotorDriver.cpp
 * DCモータードライバ実装（PWM制御）
 * 
 * @author SDDL Project
 * @date 2026/01/18
 */

#include "DCMotorDriver.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

DCMotorDriver::DCMotorDriver() 
  : currentSpeed_(0.0f),
    targetSpeed_(0.0f),
    isRunning_(false) {
}

DCMotorDriver::~DCMotorDriver() {
  if (isRunning_) {
    softStop();
  }
}

// ============================================================================
// 公開メソッド
// ============================================================================

bool DCMotorDriver::init() {
  Serial.println("[DCMotor] Initializing DC Motor Driver...");
  Serial.printf("[DCMotor] Pin Config - PWM_A:%d PWM_B:%d ENABLE:%d\n",
                DC_PIN_PWM_A, DC_PIN_PWM_B, DC_PIN_ENABLE);
  
  // PWMピン設定
  pinMode(DC_PIN_PWM_A, OUTPUT);
  pinMode(DC_PIN_PWM_B, OUTPUT);
  pinMode(DC_PIN_ENABLE, OUTPUT);
  
  // PWMチャンネル設定（ESP32のLEDC機能を使用）
  ledcSetup(DC_PWM_CHANNEL_A, DC_PWM_FREQ, DC_PWM_RESOLUTION);
  ledcSetup(DC_PWM_CHANNEL_B, DC_PWM_FREQ, DC_PWM_RESOLUTION);
  
  // PWMチャンネルをピンに割り当て
  ledcAttachPin(DC_PIN_PWM_A, DC_PWM_CHANNEL_A);
  ledcAttachPin(DC_PIN_PWM_B, DC_PWM_CHANNEL_B);
  
  // イネーブル信号をHIGHに（モータードライバー有効化）
  digitalWrite(DC_PIN_ENABLE, HIGH);
  
  // 初期状態：停止
  ledcWrite(DC_PWM_CHANNEL_A, 0);
  ledcWrite(DC_PWM_CHANNEL_B, 0);
  
  Serial.println("[DCMotor] DC Motor Driver initialized successfully");
  Serial.printf("[DCMotor]   - PWM Frequency: %d Hz\n", DC_PWM_FREQ);
  Serial.printf("[DCMotor]   - PWM Resolution: %d bit (0-255)\n", DC_PWM_RESOLUTION);
  
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
  isRunning_ = false;
  
  return true;
}

bool DCMotorDriver::setSpeed(float speed, StepMode& currentStepMode) {
  // speedは-100.0〜+100.0の範囲（%）を想定
  // ステッピングモーターとの互換性のため、大きな値も受け入れて正規化
  
  // 速度を-100〜+100の範囲にクランプ
  float normalizedSpeed = speed;
  if (abs(speed) > 100.0f) {
    // ステッピングモーター用の大きな値の場合、スケーリング
    // 例: 3000 step/s → 100%として扱う
    normalizedSpeed = (speed / MOTOR_MAX_SPEED) * 100.0f;
    normalizedSpeed = constrain(normalizedSpeed, -100.0f, 100.0f);
  }
  
  Serial.printf("[DCMotor] setSpeed called: %.2f%% (original: %.2f)\n", 
                normalizedSpeed, speed);
  
  targetSpeed_ = normalizedSpeed;
  
  // StepModeはDCモーターでは使用しないが、互換性のため
    currentStepMode = STEP_FULL;  // ダミー値
  
  // 速度設定
  if (abs(normalizedSpeed) < 1.0f) {
    Serial.println("[DCMotor] Stopping motor (speed < 1%)");
    setPWM(0.0f);
    isRunning_ = false;
    currentSpeed_ = 0.0f;
  } else {
    Serial.printf("[DCMotor] Setting speed to %.2f%%\n", normalizedSpeed);
    setPWM(normalizedSpeed);
    isRunning_ = true;
    currentSpeed_ = normalizedSpeed;
  }
  
  Serial.printf("[DCMotor] Motor state - Running: %d, Current: %.2f%%, Target: %.2f%%\n",
                isRunning_, currentSpeed_, targetSpeed_);
  
  return true;
}

void DCMotorDriver::softStop() {
  Serial.println("[DCMotor] Soft stop");
  // DCモーターの場合、ソフトストップは段階的に減速
  // シンプルに即座に停止（必要に応じて減速処理を追加可能）
  setPWM(0.0f);
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
}

void DCMotorDriver::hardStop() {
  Serial.println("[DCMotor] HARD STOP (Emergency)");
  // ブレーキ動作（両方のPWMを同時にON）
  ledcWrite(DC_PWM_CHANNEL_A, 255);
  ledcWrite(DC_PWM_CHANNEL_B, 255);
  delay(50);  // 短時間ブレーキ
  
  // 完全停止
  ledcWrite(DC_PWM_CHANNEL_A, 0);
  ledcWrite(DC_PWM_CHANNEL_B, 0);
  
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
}

bool DCMotorDriver::isBusy() {
  // DCモーターは常に応答可能
  return false;
}

float DCMotorDriver::getCurrentSpeed() {
  // DCモーターは速度フィードバックなし（エンコーダがない場合）
  // 設定速度を返す
  return currentSpeed_;
}

bool DCMotorDriver::isRunning() const {
  return isRunning_;
}

StepMode DCMotorDriver::getStepMode() const {
  // DCモーターではステップモード不要
    return STEP_FULL;  // ダミー値
}

// ============================================================================
// プライベートメソッド
// ============================================================================

void DCMotorDriver::setPWM(float speed) {
  // speed: -100.0 〜 +100.0 (%)
  
  uint8_t duty = speedToDuty(abs(speed));
  
  if (speed > 0) {
    // 正転: PWM_Aに出力、PWM_Bは0
    Serial.printf("[DCMotor] Forward - Duty: %d/255\n", duty);
    ledcWrite(DC_PWM_CHANNEL_A, duty);
    ledcWrite(DC_PWM_CHANNEL_B, 0);
  } else if (speed < 0) {
    // 逆転: PWM_Bに出力、PWM_Aは0
    Serial.printf("[DCMotor] Reverse - Duty: %d/255\n", duty);
    ledcWrite(DC_PWM_CHANNEL_A, 0);
    ledcWrite(DC_PWM_CHANNEL_B, duty);
  } else {
    // 停止
    Serial.println("[DCMotor] Stop - Duty: 0/255");
    ledcWrite(DC_PWM_CHANNEL_A, 0);
    ledcWrite(DC_PWM_CHANNEL_B, 0);
  }
}

uint8_t DCMotorDriver::speedToDuty(float speed) {
  // speed: 0.0 〜 100.0 (%)
  // duty: 0 〜 255
  
  // リニアマッピング
  uint8_t duty = (uint8_t)((speed / 100.0f) * 255.0f);
  
  // 最小デューティ比の設定（モーターが動き始めるための最低値）
  // 必要に応じて調整
  if (duty > 0 && duty < 30) {
    duty = 30;  // 最小デューティ比を30/255 (約12%)に設定
  }
  
  return constrain(duty, 0, 255);
}
