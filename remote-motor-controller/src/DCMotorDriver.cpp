/**
 * DCMotorDriver.cpp
 * DCモータードライバ実装（PWM制御 + エンコーダーフィードバック + PID制御）
 * 
 * @author SDDL Project
 * @date 2026/01/19
 */

#include "DCMotorDriver.h"

// ============================================================================
// 静的メンバー変数の初期化
// ============================================================================
volatile unsigned long DCMotorDriver::encoderCount_ = 0;
DCMotorDriver* DCMotorDriver::instance_ = nullptr;

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

DCMotorDriver::DCMotorDriver() 
  : currentSpeed_(0.0f),
    targetSpeed_(0.0f),
    targetRPM_(0.0f),
    isRunning_(false),
    lastEncoderCount_(0),
    lastRPMUpdateTime_(0),
    currentRPM_(0.0f),
    pidIntegral_(0.0f),
    pidLastError_(0.0f),
    lastPIDTime_(0),
    pidEnabled_(true) {
  instance_ = this;
}

DCMotorDriver::~DCMotorDriver() {
  if (isRunning_) {
    softStop();
  }
  // エンコーダー割り込みを解除
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));
  instance_ = nullptr;
}

// ============================================================================
// 公開メソッド
// ============================================================================

bool DCMotorDriver::init() {
  Serial.println("[DCMotor] Initializing DC Motor Driver...");
  Serial.printf("[DCMotor] Pin Config - PWM_A:%d PWM_B:%d ENABLE:%d ENCODER:%d\n",
                DC_PIN_PWM_A, DC_PIN_PWM_B, DC_PIN_ENABLE, ENCODER_PIN);
  
  // PWMピン設定
  pinMode(DC_PIN_PWM_A, OUTPUT);
  pinMode(DC_PIN_PWM_B, OUTPUT);
  pinMode(DC_PIN_ENABLE, OUTPUT);
  
  // エンコーダーピン設定
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  
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
  Serial.printf("[DCMotor]   - Encoder PPR: %d\n", ENCODER_PPR);
  Serial.printf("[DCMotor]   - PID Enabled: %s\n", pidEnabled_ ? "Yes" : "No");
  Serial.printf("[DCMotor]   - PID Gains - Kp:%.2f Ki:%.2f Kd:%.2f\n", 
                PID_KP, PID_KI, PID_KD);
  
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
  targetRPM_ = 0.0f;
  isRunning_ = false;
  encoderCount_ = 0;
  lastEncoderCount_ = 0;
  lastRPMUpdateTime_ = millis();
  lastPIDTime_ = millis();
  
  return true;
}

bool DCMotorDriver::setSpeed(float speed, StepMode& currentStepMode) {
  // speedはRPM単位を想定（PID制御時）または%単位（互換モード）
  
  // 値が大きい場合はRPM指定とみなす
  bool isRPMMode = abs(speed) > 100.0f;
  
  if (isRPMMode) {
    // RPM指定モード
    targetRPM_ = speed;  // 符号を保持（正：正転、負：逆転）
    pidEnabled_ = true;   // PID有効化
    Serial.printf("[DCMotor] setSpeed called: %.2f RPM (PID control)\n", targetRPM_);
    
    if (abs(targetRPM_) < 1.0f) {
      Serial.println("[DCMotor] Stopping motor (RPM < 1)");
      setPWM(0.0f);
      isRunning_ = false;
      currentSpeed_ = 0.0f;
      pidIntegral_ = 0.0f;  // 積分項リセット
    } else {
      isRunning_ = true;
      // PID制御はupdatePID()で継続的に行う
    }
  } else {
    // %指定モード（互換性のため）
    float normalizedSpeed = constrain(speed, -100.0f, 100.0f);
    
    Serial.printf("[DCMotor] setSpeed called: %.2f%% (Direct PWM control)\n", normalizedSpeed);
    
    targetSpeed_ = normalizedSpeed;
    targetRPM_ = 0.0f;  // RPMモード無効
    pidEnabled_ = false;  // PID無効
    
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
  }
  
  // StepModeはDCモーターでは使用しないが、互換性のため
  currentStepMode = STEP_FULL;
  
  Serial.printf("[DCMotor] Motor state - Running: %d, Current: %.2f%%, Target RPM: %.2f\n",
                isRunning_, currentSpeed_, targetRPM_);
  
  return true;
}

void DCMotorDriver::softStop() {
  Serial.println("[DCMotor] Soft stop");
  setPWM(0.0f);
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
  targetRPM_ = 0.0f;
  pidIntegral_ = 0.0f;  // PID積分項リセット
  pidLastError_ = 0.0f;
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
  targetRPM_ = 0.0f;
  pidIntegral_ = 0.0f;
  pidLastError_ = 0.0f;
}

bool DCMotorDriver::isBusy() {
  return false;
}

float DCMotorDriver::getCurrentSpeed() {
  // エンコーダーがある場合は常にRPMを返す（手動回転も検出）
  // PID制御の有無に関わらず、実際の回転速度を表示
  return currentRPM_;
}

bool DCMotorDriver::isRunning() const {
  return isRunning_;
}

StepMode DCMotorDriver::getStepMode() const {
  return STEP_FULL;
}

float DCMotorDriver::getCurrentRPM() const {
  return currentRPM_;
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

// ============================================================================
// エンコーダー関連
// ============================================================================

void IRAM_ATTR DCMotorDriver::encoderISR() {
  encoderCount_++;
}

void DCMotorDriver::calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - lastRPMUpdateTime_;
  
  // RPM_UPDATE_INTERVAL経過していない場合はスキップ
  if (deltaTime < RPM_UPDATE_INTERVAL) {
    return;
  }
  
  // パルス数の差分を計算
  unsigned long currentCount = encoderCount_;
  unsigned long deltaPulses = currentCount - lastEncoderCount_;
  
  // RPM計算
  // RPM = (パルス数 / PPR) / (時間[秒]) * 60
  float deltaTimeSeconds = deltaTime / 1000.0f;
  float absRPM = (deltaPulses / (float)ENCODER_PPR) / deltaTimeSeconds * 60.0f;
  
  // 方向を決定：targetRPM_の符号に合わせる
  // モーター停止中は符号を保持しない
  if (isRunning_ && targetRPM_ != 0.0f) {
    currentRPM_ = (targetRPM_ > 0) ? absRPM : -absRPM;
  } else {
    // 停止中または手動回転の場合は絶対値
    currentRPM_ = absRPM;
  }
  
  // 次回計算用に値を保存
  lastEncoderCount_ = currentCount;
  lastRPMUpdateTime_ = currentTime;
  
  // デバッグ出力（頻度を抑える）
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime > 1000) {
    Serial.printf("[DCMotor] RPM: %.2f (Target: %.2f, Pulses: %lu)\n", 
                  currentRPM_, targetRPM_, deltaPulses);
    lastDebugTime = currentTime;
  }
}

// ============================================================================
// PID制御
// ============================================================================

float DCMotorDriver::calculatePID() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastPIDTime_) / 1000.0f;  // 秒単位
  
  if (deltaTime < 0.001f) {
    return currentSpeed_;  // 時間差が小さすぎる場合はスキップ
  }
  
  lastPIDTime_ = currentTime;
  
  // 誤差計算
  float error = targetRPM_ - currentRPM_;
  
  // 比例項 (P)
  float pTerm = PID_KP * error;
  
  // 積分項 (I)
  pidIntegral_ += error * deltaTime;
  pidIntegral_ = constrain(pidIntegral_, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  float iTerm = PID_KI * pidIntegral_;
  
  // 微分項 (D)
  float dTerm = PID_KD * (error - pidLastError_) / deltaTime;
  pidLastError_ = error;
  
  // PID出力計算
  float output = pTerm + iTerm + dTerm;
  output = constrain(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  
  // デバッグ出力（頻度を抑える）
  static unsigned long lastPIDDebugTime = 0;
  if (currentTime - lastPIDDebugTime > 500) {
    Serial.printf("[DCMotor] PID: Error=%.2f P=%.2f I=%.2f D=%.2f Output=%.2f%%\n",
                  error, pTerm, iTerm, dTerm, output);
    lastPIDDebugTime = currentTime;
  }
  
  return output;
}

void DCMotorDriver::updatePID() {
  // RPM計算は常に実行（手動回転も検出するため）
  calculateRPM();
  
  // PID制御はモーター動作中のみ
  if (!isRunning_ || !pidEnabled_ || targetRPM_ == 0.0f) {
    return;
  }
  
  // PID制御でPWM出力を計算
  float pidOutput = calculatePID();
  
  // PWM設定
  setPWM(pidOutput);
  currentSpeed_ = pidOutput;
}
