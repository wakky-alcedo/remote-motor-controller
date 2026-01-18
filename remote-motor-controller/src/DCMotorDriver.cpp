/**
 * DCMotorDriver.cpp
 * DCモータードライバ実装（PWM制御 + エンコーダーフィードバック + 速度型PID制御）
 * 
 * @author SDDL Project
 * @date 2026/01/19
 */

#include "DCMotorDriver.h"
#include "float.h"

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
    pidLastError_(0.0f),
    pidLastError2_(0.0f),
    pidLastOutput_(0.0f),
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
  // DCモーターは常にRPM単位でPID制御を使用
  
  targetRPM_ = speed;  // 符号を保持（正：正転、負：逆転）
  pidEnabled_ = true;   // PID有効化
  
  Serial.printf("[DCMotor] setSpeed called: %.2f RPM (PID control)\n", targetRPM_);
  
  if (abs(targetRPM_) < 1.0f) {
    Serial.println("[DCMotor] Stopping motor (RPM < 1)");
    setPWM(0.0f);
    isRunning_ = false;
    currentSpeed_ = 0.0f;
    pidLastError_ = 0.0f;
    pidLastError2_ = 0.0f;
    pidLastOutput_ = 0.0f;
  } else {
    isRunning_ = true;
    // PID制御はupdatePID()で継続的に行う
  }
  
  // StepModeはDCモーターでは使用しないが、互換性のため
  currentStepMode = STEP_FULL;
  
  Serial.printf("[DCMotor] Motor state - Running: %d, Target RPM: %.2f\n",
                isRunning_, targetRPM_);
  
  return true;
}

void DCMotorDriver::softStop() {
  Serial.println("[DCMotor] Soft stop");
  setPWM(0.0f);
  isRunning_ = false;
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
  targetRPM_ = 0.0f;
  pidLastError_ = 0.0f;
  pidLastError2_ = 0.0f;
  pidLastOutput_ = 0.0f;
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
  pidLastError_ = 0.0f;
  pidLastError2_ = 0.0f;
  pidLastOutput_ = 0.0f;
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
  
  // 0ではなく0に近い値の場合は停止とみなす→DBL_EPSILONを使用
  if (speed > DBL_EPSILON) {
    // 正転: PWM_Aに出力、PWM_Bは0
    // Serial.printf("[DCMotor] Forward - Duty: %d/255\n", duty);
    ledcWrite(DC_PWM_CHANNEL_A, duty);
    ledcWrite(DC_PWM_CHANNEL_B, 0);
  } else if (speed < -DBL_EPSILON) {
    // 逆転: PWM_Bに出力、PWM_Aは0
    // Serial.printf("[DCMotor] Reverse - Duty: %d/255\n", duty);
    ledcWrite(DC_PWM_CHANNEL_A, 0);
    ledcWrite(DC_PWM_CHANNEL_B, duty);
  } else {
    // 停止：両方を0に設定
    // Serial.println("[DCMotor] Stop - Duty: 0/255");
    ledcWrite(DC_PWM_CHANNEL_A, 0);
    ledcWrite(DC_PWM_CHANNEL_B, 0);
  }
}

uint8_t DCMotorDriver::speedToDuty(float speed) {
  // speed: 0.0 〜 100.0 (%)
  // duty: 0 〜 DC_PWM_MAX_DUTY
  
  // リニアマッピング
  uint8_t duty = (uint8_t)((speed / 100.0f) * 255.0f);
  
  // 最小デューティ比の設定（モーターが動き始めるための最低値）
  if (duty > 0 && duty < DC_PWM_MIN_DUTY) {
    duty = DC_PWM_MIN_DUTY;
  }
  
  return constrain(duty, 0, DC_PWM_MAX_DUTY);
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
  if (currentTime - lastDebugTime > 500) {
    Serial.printf("[DCMotor] Encoder: Pulses=%lu/%.1fs = %.1fRPM (Target: %.1fRPM)\n", 
                  deltaPulses, deltaTimeSeconds, absRPM, targetRPM_);
    lastDebugTime = currentTime;
  }
}

// ============================================================================
// PID制御（速度型PID）
// ============================================================================

float DCMotorDriver::calculatePID() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastPIDTime_) / 1000.0f;  // 秒単位
  
  if (deltaTime < 0.001f) {
    return pidLastOutput_;  // 時間差が小さすぎる場合は前回の出力を返す
  }
  
  lastPIDTime_ = currentTime;
  
  // 現在の誤差
  float error = targetRPM_ - currentRPM_;
  
  // 速度型PID: Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
  // 出力の増分を計算
  float deltaP = PID_KP * (error - pidLastError_);
  float deltaI = PID_KI * error * deltaTime;
  float deltaD = PID_KD * (error - 2.0f * pidLastError_ + pidLastError2_) / deltaTime;
  
  // 出力の増分
  float deltaOutput = deltaP + deltaI + deltaD;
  
  // 新しい出力 = 前回の出力 + 増分
  float output = pidLastOutput_ + deltaOutput;
  
  // 出力を制限
  output = constrain(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
  
  // 誤差履歴を更新
  pidLastError2_ = pidLastError_;
  pidLastError_ = error;
  pidLastOutput_ = output;
  
  // デバッグ出力（頻度を抑える）
  static unsigned long lastPIDDebugTime = 0;
  if (currentTime - lastPIDDebugTime > 200) {  // 200msごとに出力
    Serial.printf("[DCMotor] PID: T=%.1f C=%.1f E=%.1f ΔP=%.2f ΔI=%.2f ΔD=%.2f Out=%.2f%%\n",
                  targetRPM_, currentRPM_, error, deltaP, deltaI, deltaD, output);
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
