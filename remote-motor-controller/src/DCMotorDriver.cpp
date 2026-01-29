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
volatile unsigned long DCMotorDriver::lastEdgeTime_ = 0;
volatile unsigned long DCMotorDriver::edgeInterval_ = 0;
volatile unsigned long DCMotorDriver::firstEdgeTime_ = 0;
volatile unsigned long DCMotorDriver::edgeCountInPeriod_ = 0;
DCMotorDriver* DCMotorDriver::instance_ = nullptr;
hw_timer_t* DCMotorDriver::timer_ = nullptr;

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

DCMotorDriver::DCMotorDriver() 
  : currentSpeed_(0.0f),
    targetSpeed_(0.0f),
    targetRPM_(0.0f),
    isRunning_(false),
    currentDuty_(0.0f),
    lastEncoderCount_(0),
    controlCycleCount_(0),
    rpmCalcCycleCount_(0),
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
  
  // タイマーを停止
  if (timer_ != nullptr) {
    timerEnd(timer_);
    timer_ = nullptr;
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
  
#if ENCODER_USE_BOTH_EDGES
  // 両エッジ検出（RISING + FALLING）精度2倍だが割り込み頻度も2倍
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, CHANGE);
  Serial.println("[DCMotor]   - Encoder: BOTH EDGES mode (RISING + FALLING)");
#else
  // 片エッジ検出（RISING のみ）標準設定
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, RISING);
  Serial.println("[DCMotor]   - Encoder: RISING edge only");
#endif
  
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
  Serial.printf("[DCMotor]   - Encoder Debounce: %d us\n", ENCODER_DEBOUNCE_US);
  Serial.printf("[DCMotor]   - Control Frequency: %d Hz (1ms cycle)\n", CONTROL_FREQ);
  Serial.printf("[DCMotor]   - RPM Calc Interval: %d ms (M/T method)\n", RPM_CALC_CYCLES);
  Serial.println("[DCMotor]   - RPM Algorithm: M/T Method (high precision at all speeds)");
  Serial.printf("[DCMotor]   - Max Pulse Freq @%dRPM: %.1f Hz (Period: %.2f ms)\n",
                DC_MOTOR_MAX_RPM, (DC_MOTOR_MAX_RPM * ENCODER_PPR / 60.0f),
                1000.0f / (DC_MOTOR_MAX_RPM * ENCODER_PPR / 60.0f));
  Serial.printf("[DCMotor]   - PID Enabled: %s\n", pidEnabled_ ? "Yes" : "No");
  Serial.printf("[DCMotor]   - PID Gains - Kp:%.2f Ki:%.2f Kd:%.2f\n", 
                PID_KP, PID_KI, PID_KD);
  
  currentSpeed_ = 0.0f;
  targetSpeed_ = 0.0f;
  targetRPM_ = 0.0f;
  isRunning_ = false;
  encoderCount_ = 0;
  lastEncoderCount_ = 0;
  lastEdgeTime_ = 0;
  edgeInterval_ = 0;
  firstEdgeTime_ = 0;
  edgeCountInPeriod_ = 0;
  controlCycleCount_ = 0;
  rpmCalcCycleCount_ = 0;
  lastPIDTime_ = millis();
  
  // ハードウェアタイマー設定（タイマー0、プリスケーラ80で1MHz、1000カウントで1kHz）
  timer_ = timerBegin(0, 80, true);  // タイマー0、80分周（1MHz）、カウントアップ
  timerAttachInterrupt(timer_, &timerISR, true);  // 割り込みハンドラを登録
  timerAlarmWrite(timer_, 1000, true);  // 1000μs（1ms）周期、自動リロード
  timerAlarmEnable(timer_);  // タイマー開始
  
  Serial.println("[DCMotor] Hardware timer started - 1kHz control loop");
  
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

float DCMotorDriver::getCurrentDuty() const {
  return currentDuty_;
}

// ============================================================================
// プライベートメソッド
// ============================================================================

void DCMotorDriver::setPWM(float speed) {
  // speed: -100.0 〜 +100.0 (%)
  
  uint8_t duty = speedToDuty(abs(speed));
  
  // 現在のduty比を記録 (0-100%)
  currentDuty_ = (duty / 255.0f) * 100.0f;
  
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
// エンコーダー関連（M/T法対応）
// ============================================================================

void IRAM_ATTR DCMotorDriver::encoderISR() {
  // ソフトウェアデバウンス（チャタリング対策）
  unsigned long interruptTime = micros();
  unsigned long interval = interruptTime - lastEdgeTime_;
  
  // 前回の割り込みから ENCODER_DEBOUNCE_US 以上経過していることを確認
  if (interval > ENCODER_DEBOUNCE_US) {
    // M法: パルスカウント
    encoderCount_++;
    
    // T法: エッジ間隔を記録
    edgeInterval_ = interval;
    lastEdgeTime_ = interruptTime;
    
    // M/T法: 計測期間内のエッジ数をカウント
    edgeCountInPeriod_++;
    
    // 計測期間の最初のエッジ時刻を記録（タイマーISRでリセット）
    if (edgeCountInPeriod_ == 1) {
      firstEdgeTime_ = interruptTime;
    }
  }
}
void IRAM_ATTR DCMotorDriver::timerISR() {
  if (instance_ == nullptr) {
    return;
  }
  
  // 制御周期カウンタをインクリメント
  instance_->controlCycleCount_++;
  instance_->rpmCalcCycleCount_++;
  
  // RPM計算（RPM_CALC_CYCLES毎にM/T法で計算）
  if (instance_->rpmCalcCycleCount_ >= RPM_CALC_CYCLES) {
    float rawRPM = 0.0f;
    
    // ============================================
    // M/T法によるRPM計算
    // ============================================
    // M = 計測期間内のパルス数
    // T = 最初のエッジから最後のエッジまでの実時間
    // RPM = (M - 1) × 60 / (T × PPR)
    // ※ M-1 を使うのは、M個のエッジ間には M-1 個の周期があるため
    
    unsigned long M = edgeCountInPeriod_;
    unsigned long nowTime = micros();
    
    if (M >= 2) {
      // M/T法: 2つ以上のエッジがある場合は高精度計算
      // T = 最後のエッジ時刻 - 最初のエッジ時刻
      unsigned long T_us = lastEdgeTime_ - firstEdgeTime_;
      
      if (T_us > 0) {
        // RPM = (M - 1) × 60,000,000 / (T_us × PPR)
        rawRPM = ((float)(M - 1) * 60000000.0f) / ((float)T_us * ENCODER_PPR);
      }
    } else if (M == 1) {
      // T法フォールバック: 1つのエッジしかない場合
      // 前回のエッジ間隔から推定
      unsigned long interval = edgeInterval_;
      if (interval > 0 && interval < 1000000) {  // 1秒以内
        rawRPM = 60000000.0f / ((float)interval * ENCODER_PPR);
      }
    } else {
      // パルスなし: 停止とみなす or 前回値を維持
      // 一定時間パルスがなければ速度を減衰させる
      unsigned long timeSinceLastEdge = nowTime - lastEdgeTime_;
      if (timeSinceLastEdge > 100000) {  // 100ms以上パルスなし
        rawRPM = 0.0f;
      } else if (edgeInterval_ > 0) {
        // 減衰推定: 前回の周期から推定するが時間経過で減衰
        float estimatedRPM = 60000000.0f / ((float)edgeInterval_ * ENCODER_PPR);
        float decay = 1.0f - (float)timeSinceLastEdge / 100000.0f;
        rawRPM = estimatedRPM * decay;
      }
    }
    
    // 次の計測期間のためにリセット
    edgeCountInPeriod_ = 0;
    firstEdgeTime_ = 0;
    
    // 方向を決定：targetRPM_の符号に合わせる
    if (instance_->isRunning_ && instance_->targetRPM_ != 0.0f) {
      rawRPM = (instance_->targetRPM_ > 0) ? rawRPM : -rawRPM;
    }
    
    // M/T法の生の値を直接使用（移動平均なし）
    instance_->currentRPM_ = rawRPM;
    
    // 次回計算用にリセット
    instance_->rpmCalcCycleCount_ = 0;
  }
  
  // PID制御（1ms毎に実行）
  if (instance_->isRunning_ && instance_->pidEnabled_ && instance_->targetRPM_ != 0.0f) {
    // 現在の誤差
    float error = instance_->targetRPM_ - instance_->currentRPM_;
    
    // 速度型PID: Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k)*Δt + Kd*(e(k)-2*e(k-1)+e(k-2))/Δt
    // Δt = 0.001秒（1ms）
    const float deltaTime = 0.001f;
    
    float deltaP = PID_KP * (error - instance_->pidLastError_);
    float deltaI = PID_KI * error * deltaTime;
    float deltaD = PID_KD * (error - 2.0f * instance_->pidLastError_ + instance_->pidLastError2_) / deltaTime;
    
    // 出力の増分
    float deltaOutput = deltaP + deltaI + deltaD;
    
    // 新しい出力 = 前回の出力 + 増分
    float output = instance_->pidLastOutput_ + deltaOutput;
    
    // 出力を制限
    output = constrain(output, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    
    // 誤差履歴を更新
    instance_->pidLastError2_ = instance_->pidLastError_;
    instance_->pidLastError_ = error;
    instance_->pidLastOutput_ = output;
    
    // PWM設定（割り込み内で直接実行）
    uint8_t duty = instance_->speedToDuty(abs(output));
    
    // 現在のduty比を記録
    instance_->currentDuty_ = (duty / 255.0f) * 100.0f;
    
    if (output > 1e-6) {
      // 正転
      ledcWrite(DC_PWM_CHANNEL_A, duty);
      ledcWrite(DC_PWM_CHANNEL_B, 0);
    } else if (output < -1e-6) {
      // 逆転
      ledcWrite(DC_PWM_CHANNEL_A, 0);
      ledcWrite(DC_PWM_CHANNEL_B, duty);
    } else {
      // 停止
      ledcWrite(DC_PWM_CHANNEL_A, 0);
      ledcWrite(DC_PWM_CHANNEL_B, 0);
    }
    
    instance_->currentSpeed_ = output;
  }
}

void DCMotorDriver::calculateRPM() {
  // タイマー割り込みで処理されるため、この関数は空実装
  // デバッグ出力のみ実行
  static unsigned long lastDebugTime = 0;
  static unsigned long lastPulseCount = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastDebugTime > 500) {
    unsigned long deltaPulses = encoderCount_ - lastPulseCount;
    float pulsesPerSecond = deltaPulses * 2.0f;  // 500ms間隔なので2倍
    float expectedPulses = abs(currentRPM_) * ENCODER_PPR / 60.0f;
    float ratio = (expectedPulses > 0) ? (pulsesPerSecond / expectedPulses) : 0;
    
    Serial.printf("[DCMotor] RPM: C=%.1f T=%.1f | Pulses=%lu PPS=%.1f Expect=%.1f Ratio=%.2fx\n", 
                  currentRPM_, targetRPM_, encoderCount_, pulsesPerSecond, expectedPulses, ratio);
    
    if (ratio > 1.5f && currentRPM_ > 10) {
      Serial.println("[DCMotor] WARNING: Pulse count is too high! Possible chattering.");
      Serial.printf("[DCMotor]   -> Increase ENCODER_DEBOUNCE_US (current: %d us)\n", ENCODER_DEBOUNCE_US);
    }
    
    lastDebugTime = currentTime;
    lastPulseCount = encoderCount_;
  }
}

// ============================================================================
// PID制御（速度型PID）
// ============================================================================

float DCMotorDriver::calculatePID() {
  // タイマー割り込みで処理されるため、デバッグ出力のみ
  static unsigned long lastPIDDebugTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastPIDDebugTime > 200) {
    float error = targetRPM_ - currentRPM_;
    Serial.printf("[DCMotor] PID: T=%.1f C=%.1f E=%.1f Out=%.2f%%\n",
                  targetRPM_, currentRPM_, error, pidLastOutput_);
    lastPIDDebugTime = currentTime;
  }
  
  return pidLastOutput_;
}

void DCMotorDriver::updatePID() {
  // タイマー割り込みで処理されるため、デバッグ出力のみ実行
  calculateRPM();
  calculatePID();
}
