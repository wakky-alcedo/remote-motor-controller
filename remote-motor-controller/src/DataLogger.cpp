/**
 * DataLogger.cpp
 * 回転数データのロギングクラス実装
 * 
 * @author SDDL Project
 * @date 2026/01/29
 */

#include "DataLogger.h"

// ============================================================================
// コンストラクタ / デストラクタ
// ============================================================================

DataLogger::DataLogger()
  : buffer_(nullptr),
    recordCount_(0),
    recording_(false),
    startTime_(0),
    lastRecordTime_(0) {
  // バッファを動的確保
  buffer_ = new LogRecord[LOG_MAX_RECORDS];
  if (buffer_ == nullptr) {
    Serial.println("[DataLogger] ERROR: Failed to allocate buffer!");
  } else {
    Serial.printf("[DataLogger] Buffer allocated: %d records (%d bytes)\n", 
                  LOG_MAX_RECORDS, LOG_MAX_RECORDS * sizeof(LogRecord));
  }
}

DataLogger::~DataLogger() {
  if (buffer_ != nullptr) {
    delete[] buffer_;
    buffer_ = nullptr;
  }
}

// ============================================================================
// 公開メソッド
// ============================================================================

void DataLogger::startRecording() {
  if (buffer_ == nullptr) {
    Serial.println("[DataLogger] ERROR: Buffer not allocated!");
    return;
  }
  
  // バッファをクリア
  clear();
  
  // 記録開始
  recording_ = true;
  startTime_ = millis();
  lastRecordTime_ = startTime_;
  
  Serial.println("[DataLogger] Recording started");
}

void DataLogger::stopRecording() {
  if (recording_) {
    recording_ = false;
    Serial.printf("[DataLogger] Recording stopped. Total records: %d, Duration: %.2f sec\n",
                  recordCount_, getRecordDuration());
  }
}

bool DataLogger::isRecording() const {
  return recording_;
}

void DataLogger::record(float currentSpeed, float targetSpeed) {
  if (!recording_ || buffer_ == nullptr) {
    return;
  }
  
  // バッファフル時は記録停止
  if (recordCount_ >= LOG_MAX_RECORDS) {
    Serial.println("[DataLogger] Buffer full, stopping recording");
    stopRecording();
    return;
  }
  
  // 記録間隔チェック
  unsigned long now = millis();
  if (now - lastRecordTime_ < LOG_INTERVAL_MS) {
    return;
  }
  
  // データを記録
  buffer_[recordCount_].timestamp = now - startTime_;
  buffer_[recordCount_].speed = currentSpeed;
  buffer_[recordCount_].target = targetSpeed;
  
  recordCount_++;
  lastRecordTime_ = now;
}

void DataLogger::exportCSV(String& output) const {
  output = "";
  
  // ヘッダー
  output += "timestamp_ms,speed,target\n";
  
  // データ
  for (size_t i = 0; i < recordCount_; i++) {
    output += String(buffer_[i].timestamp) + ",";
    output += String(buffer_[i].speed, 2) + ",";
    output += String(buffer_[i].target, 2) + "\n";
  }
}

size_t DataLogger::exportCSVChunk(size_t startIndex, size_t count, String& output) const {
  output = "";
  
  if (startIndex >= recordCount_) {
    return 0;
  }
  
  // ヘッダー（最初のチャンクのみ）
  if (startIndex == 0) {
    output += "timestamp_ms,speed,target\n";
  }
  
  // 実際に出力するレコード数
  size_t endIndex = min(startIndex + count, recordCount_);
  size_t actualCount = endIndex - startIndex;
  
  // データ
  for (size_t i = startIndex; i < endIndex; i++) {
    output += String(buffer_[i].timestamp) + ",";
    output += String(buffer_[i].speed, 2) + ",";
    output += String(buffer_[i].target, 2) + "\n";
  }
  
  return actualCount;
}

size_t DataLogger::getRecordCount() const {
  return recordCount_;
}

float DataLogger::getRecordDuration() const {
  if (recordCount_ == 0) {
    return 0.0f;
  }
  return buffer_[recordCount_ - 1].timestamp / 1000.0f;
}

void DataLogger::clear() {
  recordCount_ = 0;
  startTime_ = 0;
  lastRecordTime_ = 0;
}
