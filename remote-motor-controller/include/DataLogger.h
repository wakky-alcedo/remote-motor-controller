/**
 * DataLogger.h
 * 回転数データのロギングクラス
 * 
 * ESP32のRAM上にリングバッファでデータを一時保存し、
 * CSV形式でダウンロード可能にする
 * 
 * @author SDDL Project
 * @date 2026/01/29
 */

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include <Arduino.h>

// ============================================================================
// ロガー設定
// ============================================================================
// 1レコード = 12バイト (timestamp: 4, speed: 4, target: 4)
// 10000レコード = 120KB (ESP32C3のRAM約320KBで余裕あり)
#define LOG_MAX_RECORDS 10000
#define LOG_INTERVAL_MS 10  // 10msごとに記録 (100Hz)

// ============================================================================
// ログレコード構造体
// ============================================================================
struct LogRecord {
  uint32_t timestamp;   // 記録開始からの経過時間 (ms)
  float speed;          // 現在速度 (RPM or step/s)
  float target;         // 目標速度
};

// ============================================================================
// DataLogger クラス
// ============================================================================
class DataLogger {
public:
  /**
   * コンストラクタ
   */
  DataLogger();
  
  /**
   * デストラクタ
   */
  ~DataLogger();
  
  /**
   * 記録を開始
   * バッファをクリアして新規記録開始
   */
  void startRecording();
  
  /**
   * 記録を停止
   */
  void stopRecording();
  
  /**
   * 記録中かどうか
   * @return 記録中ならtrue
   */
  bool isRecording() const;
  
  /**
   * データを記録（定期的に呼び出す）
   * @param currentSpeed 現在速度
   * @param targetSpeed 目標速度
   */
  void record(float currentSpeed, float targetSpeed);
  
  /**
   * CSV形式でデータを出力
   * @param output 出力先String
   */
  void exportCSV(String& output) const;
  
  /**
   * CSV形式でデータをチャンク出力（メモリ節約版）
   * @param startIndex 開始インデックス
   * @param count 取得レコード数
   * @param output 出力先String
   * @return 実際に出力したレコード数
   */
  size_t exportCSVChunk(size_t startIndex, size_t count, String& output) const;
  
  /**
   * 記録済みレコード数を取得
   * @return レコード数
   */
  size_t getRecordCount() const;
  
  /**
   * 記録時間（秒）を取得
   * @return 記録時間
   */
  float getRecordDuration() const;
  
  /**
   * バッファをクリア
   */
  void clear();
  
  /**
   * 最大記録可能数を取得
   * @return 最大レコード数
   */
  size_t getMaxRecords() const { return LOG_MAX_RECORDS; }
  
  /**
   * 記録開始時刻を取得（millis）
   * @return 開始時刻
   */
  unsigned long getStartTime() const { return startTime_; }

private:
  LogRecord* buffer_;           // ログバッファ
  size_t recordCount_;          // 現在のレコード数
  bool recording_;              // 記録中フラグ
  unsigned long startTime_;     // 記録開始時刻 (millis)
  unsigned long lastRecordTime_;// 最後の記録時刻
};

#endif // DATA_LOGGER_H
