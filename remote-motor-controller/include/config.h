/**
 * WiFi設定ファイル
 * 
 * このファイルをコピーして使用環境に合わせて編集してください
 */

#ifndef CONFIG_H
#define CONFIG_H

// Wi-Fi設定（ここを環境に合わせて変更してください）
#define WIFI_SSID     "Kawasemi-G"      // 例: "your_wifi_ssid"
#define WIFI_PASSWORD "most9849"        // 例: "your_wifi_password"

// mDNS設定
#define MDNS_HOSTNAME "motor"  // motor.local でアクセス可能

// UDP設定
#define UDP_PORT 8888

// ウォッチドッグタイムアウト（ミリ秒）
#define WATCHDOG_TIMEOUT_MS 1000

// L6470 電気的パラメータ
#define KVAL_PARAM 0x29        // KVAL_HOLD/RUN/ACC/DEC
#define OCD_THRESHOLD 0x0F     // 過電流検出 (3.375A)

// L6470 動作パラメータ
#define MOTOR_MAX_SPEED 3000      // 最大速度 (step/s)
#define MOTOR_MIN_SPEED 1         // 最小速度 (step/s)
#define MOTOR_ACCELERATION 100    // 加速度 (step/s/s)
#define MOTOR_THRESHOLD_SPEED 1000 // フルステップ切替速度 (step/s) - この速度を超えるとフルステップモードに移行

// 動的マイクロステップ最適化の閾値
#define SPEED_THRESHOLD_LOW  500    // 128→32分割への遷移点
#define SPEED_THRESHOLD_HIGH 2000   // 32→8分割への遷移点

#endif // CONFIG_H
