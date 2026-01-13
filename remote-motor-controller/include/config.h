/**
 * WiFi設定ファイル
 * 
 * このファイルをコピーして使用環境に合わせて編集してください
 */

#ifndef CONFIG_H
#define CONFIG_H

// Wi-Fi設定
#define WIFI_SSID     "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// mDNS設定
#define MDNS_HOSTNAME "motor"  // motor.local でアクセス可能

// UDP設定
#define UDP_PORT 8888

// ウォッチドッグタイムアウト（ミリ秒）
#define WATCHDOG_TIMEOUT_MS 1000

// L6470 電気的パラメータ
#define KVAL_PARAM 0x29        // KVAL_HOLD/RUN/ACC/DEC
#define OCD_THRESHOLD 0x0F     // 過電流検出 (3.375A)

// 動的マイクロステップ最適化の閾値
#define SPEED_THRESHOLD_LOW  500    // 128→32分割への遷移点
#define SPEED_THRESHOLD_HIGH 2000   // 32→8分割への遷移点

#endif // CONFIG_H
