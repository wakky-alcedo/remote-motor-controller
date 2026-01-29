/**
 * WiFi設定ファイル
 * 
 * このファイルをコピーして使用環境に合わせて編集してください
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// モータータイプ選択
// ============================================================================
// 使用するモータータイプを選択してください
// MOTOR_TYPE_STEPPER: L6470ステッピングモーター
// MOTOR_TYPE_DC:      DCモーター（PWM制御）
#define MOTOR_TYPE_DC       // DCモーターを使用
// #define MOTOR_TYPE_STEPPER  // ステッピングモーターを使用（コメントアウトを切り替え）

// Wi-Fi設定（ここを環境に合わせて変更してください）
// #define WIFI_SSID     "Kawasemi-G"      // 例: "your_wifi_ssid"
// #define WIFI_PASSWORD "most9849"        // 例: "your_wifi_password"
// #define WIFI_SSID     "SDDLnet"
// #define WIFI_PASSWORD "smallbear"
#define WIFI_SSID     "Hippopotamus"
#define WIFI_PASSWORD "origami2827"

// mDNS設定
#define MDNS_HOSTNAME "motor"  // motor.local でアクセス可能

// UDP設定
#define UDP_PORT 8888

// ウォッチドッグタイムアウト（ミリ秒）
#define WATCHDOG_TIMEOUT_MS 1000

// ============================================================================
// L6470 電気的パラメータ
// ============================================================================

// KVAL (Konstant Voltage) - モーターへの供給電圧制御
// 範囲: 0x00～0xFF (0～255)
// 計算式: 実効電圧% = (KVAL / 255) × 100
// 
// KVAL_HOLD: 停止時の保持トルク電圧
// KVAL_RUN:  定速運転時の電圧
// KVAL_ACC:  加速時の電圧
// KVAL_DEC:  減速時の電圧
//
// 設定値の目安:
//   0x29 (41)  = 16%  - 低電圧、低トルク、省電力（脱調しやすい）
//   0x80 (128) = 50%  - 標準的な設定
//   0xC0 (192) = 75%  - 高トルク、高速回転に対応（推奨）
//   0xFF (255) = 100% - 最大電圧（発熱・過電流に注意）
//
// 注意: 低すぎると高速時に脱調、高すぎると発熱や過電流のリスク
#define KVAL_PARAM 0xC0        // 0xC0 = 192 = 75%電圧（高速・高トルク用）

// 過電流検出閾値
// 範囲: 0x00～0x0F (375mA～6000mA、15段階)
// 計算式: 電流(A) = (THRESHOLD + 1) × 0.375A
// 0x0F = 16 × 0.375A = 6.0A
#define OCD_THRESHOLD 0x0F     // 過電流検出 (6.0A)

// L6470 動作パラメータ
#define MOTOR_MAX_SPEED 3000      // 最大速度 (step/s)
#define MOTOR_MIN_SPEED 1         // 最小速度 (step/s)
#define MOTOR_ACCELERATION 50     // 加速度 (step/s/s) - ゆっくり加速して脱調を防止
#define MOTOR_THRESHOLD_SPEED 800 // フルステップ切替速度 (step/s) - この速度を超えるとフルステップモードに移行

// 動的マイクロステップ最適化の閾値
#define SPEED_THRESHOLD_LOW  300    // 128→32分割への遷移点（低速域）
#define SPEED_THRESHOLD_HIGH 800    // 32→8分割への遷移点（高速域、脱調防止のため早めに切替）

// ============================================================================
// DCモーター - エンコーダー設定
// ============================================================================
#define ENCODER_PPR (36*2)        // エンコーダーのパルス/回転数（Pulse Per Revolution）
                                  // ※両エッジ検出を有効にする場合は (36*2) = 72 に設定
#define ENCODER_USE_BOTH_EDGES 1  // 0: RISING のみ, 1: RISING + FALLING（精度2倍、高負荷）
#define ENCODER_DEBOUNCE_US 500   // デバウンス時間（マイクロ秒）チャタリング対策
                                  // フォトインタラプタの場合: 100〜500μs 推奨
                                  // スロット数の倍が取れる場合は値を増やす
#define CONTROL_FREQ 1000       // 制御周波数 (Hz) - タイマー割り込み周期
#define RPM_CALC_CYCLES 10      // RPM生値計算周期（制御周期の倍数、10cycles = 10ms）
                                // 110RPM時: 10msで約1.3パルス（測定精度向上）
                                // 移動平均フィルタ（20サンプル）で平滑化
                                // 実質的な応答時間: 10ms × 20 = 200ms
#define DC_MOTOR_MAX_RPM 300    // DCモーター最大RPM（Web UI スライダー範囲）

// ============================================================================
// DCモーター - PWM設定
// ============================================================================
#define DC_PWM_MIN_DUTY 30      // 最小デューティ比 (0-255) - モーター始動に必要な最低値
#define DC_PWM_MAX_DUTY 250     // 最大デューティ比 (0-255) - 安全のための上限値

// ============================================================================
// DCモーター - PID制御パラメータ（速度型PID）
// ============================================================================
// 速度型PID: Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k)*Δt + Kd*(e(k)-2*e(k-1)+e(k-2))/Δt
// 出力 u(k) = u(k-1) + Δu(k)
//
// 速度型PIDの利点:
//   - 積分飽和（ワインドアップ）が起こりにくい
//   - 出力制限時の挙動が安定
//   - モーター制御に適している
//
// 調整の目安:
//   Kp: 誤差の変化に対する応答性。大きいほど速く反応するが振動しやすい
//   Ki: 定常偏差を減らす。大きいほど正確に収束するが、オーバーシュートしやすい
//   Kd: 振動を抑える。急激な変化にブレーキをかける
//
#define PID_KP 0.01f             // 比例ゲイン（Proportional）
#define PID_KI 0.15f             // 積分ゲイン（Integral）
#define PID_KD 0.00f            // 微分ゲイン（Derivative）
#define PID_OUTPUT_LIMIT 100.0f // PID出力上限（PWM %）

#endif // CONFIG_H
