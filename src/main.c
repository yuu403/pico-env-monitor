/**
 * main.c — Pico W 環境モニター メインエントリ
 *
 * core0: タイマー割り込みでBME280を100ms周期でサンプリング
 * core1: queue_tからデータを受け取りWiFi/HTTP送信（core1_entry.c）
 *
 * デュアルコア分割の設計根拠:
 *   lwIPのコールバックはcore0コンテキストで動作する前提で設計されている。
 *   センサー割り込みもcore0で受ける場合、WiFiドライバとの排他制御が必要になる。
 *   core1にWiFi送信を分離し、queue_tで非同期受け渡しすることで
 *   センサーのリアルタイム性とWiFiレイテンシを完全に分離する。
 *
 * タイマー設計根拠:
 *   BME280 OSR×4 の最大変換時間 ≈ 9.3ms（BST-BME280-DS002 Table 13）
 *   100ms周期はその約10倍のマージン。
 *   repeating_timer はcore0のFIFOタイマーを使用し、
 *   ジッタは最大1μs（125MHz時）。
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "pico/util/queue.h"

#include "pio_i2c.h"
#include "bme280.h"

#include "pico/cyw43_arch.h"
// -------------------------------------------------------
// ピン定義
// -------------------------------------------------------
#define PIN_SDA     0   // I2C SDA (PIO制御)
#define PIN_SCL     1   // I2C SCL (PIO制御, サイドセット)
#define BME280_ADDR BME280_I2C_ADDR_LOW  // SDO=GND → 0x76

// -------------------------------------------------------
// サンプリング周期
// 設計根拠: BME280最大変換時間9.3ms の約10倍マージン
// -------------------------------------------------------
#define SAMPLE_INTERVAL_MS  100

// -------------------------------------------------------
// コア間通信キュー
// queue_t はPico SDKが提供するスレッドセーフFIFO
// 容量: 8エントリ（WiFi遅延時のバッファ）
// -------------------------------------------------------
#define QUEUE_DEPTH  8
queue_t sensor_queue;

// -------------------------------------------------------
// グローバルデバイス状態（core0専用）
// -------------------------------------------------------
static bme280_dev_t bme280;

// -------------------------------------------------------
// core1エントリ（WiFi/HTTP処理）
// 実装は core1_entry.c に分離
// -------------------------------------------------------
void core1_entry(void);

// -------------------------------------------------------
// タイマー割り込みコールバック
// 100ms周期でBME280を読み取りキューへ積む
// -------------------------------------------------------
static bool on_timer(struct repeating_timer *t) {
    bme280_data_t data;

    if (!bme280_read(&bme280, &data)) {
        // 計測中（measuring=1）はスキップ。次の周期で再試行。
        return true;
    }

    // キューが満杯の場合は最古データを捨てて新データを優先
    if (queue_is_full(&sensor_queue)) {
        bme280_data_t discard;
        queue_try_remove(&sensor_queue, &discard);
    }

    queue_try_add(&sensor_queue, &data);
    return true;  // true を返すとタイマー継続
}

// -------------------------------------------------------
// メイン（core0）
// -------------------------------------------------------
int main(void) {
    stdio_init_all();
    sleep_ms(2000);  // USB-CDC接続待ち（デバッグ用）

    if (cyw43_arch_init()) {
        return -1;
    }

    // コア間キュー初期化
    queue_init(&sensor_queue, sizeof(bme280_data_t), QUEUE_DEPTH);

    // PIO I2C初期化
    uint pio_offset = pio_i2c_init(pio0, 0, PIN_SDA, PIN_SCL);

    // BME280初期化
    if (!bme280_init(&bme280, pio0, 0, pio_offset, BME280_ADDR)) {
        // 初期化失敗: LEDを点滅させてエラーを表示
        // 本番実装ではシリアルへのエラー出力を追加すること
        while (true) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1); sleep_ms(100);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); sleep_ms(100);
        }
    }

    // core1起動（WiFi/HTTP処理）
    // core1_entry は sensor_queue からデータを読んで送信する
    multicore_launch_core1(core1_entry);

    // タイマー割り込み設定（100ms周期、ネガティブ値=最後の実行完了から計測）
    struct repeating_timer timer;
    add_repeating_timer_ms(-SAMPLE_INTERVAL_MS, on_timer, NULL, &timer);

    // core0はタイマー割り込みに任せて省電力待機
    while (true) {
        __wfi();  // Wait For Interrupt: 割り込みまでCPUをスリープ
    }

    return 0;  // 到達しない
}
