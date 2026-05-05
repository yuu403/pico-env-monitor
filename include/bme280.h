#pragma once
/**
 * bme280.h — BME280 環境センサードライバ
 *
 * ライブラリ不使用。BST-BME280-DS002 を直接参照して実装。
 *
 * 実装範囲:
 *   - レジスタマップ: Section 5.3
 *   - 補正計算:       Appendix B (trimming parameter)
 *   - 動作モード:     Section 3.3 (Normal mode, 100ms standby)
 *   - OSR設定:        温度×2, 湿度×1, 気圧×4
 *
 * サンプリング設計根拠:
 *   OSR×4(気圧)の最大変換時間 ≈ 9.3ms → 100ms周期は十分なマージン
 *   参照: Table 13 (Measurement time)
 */

#include "hardware/pio.h"
#include <stdint.h>
#include <stdbool.h>

// -------------------------------------------------------
// I2Cアドレス（SDO=Low: 0x76, SDO=High: 0x77）
// -------------------------------------------------------
#define BME280_I2C_ADDR_LOW  0x76
#define BME280_I2C_ADDR_HIGH 0x77

// -------------------------------------------------------
// レジスタマップ (BST-BME280-DS002 Table 18)
// -------------------------------------------------------
#define BME280_REG_CALIB_00   0x88  // 補正データ先頭 (T/P: 0x88-0x9F)
#define BME280_REG_CHIP_ID    0xD0  // 固定値 0x60
#define BME280_REG_RESET      0xE0  // 0xB6 書き込みでソフトリセット
#define BME280_REG_CALIB_26   0xE1  // 補正データ続き (H: 0xE1-0xE7)
#define BME280_REG_CTRL_HUM   0xF2  // 湿度OSR設定
#define BME280_REG_STATUS     0xF3  // bit3=measuring, bit0=im_update
#define BME280_REG_CTRL_MEAS  0xF4  // 温度/気圧OSR + mode
#define BME280_REG_CONFIG     0xF5  // standby time, IIRフィルタ
#define BME280_REG_PRESS_MSB  0xF7  // 気圧データ先頭（0xF7-0xF9）
#define BME280_REG_TEMP_MSB   0xFA  // 温度データ先頭（0xFA-0xFC）
#define BME280_REG_HUM_MSB    0xFD  // 湿度データ先頭（0xFD-0xFE）

// -------------------------------------------------------
// ctrl_meas レジスタ設定値
// bit7-5: osrs_t (温度OSR), bit4-2: osrs_p (気圧OSR), bit1-0: mode
// -------------------------------------------------------
#define BME280_OSRS_T_X2    (0x2 << 5)  // 温度 ×2 過サンプリング
#define BME280_OSRS_P_X4    (0x3 << 2)  // 気圧 ×4 過サンプリング
#define BME280_MODE_NORMAL  (0x3)       // Normalモード（連続計測）

// ctrl_hum レジスタ
#define BME280_OSRS_H_X1    (0x1)       // 湿度 ×1 過サンプリング

// config レジスタ
// bit7-5: t_sb (standby時間), bit4-2: filter, bit0: spi3w_en
#define BME280_TSB_100MS    (0x2 << 5)  // 100ms スタンバイ
#define BME280_FILTER_OFF   (0x0 << 2)  // IIRフィルタなし

// -------------------------------------------------------
// 補正係数（trimming parameter）構造体
// BST-BME280-DS002 Appendix B, Table 1
// -------------------------------------------------------
typedef struct {
    // 温度補正 (T1-T3)
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    // 気圧補正 (P1-P9)
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    // 湿度補正 (H1-H6)
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;  // 注意: 特殊なビット配置（Appendix B参照）
    int16_t  dig_H5;  // 注意: 特殊なビット配置
    int8_t   dig_H6;
} bme280_calib_t;

// -------------------------------------------------------
// 計測結果
// -------------------------------------------------------
typedef struct {
    float temperature;  // [°C]
    float pressure;     // [hPa]
    float humidity;     // [%RH]
} bme280_data_t;

// -------------------------------------------------------
// デバイス状態
// -------------------------------------------------------
typedef struct {
    PIO          pio;
    uint         sm;
    uint         pio_offset;
    uint8_t      i2c_addr;
    bme280_calib_t calib;
} bme280_dev_t;

/**
 * BME280 初期化
 * @return true=成功, false=チップID不一致 or NACKエラー
 */
bool bme280_init(bme280_dev_t *dev, PIO pio, uint sm,
                 uint pio_offset, uint8_t addr);

/**
 * BME280 計測データ取得
 * Normalモードで動作中のデータをレジスタから読み取り、補正計算を行う
 */
bool bme280_read(bme280_dev_t *dev, bme280_data_t *out);
