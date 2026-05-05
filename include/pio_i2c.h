#pragma once
/**
 * pio_i2c.h — PIO bitbang I2C ドライバ
 *
 * 制約:
 *   - クロックストレッチング非対応（BME280専用設計）
 *   - マルチマスター非対応
 *   - アドレス: 7bit のみ
 *
 * 参照: BST-BME280-DS002 Section 4 (BME280 I2C仕様)
 */

#include "hardware/pio.h"
#include <stdint.h>
#include <stdbool.h>

// I2Cクロック周波数設定
// 125MHz / PIO_I2C_CLK_DIV / 8サイクル ≈ 244kHz
#define PIO_I2C_CLK_DIV  64

// エラーコード
#define PIO_I2C_OK        0
#define PIO_I2C_ERR_NACK -1

/**
 * PIO I2C初期化
 * @param pio   PIOインスタンス (pio0 or pio1)
 * @param sm    ステートマシン番号 (0–3)
 * @param sda   SDAピン番号
 * @param scl   SCLピン番号 (sda+1 推奨)
 * @return      ステートマシンのオフセット
 */
uint pio_i2c_init(PIO pio, uint sm, uint sda, uint scl);

/**
 * I2C書き込み（レジスタへの書き込み）
 * @param pio       PIOインスタンス
 * @param sm        ステートマシン番号
 * @param addr      7bitスレーブアドレス
 * @param reg       レジスタアドレス
 * @param data      書き込みデータ
 * @return          PIO_I2C_OK / PIO_I2C_ERR_NACK
 */
int pio_i2c_write_reg(PIO pio, uint sm, uint8_t addr,
                      uint8_t reg, uint8_t data);

/**
 * I2C読み出し（レジスタからの連続読み取り）
 * @param pio       PIOインスタンス
 * @param sm        ステートマシン番号
 * @param addr      7bitスレーブアドレス
 * @param reg       開始レジスタアドレス
 * @param buf       受信バッファ
 * @param len       読み取りバイト数
 * @return          PIO_I2C_OK / PIO_I2C_ERR_NACK
 */
int pio_i2c_read_regs(PIO pio, uint sm, uint8_t addr,
                      uint8_t reg, uint8_t *buf, size_t len);
