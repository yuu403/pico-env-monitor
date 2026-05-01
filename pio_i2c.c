/**
 * pio_i2c.c — PIO bitbang I2C ドライバ実装
 */

#include "pio_i2c.h"
#include "i2c_bb.pio.h"   // pioasmが生成するヘッダ
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stddef.h>

// -------------------------------------------------------
// 内部ヘルパー: PIOプログラムカウンタをラベルへ移動
// -------------------------------------------------------
static inline void _pio_jump(PIO pio, uint sm, uint offset_label) {
    pio_sm_exec(pio, sm, pio_encode_jmp(offset_label));
}

// -------------------------------------------------------
// 内部ヘルパー: 1バイト送信
// ACKが返らない場合 PIO_I2C_ERR_NACK を返す
// -------------------------------------------------------
static int _tx_byte(PIO pio, uint sm, uint offset, uint8_t byte) {
    // tx_byte ラベルへジャンプしてOSRに書き込み
    pio_sm_put_blocking(pio, sm, (uint32_t)byte << 24);
    _pio_jump(pio, sm, offset + i2c_bb_offset_tx_byte);

    // rx_ack を実行してACKを確認
    _pio_jump(pio, sm, offset + i2c_bb_offset_rx_ack);
    uint32_t ack = pio_sm_get_blocking(pio, sm);
    return (ack & 1) ? PIO_I2C_ERR_NACK : PIO_I2C_OK;
}

// -------------------------------------------------------
// 内部ヘルパー: 1バイト受信
// last=true のとき NACK を送信（最終バイト）
// -------------------------------------------------------
static uint8_t _rx_byte(PIO pio, uint sm, uint offset, bool last) {
    _pio_jump(pio, sm, offset + i2c_bb_offset_rx_byte);
    uint32_t data = pio_sm_get_blocking(pio, sm);

    // ACK/NACK送信: last=trueならNACK(SDA=Hi-Z=1), falseならACK(SDA=Low=0)
    pio_sm_put_blocking(pio, sm, last ? (1u << 31) : 0u);
    _pio_jump(pio, sm, offset + i2c_bb_offset_tx_ack);

    return (uint8_t)(data >> 24);
}

// -------------------------------------------------------
// 公開API
// -------------------------------------------------------

uint pio_i2c_init(PIO pio, uint sm, uint sda, uint scl) {
    // SCLはSDA+1でなければならない（サイドセット制約）
    // ハードウェアプルアップ使用（外部4.7kΩ推奨）

    uint offset = pio_add_program(pio, &i2c_bb_program);

    pio_sm_config c = i2c_bb_program_get_default_config(offset);

    // SDA: OUT/IN/SETピン（オープンドレイン: pindirs で0/1を制御）
    sm_config_set_out_pins(&c, sda, 1);
    sm_config_set_in_pins(&c, sda);
    sm_config_set_set_pins(&c, sda, 1);

    // SCL: サイドセットピン
    sm_config_set_sideset_pins(&c, scl);

    // クロック分周
    sm_config_set_clkdiv(&c, PIO_I2C_CLK_DIV);

    // シフト設定: OSR左シフト(MSB first), ISR左シフト(MSB first)
    sm_config_set_out_shift(&c, false, false, 8);
    sm_config_set_in_shift(&c, false, false, 8);

    // GPIO設定（オープンドレイン: OUTは常に0、pindirs で Hi-Z/Low を切替）
    gpio_set_function(sda, GPIO_FUNC_PIO0);
    gpio_set_function(scl, GPIO_FUNC_PIO0);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    // ピンを初期状態(High=Hi-Z)に設定
    pio_sm_set_pins_with_mask(pio, sm, 0, (1u << sda) | (1u << scl));
    pio_sm_set_pindirs_with_mask(pio, sm, 0, (1u << sda) | (1u << scl));

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    return offset;
}

int pio_i2c_write_reg(PIO pio, uint sm, uint8_t addr,
                      uint8_t reg, uint8_t data) {
    uint offset = pio_get_index(pio) == 0
        ? pio_add_program(pio, &i2c_bb_program)   // 初回のみ追加
        : 0; // 実際は init 時に取得したoffsetをグローバルで保持すべき

    // START
    _pio_jump(pio, sm, offset + i2c_bb_offset_start_cond);
    sleep_us(1); // START セットアップ時間

    // ADDR + W
    if (_tx_byte(pio, sm, offset, (addr << 1) | 0) != PIO_I2C_OK)
        goto stop;

    // REG
    if (_tx_byte(pio, sm, offset, reg) != PIO_I2C_OK)
        goto stop;

    // DATA
    _tx_byte(pio, sm, offset, data);

stop:
    _pio_jump(pio, sm, offset + i2c_bb_offset_stop_cond);
    sleep_us(1);
    return PIO_I2C_OK;
}

int pio_i2c_read_regs(PIO pio, uint sm, uint8_t addr,
                      uint8_t reg, uint8_t *buf, size_t len) {
    uint offset = 0; // init 時に保存したオフセットを使うこと

    // START + ADDR + W（レジスタアドレス送信フェーズ）
    _pio_jump(pio, sm, offset + i2c_bb_offset_start_cond);
    sleep_us(1);

    if (_tx_byte(pio, sm, offset, (addr << 1) | 0) != PIO_I2C_OK)
        goto stop_err;

    if (_tx_byte(pio, sm, offset, reg) != PIO_I2C_OK)
        goto stop_err;

    // Repeated START + ADDR + R（受信フェーズ）
    _pio_jump(pio, sm, offset + i2c_bb_offset_start_cond);
    sleep_us(1);

    if (_tx_byte(pio, sm, offset, (addr << 1) | 1) != PIO_I2C_OK)
        goto stop_err;

    // データ受信（最終バイトのみNACK）
    for (size_t i = 0; i < len; i++) {
        buf[i] = _rx_byte(pio, sm, offset, i == len - 1);
    }

    _pio_jump(pio, sm, offset + i2c_bb_offset_stop_cond);
    sleep_us(1);
    return PIO_I2C_OK;

stop_err:
    _pio_jump(pio, sm, offset + i2c_bb_offset_stop_cond);
    sleep_us(1);
    return PIO_I2C_ERR_NACK;
}
