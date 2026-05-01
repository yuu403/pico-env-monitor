/**
 * bme280.c — BME280 ドライバ実装
 *
 * 補正計算はBoschデータシート(BST-BME280-DS002)Appendix Bの
 * Cコードリファレンスを参照し、int32_t/int64_t の型を厳密に再現。
 * 型ミスによる測定値の破綻（例: 気圧が10倍ズレ）を防ぐため
 * すべてのキャストを明示的に記述する。
 */

#include "bme280.h"
#include "pio_i2c.h"
#include "pico/stdlib.h"
#include <string.h>

// -------------------------------------------------------
// 内部: 補正係数の読み取り
// Appendix B, Section 4.2.2 に従いレジスタを解釈
// -------------------------------------------------------
static bool _read_calib(bme280_dev_t *dev) {
    uint8_t buf[26];
    bme280_calib_t *c = &dev->calib;

    // 0x88-0xA1: T1-T3, P1-P9, H1
    if (pio_i2c_read_regs(dev->pio, dev->sm, dev->i2c_addr,
                          BME280_REG_CALIB_00, buf, 26) != PIO_I2C_OK)
        return false;

    // 温度補正係数（Little Endian）
    c->dig_T1 = (uint16_t)(buf[1]  << 8 | buf[0]);
    c->dig_T2 = (int16_t) (buf[3]  << 8 | buf[2]);
    c->dig_T3 = (int16_t) (buf[5]  << 8 | buf[4]);

    // 気圧補正係数（Little Endian）
    c->dig_P1 = (uint16_t)(buf[7]  << 8 | buf[6]);
    c->dig_P2 = (int16_t) (buf[9]  << 8 | buf[8]);
    c->dig_P3 = (int16_t) (buf[11] << 8 | buf[10]);
    c->dig_P4 = (int16_t) (buf[13] << 8 | buf[12]);
    c->dig_P5 = (int16_t) (buf[15] << 8 | buf[14]);
    c->dig_P6 = (int16_t) (buf[17] << 8 | buf[16]);
    c->dig_P7 = (int16_t) (buf[19] << 8 | buf[18]);
    c->dig_P8 = (int16_t) (buf[21] << 8 | buf[20]);
    c->dig_P9 = (int16_t) (buf[23] << 8 | buf[22]);
    c->dig_H1 = buf[25];  // 0xA1

    // 0xE1-0xE7: H2-H6（特殊なビット配置に注意）
    if (pio_i2c_read_regs(dev->pio, dev->sm, dev->i2c_addr,
                          BME280_REG_CALIB_26, buf, 7) != PIO_I2C_OK)
        return false;

    c->dig_H2 = (int16_t)(buf[1] << 8 | buf[0]);
    c->dig_H3 = buf[2];

    // dig_H4: 0xE4[7:4] + 0xE5[3:0]  ← 特殊（Appendix B Table 1）
    c->dig_H4 = (int16_t)((buf[3] << 4) | (buf[4] & 0x0F));

    // dig_H5: 0xE5[7:4] + 0xE6[7:0]  ← 特殊
    c->dig_H5 = (int16_t)((buf[5] << 4) | (buf[4] >> 4));

    c->dig_H6 = (int8_t)buf[6];

    return true;
}

// -------------------------------------------------------
// 内部: 温度補正
// 戻り値: t_fine（気圧・湿度補正で再利用する内部変数）
// 出力: *temp_c に温度[°C]を書く
//
// Appendix B, bmp280_compensate_temperature_double() 準拠
// -------------------------------------------------------
static int32_t _compensate_temp(const bme280_calib_t *c,
                                int32_t adc_T,
                                float *temp_c) {
    int32_t var1, var2, t_fine;

    // Bosch リファレンスの式をそのまま再現（型は厳密に int32_t）
    var1 = ((((adc_T >> 3) - ((int32_t)c->dig_T1 << 1)))
            * ((int32_t)c->dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)c->dig_T1))
            * ((adc_T >> 4) - ((int32_t)c->dig_T1))) >> 12)
            * ((int32_t)c->dig_T3)) >> 14;

    t_fine = var1 + var2;
    *temp_c = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

    return t_fine;
}

// -------------------------------------------------------
// 内部: 気圧補正
// 注意: var1, var2 は int64_t（int32_tでオーバーフローする）
//
// Appendix B, bmp280_compensate_pressure_int64() 準拠
// -------------------------------------------------------
static float _compensate_press(const bme280_calib_t *c,
                               int32_t adc_P,
                               int32_t t_fine) {
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)c->dig_P6;
    var2 = var2 + ((var1 * (int64_t)c->dig_P5) << 17);
    var2 = var2 + (((int64_t)c->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)c->dig_P3) >> 8)
         + ((var1 * (int64_t)c->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)c->dig_P1) >> 33;

    if (var1 == 0) return 0.0f;  // ゼロ除算防止

    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)c->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)c->dig_P8) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)c->dig_P7) << 4);

    return (float)p / 25600.0f;  // Pa → hPa 変換（÷256÷100）
}

// -------------------------------------------------------
// 内部: 湿度補正
// Appendix B, bme280_compensate_humidity_double() 準拠
// -------------------------------------------------------
static float _compensate_hum(const bme280_calib_t *c,
                             int32_t adc_H,
                             int32_t t_fine) {
    int32_t v;

    v = t_fine - 76800;
    v = (((((adc_H << 14) - (((int32_t)c->dig_H4) << 20)
           - (((int32_t)c->dig_H5) * v)) + 16384) >> 15)
         * (((((((v * ((int32_t)c->dig_H6)) >> 10)
                * (((v * ((int32_t)c->dig_H3)) >> 11) + 32768)) >> 10)
               + 2097152) * ((int32_t)c->dig_H2) + 8192) >> 14));

    v = v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)c->dig_H1)) >> 4);
    v = (v < 0) ? 0 : v;
    v = (v > 419430400) ? 419430400 : v;

    return (float)(v >> 12) / 1024.0f;
}

// -------------------------------------------------------
// 公開API
// -------------------------------------------------------

bool bme280_init(bme280_dev_t *dev, PIO pio, uint sm,
                 uint pio_offset, uint8_t addr) {
    dev->pio        = pio;
    dev->sm         = sm;
    dev->pio_offset = pio_offset;
    dev->i2c_addr   = addr;

    // チップID確認（0x60 固定）
    uint8_t chip_id;
    if (pio_i2c_read_regs(pio, sm, addr, BME280_REG_CHIP_ID,
                          &chip_id, 1) != PIO_I2C_OK)
        return false;
    if (chip_id != 0x60) return false;

    // ソフトリセット（電源投入直後の状態をクリア）
    pio_i2c_write_reg(pio, sm, addr, BME280_REG_RESET, 0xB6);
    sleep_ms(10);  // リセット完了待ち（データシート: 2ms以上）

    // 補正係数読み取り
    if (!_read_calib(dev)) return false;

    // 湿度OSR設定（ctrl_humはctrl_measより先に書く必要がある）
    pio_i2c_write_reg(pio, sm, addr,
                      BME280_REG_CTRL_HUM, BME280_OSRS_H_X1);

    // standby時間とIIRフィルタ設定
    pio_i2c_write_reg(pio, sm, addr,
                      BME280_REG_CONFIG, BME280_TSB_100MS | BME280_FILTER_OFF);

    // 温度・気圧OSR + Normalモード開始
    pio_i2c_write_reg(pio, sm, addr, BME280_REG_CTRL_MEAS,
                      BME280_OSRS_T_X2 | BME280_OSRS_P_X4 | BME280_MODE_NORMAL);

    return true;
}

bool bme280_read(bme280_dev_t *dev, bme280_data_t *out) {
    // status レジスタで計測完了を確認
    uint8_t status;
    pio_i2c_read_regs(dev->pio, dev->sm, dev->i2c_addr,
                      BME280_REG_STATUS, &status, 1);
    if (status & 0x08) return false;  // measuring中はスキップ

    // 0xF7-0xFE: press(3), temp(3), hum(2) を一括読み取り
    uint8_t raw[8];
    if (pio_i2c_read_regs(dev->pio, dev->sm, dev->i2c_addr,
                          BME280_REG_PRESS_MSB, raw, 8) != PIO_I2C_OK)
        return false;

    // ADCデータ抽出（20bit、MSB first、下位4bitはxlsb）
    int32_t adc_P = (int32_t)(((uint32_t)raw[0] << 12)
                             | ((uint32_t)raw[1] << 4)
                             | ((uint32_t)raw[2] >> 4));

    int32_t adc_T = (int32_t)(((uint32_t)raw[3] << 12)
                             | ((uint32_t)raw[4] << 4)
                             | ((uint32_t)raw[5] >> 4));

    // 湿度: 16bit（MSB=raw[6], LSB=raw[7]）
    int32_t adc_H = (int32_t)(((uint32_t)raw[6] << 8) | raw[7]);

    // 補正計算（順序重要: tempが t_fine を生成し、press/humが使う）
    int32_t t_fine;
    t_fine = _compensate_temp(&dev->calib, adc_T, &out->temperature);
    out->pressure = _compensate_press(&dev->calib, adc_P, t_fine);
    out->humidity = _compensate_hum(&dev->calib, adc_H, t_fine);

    return true;
}
