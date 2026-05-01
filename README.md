# Pico W 環境モニター

Raspberry Pi Pico W + BME280 による温度・湿度・気圧のWiFiモニタリングシステム。

## 技術的特徴

- **PIOステートマシン**でI2Cを自前実装（`hardware_i2c` 不使用）
- **BME280補正計算**をデータシート(BST-BME280-DS002)から直接実装（ライブラリ不使用）
- **デュアルコア**：core0=センサー計測、core1=WiFi/HTTP
- **タイマー割り込み**で100ms周期の決定論的サンプリング
- **KiCad**回路図付き

---

## なぜこう設計したか

### PIOでI2Cを実装した理由

Pico SDKの `hardware_i2c` は内部でDMAとIRQを使い、実際の送信完了タイミングがソフトウェアから見えにくい。PIOステートマシンで直接SCL/SDAを制御することで、ビットレベルのタイミングが決定論的になり、タイマー割り込みとの干渉を排除できる。

**クロックストレッチング非対応の根拠**：BME280は測定完了をステータスレジスタ（0xF3 bit3）でポーリング通知し、クロックストレッチを行わない（BST-BME280-DS002 Section 4）。汎用I2Cドライバとしての使用は意図していない。

### デュアルコア分割の根拠

`pico_cyw43_arch_lwip_threadsafe_background` を使用し、lwIPのコールバックをcore0コンテキストで受けつつ、WiFi送信処理の実体をcore1に置く。`queue_t`（スレッドセーフFIFO）を介してデータを受け渡すことで、センサーのリアルタイム性（100ms周期）とWiFiの可変レイテンシを完全に分離した。

### サンプリング周期100msの根拠

| 設定 | 値 |
|------|----|
| 気圧OSR | ×4 |
| 最大変換時間 | 約9.3ms（BST-BME280-DS002 Table 13） |
| サンプリング周期 | 100ms（≒10倍マージン） |
| タイマー精度 | ±1μs（125MHz PIOクロック） |

環境計測として100msは十分な時間分解能であり、センサーに不要な負荷をかけない。

---

## 詰まった問題と解決策

### 問題1: dig_H4/H5 の補正係数が正しく取れない

**症状**：湿度が0%か100%に張り付く。

**原因**：BME280の湿度補正係数 dig_H4 と dig_H5 は、通常のLittle Endianではなく2バイトをビットフィールドで共有する特殊な配置（BST-BME280-DS002 Appendix B Table 1）。

```
dig_H4 = (E4[7:4] << 4) | E5[3:0]   ← E4とE5をまたぐ
dig_H5 = (E5[7:4] << 4) | E6[7:0]   ← E5とE6をまたぐ
```

**解決**：ライブラリのソースを確認してビット操作を手で再実装。`(buf[3] << 4) | (buf[4] & 0x0F)` が正しい dig_H4 の取り出し方。

---

### 問題2: 気圧補正で int32_t がオーバーフロー

**症状**：気圧が -300hPa などの異常値になる。

**原因**：Appendix Bの気圧補正式の中間計算値が int32_t の最大値（約2.1×10⁹）を超える。Boschのリファレンスコードは `int64_t` を明示しているが、見落としやすい。

**解決**：`var1`, `var2`, `p` をすべて `int64_t` で宣言。コードに `// int64_t 必須（int32_tでオーバーフロー）` コメントを追加して再発防止。

---

### 問題3: lwIPをcore1から直接呼ぶとハング

**症状**：WiFi接続後、数分で Pico W がハング。デバッガで止めると lwIP の内部ミューテックス待ちで止まっていた。

**原因**：`pico_cyw43_arch_lwip_poll` モードを使っていたが、core1から `tcp_write` を呼ぶと core0 の lwIP タイマーと競合する。

**解決**：`pico_cyw43_arch_lwip_threadsafe_background` に切り替え、`cyw43_arch_poll()` をcore1のメインループで明示的に呼ぶ。

---

## ビルド方法

```bash
git clone <this-repo>
cd pico-env-monitor

mkdir build && cd build
cmake .. \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DWIFI_SSID="your-ssid" \
  -DWIFI_PASSWORD="your-password"
make -j4

# build/pico_env_monitor.uf2 を Pico W にコピー
```

## 回路図

`docs/schematic.kicad_sch` を参照。

主な設計ポイント：
- SDA/SCL に 4.7kΩ プルアップ（3.3Vへ）
- BME280 VDD/VDDIO に 100nF デカップリングコンデンサ
- Pico W の 3V3(OUT) ピンから給電（BME280は最大3.6V）

## API動作確認

Pico W のIPアドレスに対してHTTPリクエストを送ると、JSONでセンサーデータが返る。

```bash
curl http://<pico-ip>/
# {"temperature":24.53,"pressure":1013.25,"humidity":58.12}
```
