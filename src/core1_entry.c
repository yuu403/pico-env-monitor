/**
 * core1_entry.c — core1 WiFi/HTTP処理
 *
 * lwIP設計上の注意:
 *   pico_cyw43_arch_lwip_threadsafe_background を使用することで
 *   core1からのlwIP API呼び出しを安全に行う。
 *   これを使わずにcore1からlwIP APIを直接呼ぶと内部状態が破壊される。
 *
 * HTTP送信方式:
 *   シンプルなTCPソケット + JSON payload。
 *   MQTTやWebSocketは実装複雑度が増すため今回のスコープ外。
 *   Webダッシュボードは Pico W 自身がHTTPサーバーとして動作し、
 *   ブラウザからポーリング（5秒周期）でデータを取得する構成。
 */

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/util/queue.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "bme280.h"
#include <stdio.h>
#include <string.h>

// WiFi認証情報（実際の運用では環境変数や設定ファイルで管理）
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_PASSWORD"
#endif

// HTTPサーバーポート
#define HTTP_PORT 80

// 最新センサーデータ（HTTPレスポンス用）
static bme280_data_t latest_data = {0};
static bool data_available = false;

// core0が初期化したキューへの参照（外部宣言）
extern queue_t sensor_queue;

// -------------------------------------------------------
// HTTP レスポンス生成
// -------------------------------------------------------
static void send_http_response(struct tcp_pcb *pcb, const bme280_data_t *d) {
    char json[128];
    int json_len = snprintf(json, sizeof(json),
        "{\"temperature\":%.2f,\"pressure\":%.2f,\"humidity\":%.2f}",
        d->temperature, d->pressure, d->humidity);

    char header[256];
    int header_len = snprintf(header, sizeof(header),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n\r\n",
        json_len);

    tcp_write(pcb, header, header_len, TCP_WRITE_FLAG_COPY);
    tcp_write(pcb, json, json_len, TCP_WRITE_FLAG_COPY);
    tcp_output(pcb);
}

// -------------------------------------------------------
// TCP受信コールバック
// -------------------------------------------------------
static err_t on_recv(void *arg, struct tcp_pcb *pcb,
                     struct pbuf *p, err_t err) {
    if (!p) {
        tcp_close(pcb);
        return ERR_OK;
    }
    pbuf_free(p);

    if (data_available) {
        send_http_response(pcb, &latest_data);
    } else {
        const char *err_resp =
            "HTTP/1.1 503 Service Unavailable\r\n"
            "Content-Length: 0\r\n\r\n";
        tcp_write(pcb, err_resp, strlen(err_resp), TCP_WRITE_FLAG_COPY);
        tcp_output(pcb);
    }

    tcp_close(pcb);
    return ERR_OK;
}

// -------------------------------------------------------
// TCP接続受け入れコールバック
// -------------------------------------------------------
static err_t on_accept(void *arg, struct tcp_pcb *new_pcb, err_t err) {
    if (err != ERR_OK || !new_pcb) return ERR_VAL;
    tcp_recv(new_pcb, on_recv);
    return ERR_OK;
}

// -------------------------------------------------------
// core1エントリ
// -------------------------------------------------------
void core1_entry(void) {
    // lwIPスレッドセーフバックグラウンドモードで初期化
    if (cyw43_arch_init()) {
        // 初期化失敗: このコアはそのまま停止
        return;
    }

    cyw43_arch_enable_sta_mode();

    // WiFi接続（最大30秒タイムアウト）
    if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID, WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        // 接続失敗
        cyw43_arch_deinit();
        return;
    }

    // TCPサーバー起動
    struct tcp_pcb *server = tcp_new();
    tcp_bind(server, IP_ADDR_ANY, HTTP_PORT);
    server = tcp_listen(server);
    tcp_accept(server, on_accept);

    // メインループ: キューからデータを取り出し、lwIPをポーリング
    while (true) {
        bme280_data_t data;
        if (queue_try_remove(&sensor_queue, &data)) {
            latest_data = data;
            data_available = true;
        }

        // lwIPのタイマー処理（threadsafe_backgroundモードでは必須）
        cyw43_arch_poll();
        sleep_ms(1);
    }
}
