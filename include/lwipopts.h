#ifndef LWIPOPTS_H
#define LWIPOPTS_H

// OSなし（bare metal）
#define NO_SYS 1

// メモリ設定
#define MEM_ALIGNMENT 4
#define MEM_SIZE 16000

// プロトコル有効化
#define LWIP_TCP 1
#define LWIP_UDP 1
#define LWIP_DHCP 1

// API（使わない）
#define LWIP_SOCKET 0
#define LWIP_NETCONN 0

// 軽量設定
#define LWIP_NETIF_STATUS_CALLBACK 1
#define LWIP_NETIF_LINK_CALLBACK 1

// チェックサム
#define CHECKSUM_GEN_IP 1
#define CHECKSUM_GEN_UDP 1
#define CHECKSUM_GEN_TCP 1
#define CHECKSUM_CHECK_IP 1
#define CHECKSUM_CHECK_UDP 1
#define CHECKSUM_CHECK_TCP 1

#endif
