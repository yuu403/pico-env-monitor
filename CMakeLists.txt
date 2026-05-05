cmake_minimum_required(VERSION 3.13)

# Pico SDKのパスを設定（環境変数 PICO_SDK_PATH が必要）
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Pico W指定（重要）
set(PICO_BOARD pico_w)

project(pico_env_monitor C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# PIOプログラムのコード生成
# pioasm が i2c_bb.pio → i2c_bb.pio.h を生成する

add_executable(pico_env_monitor
    src/main.c
    src/pio_i2c.c
    src/bme280.c
    src/core1_entry.c
)

pico_generate_pio_header(pico_env_monitor
    ${CMAKE_CURRENT_SOURCE_DIR}/i2c_bb.pio
)

target_include_directories(pico_env_monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}  # pioasm生成ヘッダのパス
)

# WiFi認証情報をビルド時に渡す
# 使い方: cmake -DWIFI_SSID="mynet" -DWIFI_PASSWORD="pass" ..
if(DEFINED WIFI_SSID)
    target_compile_definitions(pico_env_monitor PRIVATE
        WIFI_SSID="${WIFI_SSID}"
        WIFI_PASSWORD="${WIFI_PASSWORD}"
    )
endif()

target_link_libraries(pico_env_monitor
    pico_stdlib
    pico_stdlib_headers
    pico_multicore
    hardware_pio
    hardware_gpio
    pico_cyw43_arch_lwip_threadsafe_background  # core1からのlwIP安全使用
)


# UARTとUSBの両方でstdioを有効化
pico_enable_stdio_uart(pico_env_monitor 1)
pico_enable_stdio_usb(pico_env_monitor 1)

# UF2/BIN/HEXの生成
pico_add_extra_outputs(pico_env_monitor)
