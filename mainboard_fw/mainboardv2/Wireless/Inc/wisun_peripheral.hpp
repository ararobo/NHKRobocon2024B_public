/**
 * @file wisun_peripheral.hpp
 * @author Gento Aiba (GN10)
 * @brief Wi-SUNのモジュールを扱うクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#include <memory>
#include <string.h>

// DMAバッファのサイズ
#define WISUN_DMA_BUFFER_SIZE 1024

class WiSUNPeripheral
{
private:
    // DMAバッファの受信データ
    uint8_t dma_rx_buffer[WISUN_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
    // 前のDMA受信バッファのインデックス
    uint16_t dma_rx_prev_index;

protected:
public:
    WiSUNPeripheral();
    ~WiSUNPeripheral();

    /**
     * @brief DMAの初期化
     *
     */
    void DMAinit();

    /**
     * @brief BP35C5のリセット
     *
     */
    void BP35C5Reset();

    /**
     * @brief データを送信する
     *
     * @param data 送信するデータ
     * @param size 送信するデータのサイズ
     * @return true 送信成功
     */
    bool sendData(uint8_t *data, uint16_t size);

    /**
     * @brief データを受信する
     *
     * @param data 受信した行データ
     * @return uint16_t 受信した行データのサイズ
     */
    uint16_t getLineData(char *line_buffer);
};
