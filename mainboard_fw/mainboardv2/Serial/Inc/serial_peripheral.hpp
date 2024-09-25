/**
 * @file serial_peripheral.hpp
 * @author Gento Aiba (GN10)
 * @brief シリアル通信をするためにUARTを扱うクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>
#include <usart.h>
#include <memory.h>

#define SERIAL_DMA_BUFFER_SIZE 255

class SerialPeripheral
{
private:
    // DMAバッファの受信データ
    uint8_t dma_rx_buffer[SERIAL_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
    uint16_t dma_rx_prev_index = 0;

public:
    SerialPeripheral();
    ~SerialPeripheral();

    void init();

    void send(uint8_t *data, uint8_t size);

    uint16_t recv(uint8_t *buffer);
};
