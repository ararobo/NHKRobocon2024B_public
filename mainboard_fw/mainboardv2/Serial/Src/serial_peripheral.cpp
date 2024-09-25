/**
 * @file serial_peripheral.cpp
 * @author Gento Aiba (GN10)
 * @brief シリアル通信をするためにUARTを扱うクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "serial_peripheral.hpp"

SerialPeripheral::SerialPeripheral()
{
    memset(dma_rx_buffer, 0, SERIAL_DMA_BUFFER_SIZE);
}

SerialPeripheral::~SerialPeripheral()
{
}

void SerialPeripheral::init()
{
    HAL_UART_Receive_DMA(&huart2, dma_rx_buffer, SERIAL_DMA_BUFFER_SIZE);
}

void SerialPeripheral::send(uint8_t *data, uint8_t size)
{
    HAL_UART_Transmit(&huart2, data, size, 100);
}

uint16_t SerialPeripheral::recv(uint8_t *buffer)
{
    uint16_t dma_rx_index = SERIAL_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx); // DMAの受信バッファのインデックスを取得
    if (dma_rx_index == dma_rx_prev_index)                                                 // 受信データがない場合
    {
        return 0;
    }
    uint16_t size = (dma_rx_index >= dma_rx_prev_index) ? (dma_rx_index - dma_rx_prev_index) : (SERIAL_DMA_BUFFER_SIZE - dma_rx_prev_index + dma_rx_index); // 受信データのサイズを取得
    if (size > 0)
    {
        for (uint16_t i = 0; i < size; i++)
        {
            buffer[i] = dma_rx_buffer[(dma_rx_prev_index + i) % SERIAL_DMA_BUFFER_SIZE];
        }
    }
    dma_rx_prev_index = dma_rx_index; // DMAの受信バッファのインデックスを更新
    return size;
}