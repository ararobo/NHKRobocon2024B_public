/**
 * @file wisun_peripheral.cpp
 * @author Gento Aiba (GN10)
 * @brief Wi-SUNのモジュールを扱うクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "wisun_peripheral.hpp"
#include "serial_printf.hpp"

WiSUNPeripheral::WiSUNPeripheral()
{
    dma_rx_prev_index = 0;
    memset(dma_rx_buffer, 0, WISUN_DMA_BUFFER_SIZE);
}

WiSUNPeripheral::~WiSUNPeripheral()
{
}

void WiSUNPeripheral::DMAinit()
{
    // UARTの受信割り込みを有効にする
    HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, WISUN_DMA_BUFFER_SIZE);
}

void WiSUNPeripheral::BP35C5Reset()
{
    HAL_GPIO_WritePin(BP35_RST_GPIO_Port, BP35_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BP35_RST_GPIO_Port, BP35_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BP35_RST_GPIO_Port, BP35_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(BP35_RST_GPIO_Port, BP35_RST_Pin, GPIO_PIN_SET);
}

uint16_t WiSUNPeripheral::getLineData(char *line_buffer)
{
    uint16_t dma_rx_index = WISUN_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx); // DMAの受信バッファのインデックスを取得
    if (dma_rx_index == dma_rx_prev_index)                                                // 受信データがない場合
        return 0;

    uint16_t rx_data_size;
    if (dma_rx_index >= dma_rx_prev_index) // indexが増加した場合
    {
        rx_data_size = dma_rx_index - dma_rx_prev_index; // 受信したデータのサイズ
        uint16_t line_data_index = 0;
        uint16_t procced_dma_size = 1; // ループ終わりにインクリメントするサイズなので1からスタート
        while (line_data_index < rx_data_size)
        {
            uint8_t data = dma_rx_buffer[(dma_rx_prev_index + procced_dma_size) % WISUN_DMA_BUFFER_SIZE];
            if (data == '\n')
            {
                // 行データに入れないので上書きする。そのため何もしない
            }
            else if (data == '\r')
            {
                line_buffer[line_data_index] = '\0';
                line_data_index++;
                dma_rx_prev_index = (dma_rx_prev_index + procced_dma_size) % WISUN_DMA_BUFFER_SIZE; // DMAの受信バッファのインデックスを更新
                return line_data_index;                                                             // インデックスとサイズの変換
            }
            else
            {
                line_buffer[line_data_index] = data;
                line_data_index++;
            }
            procced_dma_size++; // DMAを順次処理しているのでインクリメント
        }
    }
    else // indexが減少しDMAバッファを受信データが折り返した場合
    {
        rx_data_size = WISUN_DMA_BUFFER_SIZE - dma_rx_prev_index + dma_rx_index;
        uint16_t line_data_index = 0;
        uint16_t procced_dma_size = 1; // ループ終わりにインクリメントするサイズなので1からスタート
        while (line_data_index < rx_data_size)
        {
            uint8_t data = dma_rx_buffer[(dma_rx_prev_index + procced_dma_size) % WISUN_DMA_BUFFER_SIZE];
            if (data == '\n')
            {
                // 行データに入れないので上書きする。そのため何もしない
            }
            else if (data == '\r')
            {
                line_buffer[line_data_index] = '\0';
                line_data_index++;
                dma_rx_prev_index = (dma_rx_prev_index + procced_dma_size) % WISUN_DMA_BUFFER_SIZE; // DMAの受信バッファのインデックスを更新
                return line_data_index;                                                             // インデックスとサイズの変換
            }
            else
            {
                line_buffer[line_data_index] = data;
                line_data_index++;
            }
            procced_dma_size++; // DMAを順次処理しているのでインクリメント
        }
    }
    return 0;
}

bool WiSUNPeripheral::sendData(uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart1, data, size);
    if (status == HAL_OK)
    {
        return true;
    }
    return false;
}