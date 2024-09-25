/**
 * @file serial_packetter.hpp
 * @author Gento Aiba (GN10)
 * @brief シリアル通信でパケット化するクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "serial_peripheral.hpp"

class SerialPacketter
{
private:
    static constexpr uint8_t END_OF_PACKET = 0xFF;
    static constexpr uint8_t START_OF_PACKET = 0x44;

public:
    static constexpr uint8_t META_DATA_SIZE = 3;

    /**
     * @brief シリアル通信の受信データを解析して内容を取り出す
     *
     * @param raw_data 受信したデータ
     * @param raw_data_size 受信したデータのサイズ
     * @param data_buffer 内容
     * @param data_size 内容のサイズ
     * @return uint8_t 処理した受信データのサイズ
     */
    uint8_t decodeSerialPacket(const uint8_t *raw_data, const uint8_t raw_data_size, uint8_t *data_buffer, uint8_t *data_size);

    /**
     * @brief シリアル通信の送信データを作成する
     *
     * @param data 内容
     * @param data_size 内容のサイズ
     * @param raw_data 送信データ
     * @return uint8_t 送信データのサイズ
     */
    uint8_t encodeSerialPacket(const uint8_t *data, const uint8_t data_size, uint8_t *raw_data);
};