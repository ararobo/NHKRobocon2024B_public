/**
 * @file serial_packetter.cpp
 * @author Gento Aiba (GN10)
 * @brief シリアル通信でパケット化するクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "serial_packetter.hpp"

uint8_t SerialPacketter::decodeSerialPacket(const uint8_t *raw_data, const uint8_t raw_data_size, uint8_t *data_buffer, uint8_t *data_size)
{
    bool found_start_of_data = false;
    *data_size = 0;
    uint8_t data_count = 0;
    for (uint8_t raw_data_count = 0; raw_data_count < raw_data_size; raw_data_count++)
    {
        if (found_start_of_data)
        {
            *data_size = raw_data[raw_data_count];
            data_count = *data_size;
            found_start_of_data = false;
        }
        if (raw_data[raw_data_count] == START_OF_PACKET)
            found_start_of_data = true;
        if (0 < data_count)
        {
            data_buffer[*data_size - data_count] = raw_data[raw_data_count];
            data_count--;
        }
        else if (raw_data[raw_data_count] == END_OF_PACKET)
        {
            return raw_data_count;
        }
    }
    return raw_data_size;
}

uint8_t SerialPacketter::encodeSerialPacket(const uint8_t *data, const uint8_t data_size, uint8_t *raw_data)
{
    if (data_size > (UINT8_MAX - META_DATA_SIZE))
        return 0;
    raw_data[0] = START_OF_PACKET;
    raw_data[1] = data_size;
    for (uint8_t i = 0; i < data_size; i++)
    {
        raw_data[i + 2] = data[i];
    }
    raw_data[data_size + 2] = END_OF_PACKET;
    return data_size + META_DATA_SIZE;
}