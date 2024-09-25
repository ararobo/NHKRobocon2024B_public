/**
 * @file wisun_tx.cpp
 * @author Gento Aiba (GN10)
 * @brief Wi-SUNモジュールにコマンドを送信するプログラム
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "wisun_tx.hpp"
#include "simple_udp.hpp"
#include <string>
#include <iostream>

SimpleUDP simple_udp;

int main(int argc, char const *argv[])
{
    data_kind_buffer = 0;
    e_flag = 0;
    simple_udp.initSocket();
    simple_udp.setTxAddr(udp_configure::ip::mainboard_robot1, udp_configure::port::debug);
    simple_udp.bindSocket(udp_configure::ip::default_route, udp_configure::port::debug);
    printf("To stop the program, input 'c'.\n");
    printf("started\n");
    if (signal(SIGINT, abrt_handler) == SIG_ERR)
    {
        printf("ctrl+C flag error!\n");
        exit(1);
    }
    while (!e_flag)
    {
        std::string input;
        getline(std::cin, input);
        if (input == "c")
        {
            break;
        }

        input = input + "\n\r";
        encode_debug_packet(data_kind_buffer, (uint8_t *)input.c_str(), input.size());
        simple_udp.sendPacket(debug_packet_buffer_tx, input.size() + DEBUG_PACKET_META_DATA_SIZE);
    }
    simple_udp.closeSocket();
    return 0;
}

void abrt_handler(int sig)
{
    e_flag = 1;
}

bool encode_debug_packet(uint8_t data_kind, uint8_t *data, uint8_t data_size)
{
    if (data_size > DEBUG_PACKET_SIZE)
    {
        printf("failed to encode debug packet : invided data size\n");
        return false;
    }
    debug_packet_buffer_tx[0] = data_kind;
    debug_packet_buffer_tx[1] = data_size;
    for (uint8_t i = 0; i < data_size; i++)
    {
        debug_packet_buffer_tx[i + 2] = data[i];
    }
    return true;
}