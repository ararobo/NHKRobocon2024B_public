/**
 * @file simple_udp.cpp
 * @author Gento Aiba (GN10)
 * @brief LinuxでUDP通信を楽にやるためのクラス
 * @version 1.0
 * @date 2024-08-31
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <memory>
#include <sys/socket.h>
#include <sys/types.h>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h> // For fcntl()
#include <errno.h> // For errno and EWOULDBLOCK
#include "simple_udp.hpp"

SimpleUDP::SimpleUDP()
{
}

/**
 * @brief ソケットを作成しアドレスを初期化する
 *
 * @return true
 * @return false
 */
bool SimpleUDP::initSocket()
{
    // DGRAMでUDPとしてソケット作成
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0)
    {
        perror("socket");
        return false;
    }
    // 他でもソケットを開けるように設定
    int on = 1;
    setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    // ノンブロックモードにする
    int flags = fcntl(sock_, F_GETFL, 0);
    fcntl(sock_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

bool SimpleUDP::setTxAddr(const uint8_t ip_address[4], const uint16_t port)
{
    tx_addr_.sin_family = AF_INET;
    tx_addr_.sin_port = htons(port);
    tx_addr_.sin_addr.s_addr = (ip_address[0] << 24) | (ip_address[1] << 16) | (ip_address[2] << 8) | ip_address[3];
    tx_addr_.sin_addr.s_addr = htonl(tx_addr_.sin_addr.s_addr);

    printf("tx_address: %s\n", inet_ntoa(tx_addr_.sin_addr));
    printf("tx_port: %d\n", ntohs(tx_addr_.sin_port));
}

bool SimpleUDP::bindSocket(const uint8_t ip_address[4], const uint16_t port)
{
    rx_addr_.sin_family = AF_INET;
    rx_addr_.sin_port = htons(port);
    rx_addr_.sin_addr.s_addr = (ip_address[0] << 24) | (ip_address[1] << 16) | (ip_address[2] << 8) | ip_address[3];
    rx_addr_.sin_addr.s_addr = htonl(rx_addr_.sin_addr.s_addr);
    if (bind(sock_, (struct sockaddr *)&rx_addr_, sizeof(rx_addr_)) < 0)
    {
        perror("bind");
        return false;
    }
    printf("rx_address: %s\n", inet_ntoa(rx_addr_.sin_addr));
    printf("rx_port: %d\n", ntohs(rx_addr_.sin_port));
    return true;
}

bool SimpleUDP::sendPacket(uint8_t *data, uint8_t size, const uint8_t *ip_address, const uint16_t port)
{
    struct sockaddr_in tx_addr;
    tx_addr.sin_family = AF_INET;
    tx_addr.sin_port = htons(port);
    tx_addr.sin_addr.s_addr = (ip_address[0] << 24) | (ip_address[1] << 16) | (ip_address[2] << 8) | ip_address[3];
    tx_addr.sin_addr.s_addr = htonl(tx_addr.sin_addr.s_addr);
    int status = sendto(sock_, data, size, 0, (struct sockaddr *)&tx_addr_, sizeof(tx_addr_));
    if (status < 0)
    {
        perror("sendto");
        return false;
    }
    return true;
}
bool SimpleUDP::sendPacket(uint8_t *data, uint8_t size)
{
    int status = sendto(sock_, data, size, 0, (struct sockaddr *)&tx_addr_, sizeof(tx_addr_));
    if (status < 0)
    {
        perror("sendto");
        return false;
    }
    return true;
}

int SimpleUDP::recvPacket(uint8_t *buffer, uint8_t buffer_size)
{
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    int rx_data_size = recvfrom(sock_, buffer, buffer_size, 0, (struct sockaddr *)&addr, &addr_len);
    if (rx_data_size < 0)
    {
        if (errno == EWOULDBLOCK)
        {
            return 0;
        }
        else
        {
            perror("recvfrom");
            return -1;
        }
    }
    return rx_data_size;
}

void SimpleUDP::closeSocket()
{
    close(sock_);
}