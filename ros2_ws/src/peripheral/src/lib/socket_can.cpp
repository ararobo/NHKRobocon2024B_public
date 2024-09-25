/**
 * @file socket_can.cpp
 * @author Gento Aiba (GN10)
 * @brief CAN通信をLinuxで行う
 * @version 1.1
 * @date 2024-04-06
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "lib/socket_can.hpp"
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cerrno>

bool SocketCAN::sendPacket(uint16_t id, uint8_t *send_buffer, uint8_t data_length)
{
    can_tx_frame.can_id = id;
    can_tx_frame.can_dlc = data_length;
    for (__u8 i = 0; i < data_length; i++)
    {
        can_tx_frame.data[i] = send_buffer[i];
    }
    ssize_t nbytes = write(can_socket, &can_tx_frame, sizeof(can_tx_frame));
    if (nbytes == -1)
    {
        perror("write");
        return false;
    }
    if (nbytes != sizeof(can_tx_frame))
    {
        perror("write a frame");
        return false;
    }
    return true;
}

bool SocketCAN::readPacket(uint8_t *receive_buffer, uint16_t *rx_id, uint8_t *rx_data_length)
{
    rx_numbytes = read(can_socket, &can_rx_frame, sizeof(can_rx_frame));
    if (rx_numbytes < 0 && errno != EAGAIN)
    {
        perror("read");
        return false;
    }
    else if (rx_numbytes <= 0)
    {
        return false;
    }
    for (uint8_t i = 0; i < can_rx_frame.can_dlc; i++)
    {
        receive_buffer[i] = can_rx_frame.data[i];
    }
    *rx_id = can_rx_frame.can_id;
    *rx_data_length = can_rx_frame.can_dlc;
    return true;
}

void SocketCAN::initCAN()
{
    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("make socket");
        return;
    }
    strcpy(ifr.ifr_name, "can0");
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        return; // エラーが発生した場合、関数を終了
    }
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind device");
        return;
    }
    // ソケットを非ブロックモードに設定
    int flags = fcntl(can_socket, F_GETFL, 0);
    fcntl(can_socket, F_SETFL, flags | O_NONBLOCK);
}

void SocketCAN::closeSocket()
{
    if (close(can_socket) < 0)
    {
        perror("close socket");
    }
}