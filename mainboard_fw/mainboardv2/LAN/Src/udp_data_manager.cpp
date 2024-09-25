/**
 * @file udp_data_manager.cpp
 * @author Gento Aiba (GN10)
 * @brief UDP通信を使ってロボ１のPCとメイン基板を通信するためのクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "udp_data_manager.hpp"
#include "tim.h"
#include "serial_printf.hpp"

UDPDataManager::UDPDataManager()
{
}

UDPDataManager::~UDPDataManager()
{
}

void UDPDataManager::init(bool is_controller)
{
    if (is_controller)
    {
        init_peripheral(udp_configure::ip::mainboard_controller, udp_configure::mac::mainboard_controller);
    }
    else
    {
        init_peripheral(udp_configure::ip::mainboard_robot1, udp_configure::mac::mainboard_robot1);
    }
    init_socket();
}

void UDPDataManager::init_socket()
{
    // 非ブロックモードでソケットを開く
    socket_controller = 0;
    socket_can = 1;
    socket_robot2 = 2;
    socket_debug = 3;
    socket(socket_controller, Sn_MR_UDP, udp_configure::port::controller, SF_IO_NONBLOCK); // SF_IO_NONBLOCK: 非ブロックモード
    socket(socket_can, Sn_MR_UDP, udp_configure::port::udp_to_can, SF_IO_NONBLOCK);
    socket(socket_robot2, Sn_MR_UDP, udp_configure::port::robot2, SF_IO_NONBLOCK);
    socket(socket_debug, Sn_MR_UDP, udp_configure::port::debug, SF_IO_NONBLOCK);
    setSn_CR(socket_controller, Sn_CR_RECV);
    setSn_CR(socket_can, Sn_CR_RECV);
    setSn_CR(socket_robot2, Sn_CR_RECV);
    setSn_CR(socket_debug, Sn_CR_RECV);
    // ソケットのオープン確認
    if (getSn_SR(socket_controller) == SOCK_UDP)
        serial_printf("[INFO] UDP# Opened Controller socket.\n");
    else
        serial_printf("[ERROR]UDP# Failed to open Controller socket.\n");
    if (getSn_SR(socket_can) == SOCK_UDP)
        serial_printf("[INFO] UDP# Opened CAN socket.\n");
    else
        serial_printf("[ERROR]UDP# Failed to open CAN socket.\n");
    if (getSn_SR(socket_robot2) == SOCK_UDP)
        serial_printf("[INFO] UDP# Opened Socket Robot2.\n");
    else
        serial_printf("[ERROR]UDP# Failed to open Robot2 socket.\n");
    if (getSn_SR(socket_debug) == SOCK_UDP)
        serial_printf("[INFO] UDP# Opened Debug socket.\n");
    else
        serial_printf("[ERROR]UDP# Failed to open Debug socket.\n");
}

void UDPDataManager::send_can_packet(udp_can_t packet)
{
    uint8_t ip_address[4] = {udp_configure::ip::pc_robo1[0], udp_configure::ip::pc_robo1[1], udp_configure::ip::pc_robo1[2], udp_configure::ip::pc_robo1[3]};
    sendto(socket_can, packet.raw, sizeof(packet.raw), ip_address, udp_configure::port::udp_to_can);
}

void UDPDataManager::send_controller_packet(controller_t packet)
{
    uint8_t ip_address[4] = {udp_configure::ip::pc_robo1[0], udp_configure::ip::pc_robo1[1], udp_configure::ip::pc_robo1[2], udp_configure::ip::pc_robo1[3]};
    sendto(socket_controller, packet.raw, sizeof(packet.raw), ip_address, udp_configure::port::controller);
}

void UDPDataManager::send_debug_packet(uint8_t *data, uint8_t size)
{
    uint8_t ip_address[4] = {udp_configure::ip::pc_robo1[0], udp_configure::ip::pc_robo1[1], udp_configure::ip::pc_robo1[2], udp_configure::ip::pc_robo1[3]};
    sendto(socket_debug, data, size, ip_address, udp_configure::port::debug);
}

bool UDPDataManager::get_can_packet(udp_can_t *packet)
{
    uint8_t destip[4];
    uint16_t destport;
    int32_t len = recvfrom(socket_can, packet->raw, sizeof(packet->raw), destip, &destport);
    if (len == sizeof(udp_can_t))
    {
        return true;
    }
    return false;
}

bool UDPDataManager::get_controller_packet(controller_t *packet)
{
    uint8_t destip[4];
    uint16_t destport;
    int32_t len = recvfrom(socket_controller, packet->raw, sizeof(packet->raw), destip, &destport);
    if (len == sizeof(controller_t))
    {
        return true;
    }
    return false;
}

bool UDPDataManager::get_robot2_packet(robot2_t *packet)
{
    uint8_t destip[4];
    uint16_t destport;
    int32_t len = recvfrom(socket_robot2, packet->raw, sizeof(packet->raw), destip, &destport);
    if (len == sizeof(robot2_t))
    {
        return true;
    }
    return false;
}

bool UDPDataManager::get_debug_packet(uint8_t *data_kind, uint8_t *data, uint8_t *data_size)
{
    uint8_t destip[4];
    uint16_t destport;
    int32_t len = recvfrom(socket_debug, rx_debug_packet_data, RX_BUFFER_SIZE, destip, &destport);
    if (len > 0)
    {
        // if (len == rx_buff[1] + DEBUG_PACKET_META_DATA_SIZE) // 定義上のサイズと受信したデータのサイズが合うかどうか
        //{
        if (!decode_debug_packet(data_kind, data, data_size)) // decodeが成功しなかった場合
        {
            return false;
        }
        return true;
        //}
    }
    return false; // 受信されなかった場合やサイズが合わなかった場合
}

bool UDPDataManager::decode_debug_packet(uint8_t *data_kind, uint8_t *data, uint8_t *data_size)
{
    *data_kind = rx_debug_packet_data[0];
    *data_size = rx_debug_packet_data[1];
    if (*data_size > DEBUG_PACKET_SIZE)
    {
        serial_printf("[WARN] UDP# Failed to decode debug packet : invalid data size\n");
        return false;
    }
    for (uint8_t i = 0; i < *data_size; i++)
    {
        data[i] = rx_debug_packet_data[i + 2];
    }
    return true;
}

bool UDPDataManager::encode_debug_packet(uint8_t data_kind, uint8_t *data, uint8_t data_size)
{
    if (data_size > DEBUG_PACKET_SIZE)
    {
        serial_printf("[WARN] UDP# failed to encode debug packet : invalid data size\n");
        return false;
    }
    tx_debug_packet_data[0] = data_kind;
    tx_debug_packet_data[1] = data_size;
    for (uint8_t i = 0; i < data_size; i++)
    {
        tx_debug_packet_data[i + 2] = data[i];
    }
    return true;
}