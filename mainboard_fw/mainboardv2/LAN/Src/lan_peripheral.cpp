/**
 * @file lan_peripheral.cpp
 * @author Gento Aiba (GN10)
 * @brief Ethernetコントローラーの管理用クラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "lan_peripheral.hpp"
#include "serial_printf.hpp"

LANPeripheral::LANPeripheral()
{
}

LANPeripheral::~LANPeripheral()
{
}

void LANPeripheral::init_peripheral(const uint8_t ip_address[4], const uint8_t mac_address[6])
{
    if (W5500Init())
    {
        serial_printf("[INFO] LAN# Success W5500 Init.\n");
    }
    else
    {
        serial_printf("[ERROR]LAN# Failed W5500 Init.\n");
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
        return;
    }

    // W5500のネットワーク情報を設定
    netInfo = {
        .mac = {mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]},
        .ip = {ip_address[0], ip_address[1], ip_address[2], ip_address[3]},
        .sn = {255, 255, 255, 0},
        .gw = {192, 168, 1, 1},
        .dns = {192, 168, 1, 1},
        .dhcp = NETINFO_DHCP};

    wizchip_setnetinfo(&netInfo);
    // ネットワーク情報の確認
    wiz_NetInfo tmpNetInfo;
    wizchip_getnetinfo(&tmpNetInfo);

    serial_printf("[INFO] LAN# MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", tmpNetInfo.mac[0], tmpNetInfo.mac[1], tmpNetInfo.mac[2], tmpNetInfo.mac[3], tmpNetInfo.mac[4], tmpNetInfo.mac[5]);
    serial_printf("[INFO] LAN# IP: %d.%d.%d.%d\n", tmpNetInfo.ip[0], tmpNetInfo.ip[1], tmpNetInfo.ip[2], tmpNetInfo.ip[3]);
    serial_printf("[INFO] LAN# SN: %d.%d.%d.%d\n", tmpNetInfo.sn[0], tmpNetInfo.sn[1], tmpNetInfo.sn[2], tmpNetInfo.sn[3]);
    serial_printf("[INFO] LAN# GW: %d.%d.%d.%d\n", tmpNetInfo.gw[0], tmpNetInfo.gw[1], tmpNetInfo.gw[2], tmpNetInfo.gw[3]);
    serial_printf("[INFO] LAN# DNS: %d.%d.%d.%d\n", tmpNetInfo.dns[0], tmpNetInfo.dns[1], tmpNetInfo.dns[2], tmpNetInfo.dns[3]);
}
