/**
 * @file lan_peripheral.hpp
 * @author Gento Aiba (GN10)
 * @brief Ethernetコントローラーの管理用クラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "w5500_spi.hpp"
#include "wizchip_conf.hpp"
#include "socket.hpp"
#include "gpio.h"
#include "udp_data_configure.hpp"

class LANPeripheral
{
private:
    wiz_NetInfo netInfo;

protected:
    void init_peripheral(const uint8_t ip_address[4], const uint8_t mac_address[6]);

public:
    LANPeripheral();
    ~LANPeripheral();
};