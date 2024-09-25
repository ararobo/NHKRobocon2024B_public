/**
 * @file wisun_data_configure.hpp
 * @author Gento Aiba (GN10)
 * @brief 無線通信するときのデータの設定
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "udp_data_configure.hpp"

namespace wisun_configure
{
}

union robot2_wisun_t
{
    robot2_data_t data;
    uint8_t raw[sizeof(robot2_data_t)];
} __attribute__((__packed__));