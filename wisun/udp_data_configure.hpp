/**
 * @file udp_data_configure.hpp
 * @author Gento Aiba
 * @brief 通信で用いるデータの設定
 * @version 1.0
 * @date 2024-08-29
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>

/**
 * @brief UDP通信の基本的な定数
 *
 */
namespace udp_configure
{
    namespace port
    {
        static constexpr uint16_t controller = 5000;
        static constexpr uint16_t udp_to_can = 5001;
        static constexpr uint16_t robot2 = 5002;
        static constexpr uint16_t debug = 5003;
    }
    namespace ip
    {
        static constexpr uint8_t broadcast_address[4] = {255, 255, 255, 255};
        static constexpr uint8_t default_route[4] = {0, 0, 0, 0};
        static constexpr uint8_t pc_robo1[4] = {192, 168, 1, 11};
        static constexpr uint8_t pc_controller[4] = {192, 168, 1, 12};
        static constexpr uint8_t mainboard_controller[4] = {192, 168, 1, 13};
        static constexpr uint8_t mainboard_robot1[4] = {192, 168, 1, 14};
    }
    namespace mac
    {
        static constexpr uint8_t mainboard_controller[6] = {0x52, 0x42, 0x00, 0x0f, 0x04, 0x24};
        static constexpr uint8_t mainboard_robot1[6] = {0x52, 0x42, 0x00, 0x63, 0x58, 0x52};
    }
    namespace debug
    {
        static constexpr uint8_t kind_wisun = 0;
    }
}

/**
 * @brief robot2の制御値の構造体[1byte]
 *
 */
struct robot2_data_t
{
    unsigned char mode : 1;
    unsigned char slow : 1;
    unsigned char forward : 1;
    unsigned char backward : 1;
    unsigned char right : 1;
    unsigned char left : 1;
    unsigned char action : 2;
} __attribute__((__packed__));

/**
 * @brief キーボードの状態の構造体[3byte]
 *
 */
struct controller_data_t
{
    unsigned char w : 1;
    unsigned char a : 1;
    unsigned char s : 1;
    unsigned char d : 1;
    unsigned char t : 1;
    unsigned char f : 1;
    unsigned char g : 1;
    unsigned char h : 1;
    unsigned char i : 1;
    unsigned char j : 1;
    unsigned char k : 1;
    unsigned char l : 1;
    unsigned char shift : 1;
    unsigned char lotate_l : 1;
    unsigned char lotate_r : 1;
    unsigned char num_1 : 1;
    unsigned char num_2 : 1;
    unsigned char num_3 : 1;
    unsigned char num_4 : 1;
    unsigned char num_5 : 1;
    unsigned char num_6 : 1;
    unsigned char num_7 : 1;
    unsigned char num_8 : 1;
    unsigned char num_9 : 1;
} __attribute__((__packed__));

/**
 * @brief キーボードの状態の構造体とuint8型の共用体
 *
 */
union controller_t
{
    controller_data_t data;
    uint8_t raw[sizeof(controller_data_t)];
} __attribute__((__packed__));

/**
 * @brief UDP通信用にCANのデータをuint8と共用した
 *
 */
union udp_can_t
{
    struct
    {
        uint16_t id;
        uint8_t dlc;
        uint8_t data[8];
    } __attribute__((__packed__)) can;
    uint8_t raw[11];
} __attribute__((__packed__));

/**
 * @brief ロボット2の制御値x2のuint8との共用体
 *
 */
union robot2_t
{
    robot2_data_t data[2];
    uint8_t raw[2];
} __attribute__((__packed__));