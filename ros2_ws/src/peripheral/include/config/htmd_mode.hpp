/**
 * @file htmd_mode.hpp
 * @author Gento Aiba (GN10)
 * @brief MDのモードの共用体(通信時変換用)
 * @version 2.0
 * @date 2024-05-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

union md_mode_t
{
    struct
    {
        struct
        {
            unsigned char incremental_encoder : 1;
            unsigned char absolute_encoder : 1;
            unsigned char reverse_encoder : 1;
            unsigned char brake : 1;
            unsigned char pid : 1;
            unsigned char stop_by_limit_switch : 1;
            unsigned char torque_control : 1;
            unsigned char state : 1;
        } __attribute__((__packed__)) flags;
        struct
        {
            uint8_t max_acceleration;
            uint8_t max_current;
            uint8_t report_rate;
            uint16_t max_output;
            uint16_t motor_transfer_coefficient;
        } __attribute__((__packed__)) values;
    } __attribute__((__packed__));
    uint8_t code[8];
} __attribute__((__packed__));