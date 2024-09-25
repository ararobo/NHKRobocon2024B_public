/**
 * @file app_configure.hpp
 * @author Gento Aiba (GN10)
 * @brief メイン基板の設定
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

static constexpr uint8_t robot2_id = 0;
namespace app_configure
{
    namespace enable // 機能を有効にするかどうか（基板の実装率に合わせて調節する。）
    {
        static constexpr bool wisun = true;
        static constexpr bool lan = true;
        static constexpr bool yama_hex = true;
        static constexpr bool rgb_led = true;
        static constexpr bool bluetooth = false;
    }
    namespace task // // 機能を有効にするかどうか（基板の実装率に合わせて調節する。）
    {
        static constexpr bool pc_to_can = true;      // LAN
        static constexpr bool can_to_pc = true;      // LAN
        static constexpr bool wisun_to_pc = true;    // WiSUN LAN
        static constexpr bool pc_to_wisun = true;    // WiSUN LAN
        static constexpr bool wisun_to_can = true;   // WiSUN YamaHex
        static constexpr bool wisun_debugger = true; // WiSUN LAN
    }
    namespace mode
    {
        static constexpr bool debug = true;
    }
}
