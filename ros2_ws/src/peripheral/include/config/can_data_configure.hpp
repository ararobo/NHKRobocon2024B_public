/**
 * @file can_data_configure.hpp
 * @author Gento Aiba (GN10)
 * @brief CAN通信の設定
 * @version 4.2
 * @date 2024-09-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>

namespace can_config
{
    namespace dir // 通信方向
    {
        static constexpr uint16_t to_slave = 0;  // PCやマザーへの通信
        static constexpr uint16_t to_master = 1; // MDやドライバへの通信
    }
    namespace dev // デバイスの種類
    {
        static constexpr uint16_t common = 0;          // 共通
        static constexpr uint16_t motor_driver = 1;    // モータードライバ
        static constexpr uint16_t servo_driver = 2;    // サーボドライバ
        static constexpr uint16_t solenoid_driver = 3; // ソレノイドドライバ
        static constexpr uint16_t led_driver = 4;      // LEDドライバ
    }
    namespace data_name // データの種類
    {
        namespace common
        {
            static constexpr uint16_t init = 0;       // 初期化コマンド
            static constexpr uint16_t tof_sensor = 1; // ToF距離センサ
            static constexpr uint16_t robot2 = 2;
            static constexpr uint16_t gyro = 3;
        }
        namespace md
        {
            static constexpr uint16_t init = 0;    // 初期化コマンド
            static constexpr uint16_t targets = 1; // 目標値
            static constexpr uint16_t mode = 2;    // モード
            static constexpr uint16_t p_gain = 3;  // PID制御の比例ゲイン
            static constexpr uint16_t i_gain = 4;  // PID制御の積分ゲイン
            static constexpr uint16_t d_gain = 5;  // PID制御の微分ゲイン
            static constexpr uint16_t sensor = 6;  // センサー(リミットスイッチ、エンコーダ、電流センサー)
            static constexpr uint16_t status = 7;  // ステータス
        }
        namespace servo
        {
            static constexpr uint16_t init = 0;        // 初期化コマンド
            static constexpr uint16_t target = 1;      // 目標値
            static constexpr uint16_t targets_1_4 = 2; // 1~4のサーボの目標値
            static constexpr uint16_t targets_5_8 = 3; // 5~8のサーボの目標値
            static constexpr uint16_t freq = 4;        // 周波数
        }
        namespace solenoid
        {
            static constexpr uint16_t init = 0;    // 初期化コマンド
            static constexpr uint16_t targets = 1; // 目標値
        }
        namespace led
        {
            static constexpr uint16_t init = 0;    // 初期化コマンド
            static constexpr uint16_t targets = 1; // 目標値
        }
    }
    namespace dlc // Data Length Code
    {
        namespace common
        {
            static constexpr uint8_t init = 1; // 初期化コマンド
            static constexpr uint8_t tof_sensor = 2;
            static constexpr uint8_t robot2 = 1;
            static constexpr uint8_t gyro = 2;
        }
        namespace md
        {
            static constexpr uint8_t targets_1 = 2;             // 1のモーターの目標値
            static constexpr uint8_t targets_4 = 8;             // 1~4のモーターの目標値
            static constexpr uint8_t init = 1;                  // 初期化コマンド
            static constexpr uint8_t mode = 8;                  // モード
            static constexpr uint8_t p_gain = 4;                // PID制御の比例ゲイン
            static constexpr uint8_t i_gain = 4;                // PID制御の積分ゲイン
            static constexpr uint8_t d_gain = 4;                // PID制御の微分ゲイン
            static constexpr uint8_t limit = 1;                 // リミットスイッチの状態
            static constexpr uint8_t limit_encoder = 3;         // エンコーダとリミットスイッチの状態
            static constexpr uint8_t limit_encoder_current = 7; // エンコーダとリミットスイッチの状態と電流
            static constexpr uint8_t state = 1;                 // MDの状態
            static constexpr uint8_t state_temp = 3;            // MDの状態と温度
        }
        namespace servo
        {
            static constexpr uint8_t targets_1 = 2;   // 1のサーボの目標値
            static constexpr uint8_t targets_1_4 = 8; // 1~4のサーボの目標値
            static constexpr uint8_t targets_5_8 = 8; // 5~8のサーボの目標値
            static constexpr uint8_t init = 1;        // 初期化コマンド
            static constexpr uint8_t freq = 1;        // 周波数
        }
        namespace solenoid
        {
            static constexpr uint8_t targets = 1; // ソレノイドx8の目標値(1bitづつ対応)
            static constexpr uint8_t init = 1;    // 初期化コマンド
        }
        namespace led
        {
            static constexpr uint8_t targets = 1; // LEDの点灯パターン
            static constexpr uint8_t init = 1;    // 初期化コマンド
        }
    }
    namespace code // 通信コード
    {
        namespace state
        {
            static constexpr int init = 0x00;
            static constexpr int ready = 0x01;
            static constexpr int busy = 0x02;
            static constexpr int error = 0x03;
        }
        namespace command
        {
            static constexpr int init = 0x00;
        }
    }
}

uint16_t encodeCanID(uint8_t dir, uint8_t dev, uint8_t device_id, uint8_t data_name);

void decodeCanID(uint16_t can_id, uint8_t *dir, uint8_t *dev, uint8_t *device_id, uint8_t *data_name);