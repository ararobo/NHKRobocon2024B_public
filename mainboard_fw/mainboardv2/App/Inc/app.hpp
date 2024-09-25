/**
 * @file app.hpp
 * @author Gento Aiba (GN10)
 * @brief メイン基板のメインプログラム
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "usart.h"
#include "socket.hpp"
#include "wisun_data_configure.hpp"
#include "app_configure.hpp"
#include "udp_data_manager.hpp"
#include "wisun_data_manager.hpp"
#include "serial_peripheral.hpp"

class App
{
private:
    // ******* Packet（一時使用変数） ********
    controller_t controller_packet;     // コントローラーのパケット(wasd等)
    udp_can_t can_udp_packet;           // UDP通信とCAN通信の橋渡しに使用するパケット
    robot2_t robot2_packet;             // R2の制御値が２つ入っているUDP用のパケット
    robot2_wisun_t robot2_wisun_packet; // R2の制御値が１つ入っているWiSUN用のパケット
    // ******* CANのIDを生成する際に使用する一時使用変数 ********
    uint8_t can_dir;       // CANの通信方向
    uint8_t can_dev;       // CANの通信相手
    uint8_t can_data_name; // CANの通信内容
    uint8_t can_dev_id;    // 通信相手のデバイスのID
    // ******* Robot2 ********

    // ******* Debug ********
    uint32_t count_led; // loopごとにカウントして、LEDを点滅させる
    uint8_t debug_kind;
    uint8_t debug_data_size;
    uint8_t debug_data[DEBUG_PACKET_SIZE];

    // ******* ModeFlag DIPスイッチの状態を表すフラグ ********
    bool is_controller; // コントローラーモード
    bool is_panjan;     // パンジャンモード(コントローラーモードの場合はパンジャンのIPアドレスを予備に切り替える)
    bool is_bridge;     // 橋モード（コントローラーモードの場合は橋のIPアドレスを予備に切り替える）
    // ******* WiSUN *******************
    char wisun_line_data_buffer[WISUN_LINE_BUFFER_SIZE];
    // ******* Bluetooth ************
    uint8_t bluetooth_raw_data_buffer[SERIAL_DMA_BUFFER_SIZE];
    uint8_t bluetooth_data_buffer[SERIAL_DMA_BUFFER_SIZE];
    uint8_t bluetooth_data_size;
    // ******* Bluetooth or WiSUN flag *******
    bool live_bluetooth_robot2;
    bool live_bluetooth_controller;
    // ******* LEDDriver **********
    uint8_t led_driver_target;
    bool solenoid_state = false;
    bool motor_state = false;
    bool is_emergency = false;
    uint32_t led_driver_count = 0;

    /**
     * @brief LEDを点滅させる関数
     *
     */
    void led_blink();

    /**
     * @brief DIPスイッチの状態を変数に反映する関数
     *
     */
    void update_mode_by_DIP_switch();

public:
    /**
     * @brief 一回だけ実行される関数
     *
     */
    void init();

    /**
     * @brief init後に繰り返し実行される関数
     *
     */
    void loop();

    /**
     * @brief 100Hz周期で実行される関数
     *
     */
    void timer_rgb_led();

    /**
     * @brief CAN通信の受信コールバック関数
     *
     * @param hfdcan ハンドラ
     * @param RxFifo0ITs よくわからん
     */
    void CANCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
};
