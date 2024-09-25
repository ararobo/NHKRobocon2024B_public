/**
 * @file bluetooth_data_manager.hpp
 * @author Gento Aiba (GN10)
 * @brief ESP32にシリアル通信をしてBluetoothで通信するクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>
#include "wisun_data_configure.hpp"

class BluetoothDataManager
{
private:
    // robot2パケット
    robot2_wisun_t robot2_packet;
    bool flag_robot2_packet;
    // controllerパケット
    controller_t controller_packet;
    bool flag_controller_packet;

public:
    /**
     * @brief データを分類してバッファに入れる
     *
     * @param data データ
     * @param data_size データのサイズ[byte]
     */
    void classifyData(uint8_t *data, uint8_t data_size);

    /**
     * @brief robot2パケットを取得する
     *
     * @param packet robot2パケット
     * @return true robot2パケット取得
     * @return false robot2パケット未取得
     */
    bool getRobot2Packet(robot2_wisun_t *packet);

    /**
     * @brief controllerパケットを取得する
     *
     * @param packet controllerパケット
     * @return true controllerパケット取得
     * @return false controllerパケット未取得
     */
    bool getControllerPacket(controller_t *packet);
};