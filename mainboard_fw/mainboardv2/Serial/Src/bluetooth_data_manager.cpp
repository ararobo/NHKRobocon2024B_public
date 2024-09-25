/**
 * @file bluetooth_data_manager.cpp
 * @author Gento Aiba (GN10)
 * @brief ESP32にシリアル通信をしてBluetoothで通信するクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "bluetooth_data_manager.hpp"

void BluetoothDataManager::classifyData(uint8_t *data, uint8_t data_size)
{
    switch (data_size)
    {
    case sizeof(robot2_wisun_t):
        for (uint8_t i = 0; i < sizeof(robot2_wisun_t); i++)
        {
            robot2_packet.raw[i] = data[i];
        }
        break;
    case sizeof(controller_t):
        for (uint8_t i = 0; i < sizeof(controller_t); i++)
        {
            controller_packet.raw[i] = data[i];
        }
        break;

    default:
        break;
    }
}

bool BluetoothDataManager::getRobot2Packet(robot2_wisun_t *data)
{
    if (flag_robot2_packet)
    {
        for (uint8_t i = 0; i < sizeof(robot2_wisun_t); i++)
        {
            data->raw[i] = robot2_packet.raw[i];
        }
        flag_robot2_packet = false;
        return true;
    }
    return false;
}

bool BluetoothDataManager::getControllerPacket(controller_t *data)
{
    if (flag_controller_packet)
    {
        for (uint8_t i = 0; i < sizeof(controller_t); i++)
        {
            data->raw[i] = controller_packet.raw[i];
        }
        flag_controller_packet = false;
        return true;
    }
    return false;
}