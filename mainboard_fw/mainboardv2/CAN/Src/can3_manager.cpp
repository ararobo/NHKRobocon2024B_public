/**
 * @file can3_manager.cpp
 * @author Gento Aiba (GN10)
 * @brief CAN通信用のクラス(モータードライバ以外)
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "can3_manager.hpp"
#include "can_data_configure.hpp"
#include "serial_printf.hpp"
#include "gpio.h"

CAN3Manager::CAN3Manager()
{
}

void CAN3Manager::init()
{
    // CANのフィルタ設定
    RxFilter.IdType = FDCAN_STANDARD_ID;             // 標準ID
    RxFilter.FilterIndex = 0;                        // フィルタインデックス
    RxFilter.FilterType = FDCAN_FILTER_MASK;         // マスク
    RxFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // FIFO0にフィルタ
    RxFilter.FilterID1 = 0x000;
    RxFilter.FilterID2 = 0x000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &RxFilter) != HAL_OK)
    {
        Error_Handler();
        serial_printf("[ERROR]CAN3# HAL_FDCAN3_ConfigFilter error.\n");
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
    }
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
        serial_printf("[ERROR]CAN3# HAL_FDCAN3_Start error.\n");
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
    }
    // 割り込み有効
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
        serial_printf("[ERROR]CAN3# HAL_FDCAN3_ActivateNotification error.\n");
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
    }
}

void CAN3Manager::onReceiveTask(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan3.Instance)
    {
        if (RxFifo0ITs != 0)
        {
            if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
            {
                Error_Handler();
                serial_printf("[ERROR]CAN3# HAL_FDCAN2_GetRxMessage error.\n");
                HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
            }
            uint8_t _;
            uint8_t dir;
            decodeCanID(RxHeader.Identifier, &dir, &_, &_, &_);
            if (dir == can_config::dir::to_master)
            {
                to_master_packet.can.id = RxHeader.Identifier;
                to_master_packet.can.dlc = RxHeader.DataLength;
                for (uint8_t i = 0; i < to_master_packet.can.dlc; i++)
                {
                    to_master_packet.can.data[i] = RxData[i];
                }
                flag_rx_data = true;
            }
        }
    }
}

bool CAN3Manager::sendPacket(uint16_t can_id, uint8_t *tx_buffer, uint8_t data_length)
{

    if (data_length > 8)
    {
    }

    TxHeader.Identifier = can_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = data_length;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (0 < HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan3))
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxHeader, tx_buffer) != HAL_OK)
        {
            Error_Handler();
            serial_printf("[ERROR]CAN3# HAL_FDCAN3_AddMessageToTxFifoQ error.\n");
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
            return false;
        }
    }
    else
    {
        serial_printf("[ERROR]CAN3# HAL_FDCAN3_GetTxFifoFreeLevel error.\n");
        serial_printf("[ERROR]CAN3# Check 5V & CAN connection!\n");
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
        return false;
    }
    return true;
}

void CAN3Manager::sendHexInit(uint8_t device_id)
{
    uint8_t tx_buffer[] = {0};
    tx_buffer[0] = can_config::code::command::init;
    uint16_t can_id = encodeCanID(can_config::dir::to_slave, can_config::dev::common, device_id, can_config::data_name::common::init);
    sendPacket(can_id, tx_buffer, can_config::dlc::common::init);
}

void CAN3Manager::sendHexMotorTargets(uint8_t device_id, int16_t *target)
{
    uint8_t tx_buffer[8] = {0};
    for (int i = 0; i < 4; i++)
    {
        tx_buffer[i * 2] = target[i] & 0xFF;
        tx_buffer[i * 2 + 1] = (target[i] >> 8) & 0xFF;
    }
    uint16_t can_id = encodeCanID(can_config::dir::to_slave, can_config::dev::motor_driver, device_id, can_config::data_name::md::targets);
    sendPacket(can_id, tx_buffer, can_config::dlc::md::targets_4);
}

void CAN3Manager::sendHexServoTargets_1_4(uint8_t device_id, uint16_t *target)
{
    uint8_t tx_buffer[8] = {0};
    for (int i = 0; i < 4; i++)
    {
        tx_buffer[i * 2] = target[i] & 0xFF;
        tx_buffer[i * 2 + 1] = (target[i] >> 8) & 0xFF;
    }
    uint16_t can_id = encodeCanID(can_config::dir::to_slave, can_config::dev::servo_driver, device_id, can_config::data_name::servo::targets_1_4);
    sendPacket(can_id, tx_buffer, can_config::dlc::servo::targets_1_4);
}

void CAN3Manager::sendHexSolenoidTargets(uint8_t device_id, uint8_t target)
{
    uint8_t tx_buffer[1] = {0};
    tx_buffer[0] = target;
    uint16_t can_id = encodeCanID(can_config::dir::to_slave, can_config::dev::solenoid_driver, device_id, can_config::data_name::solenoid::targets);
    sendPacket(can_id, tx_buffer, can_config::dlc::solenoid::targets);
}

void CAN3Manager::sendLedTarget(uint8_t device_id, uint8_t target)
{
    uint8_t tx_buffer[1] = {0};
    tx_buffer[0] = target;
    uint16_t can_id = encodeCanID(can_config::dir::to_slave, can_config::dev::led_driver, device_id, can_config::data_name::led::targets);
    sendPacket(can_id, tx_buffer, can_config::dlc::md::targets_1);
}

bool CAN3Manager::getToMasterPacket(udp_can_t *data)
{
    if (flag_rx_data)
    {
        *data = to_master_packet;
        flag_rx_data = false;
        return true;
    }
    return false;
}