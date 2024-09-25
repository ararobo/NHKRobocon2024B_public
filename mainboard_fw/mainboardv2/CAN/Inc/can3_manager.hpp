/**
 * @file can3_manager.hpp
 * @author Gento Aiba (GN10)
 * @brief CAN通信用のクラス(モータードライバ以外)
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "fdcan.h"
#include <stdbool.h>
#include <stdint.h>
#include "htmd_mode.hpp"
#include "udp_data_configure.hpp"

class CAN3Manager
{
private:
    // 内部使用データ
    FDCAN_RxHeaderTypeDef RxHeader;
    FDCAN_FilterTypeDef RxFilter;
    FDCAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t RxData[8];
    // To Master Packet
    udp_can_t to_master_packet;
    bool flag_rx_data;

public:
    CAN3Manager();

    void init();

    bool sendPacket(uint16_t can_id, uint8_t *tx_buffer, uint8_t data_length);

    void onReceiveTask(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

    /* YamaHexDeviceController*/
    /**
     * @brief 共通として初期化
     *
     * @param device_id デバイスID
     */
    void sendHexInit(uint8_t device_id);
    /**
     * @brief YamaHexモーターの目標値4つを設定
     *
     * @param device_id デバイスID
     * @param target 目標値 (4つ)
     */
    void sendHexMotorTargets(uint8_t device_id, int16_t *target);
    /**
     * @brief YamaHexサーボ1~4の目標値を設定
     *
     * @param device_id デバイスID
     * @param target PWMの目標値[us] (4つ)
     */
    void sendHexServoTargets_1_4(uint8_t device_id, uint16_t *target);
    /**
     * @brief YamaHexソレノイドの目標値を設定
     *
     * @param device_id デバイスID
     * @param target 目標値(1bitづつ対応)
     */
    void sendHexSolenoidTargets(uint8_t device_id, uint8_t target);

    /* HTLED*/
    /**
     * @brief LEDの点灯パターンを設定
     *
     * @param device_id デバイスID
     * @param target 点灯パターン
     */
    void sendLedTarget(uint8_t device_id, uint8_t target);

    /**
     * @brief マスターへのパケットを取得
     *
     * @param data UDP_CANデータのポインタ
     * @return true 取得成功
     * @return false 取得失敗
     */
    bool getToMasterPacket(udp_can_t *data);
};