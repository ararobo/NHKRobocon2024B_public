/**
 * @file can2_manager.hpp
 * @author Gento Aiba (GN10)
 * @brief CAN通信用のクラス(モータードライバ)
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

class CAN2Manager
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
    CAN2Manager();

    void init();

    bool sendPacket(uint16_t can_id, uint8_t *tx_buffer, uint8_t data_length);

    void onReceiveTask(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

    /* HTMD*/
    /**
     * @brief モータードライバとして初期化
     *
     * @param device_id デバイスID
     */
    void sendMDInit(uint8_t device_id);
    /**
     * @brief モータードライバのモードを設定
     *
     * @param device_id デバイスID
     * @param mode モード
     */
    void sendMDMode(uint8_t device_id, md_mode_t mode);
    /**
     * @brief モータードライバのPIDゲインを設定
     *
     * @param device_id デバイスID
     * @param p_gain 比例ゲイン
     * @param i_gain 積分ゲイン
     * @param d_gain 微分ゲイン
     */
    void sendMDPIDGain(uint8_t device_id, float p_gain, float i_gain, float d_gain);
    /**
     * @brief モータードライバの目標値を設定
     *
     * @param device_id デバイスID
     * @param target 目標値
     */
    void sendMDMotorTarget(uint8_t device_id, int16_t target);

    /**
     * @brief マスターへのパケットを取得
     *
     * @param data UDP_CANデータのポインタ
     * @return true 取得成功
     * @return false 取得失敗
     */
    bool getToMasterPacket(udp_can_t *data);

private:
    void sendGain(uint16_t can_id, float gain);
};