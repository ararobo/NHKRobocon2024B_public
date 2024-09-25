/**
 * @file udp_data_manager.hpp
 * @author Gento Aiba (GN10)
 * @brief UDP通信を使ってロボ１のPCとメイン基板を通信するためのクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "lan_peripheral.hpp"
#include "udp_data_configure.hpp"

#define RX_BUFFER_SIZE 255
#define DEBUG_PACKET_SIZE 255
#define DEBUG_PACKET_META_DATA_SIZE 2

class UDPDataManager : public LANPeripheral
{
private:
    uint8_t rx_buff[RX_BUFFER_SIZE];

    uint8_t socket_can;
    uint8_t socket_controller;
    uint8_t socket_robot2;
    uint8_t socket_debug;

    uint8_t rx_debug_packet_data[DEBUG_PACKET_SIZE + DEBUG_PACKET_META_DATA_SIZE];
    uint8_t tx_debug_packet_data[DEBUG_PACKET_SIZE + DEBUG_PACKET_META_DATA_SIZE];

    /**
     * @brief socketの初期化
     *
     */
    void init_socket();

    /**
     * @brief デバッグパケットを構築する
     *
     * @param data_kind 通信データの種類[wisun_debug...etc.]
     * @param data データ
     * @param data_size データ長[byte]
     * @return true 問題なし
     * @return false 問題あり
     */
    bool encode_debug_packet(uint8_t data_kind, uint8_t *data, uint8_t data_size);

    /**
     * @brief デバッグパケットを解読する
     *
     * @param data_kind 通信データの種類[wisun_debug...etc.]
     * @param data データ
     * @param data_size データ長[byte]
     * @return true 問題なし
     * @return false 問題あり
     */
    bool decode_debug_packet(uint8_t *data_kind, uint8_t *data, uint8_t *data_size);

public:
    UDPDataManager();
    ~UDPDataManager();

    /**
     * @brief デバッグのパケットを送信する
     *
     * @param packet 送信するパケット
     */
    void send_debug_packet(uint8_t *data, uint8_t size);

    /**
     * @brief peripheralの初期化とsocketの初期化
     *
     */
    void init(bool is_controller);

    /**
     * @brief CANのパケットを送信する
     *
     * @param packet 送信するパケット
     */
    void send_can_packet(udp_can_t packet);

    /**
     * @brief コントローラのパケットを送信する
     *
     * @param packet 送信するパケット
     */
    void send_controller_packet(controller_t packet);

    /**
     * @brief CANのパケットを受信する
     *
     * @param packet 受信したパケット
     * @return true 受信成功
     * @return false 受信失敗
     */
    bool get_can_packet(udp_can_t *packet);

    /**
     * @brief コントローラのパケットを受信する
     *
     * @param packet 受信したパケット
     * @return true 受信成功
     * @return false 受信失敗
     */
    bool get_controller_packet(controller_t *packet);

    /**
     * @brief ロボット2のパケットを受信する
     *
     * @param packet 受信したパケット
     * @return true 受信成功
     * @return false 受信失敗
     */
    bool get_robot2_packet(robot2_t *packet);

    /**
     * @brief debug_packetのデータを解読しデータやMeta情報を取り出す
     *
     * @param data_kind データの種類[udp_configure::debug]
     * @param data データ
     * @param data_size データのサイズ[byte]
     *
     * @return bool
     */
    bool get_debug_packet(uint8_t *data_kind, uint8_t *data, uint8_t *data_size);
};