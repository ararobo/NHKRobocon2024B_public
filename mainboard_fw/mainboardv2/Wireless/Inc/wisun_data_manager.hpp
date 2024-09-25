/**
 * @file wisun_data_manager.hpp
 * @author Gento Aiba (GN10)
 * @brief Wi-SUN無線通信用のクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "wisun_data_configure.hpp"
#include "wisun_peripheral.hpp"
#include <string>

#define UDP_RECEIVE_COMMAND "udpr"
#define WISUN_LINE_BUFFER_SIZE 1024
#define WISUN_MAX_IP_ADDRESS_LENGTH 16
#define WISUN_MAX_UDP_DATA_SIZE 32

class WiSUNDataManager : public WiSUNPeripheral
{
private:
    // robot2パケット
    robot2_wisun_t robot2_packet;
    bool flag_robot2_packet;
    // controllerパケット
    controller_t controller_packet;
    bool flag_controller_packet;
    // UDP送信データ
    char send_data[48];

    /**
     * @brief UDPパケットを送信する
     *
     * @param data 送信するデータ
     * @param size 送信するデータのサイズ
     * @param ip_address 送信先のIPアドレス
     * @return true 送信成功
     * @return false 送信失敗
     */
    bool sendUdpPacket(uint8_t *data, uint16_t size, const char *ip_address);

    /**
     * @brief 16進数の文字を数値に変換する
     *
     * @param hexChar 16進数の文字
     * @return uint8_t 16進数の数値
     */
    uint8_t hexCharToValue(char hexChar);

    /**
     * @brief 16進数の文字列を数値に変換する
     *
     * @param hexStr 16進数の文字列
     * @return uint8_t 16進数の数値
     */
    uint8_t hexStringToUint8(const char *hexStr);

public:
    /**
     * @brief robot2パケットを取得する
     *
     * @param packet robot2パケット
     * @return true robot2パケット取得
     * @return false robot2パケット未取得
     */
    bool getRobot2Packet(robot2_wisun_t *packet);

    /**
     * @brief ロボット２のパケットを送信する
     *
     * @param packet 送信するパケット
     * @param robot2_ip_under ipアドレスの下1桁
     * @param robot_id ロボットのID（0~1）
     */
    void sendRobot2Packet(robot2_wisun_t packet, uint8_t robot2_ip_under);

    /**
     * @brief controllerパケットを取得する
     *
     * @param packet controllerパケット
     * @return true controllerパケット取得
     * @return false controllerパケット未取得
     */
    bool getControllerPacket(controller_t *packet);

    /**
     * @brief controllerパケットを送信する
     *
     * @param packet controllerパケット
     * @return true 送信成功
     * @return false 送信失敗
     */
    bool sendControllerPacket(controller_t packet);

    /**
     * @brief UDPパケットかどうかを判定する
     *
     * @param line_data 受信した行データ
     * @return true UDPパケット
     * @return false UDPパケットでない
     */
    bool isUdpPacket(char *line_data);

    /**
     * @brief UDPパケットを処理する
     *
     * @param line_data 受信した行データ
     * @param ip_address 送信元のIPアドレス
     * @param udp_data UDPデータ
     * @return UDPデータのサイズ
     */
    uint16_t processUdpPacket(char *line_data, char *ip_address, char *udp_data);

    /**
     * @brief UDPデータを分類する
     *
     * @param udp_data UDPデータ
     * @param udp_data_size UDPデータのサイズ
     */
    void classifyUdpData(char *udp_data, uint16_t udp_data_size);
};
