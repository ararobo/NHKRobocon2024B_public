/**
 * @file udp_controller.hpp
 * @author Gento Aiba (GN10)
 * @brief LinuxでUDP通信を楽にやるためのクラス
 * @version 0.1
 * @date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <netinet/in.h>
#include <stdint.h>
#include "config/udp_data_configure.hpp"

class SimpleUDP
{
private:
    int sock_;                   // ソケット
    struct sockaddr_in tx_addr_; // 送信先アドレス
    struct sockaddr_in rx_addr_; // 受信先アドレス
    uint8_t rx_buffer_[255];     // 受信データ

public:
    SimpleUDP();

    /**
     * @brief ソケットを作成する
     *
     * @return true 作成できた
     * @return false 作成できなかった
     */
    bool initSocket();

    /**
     * @brief 送信用IPアドレスとポートを設定する
     *
     * @param ip_address IPアドレス[uint8 x4]
     * @param port ポート番号
     * @return true 成功
     * @return false 失敗
     */
    bool setTxAddr(const uint8_t ip_address[4], const uint16_t port);
    /**
     * @brief バインドする(受信)
     *
     * @param ip_address 送信元のIPアドレス
     * @param port 受信用ポート番号
     * @return true 成功
     * @return false 失敗
     */
    bool bindSocket(const uint8_t ip_address[4], const uint16_t port);

    /**
     * @brief パケットを送信する(IPアドレスとポートを個別指定)
     *
     * @param data 送信データ
     * @param data_size 送信データサイズ
     * @param ip_address 送信先IPアドレス
     * @param port 送信先ポート
     * @return true 成功
     * @return false 失敗
     */
    bool sendPacket(uint8_t *data, uint8_t data_size, const uint8_t *ip_address, const uint16_t port);
    /**
     * @brief パケットを送信する(事前にIPアドレスとポートを指定)
     *
     * @param data 送信データ
     * @param data_size 送信データサイズ
     * @return true 成功
     * @return false 失敗
     */
    bool sendPacket(uint8_t *data, uint8_t data_size);
    /**
     * @brief パケットを受信する
     *
     * @param buffer 受信したデータを入れるバッファ
     * @param buffer_size 受信するデータのサイズ又はバッファのサイズ
     * @return int 受信データのサイズ
     */
    int recvPacket(uint8_t *buffer, uint8_t buffer_size);

    /**
     * @brief ソケットを閉じる
     *
     */
    void closeSocket();
};