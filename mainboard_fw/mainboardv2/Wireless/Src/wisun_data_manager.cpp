/**
 * @file wisun_data_manager.cpp
 * @author Gento Aiba (GN10)
 * @brief Wi-SUN無線通信用のクラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "wisun_data_manager.hpp"
#include "serial_printf.hpp"
#include "app_configure.hpp"

// 16進数文字を数値に変換するヘルパー関数
uint8_t WiSUNDataManager::hexCharToValue(char hexChar)
{
    if (hexChar >= '0' && hexChar <= '9') // 0~9の場合
    {
        return hexChar - '0'; // 文字コードの差を取ることで数値に変換
    }
    else if (hexChar >= 'A' && hexChar <= 'F') // A~Fの場合
    {
        return hexChar - 'A' + 10; // 文字コードの差を取り、10を加算することで数値に変換
    }
    else if (hexChar >= 'a' && hexChar <= 'f')
    {
        return hexChar - 'a' + 10;
    }
    HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
    serial_printf("[ERROR] WiSUN-manager# Invalid hex char\n");
    return 0; // ここに到達することはないはず
}

// 文字列をuint8_tに変換する関数
uint8_t WiSUNDataManager::hexStringToUint8(const char *hexStr)
{
    if (hexStr == NULL || hexStr[0] == '\0' || hexStr[1] == '\0')
    {
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET);
        serial_printf("[ERROR] WiSUN-manager# Invalid hex string\n");
        return 0; // 無効な入力に対して0を返す
    }

    uint8_t highNibble = hexCharToValue(hexStr[0]);
    uint8_t lowNibble = hexCharToValue(hexStr[1]);

    return (highNibble << 4) | lowNibble;
}

bool WiSUNDataManager::sendUdpPacket(uint8_t *data, uint16_t size, const char *ip_address)
{
    if (size > 12)
    {
        serial_printf("[WARN] WiSUN-manager# send packet too big.\n");
        return false;
    }
    char hex_data[size * 2 + 1]; // データを16進数文字列に変換するためのバッファ
    for (uint16_t i = 0; i < size; ++i)
    {
        sprintf(&hex_data[i * 2], "%02X", data[i]); // 16進数文字列に変換
    }
    hex_data[size * 2] = '\0';                                                 // 文字列の終端を追加
    sprintf(send_data, "udps %s %s\n", ip_address, hex_data);                  // 送信データを作成
    return WiSUNPeripheral::sendData((uint8_t *)send_data, strlen(send_data)); // 送信
}

bool WiSUNDataManager::isUdpPacket(char *data)
{
    return strncmp(data, UDP_RECEIVE_COMMAND, strlen(UDP_RECEIVE_COMMAND)) == 0; // UDPパケットかどうかを判定
}

uint16_t WiSUNDataManager::processUdpPacket(char *line_data, char *ip_address, char *udp_data)
{
    uint16_t udp_data_size = 0;
    ////////////////////////////////////////// IPアドレスの取得 //////////////////////////////////////////
    uint8_t ip_start_index = strlen(UDP_RECEIVE_COMMAND) + 2; // "udpr <"の後ろからIPアドレスが始まる
    uint8_t ip_index;
    for (ip_index = ip_start_index; ip_index < WISUN_LINE_BUFFER_SIZE; ip_index++)
    {
        uint16_t ip_address_size = ip_index - ip_start_index;
        if (ip_address_size > WISUN_MAX_IP_ADDRESS_LENGTH)
        {
            serial_printf("[WARN] WiSUN-manager# ip address length too long.\n");
            return 0;
        }
        if (line_data[ip_index] == '>' && line_data[ip_index + 1] == ' ') // IPアドレスの終端
        {
            break; // 終端の場合はループを抜ける
        }
        else
        {
            ip_address[ip_address_size] = line_data[ip_index]; // IPアドレスを取得
        }
    }
    ip_address[ip_index - ip_start_index] = '\0'; // IPアドレスの終端を追加

    ////////////////////////////////////////// UDPデータの取得 //////////////////////////////////////////
    uint16_t data_start_index = ip_index + 2; // アドレスの後ろからデータが始まる
    uint16_t data_index;
    for (data_index = data_start_index; data_index < WISUN_LINE_BUFFER_SIZE; data_index++) // データの取得
    {
        if (line_data[data_index] == '\0') // 終端の場合
        {
            break; // ループを抜ける
        }
        else
        {
            udp_data[data_index - data_start_index] = line_data[data_index]; // データを取得
        }
    }
    udp_data_size = data_index - data_start_index; // データのサイズを取得
    udp_data[udp_data_size] = '\0';                // 文字列の終端を追加
    return udp_data_size;
}

void WiSUNDataManager::classifyUdpData(char *udp_data, uint16_t udp_data_size)
{
    if (udp_data_size == sizeof(robot2_wisun_t) * 2) // １バイトが２文字なので２倍
    {
        for (uint8_t i = 0; i < sizeof(robot2_wisun_t); i++) // １バイトずつ処理
        {
            char hex_str[3];
            hex_str[0] = udp_data[i * 2];                     // １文字目
            hex_str[1] = udp_data[i * 2 + 1];                 // ２文字目
            hex_str[2] = '\0';                                // 文字列の終端
            robot2_packet.raw[i] = hexStringToUint8(hex_str); // 16進数文字列を数値に変換して格納
        }
        flag_robot2_packet = true; // フラグを立てる
    }
    else if (udp_data_size == sizeof(controller_t) * 2)
    {
        for (uint8_t i = 0; i < sizeof(controller_t); i++)
        {
            char hex_str[3];
            hex_str[0] = udp_data[i * 2];
            hex_str[1] = udp_data[i * 2 + 1];
            hex_str[2] = '\0';
            controller_packet.raw[i] = hexStringToUint8(hex_str);
        }
        flag_controller_packet = true;
    }
}

bool WiSUNDataManager::getRobot2Packet(robot2_wisun_t *packet)
{
    if (flag_robot2_packet)
    {
        *packet = robot2_packet;
        flag_robot2_packet = false;
        return true;
    }
    return false;
}

void WiSUNDataManager::sendRobot2Packet(robot2_wisun_t packet, uint8_t robot2_ip_under)
{
    char ip_address[40];
    sprintf(ip_address, "2001:db8::%d", robot2_ip_under);
    sendUdpPacket(packet.raw, sizeof(robot2_wisun_t), ip_address); // 送信
}

bool WiSUNDataManager::getControllerPacket(controller_t *packet)
{
    if (flag_controller_packet)
    {
        *packet = controller_packet;
        flag_controller_packet = false;
        return true;
    }
    return false;
}

bool WiSUNDataManager::sendControllerPacket(controller_t packet)
{
    char ip_address[40] = "2001:db8::2";
    return sendUdpPacket((uint8_t *)&packet, sizeof(controller_t), ip_address);
}