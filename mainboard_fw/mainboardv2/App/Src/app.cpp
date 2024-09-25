/**
 * @file app.cpp
 * @author Gento Aiba (GN10)
 * @brief メイン基板のメインプログラム
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "app.hpp"
#include "can2_manager.hpp"
#include "can3_manager.hpp"
#include "can_data_configure.hpp"
#include "serial_printf.hpp"
#include "robot2_controller.hpp"
#include "tim.h"
#include "spi.h"
#include "serial_packetter.hpp"
#include "bluetooth_data_manager.hpp"
#include "neopixel.hpp"
#include "led.hpp"
#include "i2c.h"
#include "led_target_formatter.hpp"

CAN2Manager can2;
CAN3Manager can3;
WiSUNDataManager wisun_data_manager;
WiSUNPeripheral wisun_peripheral;
UDPDataManager udp_data_manager;
Robot2Controller robot2_controller;
SerialPacketter serial_packetter;
SerialPeripheral serial_peripheral;
BluetoothDataManager bluetooth_data_manager;
LED led;

int led_times_x;

void App::init()
{
    serial_printf("[INFO] App# Program started.\n");
    update_mode_by_DIP_switch(); // DIPスイッチでモード変更

    // ********** CAN **********
    can2.init();
    can3.init();
    serial_printf("[INFO] App# CAN initialized.\n");

    // ********** YamaHex **********
    if (app_configure::enable::yama_hex)
    {
        can3.sendHexInit(0);
        serial_printf("[INFO] App# YamaHex init signal send.\n");
    }

    // ********** LAN **********
    if (app_configure::enable::lan)
    {
        udp_data_manager.init(is_controller); // コントローラーの場合とロボット１の場合でIPアドレス等が変化する。
        serial_printf("[INFO] App# LAN initialized.\n");
    }

    // ********** WiSUN **********
    if (app_configure::enable::wisun)
    {
        wisun_peripheral.DMAinit();
        wisun_peripheral.BP35C5Reset();
        serial_printf("[INFO] App# Wi-SUN initialized.\n");
    }

    // ********** debug **********
    if (app_configure::task::wisun_debugger)
    {
        serial_printf("[INFO] App# Wi-SUN-debugger initialized\n");
    }

    // ********** PS3 ************
    if (app_configure::enable::bluetooth)
    {
        serial_peripheral.init();
        serial_printf("[INFO] App# Serial initialized\n");
    }

    // ********** RGBLED ***********
    if (app_configure::enable::rgb_led && (is_bridge || is_panjan))
    {
        HAL_TIM_Base_Start_IT(&htim6); // 100Hzのタイマーを開始して、RGBLEDを光らせる。
        serial_printf("[INFO] App# RGBLED-timer started\n");
    }

    HAL_GPIO_WritePin(LED_READY_GPIO_Port, LED_READY_Pin, GPIO_PIN_SET); // 緑色のLEDを点灯
}

void App::loop()
{
    update_mode_by_DIP_switch(); // DIPスイッチでモード変更

    //    ********************** 受信等 **************************** //
    if (app_configure::enable::wisun) // WiSUN受信する設定の場合
    {
        uint16_t wisun_rx_data_size = wisun_peripheral.getLineData(wisun_line_data_buffer);
        if (wisun_rx_data_size > 0)
        {
            if (wisun_data_manager.isUdpPacket(wisun_line_data_buffer)) // UDPの受信かどうか判断する
            {
                char rx_ip_address[WISUN_MAX_IP_ADDRESS_LENGTH];
                char wisun_udp_data[WISUN_MAX_UDP_DATA_SIZE];
                uint16_t wisun_udp_data_size = wisun_data_manager.processUdpPacket(wisun_line_data_buffer, rx_ip_address, wisun_udp_data);
                if (wisun_udp_data_size > 0)
                {
                    wisun_data_manager.classifyUdpData(wisun_udp_data, wisun_udp_data_size); // 分類してget~関数で取り出せるようにする
                }
            }
            else if (app_configure::task::wisun_debugger)
            {
                serial_printf("%s\n", wisun_line_data_buffer);
            }
        }
    }
    if (app_configure::task::wisun_debugger) // WiSUN debuggerが有効の場合
    {
        if (udp_data_manager.get_debug_packet(&debug_kind, debug_data, &debug_data_size)) // デバッグパケットを受信
        {
            if (debug_kind == udp_configure::debug::kind_wisun) // 送られてきたデータがwisun debuggerの場合
            {
                wisun_data_manager.sendData(debug_data, debug_data_size); // wisunでそのまま送信
                if (app_configure::mode::debug)
                    serial_printf("[DEBUG]App# Wi-SUN send : %s\n", debug_data);
            }
        }
    }
    if (app_configure::enable::bluetooth) // ESP32によるBluetoothが有効の場合
    {
        uint16_t rx_data_size = serial_peripheral.recv(bluetooth_raw_data_buffer);
        while (rx_data_size > serial_packetter.META_DATA_SIZE)
        {
            rx_data_size -= serial_packetter.decodeSerialPacket(bluetooth_raw_data_buffer, rx_data_size, bluetooth_data_buffer, &bluetooth_data_size);
            if (bluetooth_data_size > 0)
            {
                bluetooth_data_manager.classifyData(bluetooth_data_buffer, rx_data_size);
            }
        }
    }

    // **************************** 制御 ************************
    if (is_controller) // コントローラー
    {
        /* コントローラーモード */
        if (udp_data_manager.get_controller_packet(&controller_packet)) // LANでキーボードの状態を取得
        {
            wisun_data_manager.sendControllerPacket(controller_packet); // WiSUNで送信
            if (app_configure::enable::bluetooth)
            {
                uint8_t tx_data_size = serial_packetter.encodeSerialPacket(controller_packet.raw, sizeof(controller_t), bluetooth_raw_data_buffer);
                serial_peripheral.send(bluetooth_raw_data_buffer, tx_data_size);
            }
            if (app_configure::mode::debug)
                serial_printf("[DEBUG]App# Control data send to Wi-SUN : %x%x%x\n", controller_packet.raw[0], controller_packet.raw[1], controller_packet.raw[2]);
        }

        if (app_configure::task::pc_to_wisun)
        {
            if (udp_data_manager.get_robot2_packet(&robot2_packet)) // LANでRobot2パケットを取得
            {
                robot2_wisun_packet.raw[0] = robot2_packet.raw[0];               // パンジャンの制御値をパケットに入れる
                if (is_panjan)                                                   // パンジャン
                    wisun_data_manager.sendRobot2Packet(robot2_wisun_packet, 3); // WiSUNでR2パケットを送信

                robot2_wisun_packet.raw[0] = robot2_packet.raw[1];               // 橋の制御値をパケットに入れる
                if (is_bridge)                                                   // 橋のIPが予備モード
                    wisun_data_manager.sendRobot2Packet(robot2_wisun_packet, 4); // WiSUNでR2パケットを送信

                if (app_configure::mode::debug)
                    serial_printf("[DEBUG]App# Robot2 packet send : %x, %x\n", robot2_packet.raw[0], robot2_packet.raw[1]);
            }
        }
    }
    else // R1 or R2
    {
        if (is_bridge && !is_panjan)
        {
            /* 橋モード */
            if (app_configure::enable::bluetooth)
            {
                if (bluetooth_data_manager.getRobot2Packet(&robot2_wisun_packet))
                {
                    if (app_configure::mode::debug)                                                                // デバッグモードの場合
                        serial_printf("[DEBUG]App# Robot2 Bluetooth recieved : %x\n", robot2_wisun_packet.raw[0]); // 受信した1byteのデータを出力
                    robot2_controller.manual_control(&robot2_wisun_packet);                                        // 制御値を計算しCAN通信を用いてロボットを動かす。
                    live_bluetooth_robot2 = true;
                }
            }
            if (app_configure::task::wisun_to_can) // WiSUNからCAN通信へ処理する設定の場合
            {
                if (wisun_data_manager.getRobot2Packet(&robot2_wisun_packet)) // WiSUNでRobot2パケット取得
                {
                    if (app_configure::mode::debug)                                                             // デバッグモードの場合
                        serial_printf("[DEBUG]App# Robot2 Wi-SUN recieved : %x\n", robot2_wisun_packet.raw[0]); // 受信した1byteのデータを出力
                    if (!live_bluetooth_robot2)
                        robot2_controller.manual_control(&robot2_wisun_packet); // 制御値を計算しCAN通信を用いてロボットを動かす。
                    live_bluetooth_robot2 = false;
                }
            }
        }
        else if (is_panjan && !is_bridge)
        {
            /* パンジャンモード*/
            if (app_configure::task::wisun_to_can) // WiSUNからCAN通信へ処理する設定の場合
            {
                if (app_configure::enable::wisun) // WiSUNからCAN通信へ処理する設定の場合
                {
                    if (wisun_data_manager.getRobot2Packet(&robot2_wisun_packet)) // WiSUNでRobot2パケット取得
                    {
                        if (app_configure::mode::debug)                                                             // デバッグモードの場合
                            serial_printf("[DEBUG]App# Robot2 Wi-SUN recieved : %x\n", robot2_wisun_packet.raw[0]); // 受信した1byteのデータを出力
                        robot2_controller.manual_control(&robot2_wisun_packet);                                     // 制御値を計算しCAN通信を用いてロボットを動かす。
                    }
                }
            }
        }
        else if (!is_bridge && !is_panjan)
        {
            /* R1モード */
            if (app_configure::task::wisun_to_pc)
            {
                if (app_configure::enable::bluetooth)
                {
                    if (bluetooth_data_manager.getControllerPacket(&controller_packet))
                    {
                        udp_data_manager.send_controller_packet(controller_packet); // LANでControllerパケット送信
                        live_bluetooth_controller = true;
                        if (app_configure::mode::debug)
                            serial_printf("[DEBUG]App# Control packet send to LAN : %x%x%x\n", controller_packet.raw[0], controller_packet.raw[1], controller_packet.raw[2]);
                    }
                }
                if (app_configure::enable::wisun)
                {
                    if (wisun_data_manager.getControllerPacket(&controller_packet)) // WiSUNでControllerパケット取得
                    {
                        if (!live_bluetooth_controller)
                            udp_data_manager.send_controller_packet(controller_packet); // LANでControllerパケット送信
                        live_bluetooth_controller = false;
                        if (app_configure::mode::debug)
                            serial_printf("[DEBUG]App# Control packet send to LAN : %x%x%x\n", controller_packet.raw[0], controller_packet.raw[1], controller_packet.raw[2]);
                    }
                }
            }
            if (app_configure::task::pc_to_can)
            {
                if (udp_data_manager.get_can_packet(&can_udp_packet)) // LANでCANパケット取得
                {
                    decodeCanID(can_udp_packet.can.id, &can_dir, &can_dev, &can_dev_id, &can_data_name); // CAN IDをデコード
                    if (can_dir == can_config::dir::to_slave)
                    {
                        if (can_dev == can_config::dev::motor_driver) // モータードライバ
                        {
                            can2.sendPacket(can_udp_packet.can.id, can_udp_packet.can.data, can_udp_packet.can.dlc); // CAN2に送信
                            if (can_data_name == can_config::data_name::md::targets)
                            {
                                motor_state = can_udp_packet.can.data[0] | can_udp_packet.can.data[1] | can_udp_packet.can.data[2] |
                                              can_udp_packet.can.data[3] | can_udp_packet.can.data[4] | can_udp_packet.can.data[5] |
                                              can_udp_packet.can.data[6] | can_udp_packet.can.data[7];
                            }
                        }
                        else
                        {
                            can3.sendPacket(can_udp_packet.can.id, can_udp_packet.can.data, can_udp_packet.can.dlc);                      // CAN3に送信
                            if (can_dev == can_config::dev::solenoid_driver && can_data_name == can_config::data_name::solenoid::targets) // ソレノイドの出力
                            {
                                solenoid_state = can_udp_packet.can.data[0]; // LEDDriver用にソレノイドの状態を入れる。
                            }
                        }
                    }
                }
            }
            if (app_configure::task::can_to_pc)
            {
                if (can2.getToMasterPacket(&can_udp_packet)) // CAN2からパケット取得
                {
                    udp_data_manager.send_can_packet(can_udp_packet); // LANでCANパケット送信
                }
                if (can3.getToMasterPacket(&can_udp_packet)) // CAN3からパケット取得
                {
                    udp_data_manager.send_can_packet(can_udp_packet); // LANでCANパケット送信
                    // if (app_configure::mode::debug)
                    //     serial_printf("[DEBUG]App# Received CAN3 id:%d, data:%d\n", can_udp_packet.can.id, can_udp_packet.can.data[0]);
                }
            }
        }
        else
        {
            printf("[ERROR]App# Invalid mode error.");
        }
    }
    // **************** Indicator *********************
    led_driver_count++;
    if (led_driver_count > 20)
    {
        led_driver_count = 0;
        encodeLedTarget(0, is_emergency, solenoid_state, motor_state, 0);
        can3.sendLedTarget(0, led_driver_target);
    }
    led_blink();
    HAL_Delay(1);
}

void App::timer_rgb_led()
{
    serial_printf("ltx start\n");
    if (led_times_x >= 100)
    {
        led_times_x = 1;
        serial_printf("ltx reset\n");
    }
    else
    {
        led_times_x++;
    }
    /**************test*************/
    led.led_test(&hspi3);
    serial_printf("lt\n");
    /************rainbow************/
    /*if (led.led_times_x % 5 == 0)
    {
        led.led_rainbow(&hspi3);
    }*/
    /**************line*************/
    /*if (led.led_times_x % 2 == 0)
    {
        led.led_line(&hspi1);
    }*/
    /*************error*************/
    /*if (led.led_times_x % 5 == 0)
    {
        led.led_error(&hspi1);
    }*/
    /**************off**************/
    // led.led_off(&hspi1);
    /*************spread************/
    /*if (led.led_times_x % 50 == 0)
    {
        led.led_spread(&hspi3);
    }*/
}

void App::led_blink()
{
    count_led++;
    if (count_led > 1000) // 1000を敷居にLEDを点滅
    {
        count_led = 0;
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
    }
}

void App::update_mode_by_DIP_switch()
{
    if (HAL_GPIO_ReadPin(DIP_1_GPIO_Port, DIP_1_Pin) == GPIO_PIN_SET)
    {
        if (!is_controller)
        {
            serial_printf("[INFO] App# Changed mode to controller\n");
            udp_data_manager.init(true); // IPアドレスやMACアドレスを変更する
            serial_printf("[INFO] App# Changed ip\n");
            HAL_Delay(100);
        }
        is_controller = true;
    }
    else
    {
        if (is_controller)
        {
            serial_printf("[INFO] App# Changed mode to R1 / R2\n");
            udp_data_manager.init(false);
            serial_printf("[INFO] App# Changed ip\n");
            HAL_Delay(100);
        }
        is_controller = false;
    }
    if (HAL_GPIO_ReadPin(DIP_2_GPIO_Port, DIP_2_Pin) == GPIO_PIN_SET)
    {
        if (!is_panjan)
        {
            serial_printf("[INFO] App# Changed mode to panjan / ip address changed mode\n");
            HAL_Delay(100);
        }
        is_panjan = true;
    }
    else
    {
        if (is_panjan)
        {
            serial_printf("[INFO] App# Changed mode to NOT panjan / ip address nomal mode\n");
            HAL_Delay(100);
        }
        is_panjan = false;
    }
    if (HAL_GPIO_ReadPin(DIP_3_GPIO_Port, DIP_3_Pin) == GPIO_PIN_SET)
    {
        if (!is_bridge)
        {
            serial_printf("[INFO] App# Changed mode to bridge / ip address changed mode\n");
            HAL_Delay(100);
        }
        is_bridge = true;
    }
    else
    {
        if (is_bridge)
        {
            serial_printf("[INFO] App# Changed mode to NOT bridge / address nomal mode\n");
            HAL_Delay(100);
        }
        is_bridge = false;
    }
}

void App::CANCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can2.onReceiveTask(hfdcan, RxFifo0ITs);
    can3.onReceiveTask(hfdcan, RxFifo0ITs);
}