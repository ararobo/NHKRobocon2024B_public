/**
 * @file app.cpp
 * @author Koichiro Watanabe (Watanabe-Koichiro) & Gento Aiba (GN10)
 * @brief LEDドライバーのメインプログラム
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "app.hpp"
#include "led.hpp"
#include "can_data_manager.hpp"
#include "led_target_formatter.hpp"

LED led;
CANDataManager can;

App::App()
{
}

void App::init()
{
    can.init();                    // CAN通信を開始
    HAL_TIM_Base_Start_IT(&htim6); // LEDのタイマーをスタート
}

void App::loop()
{
    // 0. led.led_test(&hspi1);
    // 1. led.led_rainbow(&hspi1);
    // 2. led.led_line(&hspi1);
    // 3. led.led_error(&hspi1);
    // 4. led.led_off(&hspi1);
    // 5. led.led_spread(&hspi1);
    if (is_red_team)
    {
        /*color_red base*/
        /*line*/
        led.led_main_g_2 = 0;
        led.led_main_r_2 = 100;
        led.led_main_b_2 = 0;
        led.led_main_g_2a = 0;
        led.led_main_r_2a = 100;
        led.led_main_b_2a = 0;
        led.led_other_g_2 = 70;
        led.led_other_r_2 = 30;
        led.led_other_b_2 = 0;
        /*spread*/
        led.led_main_g_5 = 0;
        led.led_main_r_5 = 100;
        led.led_main_b_5 = 0;
        led.led_other_g_5 = 70;
        led.led_other_r_5 = 30;
        led.led_other_b_5 = 0;
    }
    else
    {
        /*color_blue base*/
        /*line*/
        led.led_main_g_2 = 0;
        led.led_main_r_2 = 0;
        led.led_main_b_2 = 100;
        led.led_main_g_2a = 0;
        led.led_main_r_2a = 0;
        led.led_main_b_2a = 100;
        led.led_other_g_2 = 50;
        led.led_other_r_2 = 0;
        led.led_other_b_2 = 50;
        /*spread*/
        led.led_main_g_5 = 0;
        led.led_main_r_5 = 0;
        led.led_main_b_5 = 100;
        led.led_other_g_5 = 50;
        led.led_other_r_5 = 0;
        led.led_other_b_5 = 50;
    }
    if (solenoid_state)
    {
        led.led_main_g_2a = led.led_other_g_2;
        led.led_main_r_2a = led.led_other_r_2;
        led.led_main_b_2a = led.led_other_b_2;
        led.led_line(&hspi1);
        led.led_error(&hspi3);
        HAL_Delay(40);
    }
    else
    {
        if (is_emergency)
        {
            led.led_line(&hspi1);
            led.led_line(&hspi3);
            HAL_Delay(20);
        }
        else
        {
            if (motor_state)
            {
                led.led_spread(&hspi1);
                led.led_main_g_2a = led.led_other_g_2;
                led.led_main_r_2a = led.led_other_r_2;
                led.led_main_b_2a = led.led_other_b_2;
                led.led_line(&hspi3);
                HAL_Delay(40);
            }
            else
            {
                led.led_error(&hspi1);
                led.led_error(&hspi3);
                HAL_Delay(100);
            }
        }
    }
}

void App::timer()
{
}

void App::CANCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can.onReceiveTask(hfdcan, RxFifo0ITs);
    if (can.getLedTarget(&led_target))
        decodeLedTarget(led_target, &is_red_team, &is_emergency, &solenoid_state, &motor_state, &move_target);
}