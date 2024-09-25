/**
 * @file app.hpp
 * @author Gento Aiba (GN10) & Koichiro Watanabe (Watanabe-Koichiro)
 * @brief LEDドライバーのメインプログラム
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

class App
{
private:
    uint8_t led_target = 0;
    bool is_red_team = false;    /* スタートゾーンが右(優先度高) */
    bool is_emergency = false;   /* 問題が発生した(優先度高) */
    bool solenoid_state = false; /* 発射信号が出ている(優先度高) */
    bool motor_state = false;    /* モーターへ出力がある(優先度低) */
    uint8_t move_target = 0;     /* 自動移動時の目的地(優先度低) */
    /**********/
    int led_times = 0;

public:
    App();

    void init();
    void loop();
    void timer();
    void CANCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
};