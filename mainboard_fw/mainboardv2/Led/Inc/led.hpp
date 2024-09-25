/**
 * @file led.hpp
 * @author Watanabe-Koichiro
 * @brief RGBLEDの制御用クラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "main.h"

class LED
{
private:
    /*〇led_rainbow*/
    int led_times_1 = 0;
    int led_standard_1 = 0;
    int led_number_1;
    int led_1 = 0;
    int led_pieces_1 = -1 + /*→*/ 1;
    int led_other_g_1 = 0;
    int led_other_r_1 = 0;
    int led_other_b_1 = 0;
    /*〇led_line*/
    int led_times_2 = 0;
    int led_standard_2 = 0;
    int led_standard_2a = 40;
    int led_2 = 0;
    int led_pieces_2 = 8;
    int led_main_g_2 = 0;
    int led_main_r_2 = 100;
    int led_main_b_2 = 0;
    int led_other_g_2 = 10;
    int led_other_r_2 = 0;
    int led_other_b_2 = 10;
    /*〇led_error*/
    int led_standard_3 = 0;
    int led_3 = 0;
    /*〇led_off*/
    int led_standard_4 = 0;
    /*〇led_spread*/
    int led_standard_5 = 9;
    int led_pieces_5 = 3;
    int led_center_5 = 9;
    int led_main_g_5 = 0;
    int led_main_r_5 = 100;
    int led_main_b_5 = 0;
    int led_other_g_5 = 10;
    int led_other_r_5 = 0;
    int led_other_b_5 = 10;
    int led_maxlight_5 = 18;

public:
    LED(/* args */);
    ~LED();
    void led_test(SPI_HandleTypeDef *hspi);
    void led_rainbow(SPI_HandleTypeDef *hspi);
    void led_line(SPI_HandleTypeDef *hspi);
    void led_error(SPI_HandleTypeDef *hspi);
    void led_off(SPI_HandleTypeDef *hspi);
    void led_spread(SPI_HandleTypeDef *hspi);

    // int led_times_x;
};
