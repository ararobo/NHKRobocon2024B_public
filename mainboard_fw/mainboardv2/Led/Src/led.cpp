/**
 * @file led.cpp
 * @author Watanabe-Koichiro
 * @brief RGBLEDの制御用クラス
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "led.hpp"
#include "neopixel.hpp"

LED::LED(/* args */)
{
}

LED::~LED()
{
}
// 0
void LED::led_test(SPI_HandleTypeDef *hspi)
{
    Color color;
    setLed(4, 70, 30, 0, color);
    show(hspi);
}
// 1
void LED::led_rainbow(SPI_HandleTypeDef *hspi)
{
    Color color;
    if (led_standard_1 == led_pieces_1)
    {
        if (led_number_1 == 4)
        {
            led_number_1 = 1;
        }
        else
        {
            led_number_1++;
        }
    }
    if (led_number_1 == 1)
    {
        led_other_g_1 = 5;
        led_other_r_1 = 15;
        led_other_b_1 = 5;
    }
    else if (led_number_1 == 2)
    {
        led_other_g_1 = 10;
        led_other_r_1 = 0;
        led_other_b_1 = 0;
    }
    else if (led_number_1 == 3)
    {
        led_other_g_1 = 3;
        led_other_r_1 = 10;
        led_other_b_1 = 0;
    }
    else if (led_number_1 == 4)
    {
        led_other_g_1 = 5;
        led_other_r_1 = 5;
        led_other_b_1 = 5;
    }
    if (led_standard_1 - led_pieces_1 < 0)
    {
        setLed(LED_COUNT + led_standard_1 - led_pieces_1 + 1, led_other_g_1, led_other_r_1, led_other_b_1, color);
    }
    else
    {
        setLed(led_standard_1 - led_pieces_1, led_other_g_1, led_other_r_1, led_other_b_1, color);
    }
    show(hspi);
    if (led_standard_1 >= LED_COUNT)
    {
        led_standard_1 = 0;
    }
    else
    {
        led_standard_1++;
    }

    if (led_standard_1 - led_1 < 0)
    {
        for (led_times_1 = 1; led_times_1 <= led_pieces_1; led_times_1++)
        {
            setLed(LED_COUNT + led_standard_1 - led_1 + 1 - led_times_1, 10, 0, 10, color);
        }
    }
    else
    {
        setLed(led_standard_1 - led_1, 10, 0, 10, color);
    }
    show(hspi);
}
// 2
void LED::led_line(SPI_HandleTypeDef *hspi)
{
    Color color;
    // 1
    if (led_standard_2 - led_pieces_2 < 0)
    {
        setLed(LED_COUNT + led_standard_2 - led_pieces_2, led_other_g_2, led_other_r_2, led_other_b_2, color);
    }
    else
    {
        // setLed(led_standard_2 - led_pieces_2 + 1, 10, 0, 0, color);
        setLed(led_standard_2 - led_pieces_2, led_other_g_2, led_other_r_2, led_other_b_2, color);
    }
    // 2
    if (led_standard_2a - led_pieces_2 < 0)
    {
        setLed(LED_COUNT + led_standard_2a - led_pieces_2, led_other_g_2, led_other_r_2, led_other_b_2, color);
    }
    else
    {
        // setLed(led_standard_2 - led_pieces_2 + 1, 10, 0, 0, color);
        setLed(led_standard_2a - led_pieces_2, led_other_g_2, led_other_r_2, led_other_b_2, color);
    }
    show(hspi);
    // 1
    if (led_standard_2 + led_pieces_2)
    {
        if (led_standard_2 >= LED_COUNT)
        {
            led_standard_2 = 0;
        }
        else
        {
            led_standard_2++;
        }
    }
    // 2
    if (led_standard_2a + led_pieces_2)
    {
        if (led_standard_2a >= LED_COUNT)
        {
            led_standard_2a = 0;
        }
        else
        {
            led_standard_2a++;
        }
    }
    // 1
    if (led_standard_2 - led_2 < 0)
    {
        for (led_times_2 = 1; led_times_2 <= led_pieces_2; led_times_2++)
        {
            setLed(LED_COUNT + led_standard_2 - led_2 + 1 - led_times_2, led_main_g_2, led_main_r_2, led_main_b_2, color);
        }
    }
    else
    {
        setLed(led_standard_2 - led_2, led_main_g_2, led_main_r_2, led_main_b_2, color);
    }
    // 2
    if (led_standard_2a - led_2 < 0)
    {
        for (led_times_2 = 1; led_times_2 <= led_pieces_2; led_times_2++)
        {
            setLed(LED_COUNT + led_standard_2a - led_2 + 1 - led_times_2, led_main_g_2, led_main_r_2, led_main_b_2, color);
        }
    }
    else
    {
        setLed(led_standard_2a - led_2, led_main_g_2, led_main_r_2, led_main_b_2, color);
    }
}
// 3
void LED::led_error(SPI_HandleTypeDef *hspi)
{
    Color color;
    if (led_3 <= 20)
    {
        led_3++;
    }
    else
    {
        led_3 = 1;
    }
    if (led_3 >= 1 && led_3 <= 10)
    {
        for (led_standard_3 = 0; led_standard_3 <= LED_COUNT; led_standard_3++)
        {
            setLed(led_standard_3, 0, led_3 * 20, 0, color);
        }
    }
    if (led_3 >= 11 && led_3 <= 20)
    {
        for (led_standard_3 = 0; led_standard_3 <= LED_COUNT; led_standard_3++)
        {
            setLed(led_standard_3, 0, (20 - led_3) * 20, 0, color);
        }
    }
    show(hspi);
}
// 4
void LED::led_off(SPI_HandleTypeDef *hspi)
{
    Color color;
    for (led_standard_4 = 0; led_standard_4 <= LED_COUNT; led_standard_4++)
    {
        setLed(led_standard_4, 0, 0, 0, color);
    }
    show(hspi);
}
// 5
void LED::led_spread(SPI_HandleTypeDef *hspi)
{
    Color color;
    if (led_standard_5 >= led_maxlight_5)
    {
        led_standard_5 = led_center_5;
    }
    else
    {
        led_standard_5++;
    }
    setLed(led_standard_5, led_main_g_5, led_main_r_5, led_main_b_5, color);
    setLed(led_center_5 * 2 - led_standard_5 /**/ - 1, led_main_g_5, led_main_r_5, led_main_b_5, color);
    if (led_standard_5 - led_center_5 - led_pieces_5 < 0)
    {
        setLed(led_maxlight_5 + led_standard_5 - led_center_5 - led_pieces_5, led_other_g_5, led_other_r_5, led_other_b_5, color);
        setLed(0 - led_standard_5 + led_center_5 + led_pieces_5 /**/ - 1, led_other_g_5, led_other_r_5, led_other_b_5, color);
    }
    else
    {
        setLed(led_standard_5 - led_pieces_5, led_other_g_5, led_other_r_5, led_other_b_5, color);
        setLed(led_center_5 * 2 - led_standard_5 + led_pieces_5 /**/ - 1, led_other_g_5, led_other_r_5, led_other_b_5, color);
    }
    show(hspi);
}
