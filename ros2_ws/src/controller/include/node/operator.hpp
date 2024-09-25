/**
 * @file operator.hpp
 * @author Gento Aiba (GN10) & Iori Fukuda (hukudaiori)
 * @brief Linuxのキーボードの状態を読み取りTopicとUDPで送信するノード
 * @version 0.1
 * @date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include "main_msgs/msg/key_state.hpp"
#include "lib/key_code_configure.hpp"
#include "config/udp_data_configure.hpp"

class Operator : public rclcpp::Node
{
public:
    Operator();
    ~Operator();
    void timerKeyboardCallback();
    void timerTransmissionCallback();

private:
    rclcpp::Publisher<main_msgs::msg::KeyState>::SharedPtr pub_key_state_;
    rclcpp::TimerBase::SharedPtr timer_keyboard_;
    rclcpp::TimerBase::SharedPtr timer_transmission_;
    uint8_t key[3] = {0b00000000, 0b00000000, 0b00000000};
    controller_t key_state_;
    uint8_t udp_data_size_;
    std::string keyboard_event_path = "/dev/input/event3";
    uint32_t counter_of_wisun = 0;
    void eventToState(uint8_t code, uint8_t state);
    void publishKeyState();
    void declareParameters();
    void getParameters();
};