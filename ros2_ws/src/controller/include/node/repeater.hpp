/**
 * @file repeater.hpp
 * @author Gento Aiba (GN10)
 * @brief キーボードの状態をUDPで受信してTopicで送信するノード
 * @version 0.1
 * @date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <rclcpp/rclcpp.hpp>
#include <main_msgs/msg/key_state.hpp>
#include "config/udp_data_configure.hpp"

class Repeater : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<main_msgs::msg::KeyState>::SharedPtr pub_key_state_;
    rclcpp::Subscription<main_msgs::msg::KeyState>::SharedPtr sub_key_state_;
    controller_t controller_data_;
    bool rx_key_state_flag_ = true; // コントローラーとのLANがつながっていてkey_stateが受信される時はUDPtoTopicの中継しない

public:
    Repeater();
    ~Repeater();
    void timerUDPtoTopicCallback();
    void subKeyStateCallback(const main_msgs::msg::KeyState::SharedPtr msg);
};