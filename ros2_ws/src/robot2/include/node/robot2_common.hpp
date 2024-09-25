/**
 * @file robot2_common.hpp
 * @author Gento Aiba
 * @brief ロボット２の制御コード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <main_msgs/msg/key_state.hpp>
#include "lib/key_code_configure.hpp"
#include <main_msgs/msg/robot2_control.hpp>

class Robot2Core : public rclcpp::Node
{
private:
    rclcpp::Subscription<main_msgs::msg::KeyState>::SharedPtr sub_key_state_;
    rclcpp::Publisher<main_msgs::msg::Robot2Control>::SharedPtr pub_robot2_control_;
    rclcpp::TimerBase::SharedPtr timer_;
    main_msgs::msg::KeyState key_state;

public:
    Robot2Core();
    ~Robot2Core();
    void keyStateCallback(const main_msgs::msg::KeyState::SharedPtr msg);
    void timerCallback();
};