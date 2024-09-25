/**
 * @file robot2_driver.hpp
 * @author Aiba Gento (GN10)
 * @brief ロボット２の制御値をメイン基板に送信するノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <main_msgs/msg/robot2_control.hpp>
#include "config/udp_data_configure.hpp"

class Robot2Driver : public rclcpp::Node
{
private:
    rclcpp::Subscription<main_msgs::msg::Robot2Control>::SharedPtr sub_robot2_control[4];
    rclcpp::TimerBase::SharedPtr timer;
    robot2_t robot2_data;

public:
    Robot2Driver();
    ~Robot2Driver();
    void robot2_control_callback_1(const main_msgs::msg::Robot2Control::SharedPtr msg);
    void robot2_control_callback_2(const main_msgs::msg::Robot2Control::SharedPtr msg);
    void robot2_control_callback_3(const main_msgs::msg::Robot2Control::SharedPtr msg);
    void robot2_control_callback_4(const main_msgs::msg::Robot2Control::SharedPtr msg);
    void timer_callback();
};