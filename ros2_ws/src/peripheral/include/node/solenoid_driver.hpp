/**
 * @file solenoid_driver.hpp
 * @author Gento Aiba (GN10)
 * @brief 電磁弁の出力値をメイン基板に送信するノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

class SolenoidDriver : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_solenoid_output;

public:
    SolenoidDriver();
    ~SolenoidDriver();

    void solenoid_output_callback(const std_msgs::msg::UInt8::SharedPtr msg);
};