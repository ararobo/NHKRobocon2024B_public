/**
 * @file robot1_core.hpp
 * @author Iori Hukuda (hukudaiori)
 * @brief ロボット１の手動制御用コード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <htmd_manager_msgs/msg/md_target.hpp>
#include <htmd_manager_msgs/msg/md_sensor.hpp>
#include <main_msgs/msg/key_state.hpp>

class Robot1Core : public rclcpp::Node
{
private:
    rclcpp::Subscription<main_msgs::msg::KeyState>::SharedPtr sub_key_state;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_throw;
    rclcpp::Publisher<htmd_manager_msgs::msg::MdTarget>::SharedPtr pub_md_target;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_solenoid;
    rclcpp::TimerBase::SharedPtr timer;
    double x;                                     // x方向の移動速度
    double y;                                     // y方向の移動速度
    double theta;                                 // 回転速度
    bool shift_ = false;                          // shiftキーの押下フラグ
    bool wasd_[4] = {false, false, false, false}; // w, a, s, dの押下フラグ
    bool rotate_key_[2] = {false, false};         // left, rightの押下フラグ
    double max_speed = 0.6;                       // 何もしてないときの速さ(移動)
    double min_speed = 0.3;                       // shiftを押しているときの速さ（移動）
    double max_theta_speed = 0.6;                 // 何もしてないときの速さ（回転）
    double min_theta_speed = 0.3;                 // shiftを押しているときの速さ（回転）
    bool weapon[4] = {false, false, false, false};
    uint8_t throw_ = 0b0;
    bool throw_key_ = false;
    bool throw_key_prev_ = false;

public:
    Robot1Core();
    ~Robot1Core();

    void key_state_callback(const main_msgs::msg::KeyState::SharedPtr msg);
    void timer_callback();

    void move_pub();
    void throw_solenoid();
};