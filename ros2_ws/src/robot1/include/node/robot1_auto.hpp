/**
 * @file robot1_auto.hpp
 * @author Gento Aiba (GN10) & Iori Fukuda (hukudaiori)
 * @brief ロボット１の制御用ノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <htmd_manager_msgs/msg/md_target.hpp>
#include <main_msgs/msg/key_state.hpp>
#include <std_msgs/msg/bool.hpp>
class Robot1Core : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_auto_movement;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_manual_movement;
    rclcpp::Publisher<htmd_manager_msgs::msg::MdTarget>::SharedPtr pub_md_target;
    rclcpp::Subscription<main_msgs::msg::KeyState>::SharedPtr sub_key_state;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_reset;
    double x;     // x方向の移動速度
    double y;     // y方向の移動速度
    double theta; // 回転速度
    bool manual = true;
    bool m_key_prev = false;

public:
    Robot1Core();
    ~Robot1Core();

    void sub_auto_movement_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void sub_manual_movement_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void key_state_callback(const main_msgs::msg::KeyState::SharedPtr msg);
    void move_pub();
};