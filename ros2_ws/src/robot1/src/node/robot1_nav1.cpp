/**
 * @file robot1_nav1.cpp
 * @author Naoki Oba (poccha5)
 * @brief ロボット１の自動制御用ノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/bool.hpp"
#include "main_msgs/msg/key_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
class DebugNode : public rclcpp::Node
{
public:
    DebugNode() : Node("debug_node"), position_x(0.0), position_y(0.0), position_theta(0.0)
    {
        sub_pose2d = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "robot1/movement", 10, std::bind(&DebugNode::pose2d_callback, this, std::placeholders::_1));
        sub_throw_1 = this->create_subscription<std_msgs::msg::Bool>(
            "robot1/throw_1", 10, std::bind(&DebugNode::throw_1_callback, this, std::placeholders::_1));
        sub_throw_2 = this->create_subscription<std_msgs::msg::Bool>(
            "robot1/throw_2", 10, std::bind(&DebugNode::throw_2_callback, this, std::placeholders::_1));
        sub_key_state = this->create_subscription<main_msgs::msg::KeyState>(
            "key_state", 10, std::bind(&DebugNode::key_state_callback, this, std::placeholders::_1));
        pub_distance = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot1/distance", 10);
    }

private:
    void pose2d_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Pose2D: x=%f, y=%f, theta=%f", msg->x, msg->y, msg->theta);

        // 速度ベクトルから距離を計算
        double distance_x = msg->x * control_cycle;
        double distance_y = msg->y * control_cycle;
        double jyro = msg->theta;

        // 現在の位置に距離を加算
        position_x += distance_x;
        position_y += distance_y;
        position_theta += jyro;

        RCLCPP_INFO(this->get_logger(), "Calculated Position: x=%f, y=%f, theta=%f", position_x, position_y, position_theta);

        // 計算結果を元のノードに送り返す
        auto distance_msg = std_msgs::msg::Float32MultiArray();
        distance_msg.data[3] = position_x;
        distance_msg.data[2] = position_y;
        pub_distance->publish(distance_msg);
    }

    void throw_1_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Throw 1: %s", msg->data ? "true" : "false");
    }

    void throw_2_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Throw 2: %s", msg->data ? "true" : "false");
    }

    void key_state_callback(const main_msgs::msg::KeyState::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received KeyState: num_1=%s, a=%s", msg->num_1 ? "true" : "false", msg->a ? "true" : "false");
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_pose2d;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_throw_1;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_throw_2;
    rclcpp::Subscription<main_msgs::msg::KeyState>::SharedPtr sub_key_state;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_distance;
    const double control_cycle = 1; // 制御周期（例として0.01秒）

    // 現在の位置を保存する変数
    double position_x;
    double position_y;
    double position_theta;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DebugNode>());
    rclcpp::shutdown();
    return 0;
}
