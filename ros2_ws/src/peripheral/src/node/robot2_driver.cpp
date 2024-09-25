/**
 * @file robot2_driver.cpp
 * @author Aiba Gento (GN10)
 * @brief ロボット２の制御値をメイン基板に送信するノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <node/robot2_driver.hpp>
#include "lib/simple_udp.hpp"

SimpleUDP simple_udp;

Robot2Driver::Robot2Driver() : Node("robot2_driver")
{
    sub_robot2_control[0] = this->create_subscription<main_msgs::msg::Robot2Control>("robot2/_1/control", 10, std::bind(&Robot2Driver::robot2_control_callback_1, this, std::placeholders::_1));
    sub_robot2_control[1] = this->create_subscription<main_msgs::msg::Robot2Control>("robot2/_2/control", 10, std::bind(&Robot2Driver::robot2_control_callback_2, this, std::placeholders::_1));
    sub_robot2_control[2] = this->create_subscription<main_msgs::msg::Robot2Control>("robot2/_3/control", 10, std::bind(&Robot2Driver::robot2_control_callback_3, this, std::placeholders::_1));
    sub_robot2_control[3] = this->create_subscription<main_msgs::msg::Robot2Control>("robot2/_4/control", 10, std::bind(&Robot2Driver::robot2_control_callback_4, this, std::placeholders::_1));
    if (simple_udp.initSocket())
    {
        RCLCPP_INFO(this->get_logger(), "Socket initialized");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize socket");
    }
    simple_udp.setTxAddr(udp_configure::ip::mainboard_controller, udp_configure::port::robot2);
    timer = this->create_wall_timer(std::chrono::milliseconds(130), std::bind(&Robot2Driver::timer_callback, this));
}

Robot2Driver::~Robot2Driver()
{
    simple_udp.closeSocket();
}

void Robot2Driver::robot2_control_callback_1(const main_msgs::msg::Robot2Control::SharedPtr msg)
{
    robot2_data.data[0].mode = msg->mode;
    robot2_data.data[0].slow = msg->slow;
    robot2_data.data[0].forward = msg->forward;
    robot2_data.data[0].backward = msg->backward;
    robot2_data.data[0].right = msg->right;
    robot2_data.data[0].left = msg->left;
    robot2_data.data[0].action = msg->action;
    RCLCPP_DEBUG(this->get_logger(), "1 Mode: %d, Slow: %d, Forward: %d, Backward: %d, Right: %d, Left: %d, Action: %d", robot2_data.data[0].mode, robot2_data.data[0].slow, robot2_data.data[0].forward, robot2_data.data[0].backward, robot2_data.data[0].right, robot2_data.data[0].left, robot2_data.data[0].action);
}

void Robot2Driver::robot2_control_callback_2(const main_msgs::msg::Robot2Control::SharedPtr msg)
{
    robot2_data.data[1].mode = msg->mode;
    robot2_data.data[1].slow = msg->slow;
    robot2_data.data[1].forward = msg->forward;
    robot2_data.data[1].backward = msg->backward;
    robot2_data.data[1].right = msg->right;
    robot2_data.data[1].left = msg->left;
    robot2_data.data[1].action = msg->action;
    RCLCPP_DEBUG(this->get_logger(), "2 Mode: %d, Slow: %d, Forward: %d, Backward: %d, Right: %d, Left: %d, Action: %d", robot2_data.data[1].mode, robot2_data.data[1].slow, robot2_data.data[1].forward, robot2_data.data[1].backward, robot2_data.data[1].right, robot2_data.data[1].left, robot2_data.data[1].action);
}

void Robot2Driver::robot2_control_callback_3(const main_msgs::msg::Robot2Control::SharedPtr msg)
{
    robot2_data.data[2].mode = msg->mode;
    robot2_data.data[2].slow = msg->slow;
    robot2_data.data[2].forward = msg->forward;
    robot2_data.data[2].backward = msg->backward;
    robot2_data.data[2].right = msg->right;
    robot2_data.data[2].left = msg->left;
    robot2_data.data[2].action = msg->action;
    RCLCPP_DEBUG(this->get_logger(), "3 Mode: %d, Slow: %d, Forward: %d, Backward: %d, Right: %d, Left: %d, Action: %d", robot2_data.data[2].mode, robot2_data.data[2].slow, robot2_data.data[2].forward, robot2_data.data[2].backward, robot2_data.data[2].right, robot2_data.data[2].left, robot2_data.data[2].action);
}

void Robot2Driver::robot2_control_callback_4(const main_msgs::msg::Robot2Control::SharedPtr msg)
{
    robot2_data.data[3].mode = msg->mode;
    robot2_data.data[3].slow = msg->slow;
    robot2_data.data[3].forward = msg->forward;
    robot2_data.data[3].backward = msg->backward;
    robot2_data.data[3].right = msg->right;
    robot2_data.data[3].left = msg->left;
    robot2_data.data[3].action = msg->action;
    RCLCPP_DEBUG(this->get_logger(), "4 Mode: %d, Slow: %d, Forward: %d, Backward: %d, Right: %d, Left: %d, Action: %d", robot2_data.data[3].mode, robot2_data.data[3].slow, robot2_data.data[3].forward, robot2_data.data[3].backward, robot2_data.data[3].right, robot2_data.data[3].left, robot2_data.data[3].action);
}

void Robot2Driver::timer_callback()
{
    if (simple_udp.sendPacket(robot2_data.raw, sizeof(robot2_t)))
    {
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to send data");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot2Driver>());
    rclcpp::shutdown();
    return 0;
}