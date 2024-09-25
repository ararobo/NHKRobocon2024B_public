/**
 * @file ijkl.cpp
 * @author Gento Aiba (GN10)
 * @brief ロボット２をijklキーで操縦するノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "node/robot2_common.hpp"

Robot2Core::Robot2Core() : Node("ijkl")
{
    pub_robot2_control_ = this->create_publisher<main_msgs::msg::Robot2Control>("robot2/_2/control", 10);
    sub_key_state_ = this->create_subscription<main_msgs::msg::KeyState>("key_state", 10, std::bind(&Robot2Core::keyStateCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Robot2Core::timerCallback, this));
}

Robot2Core::~Robot2Core()
{
}

void Robot2Core::keyStateCallback(const main_msgs::msg::KeyState::SharedPtr msg)
{
    key_state = *msg;
}

void Robot2Core::timerCallback()
{
    main_msgs::msg::Robot2Control robot2_control;
    robot2_control.mode = 0;
    robot2_control.action = 0;
    if (key_state.num_8)
        robot2_control.action |= 0b1;
    if (key_state.num_9)
        robot2_control.action |= 0b10;
    robot2_control.slow = key_state.shift;
    robot2_control.forward = key_state.i;
    robot2_control.left = key_state.j;
    robot2_control.backward = key_state.k;
    robot2_control.right = key_state.l;
    pub_robot2_control_->publish(robot2_control);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot2Core>());
    rclcpp::shutdown();
    return 0;
}
