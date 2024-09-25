/**
 * @file robot1_auto.cpp
 * @author Gento Aiba (GN10)
 * @brief ロボット１の制御用ノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <node/robot1_auto.hpp>

Robot1Core::Robot1Core() : Node("robot1_auto")
{
    pub_md_target = this->create_publisher<htmd_manager_msgs::msg::MdTarget>("md/target", 10);
    sub_auto_movement = this->create_subscription<geometry_msgs::msg::Pose2D>("robot1/auto/movement", 10, std::bind(&Robot1Core::sub_auto_movement_callback, this, std::placeholders::_1));
    sub_manual_movement = this->create_subscription<geometry_msgs::msg::Pose2D>("robot1/manual/movement", 10, std::bind(&Robot1Core::sub_manual_movement_callback, this, std::placeholders::_1));
    sub_key_state = this->create_subscription<main_msgs::msg::KeyState>("key_state", 10, std::bind(&Robot1Core::key_state_callback, this, std::placeholders::_1));
    pub_reset = this->create_publisher<std_msgs::msg::Bool>("robot1/auto/reset", 10);
}

Robot1Core::~Robot1Core()
{
}

void Robot1Core::key_state_callback(const main_msgs::msg::KeyState::SharedPtr msg)
{
    if (!m_key_prev && msg->m)
    {
        manual = !manual;
        auto msg = std_msgs::msg::Bool();
        msg.data = !manual;
        pub_reset->publish(msg);
    }
    m_key_prev = msg->m;
}

/**
 * @brief ロボットの移動指令を受け取るコールバック関数
 *
 * @param msg x, y, thetaの値を持つPose2Dメッセージ
 */
void Robot1Core::move_pub()
{
    auto motor_msg = htmd_manager_msgs::msg::MdTarget();
    motor_msg.md_id = 0;             // モータID
    motor_msg.target = (-x + theta); // モータの目標速度をオムニ４輪の逆運動学から求める
    pub_md_target->publish(motor_msg);
    motor_msg.md_id = 1;
    motor_msg.target = (y + theta);
    pub_md_target->publish(motor_msg);
    motor_msg.md_id = 2;
    motor_msg.target = (x + theta);
    pub_md_target->publish(motor_msg);
    motor_msg.md_id = 3;
    motor_msg.target = (-y + theta);
    pub_md_target->publish(motor_msg); // モータへの指令を送信
}

/**
 * @brief タイマコールバック関数
 */

void Robot1Core::sub_auto_movement_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    if (!manual)
    {
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
        move_pub();
    }
}

/**
 * @brief タイマコールバック関数
 */

void Robot1Core::sub_manual_movement_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
    if (manual)
    {
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
        move_pub();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot1Core>());
    rclcpp::shutdown();
    return 0;
}
