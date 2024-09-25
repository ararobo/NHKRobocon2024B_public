/**
 * @file robot1_core.cpp
 * @author Iori Fukuda (hukudaiori)
 * @brief ロボット１の手動制御用コード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <node/robot1_core.hpp>
#define THREE_WHEEL 0

Robot1Core::Robot1Core() : Node("robot1_core")
{
    pub_md_target = this->create_publisher<htmd_manager_msgs::msg::MdTarget>("md/target", 10);
    pub_solenoid = this->create_publisher<std_msgs::msg::UInt8>("solenoid/output", 10);
    sub_key_state = this->create_subscription<main_msgs::msg::KeyState>("key_state", 10, std::bind(&Robot1Core::key_state_callback, this, std::placeholders::_1));
    timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Robot1Core::timer_callback, this));
}

Robot1Core::~Robot1Core()
{
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
 * @brief ロボットの投擲指令を受け取るコールバック関数
 *
 * @param msg 投擲指令を表すBoolメッセージ
 */
void Robot1Core::throw_solenoid()
{
    auto throw_msg = std_msgs::msg::UInt8();
    throw_msg.data = throw_;
    pub_solenoid->publish(throw_msg);
}

void Robot1Core::key_state_callback(const main_msgs::msg::KeyState::SharedPtr msg)
{
    if (msg->w)
        wasd_[0] = true;
    else
        wasd_[0] = false;
    if (msg->a)
        wasd_[1] = true;
    else
        wasd_[1] = false;
    if (msg->s)
        wasd_[2] = true;
    else
        wasd_[2] = false;
    if (msg->d)
        wasd_[3] = true;
    else
        wasd_[3] = false;
    if (msg->shift)
        shift_ = true;
    else
        shift_ = false;
    if (msg->num_1)
        throw_key_ = true;
    else
        throw_key_ = false;
    if (msg->lotate_l)
        rotate_key_[0] = true;
    else
        rotate_key_[0] = false;
    if (msg->lotate_r)
        rotate_key_[1] = true;
    else
        rotate_key_[1] = false;
}

void Robot1Core::timer_callback()
{
    x = 0;     // xの数値を０に初期化
    y = 0;     // ｙの数値を０に初期化
    theta = 0; // thetaの値を0に初期化
    if (throw_key_ && !throw_key_prev_)
    {
        throw_ = !throw_;
    }
    throw_key_prev_ = throw_key_;
    if (wasd_[0])
    {
        if (shift_)
        {
            y = y + min_speed; // shiftが押されていた場合は0.5を足す
        }
        else
        {
            y = y + max_speed; // 押されていない場合は1を足す
        }
    }
    if (wasd_[2])
    {
        if (shift_)
        {
            y = y - min_speed; // shiftが押されていた場合は0.5を引く
        }
        else
        {
            y = y - max_speed; // 押されていない場合は1を引く
        }
    }
    if (wasd_[1])
    {
        if (shift_)
        {
            x = x - min_speed; // shiftがいされていたら0.5を引く
        }
        else
        {
            x = x - max_speed; // 押されていなければ1を引く
        }
    }
    if (wasd_[3])
    {
        if (shift_)
        {
            x = x + min_speed; // 以下同文
        }
        else
        {
            x = x + max_speed; // 以下同文
        }
    }
    if (rotate_key_[0])
    {
        if (shift_)
        {
            theta = theta + min_theta_speed;
        }
        else
        {
            theta = theta + max_theta_speed;
        }
    }
    if (rotate_key_[1])
    {
        if (shift_)
        {
            theta = theta - min_theta_speed;
        }
        else
        {
            theta = theta - max_theta_speed;
        }
    }
    move_pub();
    throw_solenoid();
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot1Core>());
    rclcpp::shutdown();
    return 0;
}
