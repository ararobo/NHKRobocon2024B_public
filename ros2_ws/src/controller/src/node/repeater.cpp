/**
 * @file repeater.cpp
 * @author Gento Aiba (GN10)
 * @brief キーボードの状態をUDPで受信してTopicで送信するノード
 * @version 0.1
 * @date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "node/repeater.hpp"
#include "lib/simple_udp.hpp"

SimpleUDP simple_udp;

Repeater::Repeater() : Node("repeater")
{
    simple_udp.initSocket();
    simple_udp.bindSocket(udp_configure::ip::default_route, udp_configure::port::controller);
    pub_key_state_ = this->create_publisher<main_msgs::msg::KeyState>("key_state", 10);
    sub_key_state_ = this->create_subscription<main_msgs::msg::KeyState>("key_state", 10, std::bind(&Repeater::subKeyStateCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Repeater::timerUDPtoTopicCallback, this));
}

Repeater::~Repeater()
{
    simple_udp.closeSocket();
}

void Repeater::timerUDPtoTopicCallback()
{
    if (simple_udp.recvPacket(controller_data_.raw, sizeof(controller_data_)) && !rx_key_state_flag_)
    {
        auto msg = main_msgs::msg::KeyState();
        msg.w = controller_data_.data.w;
        msg.a = controller_data_.data.a;
        msg.s = controller_data_.data.s;
        msg.d = controller_data_.data.d;
        msg.t = controller_data_.data.t;
        msg.f = controller_data_.data.f;
        msg.g = controller_data_.data.g;
        msg.h = controller_data_.data.h;
        msg.i = controller_data_.data.i;
        msg.j = controller_data_.data.j;
        msg.k = controller_data_.data.k;
        msg.l = controller_data_.data.l;
        msg.m = controller_data_.data.m;
        msg.shift = controller_data_.data.shift;
        msg.lotate_l = controller_data_.data.lotate_l;
        msg.lotate_r = controller_data_.data.lotate_r;
        msg.num_1 = controller_data_.data.num_1;
        msg.num_2 = controller_data_.data.num_2;
        msg.num_3 = controller_data_.data.num_3;
        msg.num_4 = controller_data_.data.num_4;
        msg.num_5 = controller_data_.data.num_5;
        msg.num_6 = controller_data_.data.num_6;
        msg.num_8 = controller_data_.data.num_8;
        msg.num_9 = controller_data_.data.num_9;
        pub_key_state_->publish(msg);
    }
    rx_key_state_flag_ = false;
}

void Repeater::subKeyStateCallback(const main_msgs::msg::KeyState::SharedPtr msg)
{
    rx_key_state_flag_ = true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Repeater>());
    rclcpp::shutdown();
    return 0;
}