/**
 * @file operator.cpp
 * @author Gento Aiba (GN10) & Iori Fukuda (hukudaiori)
 * @brief Linuxのキーボードの状態を読み取りTopicとUDPで送信するノード
 * @version 0.1
 * @date 2024-08-22
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "node/operator.hpp"
#include "lib/simple_udp.hpp"
#include "lib/linux_keyboard_driver.hpp"

SimpleUDP simple_udp;
KeyboardDriver keyboard_driver;

Operator::Operator() : Node("operator")
{
    udp_data_size_ = sizeof(controller_t);
    declareParameters();
    getParameters();
    if (keyboard_driver.init(keyboard_event_path.c_str()))
    {
        RCLCPP_INFO(this->get_logger(), "keyboard path is %s", keyboard_event_path.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "keyboard path is %s", keyboard_event_path.c_str());
        RCLCPP_ERROR(this->get_logger(), "please run 'sudo chmod +r %s' or change the path by parameter.", keyboard_event_path.c_str());
    }
    simple_udp.initSocket();
    simple_udp.setTxAddr(udp_configure::ip::mainboard_controller, udp_configure::port::controller);
    pub_key_state_ = this->create_publisher<main_msgs::msg::KeyState>("key_state", 1);
    timer_keyboard_ = this->create_wall_timer(std::chrono::microseconds(100), std::bind(&Operator::timerKeyboardCallback, this));
    timer_transmission_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Operator::timerTransmissionCallback, this));
}

void Operator::timerKeyboardCallback()
{
    uint8_t code;
    uint8_t state;
    keyboard_driver.get_event(&code, &state);
    eventToState(code, state);
}

Operator::~Operator()
{
    keyboard_driver.release();
    simple_udp.closeSocket();
}

void Operator::eventToState(uint8_t code, uint8_t state)
{
    if (state == 0 || state == 1)
    {
        if (code == key_code::w)
        {
            key_state_.data.w = state;
        }
        if (code == key_code::a)
        {
            key_state_.data.a = state;
        }
        if (code == key_code::s)
        {
            key_state_.data.s = state;
        }
        if (code == key_code::d)
        {
            key_state_.data.d = state;
        }
        if (code == key_code::t)
        {
            key_state_.data.t = state;
        }
        if (code == key_code::f)
        {
            key_state_.data.f = state;
        }
        if (code == key_code::g)
        {
            key_state_.data.g = state;
        }
        if (code == key_code::h)
        {
            key_state_.data.h = state;
        }
        if (code == key_code::i)
        {
            key_state_.data.i = state;
        }
        if (code == key_code::j)
        {
            key_state_.data.j = state;
        }
        if (code == key_code::k)
        {
            key_state_.data.k = state;
        }
        if (code == key_code::l)
        {
            key_state_.data.l = state;
        }
        if (code == key_code::m)
        {
            key_state_.data.m = state;
        }
        if (code == key_code::num_1)
        {
            key_state_.data.num_1 = state;
        }
        if (code == key_code::num_2)
        {
            key_state_.data.num_2 = state;
        }
        if (code == key_code::num_3)
        {
            key_state_.data.num_3 = state;
        }
        if (code == key_code::num_4)
        {
            key_state_.data.num_4 = state;
        }
        if (code == key_code::num_5)
        {
            key_state_.data.num_5 = state;
        }
        if (code == key_code::num_6)
        {
            key_state_.data.num_6 = state;
        }
        if (code == key_code::num_8)
        {
            key_state_.data.num_8 = state;
        }
        if (code == key_code::num_9)
        {
            key_state_.data.num_9 = state;
        }
        if (code == key_code::left)
        {
            key_state_.data.lotate_l = state;
        }
        if (code == key_code::right)
        {
            key_state_.data.lotate_r = state;
        }
        if (code == key_code::shift_l || code == key_code::shift_r)
        {
            key_state_.data.shift = state;
        }
    }
}

void Operator::timerTransmissionCallback()
{
    if (counter_of_wisun > 11)
    {
        simple_udp.sendPacket(key_state_.raw, udp_data_size_);
        counter_of_wisun = 0;
    }
    publishKeyState();
    counter_of_wisun++;
}

void Operator::publishKeyState()
{
    auto msg = main_msgs::msg::KeyState();
    msg.w = key_state_.data.w;
    msg.a = key_state_.data.a;
    msg.s = key_state_.data.s;
    msg.d = key_state_.data.d;
    msg.t = key_state_.data.t;
    msg.f = key_state_.data.f;
    msg.g = key_state_.data.g;
    msg.h = key_state_.data.h;
    msg.i = key_state_.data.i;
    msg.j = key_state_.data.j;
    msg.k = key_state_.data.k;
    msg.l = key_state_.data.l;
    msg.m = key_state_.data.m;
    msg.shift = key_state_.data.shift;
    msg.lotate_l = key_state_.data.lotate_l;
    msg.lotate_r = key_state_.data.lotate_r;
    msg.num_1 = key_state_.data.num_1;
    msg.num_2 = key_state_.data.num_2;
    msg.num_3 = key_state_.data.num_3;
    msg.num_4 = key_state_.data.num_4;
    msg.num_5 = key_state_.data.num_5;
    msg.num_6 = key_state_.data.num_6;
    msg.num_8 = key_state_.data.num_8;
    msg.num_9 = key_state_.data.num_9;
    pub_key_state_->publish(msg);
}

void Operator::declareParameters()
{
    this->declare_parameter("event_path", keyboard_event_path);
}

void Operator::getParameters()
{
    keyboard_event_path = this->get_parameter("event_path").as_string();
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Operator>());
    rclcpp::shutdown();
    return 0;
}
