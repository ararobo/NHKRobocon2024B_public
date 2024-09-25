/**
 * @file solenoid_driver.cpp
 * @author Gento Aiba (GN10)
 * @brief 電磁弁の出力値をメイン基板に送信するノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "node/solenoid_driver.hpp"
#include "lib/simple_udp.hpp"
#include "config/can_data_configure.hpp"

SimpleUDP simple_udp;

SolenoidDriver::SolenoidDriver() : Node("solenoid_driver")
{
    simple_udp.initSocket();
    simple_udp.setTxAddr(udp_configure::ip::mainboard_robot1, udp_configure::port::udp_to_can);
    sub_solenoid_output = this->create_subscription<std_msgs::msg::UInt8>("solenoid/output", 10, std::bind(&SolenoidDriver::solenoid_output_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "solenoid driver start");
}

SolenoidDriver::~SolenoidDriver()
{
    simple_udp.closeSocket();
}

void SolenoidDriver::solenoid_output_callback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Solenoid output: %d", msg->data);
    udp_can_t tx_data;
    tx_data.can.id = encodeCanID(can_config::dir::to_slave, can_config::dev::solenoid_driver, 0, can_config::data_name::solenoid::targets);
    tx_data.can.data[0] = msg->data;
    tx_data.can.dlc = 1;
    simple_udp.sendPacket(tx_data.raw, sizeof(udp_can_t));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SolenoidDriver>());
    rclcpp::shutdown();
    return 0;
}
