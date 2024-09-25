/**
 * @file robot1_sensor.cpp
 * @author Gento Aiba (GN10)
 * @brief ロボット１のセンサーの値メイン基板からUDPで受信してROS2のTopicに流すノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "node/robot1_sensor.hpp"
#include "lib/simple_udp.hpp"

SimpleUDP simple_udp;

Robot1Sensor::Robot1Sensor() : Node("robot1_sensor")
{
    simple_udp.initSocket();
    simple_udp.bindSocket(udp_configure::ip::default_route, udp_configure::port::udp_to_can);
    pub_distance = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot1/distance", 10);
    pub_jyro = this->create_publisher<std_msgs::msg::Float32>("robot1/jyro", 10);
    timer_can = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&Robot1Sensor::socketRecvCallback, this));
    timer_pub = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Robot1Sensor::distancePublishCallback, this));
}

Robot1Sensor::~Robot1Sensor()
{
    simple_udp.closeSocket();
}

void Robot1Sensor::socketRecvCallback()
{
    if (simple_udp.recvPacket(rx_udp_can_buffer.raw, sizeof(udp_can_t)))
    {
        decodeCanID(rx_udp_can_buffer.can.id, &can_direction, &can_device, &can_device_id, &can_data_name);
        if (can_device == can_config::dev::common && can_data_name == can_config::data_name::common::tof_sensor)
        {
            for (uint8_t i = 0; i < can_config::dlc::common::tof_sensor; i++)
            {
                raw_data[can_device_id - 1].uint8[i] = rx_udp_can_buffer.can.data[i];
            }
            // RCLCPP_INFO(this->get_logger(), "id :%d rx : %d", can_device_id, raw_data[can_device_id].int16);
        }
        if (can_device == can_config::dev::common && can_data_name == can_config::data_name::common::gyro)
        {
            int16_t raw_data_gyro = static_cast<int16_t>(rx_udp_can_buffer.can.data[0] | (rx_udp_can_buffer.can.data[1] << 8));
            gyro_data = (float)raw_data_gyro / 16.0f;
        }
    }
}

void Robot1Sensor::distancePublishCallback()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        sensor[i] = convertCanDataToDistance(raw_data[i]);
    }
    // if (!checkSensorValue(field_length[0], robot1_length[0], sensor[3], sensor[1], tolerance[0]))
    // {
    //     RCLCPP_WARN(this->get_logger(), "sensor value is not mach to x length. 4 : %f, 2 : %f", sensor[3], sensor[1]);
    // }
    // if (!checkSensorValue(field_length[1], robot1_length[1], sensor[0], sensor[2], tolerance[1]))
    // {
    //     RCLCPP_WARN(this->get_logger(), "sensor value is not mach to y length. 1 : %f, 3 : %f", sensor[0], sensor[2]);
    // }
    // RCLCPP_INFO(this->get_logger(), "coord x : %f, y : %f", robot1_coord[0], robot1_coord[1]);
    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data.resize(4);
    for (uint8_t i = 0; i < 4; i++)
    {
        msg.data[i] = sensor[i];
    }
    pub_distance->publish(msg);
    auto jyro_msgs = std_msgs::msg::Float32();
    if (0.0f <= gyro_data && gyro_data <= 360.0f) // 有効範囲内
    {
        if (gyro_data > 180.0f) // 180度より大きいときはマイナスとする。
        {
            jyro_msgs.data = (gyro_data - 360.0f) * M_PI / 180.0f;
        }
        else
        {
            jyro_msgs.data = gyro_data * M_PI / 180.0f;
        }
    }
    else // 範囲外（おそらくセンサがうまく動いていない）
    {
        jyro_msgs.data = 0.0f; // ロボットが暴走しないように0.0fとする。
    }
    pub_jyro->publish(jyro_msgs);
}

float Robot1Sensor::convertCanDataToDistance(int16_to_uint8 buffer)
{
    float distance = buffer.int16;
    distance /= 1000.0f;
    return distance;
}

bool Robot1Sensor::checkSensorValue(float field, float robot, float distance_high, float distance_low, float tolerance)
{
    float error = field - (robot + distance_high + distance_low);
    if (-tolerance <= error && error <= tolerance)
        return true;
    return false;
}

float Robot1Sensor::calculateRobotCoords(float field, float robot, float distance_high, float distance_low)
{
    float coord_rate = (robot / 2 + distance_low) / (robot + distance_low + distance_high);
    float robot_coord = field * coord_rate;
    return robot_coord;
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot1Sensor>());
    rclcpp::shutdown();
    return 0;
}
