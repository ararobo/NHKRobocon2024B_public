/**
 * @file robot1_sensor.hpp
 * @author Gento Aiba (GN10)
 * @brief ロボット１のセンサーの値メイン基板からUDPで受信してROS2のTopicに流すノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <rclcpp/rclcpp.hpp>
#include "config/udp_data_configure.hpp"
#include "config/can_data_configure.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

union int16_to_uint8
{
    uint8_t uint8[2];
    int16_t int16;
};

class Robot1Sensor : public rclcpp::Node
{
private:
    float field_length[2] = {5.0f, 3.0f};
    float robot1_length[2] = {0.52f};
    float sensor[4] = {0.0f};
    float tolerance[2] = {0.1f};
    float robot1_coord[2] = {0.0f};
    bool sensor_state[2] = {true};
    rclcpp::TimerBase::SharedPtr timer_can;
    rclcpp::TimerBase::SharedPtr timer_pub;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_distance;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_jyro;
    udp_can_t rx_udp_can_buffer;
    int16_to_uint8 raw_data[4];
    uint8_t can_direction;
    uint8_t can_device;
    uint8_t can_device_id;
    uint8_t can_data_name;
    float gyro_data = 0.0f;

public:
    Robot1Sensor();
    ~Robot1Sensor();

    void socketRecvCallback();
    void distancePublishCallback();
    float convertCanDataToDistance(int16_to_uint8 buffer);
    bool checkSensorValue(float field, float robot, float distance_low, float distance_high, float tolerance);
    float calculateRobotCoords(float field, float robot, float distance_low, float distance_high);
};
