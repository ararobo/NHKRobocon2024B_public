/**
 * @file robot1_navi.hpp
 * @author Naoki Oba (poccha5)
 * @brief ロボット１の自動制御用ノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <rclcpp/rclcpp.hpp>
#include "main_msgs/msg/key_state.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cmath>
#include <main_msgs/msg/key_state.hpp>
class Robot1Navi : public rclcpp::Node
{
private:
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pub_pose2d;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_distance;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_jyro;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<main_msgs::msg::KeyState>::SharedPtr sub_key_state;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_is_home_right;
    double Kp_x = 2.00;
    double Ki_x = 0.00;
    double Kd_x = 0.0;
    double Kp_y = 2.00;
    double Ki_y = 0.00;
    double Kd_y = 0.0;
    double Kp_j = 1.00;
    double Ki_j = 0.00;
    double Kd_j = 0.00;
    float prev_error = 0.0f;    // 前回のエラー
    int16_t prev_out = 0;       // 前回の出力
    uint16_t max_output = 3199; // 最大出力
    uint8_t control_cycle = 10;
    float i_out = 0.0f;
    float jyro = 0;
    float distance_front = 0;
    float distance_right = 0;
    float distance_left = 0;
    float distance_back = 0;
    float distance_x = 0;
    float distance_y = 0;
    float vecter[3] = {0, 0, 0};
    bool rotate_key_[2] = {false, false};
    bool start = false;
    bool gool = false;
    double gool_x = 0;
    double gool_y = 0;
    double gool_x1 = 0.0;
    double gool_y1 = 0.0;
    double gool_x2 = 0.0;
    double gool_y2 = 0.0;
    double gool_x3 = 0.0;
    double gool_y3 = 0.0;
    int count = 0;
    float location = 0;
    bool enterkey = false;
    float start_point_x = 0.1f;
    float start_point_y = 0.1f;
    float Tolerance = 0.07;
    bool is_home_right = false;
    bool reset = false;
    double theta;
    double max_theta_speed = 0.6;
    double min_theta_speed = 0.3;

public:
    Robot1Navi();
    ~Robot1Navi();
    float PID(float target, float now_value, float Kp, float Ki, float Kd);
    float saturate(float input, float min, float max);
    void key_state_callback(const main_msgs::msg::KeyState::SharedPtr msg);
    void timer_callback();
    void distance_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void jyiro_callback(const std_msgs::msg::Float32::SharedPtr msg);
    float PID_vecter_spead_to_location(float spead);
    void reset_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void getParameters();
    void declareParameters();
};