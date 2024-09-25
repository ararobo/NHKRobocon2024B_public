/**
 * @file robot1_navi.cpp
 * @author Naoki Oba (poccha5)
 * @brief ロボット１の自動制御用ノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "node/robot1_navi.hpp"

Robot1Navi::Robot1Navi() : Node("robot1_navi")
{
    declareParameters();

    pub_is_home_right = this->create_publisher<std_msgs::msg::Bool>("robot1/is_home_right", 10); // スタートの位置に合わせてLEDの色を変更する。
    pub_pose2d = this->create_publisher<geometry_msgs::msg::Pose2D>("robot1/auto/movement", 10);
    sub_key_state = this->create_subscription<main_msgs::msg::KeyState>("key_state", 10, std::bind(&Robot1Navi::key_state_callback, this, std::placeholders::_1));
    sub_distance = this->create_subscription<std_msgs::msg::Float32MultiArray>("robot1/distance", 10, std::bind(&Robot1Navi::distance_callback, this, std::placeholders::_1));
    sub_jyro = this->create_subscription<std_msgs::msg::Float32>("robot1/jyro", 10, std::bind(&Robot1Navi::jyiro_callback, this, std::placeholders::_1));
    sub_reset = this->create_subscription<std_msgs::msg::Bool>("robot1/auto/reset", 10, std::bind(&Robot1Navi::reset_callback, this, std::placeholders::_1));
    timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Robot1Navi::timer_callback, this));
    RCLCPP_INFO(get_logger(), "Hello, world!");
    getParameters();
}

Robot1Navi::~Robot1Navi()
{
}
void Robot1Navi::declareParameters()
{
    this->declare_parameter("is_home_right", is_home_right);
    this->declare_parameter("x.Kp", Kp_x);
    this->declare_parameter("x.Ki", Ki_x);
    this->declare_parameter("x.Kd", Kd_x);
    this->declare_parameter("y.Kp", Kp_y);
    this->declare_parameter("y.Ki", Ki_y);
    this->declare_parameter("y.Kd", Kd_y);
    this->declare_parameter("j.Kp", Kp_j);
    this->declare_parameter("j.Ki", Ki_j);
    this->declare_parameter("j.Kd", Kd_j);
    this->declare_parameter("gool1.x", gool_x1);
    this->declare_parameter("gool1.y", gool_y1);
    this->declare_parameter("gool2.x", gool_x2);
    this->declare_parameter("gool2.y", gool_y2);
    this->declare_parameter("gool3.x", gool_x3);
    this->declare_parameter("gool3.y", gool_y3);
    this->declare_parameter("startpoint.x", start_point_x);
    this->declare_parameter("startpoint.y", start_point_y);
}

void Robot1Navi::getParameters()
{
    is_home_right = this->get_parameter("is_home_right").as_bool();
    Kp_x = this->get_parameter("x.Kp").as_double();
    Ki_x = this->get_parameter("x.Ki").as_double();
    Kd_x = this->get_parameter("x.Kd").as_double();
    Kp_y = this->get_parameter("y.Kp").as_double();
    Ki_y = this->get_parameter("y.Ki").as_double();
    Kd_y = this->get_parameter("y.Kd").as_double();
    Kp_j = this->get_parameter("j.Kp").as_double();
    Ki_j = this->get_parameter("j.Ki").as_double();
    Kd_j = this->get_parameter("j.Kd").as_double();
    gool_x1 = this->get_parameter("gool1.x").as_double();
    gool_y1 = this->get_parameter("gool1.y").as_double();
    gool_x2 = this->get_parameter("gool2.x").as_double();
    gool_y2 = this->get_parameter("gool2.y").as_double();
    gool_x3 = this->get_parameter("gool3.x").as_double();
    gool_y3 = this->get_parameter("gool3.y").as_double();
    start_point_x = this->get_parameter("startpoint.x").as_double();
    start_point_y = this->get_parameter("startpoint.y").as_double();
    RCLCPP_INFO(this->get_logger(), "param x.Kp : %f", Kp_x);
    RCLCPP_INFO(this->get_logger(), "param x.Ki : %f", Ki_x);
    RCLCPP_INFO(this->get_logger(), "param x.Kd : %f", Kd_x);
    RCLCPP_INFO(this->get_logger(), "param y.Kp : %f", Kp_y);
    RCLCPP_INFO(this->get_logger(), "param y.Ki : %f", Ki_y);
    RCLCPP_INFO(this->get_logger(), "param y.Kd : %f", Kd_y);
    RCLCPP_INFO(this->get_logger(), "param j.Kp : %f", Kp_j);
    RCLCPP_INFO(this->get_logger(), "param j.Kp : %f", Ki_j);
    RCLCPP_INFO(this->get_logger(), "param j.Kp : %f", Kd_j);
    RCLCPP_INFO(this->get_logger(), "param gool1.x : %f", gool_x1);
    RCLCPP_INFO(this->get_logger(), "param gool1.y : %f", gool_y1);
    RCLCPP_INFO(this->get_logger(), "param gool2.x : %f", gool_x2);
    RCLCPP_INFO(this->get_logger(), "param gool2.y : %f", gool_y2);
    RCLCPP_INFO(this->get_logger(), "param gool3.x : %f", gool_x3);
    RCLCPP_INFO(this->get_logger(), "param gool3.y : %f", gool_y3);
    RCLCPP_INFO(this->get_logger(), "param startpoint.x : %f", start_point_x);
    RCLCPP_INFO(this->get_logger(), "param startpoint.y : %f", start_point_y);
    auto msg = std_msgs::msg::Bool();
    if (is_home_right)
    {
        msg.data = true;
        pub_is_home_right->publish(msg);
    }
    else
    {
        msg.data = false;
        pub_is_home_right->publish(msg);
    }
}
float Robot1Navi::saturate(float input, float min, float max)
{

    float output = 0.0f;
    if (input > max)
    {
        output = max;
    }
    else if (input < min)
    {
        output = min;
    }
    else
    {
        output = input;
    }
    return output;
}

float Robot1Navi::PID(float target, float now_value, float Kp, float Ki, float Kd)
{

    RCLCPP_INFO(get_logger(), "I heard:PID [%f,%f]", target, now_value);
    // ターゲットが0の場合は出力を0にする //

    i_out = saturate(i_out, -1.0f, 1.0f);
    // i_out = saturate(i_out, -1.0f, 1.0f);                      // I制御の出力を制限
    float error = target - now_value;                          // エラーを計算
    float p_out = error;                                       // P制御
    i_out += error * (float)control_cycle;                     // I制御
    float d_out = (error - prev_error) / (float)control_cycle; // D制御
    prev_error = error;
    RCLCPP_INFO(get_logger(), "I heard:PID [%f]", p_out * Kp + i_out * Ki + d_out * Kd); // 前回のエラーを保存
    return p_out * Kp + i_out * Ki + d_out * Kd;
}
/// @brief
/// @param msg

void Robot1Navi::key_state_callback(const main_msgs::msg::KeyState::SharedPtr msg)
{
    if (msg->a)
    {

        gool = true;
        gool_x = gool_x1;
        gool_y = gool_y1;
        count = 0;
        RCLCPP_INFO(get_logger(), "gool_point1_set");
        RCLCPP_INFO(get_logger(), "I heard: goolx1_%f gooly1_%f ", gool_x, gool_y);

        RCLCPP_INFO(get_logger(), "1");
    }

    if (msg->s)
    {

        gool = true;
        gool_x = start_point_x;
        gool_y = start_point_y;
        count = 0;
        RCLCPP_INFO(get_logger(), "gool_point2_set");
        RCLCPP_INFO(get_logger(), "I heard: goolx1_%f gooly1_%f ", gool_x, gool_y);

        RCLCPP_INFO(get_logger(), "2");
    }
    if (msg->d)
    {

        gool = true;
        gool_x = gool_x2;
        gool_y = gool_y2;
        count = 0;
        RCLCPP_INFO(get_logger(), "gool_point3_set");
        RCLCPP_INFO(get_logger(), "I heard: goolx1_%f gooly1_%f ", gool_x, gool_y);

        RCLCPP_INFO(get_logger(), "3");
    }
    if (msg->w)
    {

        gool = true;
        gool_x = gool_x3;
        gool_y = gool_y3;
        count = 0;
        RCLCPP_INFO(get_logger(), "gool_point3_set");
        RCLCPP_INFO(get_logger(), "I heard: goolx1_%f gooly1_%f ", gool_x, gool_y);

        RCLCPP_INFO(get_logger(), "3");
    }
    if (msg->num_1)
    {
        enterkey = true;
        RCLCPP_INFO(get_logger(), "a");
    }
    else
    {
        enterkey = false;
    }
    if (rotate_key_[0])
    {

        theta = +max_theta_speed;
    }
    if (rotate_key_[1])
    {

        theta = -max_theta_speed;
    }
}
void Robot1Navi::reset_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Hello, world!");
    reset = msg->data;
    auto msg1 = geometry_msgs::msg::Pose2D();
    msg1.x = 0;
    msg1.y = 0;
    msg1.theta = theta; // メッセージにデータを代入
    pub_pose2d->publish(msg1);
    count = 0;
    gool = false;
}
void Robot1Navi::distance_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    distance_right = msg->data[1];
    distance_back = msg->data[2];
    distance_left = msg->data[3];
    if (is_home_right)
    {
        distance_x = distance_right;
    }
    else
    {
        distance_x = distance_left;
    }
    distance_y = distance_back;

    RCLCPP_INFO(rclcpp::get_logger("pose2d_callback"), "I heard: distance_right, distance_back, distance_left [%f, %f, %f]", distance_right, distance_back, distance_left);
    RCLCPP_INFO(rclcpp::get_logger("pose2d_callback"), "I heard: distance_x, distance_y [%f, %f]", distance_x, distance_y);
}
void Robot1Navi::jyiro_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    jyro = msg->data;
}

float Robot1Navi::PID_vecter_spead_to_location(float spead)
{
    location = spead * (float)control_cycle;
    return location;
}
void Robot1Navi::timer_callback()
{

    if (gool == true)
    {
        RCLCPP_INFO(get_logger(), "I heard:count [%d]", count);
        RCLCPP_INFO(get_logger(), "I heard:distance [%f, %f]", distance_x, distance_y);
        RCLCPP_INFO(get_logger(), "I heard:distance [%f, %f,%f]", Kp_x, Kp_y, Kp_j);
        vecter[0] = PID(gool_x, distance_x, Kp_x, Ki_x, Kd_x);
        vecter[1] = PID(gool_y, distance_y, Kp_y, Ki_y, Kd_y);
        vecter[2] = PID(0, jyro, Kp_j, Ki_j, Kd_j);
        RCLCPP_INFO(get_logger(), "I heard: gool [%f, %f]", gool_x, gool_y);
        RCLCPP_INFO(get_logger(), "I heard: gool_erorre [%f, %f]", abs(gool_x - distance_x), abs(gool_y - distance_y));
        RCLCPP_INFO(get_logger(), "I heard:vecter [%f, %f]", vecter[0], vecter[1]);
        auto msg = geometry_msgs::msg::Pose2D(); // メッセージを定義
        if (is_home_right)
        {
            msg.x = -vecter[0];
        }
        else
        {
            msg.x = vecter[0];
        }
        msg.y = vecter[1];
        msg.theta = theta; // メッセージにデータを代入
        pub_pose2d->publish(msg);
    }
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Robot1Navi>());
    rclcpp::shutdown();
    return 0;
}
