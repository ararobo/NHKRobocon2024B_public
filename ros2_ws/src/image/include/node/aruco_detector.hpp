/**
 * @file aruco_detector.hpp
 * @author ai-tr
 * @brief アルゴマーカーを認識するノード
 * @version 0.1
 * @date 2024-09-25
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <main_msgs/msg/aruco_markers.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/aruco.hpp>
#include <librealsense2/rs.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class ArucoDetector : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cam_info;
    rclcpp::Publisher<main_msgs::msg::ArucoMarkers>::SharedPtr pub_aruco;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
    cv::aruco::DetectorParameters detectorParams;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

public:
    ArucoDetector();
    ~ArucoDetector();

    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void detect_aruco(cv::Mat image);
    cv::Mat drawDetected_aruco(cv::Mat image);
    void distance_aruco();
    int aruco_dis_x;
    int aruco_dis_y;
    int image_dis_x;
    int image_dis_y;
};