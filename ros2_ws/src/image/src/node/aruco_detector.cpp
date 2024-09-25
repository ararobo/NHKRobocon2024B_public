#include <node/aruco_detector.hpp>

ArucoDetector::ArucoDetector() : Node("aruco_detector")
{
    pub_aruco = this->create_publisher<main_msgs::msg::ArucoMarkers>("robot2/_1/aruco", 10);
    pub_image = this->create_publisher<sensor_msgs::msg::Image>("robot2/_1/aruco/image", 10);
    sub_image = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10, std::bind(&ArucoDetector::image_callback, this, std::placeholders::_1));
    sub_depth_image = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/depth/image_rect_raw", 10, std::bind(&ArucoDetector::depth_image_callback, this, std::placeholders::_1));
    sub_cam_info = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera/depth/camera_info", 10, std::bind(&ArucoDetector::camera_info_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "ArucoDetector is running");
}

ArucoDetector::~ArucoDetector()
{
    cv::destroyAllWindows();
}

void ArucoDetector::camera_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr rx_msg)
{
    RCLCPP_INFO(this->get_logger(), "Camera info received"); // ログメッセージを表示
    // カメラの内部パラメータを表示
    RCLCPP_INFO(this->get_logger(), "Camera Matrix: [%f, %f, %f, %f, %f, %f, %f, %f, %f]", rx_msg->k[0], rx_msg->k[1], rx_msg->k[2], rx_msg->k[3], rx_msg->k[4], rx_msg->k[5], rx_msg->k[6], rx_msg->k[7], rx_msg->k[8]);
    RCLCPP_INFO(this->get_logger(), "Distortion Coefficients: [%f, %f, %f, %f, %f]", rx_msg->d[0], rx_msg->d[1], rx_msg->d[2], rx_msg->d[3], rx_msg->d[4]);
}

void ArucoDetector::image_callback(sensor_msgs::msg::Image::SharedPtr rx_msg)
{
    RCLCPP_INFO(this->get_logger(), "Image received"); // ログメッセージを表示
    // ROS画像メッセージをOpenCVのMatに変換
    auto cv_img = cv_bridge::toCvShare(rx_msg, rx_msg->encoding);
    if (cv_img->image.empty()) // 画像が空の場合は関数を抜ける
    {
        RCLCPP_WARN(this->get_logger(), "Received empty image");
        return;
    }

    cv::Mat image = cv_img->image; // カラー画像を取得
    // 画像サイズを表示
    RCLCPP_INFO(this->get_logger(), "Image size: %d x %d", image.cols, image.rows);
    cv::imshow("Input Image", image); // ウィンドウに表示
    detect_aruco(image);
    // 距離を計算する処理
    image = drawDetected_aruco(image); // 出力画像を取得
    // 画像を送信
    sensor_msgs::msg::Image::SharedPtr tx_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", image).toImageMsg();
    pub_image->publish(*tx_msg);
}

void ArucoDetector::detect_aruco(cv::Mat image)
{
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
}

cv::Mat ArucoDetector::drawDetected_aruco(cv::Mat image)
{
    cv::Mat outputImage = image.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
    return outputImage;
}

void ArucoDetector::distance_aruco()
{
    // Arucoマーカーの情報を表示(位置)
    for (size_t i = 0; i < markerIds.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Marker ID: %d", markerIds[i]);
        for (int j = 0; j < 4; j++)
        {
            RCLCPP_INFO(this->get_logger(), "Corner %d: (%f, %f)", j, markerCorners[i][j].x, markerCorners[i][j].y);
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetector>());
    rclcpp::shutdown();
    return 0;
}

void get_xyz(rs2::depth_frame &depth_frame, int x, int y, float &x_world, float &y_world, float &z_world)
{
    // Get depth value at (x, y)
    uint16_t depth_value = depth_frame.get_distance(x, y);

    // Convert depth value to world coordinates
    rs2_intrinsics intr = depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    float depth_scale = 0.001f; // Default depth scale
    float pixel[] = {static_cast<float>(x), static_cast<float>(y)};
    float depth_in_meters = depth_value * depth_scale;

    // Calculate x, y, z in meters
    x_world = (pixel[0] - intr.ppx) / intr.fx * depth_in_meters;
    y_world = (pixel[1] - intr.ppy) / intr.fy * depth_in_meters;
    z_world = depth_in_meters;
}

void ArucoDetector::depth_image_callback(sensor_msgs::msg::Image::SharedPtr rx_msg)
{
    RCLCPP_INFO(this->get_logger(), "Depth image received"); // ログメッセージを表示
    // ROS画像メッセージをOpenCVのMatに変換
    auto cv_img = cv_bridge::toCvShare(rx_msg, rx_msg->encoding);
    if (cv_img->image.empty()) // 画像が空の場合は関数を抜ける
    {
        RCLCPP_WARN(this->get_logger(), "Received empty depth image");
        return;
    }

    cv::Mat depth_image = cv_img->image; // 深度画像を取得
    // 画像サイズを表示
    RCLCPP_INFO(this->get_logger(), "Depth image size: %d x %d", depth_image.cols, depth_image.rows);
}