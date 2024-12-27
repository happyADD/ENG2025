//
// Created by dcy on 24-12-26.
//

#include "usb_camera.h"

usb_camera::usb_camera() : usb_camera(rclcpp::NodeOptions()){}

usb_camera::usb_camera(const rclcpp::NodeOptions& options) : Node("usb_camera", options)
{
    capture_.open(0, cv::CAP_V4L2);
    if (!capture_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera.");
    }
    using namespace std::chrono_literals;
    declare_parameter("camera_index",0);
    declare_parameter("exposure_time", 1000.0);
    declare_parameter("gain", 15.0);
    declare_parameter("frame_rate", 250.0);
    declare_parameter("width", 1440);
    declare_parameter("height", 1080);
    declare_parameter("offset_x", 0);
    declare_parameter("offset_y", 0);
    declare_parameter("bit_depth", "Bits_8");
    declare_parameter("k", std::vector<double>({
                          1786.972005, 0.000000, 728.630415,
                          0.000000, 1775.199982, 551.279527,
                          0.000000, 0.000000, 1.000000,
                      }));
    declare_parameter("d", std::vector<double>({-0.130158, 0.318803, -0.003765, 0.001686, 0.000000}));


    get_parameter("k", k);
    get_parameter("d", d);


    imagePublisher_ = create_publisher<RosImage>("image_raw", 10);
    capturedPublisher_ = imagePublisher_;
    cameraInfoPublisher_ = create_publisher<CameraInfo>("camera_info", 1);
    timer_ = create_wall_timer(1s / get_parameter("frame_rate").as_double(),
                          std::bind(&usb_camera::cameraCallback, this));


    // open_device(); //begin to get stream and turn into image and pub it
}




void usb_camera::cameraCallback()
{
    if (!capture_.isOpened()) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera.");
        return;
    }
    static rclcpp::Time start = now();
    auto lastStart = start;
    start = now();

    auto publisher = capturedPublisher_.lock();
    if (!publisher)
    {
        return;
    }

    RosImage::UniquePtr image = std::make_unique<RosImage>();
    RosImage msg;

    auto t1 = now();
    capture_.read(frame_);
    auto t2 = now();

    RCLCPP_WARN(get_logger(), "%f", (t2 - t1).seconds());

    cv_bridge::CvImagePtr cv_image_ptr(new cv_bridge::CvImage);

    sensor_msgs::msg::Image ros_image_msg;
    ros_image_msg.header.stamp = now(); // 设置时间戳
    ros_image_msg.width = frame_.cols; // OpenCV图像的宽度
    ros_image_msg.height = frame_.rows; // OpenCV图像的高度
    ros_image_msg.encoding = sensor_msgs::image_encodings::BGR8; // 设置图像编码
    ros_image_msg.data = std::vector<uint8_t>(frame_.data, frame_.data + frame_.total() * frame_.elemSize()); // 转换为字节数据
    publisher->publish(ros_image_msg);



    CameraInfo cameraInfo;
    cameraInfo.distortion_model = "plumb_bob";
    cameraInfo.d = d;
    std::array<double, 9> roiK{};
    memcpy(roiK.data(), k.data(), 9 * sizeof(double));
    roiK[2] -= (double)get_parameter("offset_x").as_int();
    roiK[5] -= (double)get_parameter("offset_y").as_int();
    cameraInfo.k = roiK; //?
    cameraInfoPublisher_->publish(cameraInfo);

    static struct Count
    {
        uint8_t i : 5;
    } count;
    static std::array<double, 32> recentDurations = {};
    recentDurations[count.i++] = (start - lastStart).seconds();
    double fps = (double)recentDurations.size() /
        std::reduce(recentDurations.begin(), recentDurations.end(), 0.0);
    RCLCPP_INFO(get_logger(), "FPS: %f", fps);
    static int killCount = 0;

    // cv::imshow("demo",frame_);
    // cv::waitKey(1);
    // if (fps < 15)
    // {
    //     killCount++;
    //     RCLCPP_WARN(get_logger(), "Camera is below 15fps");
    // }
    // if (killCount > 15)
    // {
    //     RCLCPP_ERROR(get_logger(), "Camera is below 15fps, Restarting!");
    //     exit(0);
    //
    // }
}


