//
// Created by dcy on 24-12-26.
//

//v4l2-ctl -c exposure_time_absolute=50

#ifndef USB_CAMERA_H
#define USB_CAMERA_H

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/msg/header.h>

using RosImage = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;


class usb_camera : public rclcpp::Node
{
    bool is_calibrate_ = false;

    cv::VideoCapture capture_;
    cv::Mat frame_;
    std::vector<double> k;
    std::vector<double> d;
    std::vector<double> intrinsic_;
    std::vector<double> distCoeffs_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher_; //图像发布
    std::weak_ptr<std::remove_pointer<decltype(imagePublisher_.get())>::type> capturedPublisher_; //非拥有指针 decltype断言
    rclcpp::Publisher<CameraInfo>::SharedPtr cameraInfoPublisher_; //参数发布
    rclcpp::TimerBase::SharedPtr timer_; //定时器
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr onSetParametersCallbackHandle_; //参数设置的回调

    void Calibrate_camera();

    void open_device();

    void cameraCallback();


public:
    usb_camera();

    usb_camera(const rclcpp::NodeOptions &options);
};


#endif //USB_CAMERA_H
