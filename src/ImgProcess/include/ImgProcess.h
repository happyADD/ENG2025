//
// Created by dcy on 24-11-13.
//

#ifndef IMGPROCESS_H
#define IMGPROCESS_H
//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
//tools
#include "opencv4/opencv2/opencv.hpp"
//self
#include "ImgProcessFuction/config.h"
#include "ImgProcessFuction/graphic.h"
#include "ImgProcessFuction/utils.h"

class ImgProcess_node : public rclcpp::Node{
private:
    using RosImage = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    rclcpp::Subscription<RosImage>::SharedPtr ImageSubscription_;
    rclcpp::Subscription<CameraInfo>::SharedPtr CameraInfoSubscription_;
    // rclcpp::Publisher<RosImage>::SharedPtr CalibrationImagePublisher_;

public:

    ImgProcess_node();
    ImgProcess_node(const rclcpp::NodeOptions &options);
    void imageCallback(RosImage::UniquePtr rosImage);
    void cameraInfoCallback(const CameraInfo::ConstSharedPtr &cameraInfo);//相机信息回调函数
};



#endif //IMGPROCESS_H
