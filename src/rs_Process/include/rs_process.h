//
// Created by dcy on 24-12-28.
//

#ifndef RS_PROCESS_H
#define RS_PROCESS_H

#define CV_DEBUG

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform.hpp>
//thirdparty
#include <opencv2/opencv.hpp>
#include <opencv2/surface_matching/pose_3d.hpp>
#include <librealsense2/rs.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
//self class
#include <feature_points.hpp>

//self
#include <graphic/config.h>
#include <debugconfig.h>
#include <graphic/graphic.h>


class RealSenseProcessNode:public rclcpp::Node
{
    //ros
    using RosImageMsg = sensor_msgs::msg::Image;
    using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
    using ImuMsg = sensor_msgs::msg::Imu;

    rclcpp::Subscription<RosImageMsg>::SharedPtr RosRGBImageSubscription_;
    rclcpp::Subscription<RosImageMsg>::SharedPtr RosDImageSubscription_;
    rclcpp::Subscription<CameraInfoMsg>::SharedPtr CameraRGBInfoSubscription_;
    rclcpp::Subscription<CameraInfoMsg>::SharedPtr CameraDInfoSubscription_;
    rclcpp::Subscription<ImuMsg>::SharedPtr CameraImuSubscription_;


    std::map<std::string,int> hsv_mask;

    cv::Mat color_image;
    cv::Mat depth_image;
    cv::Mat RGBD_Image;
    cv::ppf_match_3d::Pose3D silo_pose_;
    cv::ppf_match_3d::Pose3D ore_pose_;

    feature_points feature_points_;
    cv::Mat color_distCoeffs,color_cameraMatrix;


public:
    explicit RealSenseProcessNode();
    explicit RealSenseProcessNode(const rclcpp::NodeOptions &options);
    //callback
    void rs_RGBImageCallback(RosImageMsg::UniquePtr ROS_color_Image);
    void rs_DImageCallback(RosImageMsg::UniquePtr ROS_D_Image);
    void rs_CameraRGBInfoCallback(const CameraInfoMsg::ConstSharedPtr &CameraInfo);
    void rs_CameraDInfoCallback(const CameraInfoMsg::ConstSharedPtr &CameraInfo);
    void rs_ImuCallback(ImuMsg::UniquePtr Imu_data);
    //function
    void feature_points_search(cv::Mat &image,feature_points &feature_points);
};



#endif //RS_PROCESS_H
