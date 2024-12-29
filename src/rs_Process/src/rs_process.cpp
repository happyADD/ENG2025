//
// Created by dcy on 24-12-28.
//

#include "rs_process.h"

#include <graphic/graphic.h>

RealSenseProcessNode::RealSenseProcessNode(): RealSenseProcessNode(rclcpp::NodeOptions())
{
}

//construction
RealSenseProcessNode::RealSenseProcessNode(const rclcpp::NodeOptions& options): Node("rs_process_node", options)
{
    //params
    declare_parameters("hsvmask", std::map<std::string, int>{
                           {"lower_blue", 150},
                           {"upper_blue", 200},
                           {"lower_red", 0},
                           {"upper_red", 1}
                       });

    //a lot subs
    RosRGBImageSubscription_ = create_subscription<RosImageMsg>(
        "/camera/camera/color/image_raw",
        10,
        std::bind(&RealSenseProcessNode::rs_RGBImageCallback, this, std::placeholders::_1)
    );
    RosDImageSubscription_ = create_subscription<RosImageMsg>(
        "/camera/camera/aligned_depth_to_color/image_raw",
        10,
        std::bind(&RealSenseProcessNode::rs_DImageCallback, this, std::placeholders::_1)
    );
    CameraRGBInfoSubscription_ = create_subscription<CameraInfoMsg>(
        "/camera/camera/color/camera_info",
        1,
        std::bind(&RealSenseProcessNode::rs_CameraRGBInfoCallback, this, std::placeholders::_1)
    );
    CameraImuSubscription_ = create_subscription<ImuMsg>(
        "/camera/camera/imu",
        5,
        std::bind(&RealSenseProcessNode::rs_ImuCallback, this, std::placeholders::_1)
    );
    CameraDInfoSubscription_ = create_subscription<CameraInfoMsg>(
        "/camera/camera/color/camera_info",
        1,
        std::bind(&RealSenseProcessNode::rs_CameraDInfoCallback, this, std::placeholders::_1)
    );
}

/**
 * @brief 实现图像处理
 * @param ROS_color_Image 传入图像，格式是ROSmsg
 *
 * @details 1.转换为Mat
 *          2.图像处理部分，寻找正面、侧面特征点，排序 —— 函数feature_points_search
 *          3.调取对应深度信息，完成RGBD匹配
 *          4.使用ICP算法计算兑矿站姿态
 *          5.获得相机姿态并且完成坐标系变换
 *          6.计算机械臂需要的到达位置
 *          7.发布话题让串口包接受再发给电控
 */
void RealSenseProcessNode::rs_RGBImageCallback(RosImageMsg::UniquePtr ROS_color_Image)
{
    if (ROS_color_Image->encoding == "rgb8")
    {
        color_image = cv::Mat(ROS_color_Image->height, ROS_color_Image->width, CV_8UC3, ROS_color_Image->data.data());
        cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "error color image coding");
        return;
    }

#ifdef SHOW_RAW_IMAGE
    cv::imshow("raw", color_image);
    cv::waitKey(1);
#endif
    feature_points_search(color_image, feature_points_);
}

/**
 * @brief 深度图回调
 * @param ROS_D_Image
 *
 * @details 1.转换为Mat （16U_C1格式）
 *          2.储存,过滤，梯度过大不滤波，较小时平滑过滤
 *          3.对齐深度图和RGB图
 *
 * @attention 如何处理深度图和RGB图的同步与对齐问题
 */
void RealSenseProcessNode::rs_DImageCallback(RosImageMsg::UniquePtr ROS_D_Image)
{
    if (ROS_D_Image->encoding == "16UC1")
    {
        depth_image = cv::Mat(ROS_D_Image->height, ROS_D_Image->width, CV_16UC1, ROS_D_Image->data.data());
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "error color image coding");
        return;
    }
    cv::Mat depth_image_32F;
    depth_image.convertTo(depth_image_32F, CV_32F, 1.0 / 65535.0);
    cv::imshow("non_depth", depth_image_32F);

    // 创建一个新的Mat对象来存储滤波后的图像
    cv::Mat filtered_depth_image;
    cv::bilateralFilter(depth_image_32F, filtered_depth_image, 0, 0.8, 0.1);

    cv::imshow("depth", filtered_depth_image);
}

void RealSenseProcessNode::rs_CameraRGBInfoCallback(const CameraInfoMsg::ConstSharedPtr& CameraInfo)
{
    cv::Mat tempCameraMatrix(3, 3, CV_64FC1);
    memcpy(color_distCoeffs.data, CameraInfo->d.data(), CameraInfo->d.size() * sizeof(double));
    memcpy(tempCameraMatrix.data, CameraInfo->k.data(), CameraInfo->k.size() * sizeof(double));
    if (cv::countNonZero(tempCameraMatrix) == 0)
        RCLCPP_WARN(get_logger(), "Camera matrix is all of zero!");
    else
        color_cameraMatrix = tempCameraMatrix;
}

void RealSenseProcessNode::rs_CameraDInfoCallback(const CameraInfoMsg::ConstSharedPtr& CameraInfo)
{
    //实现
}

void RealSenseProcessNode::rs_ImuCallback(ImuMsg::UniquePtr Imu_data)
{
    //实现
}

void RealSenseProcessNode::feature_points_search(cv::Mat& image, feature_points& feature_points)
{
    using namespace cv;
    Mat process_image;
    image.copyTo(process_image);
    Mat gray_image(process_image.rows, process_image.cols,CV_8UC1, Scalar(255));
    cvtColor(process_image, process_image, COLOR_BGR2HSV);
    //TODO 动态参数
    get_parameters("hsvmask", hsv_mask);
    cv::Mat mask_blue, mask_red;
    cv::inRange(process_image, cv::Scalar(hsv_mask["lower_blue"], 100, 100),
                cv::Scalar(hsv_mask["upper_blue"], 255, 255), mask_blue);
    cv::inRange(process_image, cv::Scalar(hsv_mask["lower_red"], 50, 50), cv::Scalar(hsv_mask["upper_red"], 255, 255),
                mask_red);
    cv::Mat mask_combined;
    cv::bitwise_or(mask_blue, mask_red, mask_combined);
    cv::bitwise_and(process_image, process_image, process_image, mask_combined);
    process_image.copyTo(gray_image, mask_combined);
    cvtColor(gray_image, gray_image, COLOR_BGR2GRAY);
    threshold(gray_image, gray_image, 150, 255, THRESH_BINARY);

#ifdef SHOW_MASK_IMAGE
    imshow("mask", gray_image);
    waitKey(1);
#endif

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(gray_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    manager man(this->depth_image);
    for (const auto& contour : contours)
    {
        man.addContours(contour);
    }


    try
    {
        man.calculateEdges();
        man.calculateSides();
        man.calculateSquares();
        man.sortSquare();
        man.draw(gray_image);
    }
    catch (cv::Exception& e)
    {
        return;
    }

    switch (man.empty())
    {
    case man.SQUARE_FOUND:

        square squ = man.getSquare(0);
    // std::cout << "aux num: " << s.aux << std::endl;
        cvtColor(gray_image, gray_image, cv::COLOR_GRAY2BGR);
        polylines(gray_image, squ.points2draw, true, cv::Scalar(0, 255, 0), 2);
        break;

    case man.SIDES_FOUND:
        side sid = man.getsides(0);

    case man.NOT_FOUND:
        RCLCPP_INFO(get_logger(), "find no squares");
        return;
    }

#ifdef FRONT_SHOW
    imshow("Front_find", gray_image);
#endif
}
