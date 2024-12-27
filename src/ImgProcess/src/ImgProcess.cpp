//
// Created by dcy on 24-11-13.
//
//ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.026 image:=/image_raw camera:=/usb_cam
#include "ImgProcess.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

ImgProcess_node::ImgProcess_node() : ImgProcess_node(rclcpp::NodeOptions())
{
}

ImgProcess_node::ImgProcess_node(const rclcpp::NodeOptions& options): Node("demo_node", options)
{
    cameraMatrix = cv::Mat(3, 3, CV_64F, 0.0);
    distCoeffs = cv::Mat(1, 5, CV_64F, 0.0);
    RCLCPP_INFO(get_logger(), "ImgProcess Node is inited!");
    ImageSubscription_ = create_subscription<RosImage>(
        "image_raw",
        10,
        std::bind(&ImgProcess_node::imageCallback, this, std::placeholders::_1)
    );

    CameraInfoSubscription_ = create_subscription<CameraInfo>(
        "camera_info",
        1,
        std::bind(&ImgProcess_node::cameraInfoCallback, this, std::placeholders::_1)
    );
    // CalibrationImagePublisher_ = create_publisher<RosImage>("CalibrationImage",1);
}

void ImgProcess_node::imageCallback(RosImage::UniquePtr rosImage)
{
    std::vector<cv::Point3d> realPoints = {
        cv::Point3d(-0.144, 0.144, 0), cv::Point3d(0.144, 0.144, 0),
        cv::Point3d(0.144, -0.144, 0), cv::Point3d(-0.144, -0.144, 0)
    };
    std::vector<cv::Point3d> triangles = {
        cv::Point3d(-0.144 + 0.050, 0.144, 0), cv::Point3d(-0.144, 0.144, 0), cv::Point3d(-0.144, 0.144 - 0.050, 0),
        cv::Point3d(0.144 - 0.050, 0.144, 0), cv::Point3d(0.144, 0.144, 0), cv::Point3d(0.144, 0.144 - 0.050, 0),
        cv::Point3d(0.144 - 0.050, -0.144, 0), cv::Point3d(0.144, -0.144, 0), cv::Point3d(0.144, -0.144 + 0.050, 0),
        cv::Point3d(-0.144 + 0.050, -0.144, 0), cv::Point3d(-0.144, -0.144, 0), cv::Point3d(-0.144, -0.144 + 0.050, 0)
    };

    cv::Mat bgrImage;

    if (rosImage->encoding == "bgr8")
    {
        bgrImage = cv::Mat(rosImage->height, rosImage->width, CV_8UC3, rosImage->data.data());
    }
    else
    {
        cv::Mat bayerImage(rosImage->height, rosImage->width, CV_8UC1, rosImage->data.data());
        cv::cvtColor(bayerImage, bgrImage, cv::COLOR_BayerRG2RGB);
    }

    cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);
    cv::imshow("raw", bgrImage);
    PreProcessImg(bgrImage, bgrImage);
    cv::namedWindow("Process", cv::WINDOW_AUTOSIZE);
    cv::imshow("Process", bgrImage);
    cv::waitKey(1);
    threshold(bgrImage, bgrImage, 0, 255, cv::THRESH_BINARY);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bgrImage, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    manager man;
    for (const auto& contour : contours)
    {
        man.addContours(contour);
    }

    try
    {
        man.calculateEdges();
        man.calculateSquares();
        man.sortSquare();
        man.draw(bgrImage);
    }
    catch (cv::Exception& e)
    {
        return;
    }

    if (man.empty())
    {
        RCLCPP_INFO(get_logger(), "find no squares");
        return;
    }
    square s = man.getSquare(0);
    // std::cout << "aux num: " << s.aux << std::endl;
    cvtColor(bgrImage, bgrImage, cv::COLOR_GRAY2BGR);
    polylines(bgrImage, s.points2draw, true, cv::Scalar(0, 255, 0), 2);
    // s.draw(bgrImage);


    std::vector<cv::Point2f> sortedImagePoints = s.points;
    cv::Mat rvec, tvec(3, 1, CV_32FC1);
    Eigen::Quaterniond q;


    try //only by two corners
    {
        cv::Mat _tvec;
        std::vector<cv::Point2d> i_triangle_points = {
            s.corners[1].points[1], s.corners[0].points[0], s.corners[0].points[1], s.corners[0].points[2]
        };
        std::vector<cv::Point3d> w_triangle_points = {
            cv::Point3d(0.144, 0.144, 0),cv::Point3d(-0.144 + 0.050, 0.144, 0), cv::Point3d(-0.144, 0.144, 0), cv::Point3d(-0.144, 0.144 - 0.050, 0)

        };
        cv::solvePnP(w_triangle_points, i_triangle_points, cameraMatrix, distCoeffs, rvec, _tvec, false,
                     cv::SOLVEPNP_P3P);
        tvec.at<float>(0, 0) = (float)_tvec.at<double>(2, 0);
        tvec.at<float>(1, 0) = -(float)_tvec.at<double>(0, 0);
        tvec.at<float>(2, 0) = -(float)_tvec.at<double>(1, 0);
        std::cout << "tri:" << rvec << std::endl;
    }
    catch (cv::Exception& e)
    {
        RCLCPP_INFO(get_logger(), e.msg.c_str());
    }

    try
    {
        cv::Mat _tvec;
        cv::solvePnP(realPoints, sortedImagePoints, cameraMatrix, distCoeffs, rvec, _tvec, false,
                     cv::SOLVEPNP_IPPE_SQUARE);
        if (_tvec.empty())
        {
            return;
        }
        tvec.at<float>(0, 0) = (float)_tvec.at<double>(2, 0);
        tvec.at<float>(1, 0) = -(float)_tvec.at<double>(0, 0);
        tvec.at<float>(2, 0) = -(float)_tvec.at<double>(1, 0);
        std::cout << "all:" << rvec << std::endl;

        // Eigen::Vector3d rotation(rvec.at<double>(2, 0), rvec.at<double>(0, 0), rvec.at<double>(1, 0));
        // double angle = rotation.norm();
        // Eigen::Vector3d axis = rotation.normalized();
        // q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));

        std::string pitch_string = "pitch:" + std::to_string(rvec.at<double>(0) * 180 / CV_PI);
        std::string yaw_string = "yaw:" + std::to_string(rvec.at<double>(1) * 180 / CV_PI);
        std::string roll_string = "roll:" + std::to_string(rvec.at<double>(2) * 180 / CV_PI);

        std::string x_string = "x:" + std::to_string(tvec.at<float>(0, 0));
        std::string y_string = "y:" + std::to_string(tvec.at<float>(0, 1));
        std::string z_string = "z:" + std::to_string(tvec.at<float>(0, 2));

        cv::putText(bgrImage, pitch_string, cv::Point2i(0, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        cv::putText(bgrImage, yaw_string, cv::Point2i(0, 100), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        cv::putText(bgrImage, roll_string, cv::Point2i(0, 150), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        cv::putText(bgrImage, x_string, cv::Point2i(0, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        cv::putText(bgrImage, y_string, cv::Point2i(0, 250), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));
        cv::putText(bgrImage, z_string, cv::Point2i(0, 300), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255));


        cv::namedWindow("find", cv::WINDOW_AUTOSIZE);
        cv::imshow("find", bgrImage);
    }
    catch (cv::Exception& e)
    {
        std::cout << "fealtotran" << std::endl;
        return;
    }
}

void ImgProcess_node::cameraInfoCallback(const CameraInfo::ConstSharedPtr& cameraInfo)
{
    cv::Mat tempCameraMatrix(3, 3, CV_64FC1);
    memcpy(distCoeffs.data, cameraInfo->d.data(), cameraInfo->d.size() * sizeof(double));
    memcpy(tempCameraMatrix.data, cameraInfo->k.data(), cameraInfo->k.size() * sizeof(double));
    if (cv::countNonZero(tempCameraMatrix) == 0)
    {
        RCLCPP_WARN(get_logger(), "Camera matrix is all of zero!");
    }
    else
    {
        cameraMatrix = tempCameraMatrix;
    }
}
