//
// Created by dcy on 24-12-26.
//
#include <rclcpp/rclcpp.hpp>
#include "usb_camera.h"
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<usb_camera>());
    rclcpp::shutdown();
}