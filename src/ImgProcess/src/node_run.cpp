//
// Created by dcy on 24-10-20.
//
#include <rclcpp/rclcpp.hpp>
#include "ImgProcess.h"
int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ImgProcess_node>());
    rclcpp::shutdown();
}