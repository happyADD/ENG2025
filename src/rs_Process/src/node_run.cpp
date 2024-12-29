#include <rclcpp/rclcpp.hpp>
#include <rs_process.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RealSenseProcessNode>());
    rclcpp::shutdown();
}