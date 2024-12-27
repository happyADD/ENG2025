//
// Created by mijiao on 24-4-15.
//

#ifndef HIK_CAMERA_HIK_CAMERA_H
#define HIK_CAMERA_HIK_CAMERA_H

#include <MvCameraControl.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class HikCamera : public rclcpp::Node {
private:
    using CameraErrorCode = uint8_t;
    using CameraHandle = void *;
    using RosImage = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;

    static std::map<std::string, const char *> realName;

    std::vector<double> k;
    std::vector<double> d;
    /**
     * @brief 相机句柄
     * */
    CameraHandle handle = nullptr;
    rclcpp::Publisher<RosImage>::SharedPtr imagePublisher;//图像发布
    std::weak_ptr<std::remove_pointer<decltype(imagePublisher.get())>::type> capturedPublisher;//非拥有指针
    rclcpp::Publisher<CameraInfo>::SharedPtr cameraInfoPublisher;//参数发布
    rclcpp::TimerBase::SharedPtr timer;//定时器
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr onSetParametersCallbackHandle;//参数设置的回调

    void PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);//打印当前相机ip和用户自定义名字

    void enumerateAndSelectDevice();

    void openDevice();

    void setTriggerModeAndPixelFormat();

    rcl_interfaces::msg::SetParametersResult setParams(const std::vector<rclcpp::Parameter> &parameters);

    void startGrabbing();

    void stopGrabbing();

    void cameraCallback();

    bool setCameraValue(const char *strKey, int64_t value);

    bool setCameraValue(const char *strKey, float value);

    bool setCameraValue(const char *strKey, const char *value);

public:
    HikCamera();

    HikCamera(const rclcpp::NodeOptions &options);
};


#endif //HIK_CAMERA_HIK_CAMERA_H
