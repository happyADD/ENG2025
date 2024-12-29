#ifndef FEATURE_POINTS_HPP_
#define FEATURE_POINTS_HPP_
#include <opencv2/opencv.hpp>
#include <graphic/graphic.h>

class feature_points
{
    struct color_feature_points
    {
        cv::Point2i left_[3];
        cv::Point2i right_[3];
        cv::Point2i front_[3];
    } color_feature_points_;

    struct depth_feature_points
    {
        cv::Point2i left_[3];
        cv::Point2i right_[3];
        cv::Point2i front_[3];
    } depth_feature_points_;

public:
    feature_points() = default;
    void combine_color_depth();


};

// feature_points::feature_points()
// {
//     color_feature_points_.front_[0] = cv::Point2i(0,0);
// }
#endif
