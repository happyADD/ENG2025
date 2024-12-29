//
// Created by bismarck on 11/13/22.
//

#include "graphic/utils.h"

using namespace cv;
double getAngleCosByPoints(cv::Point2d e1, cv::Point2d center, cv::Point2d e2) {
    const cv::Point2d vec1 = e1 - center;
    cv::Point2d vec2 = e2 - center;
    double result = vec1.dot(vec2) / norm(vec1) / norm(vec2);
    return result;
}

angle_d getEdgeAngle(cv::Point2d p1, cv::Point2d p2) {
    cv::Point2d p = p1 - p2;
    return atan2(p.y, p.x);
}

double getLength(cv::Point2d p1, cv::Point2d p2) {
    return norm(p1 - p2);
}

void PreProcessImg(InputArray src,OutputArray dst)
{
    /*第一步：过滤红蓝*/
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    cv::Mat redMask, blueMask;
    cv::inRange(hsv, cv::Scalar(0.9327*255, 50, 100), cv::Scalar(255, 255, 255), redMask);
    // cv::inRange(hsv, cv::Scalar(150, 100, 100), cv::Scalar(179, 255, 255), redMask);
    cv::inRange(hsv, cv::Scalar(0.6392*255, 50, 0.3725*255), cv::Scalar(190, 255, 255), blueMask);
    cv::Mat colorMask = redMask | blueMask;
    cv::Mat img;
    hsv.copyTo(img, colorMask);
    // imshow("debug1",img);
    /*第二步：滤波*/
    cvtColor(img, img, COLOR_HSV2BGR);
    cvtColor(img, img, COLOR_BGR2GRAY);
    //1.开操作
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    morphologyEx(img, img, MORPH_OPEN, kernel);
    threshold(img, img, 50, 255, THRESH_BINARY);

    //2.排除小的连续区域,暂时不用，当上面一步灰度低阈值低的时候开启
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // imshow("debug2",img);

    //对区域特征进行过滤

    std::vector<std::pair<double, int>> areas_and_it;
    for (int i = 0; i < contours.size(); i++)
    {
        areas_and_it.emplace_back(contourArea(contours[i]), i);
    }
    std::sort(areas_and_it.begin(), areas_and_it.end(),
              [](const std::pair<double, int>& a,
                 const std::pair<double, int>& b)
              {
                  return a.first > b.first;
              }
    );
    cv::Mat filteredDst(img.size(), img.type(), cv::Scalar(0, 0, 0));
    for (int i = 0; i < std::min(6, static_cast<int>(contours.size())); i++)
    {
        if (!areas_and_it.empty())
            if (areas_and_it.at(i).first < areas_and_it.at(0).first * 0.9 && areas_and_it.at(i).first > areas_and_it.
                at(0).first * 1.1)
                break;
        drawContours(filteredDst, contours, areas_and_it.at(i).second, Scalar(255), FILLED);
    }
    // imshow("debug3",filteredDst);
    filteredDst.copyTo(dst);
}

