//
// Created by dcy on 24-11-11.
//

#ifndef GRAPHIC_H
#define GRAPHIC_H

#include <rs_process.h>
#include <opencv2/opencv.hpp>
#include "utils.h"
#include "config.h"

int bgr2binary(cv::Mat& srcImg, cv::Mat& img_out, int method, int thresh, bool if_Red);

class auxPoint
{
public:
    double area; //面积
    cv::Point2f center; //最小外接圆的圆心
    float radius = 0; //半径
    bool match = false;

    auxPoint(cv::InputArray contour);

    void draw(cv::Mat& image);
};

class triangle
{
public:
    double area = 0;
    double real_area = 0;
    double real_long_distance = 0;
    cv::Point2d points[3]; //[1]是最大角顶点
    double angle_cos = 0;
    angle_d edge_angle[2]{0, 0};
    int index = 0;
    int aux = 0;

    triangle() = default;
    triangle(cv::InputArray contour);

    bool isCorner();
    bool hasAux(auxPoint p);

    void draw(cv::Mat& image);
};

class edge
{
public:
    triangle corners[2];
    int pointIndex[2]{0}; // 构成边的点在triangle中的index
    angle_d edge_angle{0};

    edge() = default;
    edge(triangle t1, triangle t2);
    bool isEdge(); // 只有在isEdge执行后才会将所有的成员都初始化
    bool isInEdge(int tid);

    void draw(cv::Mat& image);
};

class square
{
private:
    double edgeAngle = 0;

public:
    triangle corners[4];
    double area = 0;
    int aux = 0;
    std::vector<cv::Point2f> points;
    std::vector<cv::Point2i> points2draw;

    square(edge e1, edge e2);
    bool isParallel() const;
    double getEdgeRate();
    void sortByAux(); // 以下标为i的三角形开头，逆时针寻找边线

    void draw(cv::Mat& image);
    void drawR(cv::Mat& image);
};

class side : public triangle
{
    bool find_side = false;
    triangle side_triangle_;

public:
    side() = default;
    side(triangle tri) { side_triangle_ = tri; }
    void draw(cv::Mat& image);
};

class manager
{
private:
    std::vector<triangle> triangles; //三角形（角）
    std::vector<edge> edges; //边界，两个三角形组成
    std::vector<square> squares; //正方形投影
    std::vector<side> sides;
    std::vector<auxPoint> auxPoints; //mag辅助点

    std::shared_ptr<cv::Mat> depth_image_;


    bool find_side, find_front;
    int nowTid = 0;
    void addContoursAsAuxPoint(cv::InputArray contour);

public:
    enum empty_t
    {
        NOT_FOUND,
        SIDES_FOUND,
        SQUARE_FOUND
    }is_empty;

    manager() = default;
    manager(cv::Mat& depth_image) { depth_image_ = std::make_shared<cv::Mat>(depth_image); }

    bool addContours(cv::InputArray contour);
    // void addtri2sides(std::vector<triangle> &tris);
    int calculateEdges();
    int calculateSquares();
    int calculateSides();
    int isSquareAux(int sid, cv::Point2d point, float size);

    void sortSquare();
    square getSquare(int index);
    side getsides(int index);

    void draw(cv::Mat& image);
    int empty();
};


#endif //GRAPHIC_H
