//
// Created by dcy on 24-11-11.
//

#ifndef GRAPHIC_H
#define GRAPHIC_H

#include <opencv2/opencv.hpp>
#include "utils.h"
#include "config.h"

int bgr2binary(cv::Mat& srcImg, cv::Mat& img_out, int method, int thresh, bool if_Red);

class auxPoint {
public:
    double aero;
    cv::Point2f center;
    float radius=0;
    bool match = false;

    auxPoint(cv::InputArray contour);

    void draw(cv::Mat& image);
};

class triangle {
public:
    double aero = 0;
    cv::Point2d points[3];
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

class edge {
public:
    triangle corners[2];
    int pointIndex[2]{0};   // 构成边的点在triangle中的index
    angle_d edge_angle{0};

    edge() = default;
    edge(triangle t1, triangle t2);
    bool isEdge();      // 只有在isEdge执行后才会将所有的成员都初始化
    bool isInEdge(int tid);

    void draw(cv::Mat& image);
};

class square {
private:
    double edgeAngle = 0;
public:
    triangle corners[4];
    double aero = 0;
    int aux = 0;
    std::vector<cv::Point2f> points;
    std::vector<cv::Point2i> points2draw;

    square(edge e1, edge e2);
    bool isParallel() const;
    double getEdgeRate();
    void sortByAux();     // 以下标为i的三角形开头，逆时针寻找边线

    void draw(cv::Mat& image);
    void drawR(cv::Mat& image);
};

class manager {
private:
    std::vector<triangle> triangles;
    std::vector<edge> edges;
    std::vector<square> squares;
    std::vector<auxPoint> auxPoints;
    int nowTid = 0;
    void addContoursAsAuxPoint(cv::InputArray contour);

public:
    bool addContours(cv::InputArray contour);
    int calculateEdges();
    int calculateSquares();
    int isSquareAux(int sid, cv::Point2d point, float size);

    void sortSquare();
    square getSquare(int sid);

    void draw(cv::Mat& image);
    bool empty();
};



#endif //GRAPHIC_H
