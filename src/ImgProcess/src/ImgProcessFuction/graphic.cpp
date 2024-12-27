//
// Created by dcy on 24-11-11.
//

#include "ImgProcessFuction/graphic.h"

using namespace cv;

int bgr2binary(Mat& srcImg, Mat& img_out, int method, int thresh, bool if_Red)
{
    if (srcImg.empty())
        return -1;
    if (method == 1)
    {
        //method 1: split channels and substract
        std::vector<Mat> imgChannels;
        split(srcImg, imgChannels);
        Mat red_channel = imgChannels.at(2);
        Mat green_channel = imgChannels.at(1);
        Mat blue_channel = imgChannels.at(0);
        Mat mid_chn_img;

        if (if_Red)
        {
            mid_chn_img = red_channel - blue_channel - green_channel;
        }
        else
        {
            mid_chn_img = blue_channel - red_channel - green_channel;
        }

        threshold(mid_chn_img, img_out, thresh, 255, THRESH_BINARY);
    }
    if (method == 2)
    {
        //method 2:
        Mat imgGray;
        cvtColor(srcImg, imgGray, COLOR_BGR2GRAY);
        threshold(imgGray, img_out, thresh, 255, THRESH_BINARY);
    }
    return 0;
}


auxPoint::auxPoint(cv::InputArray contour)
{
    aero = contourArea(contour);
    minEnclosingCircle(contour, center, radius);
}

void auxPoint::draw(cv::Mat& image)
{
    circle(image, center, radius, cv::Scalar(0, 255, 0), 3);
}


triangle::triangle(cv::InputArray contour)
{
    std::vector<cv::Point2d> _triangle;
    aero = cv::contourArea(contour);
    cv::minEnclosingTriangle(contour, _triangle);
    double minAngleCos = 1; // [0~180]度范围内cos函数单调递减，故最小cos为最大角
    int minIndex = 0;
    if(!_triangle.empty())
    {
        for (int i = 0; i < 3; i++)
        {
            double angleCos = getAngleCosByPoints(_triangle[(i + 1) % 3], _triangle[i], _triangle[(i + 2) % 3]);
            if (angleCos < minAngleCos)
            {
                minAngleCos = angleCos;
                minIndex = i;
            }
        }
        points[0] = _triangle[(minIndex + 1) % 3];
        points[1] = _triangle[minIndex];
        points[2] = _triangle[(minIndex + 2) % 3];
        angle_cos = minAngleCos;
        edge_angle[0] = getEdgeAngle(points[1], points[0]);
        edge_angle[1] = getEdgeAngle(points[1], points[2]);
    }

}

bool triangle::isCorner()
{
    if (aero < MIN_AERO)
    {
        return false;
    }
    if (angle_cos > cos(degree2rad(MIN_ANGLE)))
    {
        return false;
    }
    double l1 = getLength(points[0], points[1]);
    double l2 = getLength(points[0], points[2]);
    double diff = abs(l1 - l2) / std::min(l1, l2);
    if (diff > MAX_LENGTH_DIFF)
    {
        return false;
    }
    return true;
}

void triangle::draw(cv::Mat& image)
{
    for (int i = 0; i < 3; i++)
    {
        cv::line(image, points[i], points[(i + 1) % 3], cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
        circle(image, points[1], 3, cv::Scalar(200, 20, 20), 2);
    }
}

bool triangle::hasAux(auxPoint p)
{
    double aeroRate = p.aero / aero;
    if (aeroRate < AUX_MIN_AERO || aeroRate > AUX_MAX_AERO)
    {
        return false;
    }
    bool has = false;
    for (auto& i : edge_angle)
    {
        angle_d toAux = getEdgeAngle(points[1], p.center);
        if (toAux - i > degree2rad(AUX_MAX_ANGLE))
        {
            continue;
        }
        double length = getLength(points[1], p.center) / sqrt(p.aero);
        if (length < AUX_MIN_LENGTH || length > AUX_MAX_LENGTH)
        {
            continue;
        }
        has = true;
    }
    return has;
}


edge::edge(triangle t1, triangle t2)
{
    corners[0] = t1;
    corners[1] = t2;
}

bool edge::isEdge()
{
    if (abs(corners[0].aero - corners[1].aero) / std::min(corners[0].aero, corners[1].aero) > MAX_AERO_DIFF)
    {
        return false;
    }
    double minDiff = 3.1 * M_PIl;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {

            cv::Mat image = cv::Mat::zeros(1080, 1920, CV_8UC3);
            corners[0].draw(image);
            corners[1].draw(image);
            cv::line(image, corners[0].points[1], corners[1].points[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::line(image, corners[0].points[i*2], corners[0].points[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            cv::line(image, corners[1].points[j*2], corners[1].points[1], cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            // DEBUG: 需要详细debug边的匹配时请解注释这段代码，然后启动调试，断点设置在本行，进入断点后查看image

            angle_d tris_angle = corners[0].edge_angle[i] - corners[1].edge_angle[j].reverse();
            if (tris_angle > degree2rad(MAX_TRIANGLE_EDGE_ANGLE_DIFF))
            {
                continue;
            }
            angle_d edgeAngle = getEdgeAngle(corners[0].points[1], corners[1].points[1]);
            angle_d diff1 = edgeAngle - corners[0].edge_angle[i];
            if (diff1 > degree2rad(MAX_EDGE_TRIANGLE_DIFF))
            {
                continue;
            }
            angle_d diff2 = edgeAngle - corners[1].edge_angle[j].reverse();
            if (diff2 > degree2rad(MAX_EDGE_TRIANGLE_DIFF))
            {
                continue;
            }
            double totalDiff = diff1 + diff2 + tris_angle;
            if (totalDiff < minDiff)
            {
                edge_angle = edgeAngle;
                pointIndex[0] = i * 2; // 将[0, 1]重映射为[0, 2]
                pointIndex[1] = j * 2;
                minDiff = totalDiff;
            }
        }
    }
    return minDiff < 3 * M_PIl;
}

bool edge::isInEdge(int tid)
{
    return corners[0].index == tid || corners[1].index == tid;
}

void edge::draw(cv::Mat& image)
{
    cv::line(image, corners[0].points[1], corners[1].points[1], cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
}


square::square(edge e1, edge e2)
{
    edgeAngle = e1.edge_angle - e2.edge_angle;
    corners[0] = e1.corners[0];
    corners[1] = e1.corners[1];
    corners[2] = e2.corners[0];
    corners[3] = e2.corners[1];

    std::vector<cv::Point> temp;

    for (auto& corner : corners)
    {
        temp.push_back(corner.points[1]);
    }

    aero = cv::contourArea(temp);
}

bool square::isParallel() const
{
    return edgeAngle < MAX_EDGE_ANGLE_DIFF;
}

bool cmp_y(triangle& A, triangle& B)
{
    return A.points[1].y > B.points[1].y;
}

void swap(triangle& A, triangle& B)
{
    triangle C = A;
    A = B;
    B = C;
}

void square::sortByAux()
{
    std::sort(corners, corners + 4, cmp_y);
    if (corners[1].points[1].x < corners[0].points[1].x)
        swap(corners[0], corners[1]);
    if (corners[2].points[1].x < corners[3].points[1].x)
        swap(corners[2], corners[3]);

    int maxAux = 0;
    int maxAuxIndex = 0;
    for (int i = 0; i < 4; i++)
    {
        if (corners[i].aux > maxAux)
        {
            maxAux = corners[i].aux;
            maxAuxIndex = i;
        }
    }
    aux = maxAux;
    maxAuxIndex = 2;

    points.push_back(corners[(maxAuxIndex + 2) % 4].points[1]);
    points.push_back(corners[(maxAuxIndex + 3) % 4].points[1]);
    points.push_back(corners[maxAuxIndex].points[1]);
    points.push_back(corners[(maxAuxIndex + 1) % 4].points[1]);

    points2draw.push_back(corners[(maxAuxIndex + 2) % 4].points[1]);
    points2draw.push_back(corners[(maxAuxIndex + 3) % 4].points[1]);
    points2draw.push_back(corners[maxAuxIndex].points[1]);
    points2draw.push_back(corners[(maxAuxIndex + 1) % 4].points[1]);
}

void square::draw(cv::Mat& image)
{
    circle(image, points[0], 3, cv::Scalar(200, 20, 20), 2);
    circle(image, points[1], 3, cv::Scalar(20, 200, 20), 2);
    circle(image, points[2], 3, cv::Scalar(200, 20, 200), 2);
    circle(image, points[3], 3, cv::Scalar(200, 200, 20), 2);
}

void square::drawR(cv::Mat& image)
{
    cv::polylines(image, points2draw, true, cv::Scalar(255, 0, 0), 4);
}

double square::getEdgeRate()
{
    double min = 1e20, max = 0;
    for (int i = 0; i < 4; i++)
    {
        auto pd = points[i] - points[(i + 1) % 4];
        double len = sqrt(pow(pd.x, 2) + pow(pd.y, 2));
        if (len > max)
        {
            max = len;
        }
        if (len < min)
        {
            min = len;
        }
    }
    return min / max;
}


bool manager::addContours(cv::InputArray contour)
{
    triangle tri = triangle(contour);
    if (tri.isCorner())
    {
        tri.index = nowTid++;
        triangles.push_back(tri);
        addContoursAsAuxPoint(contour);
        return true;
    }
    addContoursAsAuxPoint(contour);
    return false;
}

int manager::calculateEdges()
{
    if (triangles.size() < 2)
    {
        return 0;
    }
    for (auto & triangle : triangles)
    {
        for (auto & auxPoint : auxPoints)
        {
            if (auxPoint.match)
            {
                continue;
            }
            if (triangle.hasAux(auxPoint))
            {
                auxPoint.match = true;
                triangle.aero += 2 * auxPoint.aero;
                triangle.aux++;
            }
        }
    }
    for (int i = 0; i < triangles.size() - 1; i++)
    {
        for (int j = i + 1; j < triangles.size(); j++)
        {
            edge e(triangles[i], triangles[j]);
            if (e.isEdge())
            {
                edges.push_back(e);
            }
        }
    }
    return (int)edges.size();
}

int manager::calculateSquares()
{
    if (edges.size() < 2)
    {
        return 0;
    }
    for (int i = 0; i < edges.size() - 1; i++)
    {
        for (int j = i + 1; j < edges.size(); j++)
        {
            square s(edges[i], edges[j]);
            if (s.isParallel())
            {
                int ok[4] = {0};
                for (int k = 0; k < edges.size(); k++)
                {
                    if (k == i || k == j)
                    {
                        continue;
                    }
                    bool isIn[4];
                    isIn[0] = edges[k].isInEdge(edges[i].corners[0].index);
                    isIn[1] = edges[k].isInEdge(edges[i].corners[1].index);
                    isIn[2] = edges[k].isInEdge(edges[j].corners[0].index);
                    isIn[3] = edges[k].isInEdge(edges[j].corners[1].index);
                    for (int u = 0; u < 2; u++)
                    {
                        for (int v = 2; v < 4; v++)
                        {
                            if (isIn[u] && isIn[v])
                            {
                                ok[u]++;
                                ok[v]++;
                            }
                        }
                    }
                }
                if (ok[0] > 0 && ok[1] > 0 && ok[2] > 0 && ok[3] > 0)
                {
                    s.sortByAux();
                    if (s.aux >= MIN_AUX_FOUND && s.aero > MIN_SQUARE_AERO && s.getEdgeRate() > MIN_EDGE_RATE)
                    {
                        squares.push_back(s);
                    }
                }
            }
        }
    }
    return (int)squares.size();
}

int manager::isSquareAux(int sid, cv::Point2d point, float size)
{
    return 0;
}

square manager::getSquare(int sid)
{
    return squares[sid];
}

bool cmp_aero(const square& s1, const square& s2)
{
    return s1.aero > s2.aero;
}

void manager::sortSquare()
{
    sort(squares.begin(), squares.end(), cmp_aero);
}

void manager::draw(cv::Mat& image)
{
    for (auto& auxPoint : auxPoints)
    {
        if (auxPoint.match)
        {
            auxPoint.draw(image);
        }
    }
    for (auto& triangle : triangles)
    {
        triangle.draw(image);
    }
    // for (auto& edge : edges)
    // {
    //     edge.draw(image);
    // }
    // for (auto& square : squares)
    // {
    //     square.draw(image);
    // }
    // squares[0].drawR(image);
}

bool manager::empty()
{
    return squares.empty();
}

void manager::addContoursAsAuxPoint(cv::InputArray contour)
{
    auxPoint p(contour);
    if (p.aero > MIN_AUX_POINT_AERO)
    {
        auxPoints.push_back(p);
    }
}
