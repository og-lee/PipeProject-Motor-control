#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <Windows.h>
#include <cmath>
#include <memory.h>
#include <algorithm>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

namespace laserProject {

    class myCircle {
    public:
        cv::Point2f center;
        float radius;
        myCircle() {}
        ~myCircle() {}
        myCircle(const myCircle& other) :center(other.center), radius(other.radius) {}
    };

    void printMes();
    cv::Mat takeImg(cv::VideoCapture capture);

    //find the contours and then push back the indexs of possible contours of ellipse 
    vector<int> findCircleContoursIndexes(vector<vector<cv::Point>>const& contour, int numOfContour, int contourSize, int contourArea);

    //check the circularity of the contours 
    bool checkCircularity(vector<cv::Point>const &contour);

    //find the biggest ellipse and return the 4 points
    std::vector<cv::Point2f> findBiggestEllipse(vector<vector<cv::Point>>const& contour, const vector<int>& indexes);

    //get the cv::point2 vector and transform to point3 homogeneous coord 
    std::vector<cv::Point3f> makeHomogeneous(std::vector<cv::Point> const & contour);
    std::vector<cv::Point3f> makeHomogeneous(std::vector<cv::Point2f> const & contour);

    //return the contour transformed
    std::vector<cv::Point2f> transform2Dpoints(std::vector<cv::Point> const& contour, cv::Mat const& T);
    std::vector<cv::Point2f> transform2Dpoints(std::vector<cv::Point2f> const& contour, cv::Mat const& T);

    void drawWithContour(const std::vector<cv::Point2f>& pt, const cv::Mat& img, const char* str);
    void drawWithContourI(const std::vector<cv::Point>& pt, const cv::Mat& img, const char* str);

    //fit the contours to circle and find the circle passing the centers of the contours  
    myCircle findCircleWithCenters(const std::vector<vector<cv::Point2f>> contours);

    std::vector<cv::Point2f> returnCirclePoints(int numberOfPoints, const myCircle& circ);

    cv::Mat thresholdBlurredImage(const cv::Mat& original, int minThresh);

    //contours sorted by size
    std::vector<std::vector<cv::Point>> getContoursSorted(const cv::Mat& thresholdImg);

    //
    cv::Mat findTmatrixWithEllipse(std::vector<std::vector<cv::Point>> contours, int numOfContour, int contourSize, int contourArea);

    myCircle getCircleWithCenters(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& tmatrix, std::vector<int> indexes, int circleNum);

    //get frame and return laser Center int warped image
    vector<vector<cv::Point>> getLaserContour(const cv::Mat& frame, const cv::Mat& originalGray, int threshVal);

    cv::Point2f getTransformedLaserCenter(const vector<cv::Point>& laserContour, const cv::Mat& tmatrix);

    cv::Point2f getOriginalLaserCenter(const cv::Point2f& targetPoint, const cv::Mat& tMat);

    std::vector<float> distanceInOriginal(cv::Point2f pt1, cv::Point2f pt2);

    //show the laser image by thresholding 
    void checkLaser(const cv::Mat& img, int minThreshold);
    std::vector<std::vector<cv::Point>> getContoursSortedExternal(const cv::Mat& thresholdImg);



}
