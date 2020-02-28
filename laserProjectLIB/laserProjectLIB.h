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

namespace laserProject{

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
	vector<int> findCircleContours(vector<vector<cv::Point>>const& contour,cv::Mat const& originalImg,int numOfContour,int contourSize,int contourArea);

    //check the circularity of the contours 
    bool checkCircularity(vector<cv::Point>const &contour);

    //find the biggest ellipse and return the 4 points
    std::vector<cv::Point2f> findBiggestEllipse(vector<vector<cv::Point>>const& contour, const vector<int>& indexes);

    //get the cv::point2 vector and transform to point3 homogeneous coord 
    std::vector<cv::Point3f> makeHomogeneous(std::vector<cv::Point> const & contour);
    std::vector<cv::Point3f> makeHomogeneous(std::vector<cv::Point2f> const & contour);

    //return the contour transformed
    std::vector<cv::Point2f> transform2Dpoints(std::vector<cv::Point> const& contour,cv::Mat const& T);
    std::vector<cv::Point2f> transform2Dpoints(std::vector<cv::Point2f> const& contour,cv::Mat const& T);

    void drawWithContour(const std::vector<cv::Point2f>& pt,const cv::Mat& img, const char* str);
    
    //fit the contours to circle and find the circle passing the centers of the contours  
    myCircle findCircleWithCenters(const std::vector<vector<cv::Point2f>> contours);
    
    std::vector<cv::Point2f> returnCirclePoints(int numberOfPoints, const myCircle& circ);

    cv::Mat thresholdBlurredImage(const cv::Mat& original,int minThresh);

    cv::Mat getTransformMatFromEllipse(const cv::Mat& thresholdImg) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(thresholdImg, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        std::sort(contours.begin(), contours.end(), [](const vector<cv::Point> &a, const vector<cv::Point> &b) {return a.size() < b.size(); });
        
    }
}


