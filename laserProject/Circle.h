#pragma once
#include <random>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
using namespace cv;
using namespace std; 


typedef struct {
    Point2f C;
    float R;
    float fitness;
    int match_cnt;
    std::vector<cv::Point2f> inliers;
} Circle;

typedef vector<Circle> Circles;
typedef std::vector<Point2f> Point2fs;


bool compareByInlierCircle(const Circle &a, const Circle &b);
bool compareByFitnessCircle(const Circle &a, const Circle &b);
void _show(Mat img);
void _wait();
