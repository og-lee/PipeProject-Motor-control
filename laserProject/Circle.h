#pragma once
using namespace cv;

typedef vector<Point2f> Point2fs;


typedef struct {
    Point2f C;
    float R;
    float fitness;
    int match_cnt;
    std::vector<cv::Point2f> inliers;
} Circle;
