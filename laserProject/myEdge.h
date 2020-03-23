#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
std::vector<cv::Point> findJunction(const cv::Mat& edgeImage);
bool isJunction(const cv::Mat& img, int x, int y);
