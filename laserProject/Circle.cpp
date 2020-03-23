#include "Circle.h"


bool compareByInlierCircle(const Circle &a, const Circle &b) {
    return a.inliers.size() > b.inliers.size();
}
bool compareByFitnessCircle(const Circle &a, const Circle &b) {
    return a.fitness > b.fitness;
}

void _show(Mat img) {
    cv::imshow("show Img", img);
}
void _wait() {
    cv::waitKey(1);
}
