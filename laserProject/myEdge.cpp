#include "myEdge.h"

#undef _DEBUG

// row = y  cols = x 
std::vector<cv::Point> findJunction(const cv::Mat& edgeImage) {
    int rows = edgeImage.rows;
    int cols = edgeImage.cols;
    std::vector<cv::Point> junctions;
    junctions.reserve(rows*cols);
    //int count = 0;
    for (int y = 1; y < rows - 1; y++) {
        for (int x = 1; x < cols - 1; x++) {
            if (isJunction(edgeImage, x,y) == true) {
                junctions.emplace_back(x, y);
            }
        }
    }
    return junctions;
}


#define MY_ASSERT_(expr,msg) if(!!(expr)){}else{std::cout<< "(" #expr ")\n" msg <<std::endl;}


inline bool isJunction(const cv::Mat& img, int x, int y) {

#if _DEBUG
    MY_ASSERT_(img.empty() == false &&
        img.type() == CV_8UC1, __FUNCSIG__);

    MY_ASSERT_(pt.x >= 1 && pt.x < img.cols - 1
        && pt.y >= 1 && pt.y < img.rows - 1, __FUNCSIG__);
#endif

    //you have image
    //you want to accxess nearby 8 pixel

    auto row1 = y - 1;
    auto row2 = y;
    auto row3 = y + 1;

    auto* ptr1 = img.ptr<uchar>(row1) + x;
    auto* ptr2 = ptr1 + img.cols;
    auto* ptr3 = ptr2 + img.cols;

    //std::vector<uchar> pixels = nearbyPixel(img, pt);

#if _DEBUG
    MY_ASSERT_(pixels.size() == 8, __FUNCSIG__);
#endif

    uchar pixels[8];
    pixels[0] = ptr1[0] > 0;
    pixels[1] = ptr1[1] > 0;
    pixels[2] = ptr1[2] > 0;
    pixels[3] = ptr2[0] > 0;
    pixels[4] = ptr2[2] > 0;
    pixels[5] = ptr3[0] > 0;
    pixels[6] = ptr3[1] > 0;
    pixels[7] = ptr3[2] > 0;

    //cv::divide(255, pixels, pixels);
    // std::vector<uchar> arr(8);
    int sum = 0;
    sum += (pixels[3] - pixels[0])?1:0;
    sum += (pixels[5] - pixels[3])?1:0;
    sum += (pixels[6] - pixels[5])?1:0;
    sum += (pixels[7] - pixels[6])?1:0;
    sum += (pixels[4] - pixels[7])?1:0;
    sum += (pixels[2] - pixels[4])?1:0;
    sum += (pixels[1] - pixels[2])?1:0;
    sum += (pixels[0] - pixels[1])?1:0;

    return img.at<unsigned char>(y,x) > 0 && sum >= 6?1:0;
}


