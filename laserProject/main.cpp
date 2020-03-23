// test branch
//check mounted scene 
#include "laserProjectLIB.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <Windows.h>
#include <stdio.h>
#include <string.h>
#include <tchar.h>
#include "SerialPort.h"
#include <atomic>
#include "MotorController.h"
#include "INAmessage.h"
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "lsdlines.h"
#include <opencv2/line_descriptor/descriptor.hpp>
#include <random>
#include "Circle.h"
#include "myEdge.h"
using namespace cv;
using namespace line_descriptor;

static int stepsX = 0;
static std::atomic<int> stepsY = 0;
static int absStepsX = 0;
static int absStepsY = 0;
static bool finished = false;
// new things to update 
// test with not subtracting images // just the laser on , can I detect laser ?? 
// 

#if 1

//Laser controller class
class LaserRangeFinder {
public:
    ~LaserRangeFinder() { close(); }

    void open(std::string const& portname) {
        std::cout << "Setting up laser range finder..." << std::endl;
        m_connected = m_port.OpenPort((char*)portname.c_str());
        if (m_connected) {
            std::cout << "Connected to " << portname << std::endl;
            m_port.ConfigurePort(CBR_19200, 8, FALSE, NOPARITY, ONESTOPBIT);
            m_port.SetCommunicationTimeouts(0, 0, 0, 0, 0);
            laserOFF();
        }
        else {
            std::cout << "Failed to connecte to port " << portname << std::endl;
        }
    }

    //close device
    void close() { if (m_connected) m_port.ClosePort(); }

    //turn laser on
    void laserON() {
        if (m_connected) {
            m_port.WriteByte('O');
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }
    //turn laser off
    void laserOFF() {
        if (m_connected) {
            m_port.WriteByte('C');
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }

    //read laser distance
    double readDistance() {
        //your code here
        if (m_connected) {
            m_port.WriteByte('C');
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }

    bool connected()const { return m_connected; }

protected:
    CSerialPort m_port;
    bool m_connected = false;
};


//struct Circle {
//    float x, y, r;
//    cv::Point center()const { return { (int)x,(int)y }; }
//    cv::Point2f centerf()const { return { x,y }; }
//};

// Function to find the circle on 
// which the given three points lie 
//Circle findCircle(int x1, int y1, int x2, int y2, int x3, int y3)
//{
//    float r;
//    cv::Point2f pt;
//    std::vector<cv::Point> arr(3);
//    arr[0] = cv::Point(x1, y1);
//    arr[1] = cv::Point(x2, y2);
//    arr[2] = cv::Point(x3, y3);
//    cv::minEnclosingCircle(arr, pt, r);
//    return { pt.x,pt.y,r };
//}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    std::vector<cv::Point> *pt = (vector<cv::Point>*)userdata;
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        pt->push_back(cv::Point(x, y));
        cout <<pt->size()<<"point"<<"(" << x << ", " << y << ") saved" << endl;
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        pt->clear();
        cout << "all the points cleared" << endl;
    }
}

struct model_cnt {
    RotatedRect ellipse;
    int count;
};

std::vector<model_cnt> models;
bool compareByLength(const model_cnt &a, const model_cnt &b) {
    return a.count > b.count;
}


cv::Mat myimg = cv::imread("C:/Users/oggyu/Pictures/Camera Roll/1.jpg", 1);
int myMin ;
int myMax;
int kernelSize=1;

static void CannyThresh(int, void* userdata) {
    int ker = kernelSize + 2;
    cv::Mat dst;
    cv::Mat blurImg;
    cv::GaussianBlur(myimg, blurImg, cv::Size(ker, ker), 2, 0);
    cv::Canny(blurImg, dst, myMin, myMax);
    cv::imshow("canny ed", dst);
}


int main() {
    std::vector<cv::Point> userPicked;
    cv::VideoCapture cap;
    LaserRangeFinder laser;
    /*  laser.open("\\\\.\\COM27");
      Sleep(10);
      if (!cap.open(0))
          return 0;*/
    cv::Mat img;
    cv::Mat roiImg;
    cv::Mat roiImgG;
    img = cv::imread("C:/Users/oggyu/Pictures/Camera Roll/1.jpg", 1);
    cv::Rect2d r = cv::selectROI(img);
    //laser.laserON();
    int cannyMin = 40;
    int cannyMax = 100;
    int gaussianFilterSize = 9;
    cv::Size gauFil =  cv::Size(gaussianFilterSize, gaussianFilterSize);
    cv::Mat edge;
    cv::GaussianBlur(img, img,gauFil,2,0);
    cv::Canny(img, edge, cannyMin, cannyMax);
    cout << "type is :" << edge.type() << endl;
    edge = edge(r);

    roiImg = img(r);
    cv::cvtColor(roiImg,roiImgG,cv::COLOR_RGB2GRAY);
    //cv::GaussianBlur(roiImgG, threshImg,Size(3,3),1);
    //cv::medianBlur(roiImgG, threshImg, 5);
    //cv::adaptiveThreshold(roiImgG, threshImg, 255, ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV,3,5);

    int scale =2;
    cv::resize(edge, edge, cv::Size(0, 0), scale, scale);
    cv::resize(roiImg, roiImg, cv::Size(0, 0), scale, scale);
    cv::resize(roiImgG, roiImgG, cv::Size(0, 0), scale, scale);
    //cv::resize(threshImg, threshImg, cv::Size(0, 0), scale, 2);

    
    //edge.convertTo(edge, CV_8U);
    cv::namedWindow("my edges");
    cv::setMouseCallback("my edges", CallBackFunc, &userPicked);
    cv::imshow("my edges",edge);
    //imshow("thresh", threshImg); 

    cv::waitKey(1);
    

    namedWindow("canny ed", cv::WINDOW_AUTOSIZE);
    createTrackbar("Min Thresh", "canny ed", &myMin, 255, CannyThresh);
    createTrackbar("Max Thresh", "canny ed", &myMax, 255, CannyThresh);
    createTrackbar("kernel Size", "canny ed",&kernelSize,15,CannyThresh);
    CannyThresh(0,0);

    cv::waitKey(0);
    //vector<vector<cv::Point>> contour = laserProject::getContoursSortedExternal(edge);

    //for (int i = 0; i < contour.size(); i++) {
    //    if (contour[i].size() < 50)
    //        continue;
    //    cv::RotatedRect rect = cv::fitEllipse(contour[i]);
    //    //cv::ellipse(roiImg, rect, cv::Scalar(0, 250, 0));
    //    laserProject::drawWithContourI(contour[i], roiImg, " ");
    //}
    //for (int i = contour.size()-1; i>contour.size()-2; i--) {
    //    cv::RotatedRect rect = cv::fitEllipse(contour[i]);
    //    //cv::ellipse(roiImg, rect, cv::Scalar(0, 250, 0),2);
    //    laserProject::drawWithContourI(contour[i], roiImg, " ");
    //}
    //imshow("roi", roiImg);
    //cv::waitKey(1);
//
    //// make matrix to array 
    //


    cv::Mat showJunc;
    cv::cvtColor(edge, showJunc, cv::COLOR_GRAY2RGB);
    std::vector<cv::Point> junctions;
    std::vector<cv::Point> endPoints;

    /*while (true) {
        auto t = clock();

        junctions = findJunction(edge);

        auto dt = clock() - t;

        std::cout << "Run time: " << (1000.0*dt / CLOCKS_PER_SEC) << "ms" << std::endl;
    }*/


    junctions = findJunction(edge);

    for (int i = 0; i < junctions.size(); i++) {
        circle(showJunc, junctions[i], 1, cv::Scalar(0, 255, 0));
    }
    cv::imshow("junction",showJunc);
    cv::waitKey(1);

    //  
    //cv::SimpleBlobDetector::Params params;
    //*params.minThreshold = 10;
    //params.maxThreshold = 200;*/
    //params.filterByArea = true;
    //params.minArea = 10;
    //
    //imshow("roiImg", roiImgG);
    //cv::waitKey(1);
    //*params.filterByCircularity = true;
    //params.minCircularity = 0.1;*/
    //
    //cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    //std::vector<cv::KeyPoint> keypoints;
    //cv::Mat im_withKeyPoints;
    //cv::Mat edgeWithColor;
    //cv::cvtColor(edge, edgeWithColor, cv::COLOR_GRAY2RGB);
    //detector->detect(edge, keypoints);
    //cv::drawKeypoints(edgeWithColor, keypoints, im_withKeyPoints, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    std::vector<cv::Point2f> ps;
    std::vector<Circle> circ;
    cv::findNonZero(edge, ps);
    int minDis = 50;
    int minR = 200;
    int maxR = 300;
    float pixDis = 2;
    float matchThresh = 15;
    float fitness = 0.5;
    int iter = 3000;
    circ = laserProject::find_circlesWithRansac(ps,roiImg,minDis,minR,maxR,pixDis,matchThresh,fitness);

    //std::sort(circ.begin(), circ.end(), compareByFitnessCircle);
    std::sort(circ.begin(), circ.end(), compareByInlierCircle);

    Mat edgeWithColor;
    Mat edgeWithEllipse;
    cv::cvtColor(edge, edgeWithColor, COLOR_GRAY2RGB);
    edgeWithColor.copyTo(edgeWithEllipse);
    std::vector<cv::Point2f> circleInliers;
    int numOfCircleToSample = circ.size();
    //int numOfCircleToSample = 5;
    for (int i = 0; i < numOfCircleToSample; i++) {
        circleInliers.insert(circleInliers.end(), circ[i].inliers.begin(), circ[i].inliers.end());
        cv::circle(edgeWithColor, circ[i].C, circ[i].R,Scalar(0,255,0));
    }
    cv::RotatedRect ellipResult = fitEllipse(circleInliers);
    cv::ellipse(edgeWithEllipse, ellipResult, cv::Scalar(0, 255, 155), 2);
    cv::imshow("detectedCircle",edgeWithColor);
    cv::imshow("detectedEllipse",edgeWithEllipse);
    cv::waitKey(0);


    // Parameters Settings (Sect. 4.2)
       // Other constant parameters settings. 

    // Gaussian filter parameters, in pre-processing
  
        

    //select contour for image

    // user picking part 
    /*while (userPicked.size() < 5) {
        cv::waitKey(10);
        if (userPicked.size() >= 5) {
            std::cout << "more than 5 picked " << endl;
            break;
        }
    }

    cv::RotatedRect myellipse = cv::fitEllipse(userPicked);
    cv::ellipse(roiImg, myellipse, cv::Scalar(0, 250, 0),2);
   
    cv::imshow("roi Img", roiImg);*/

    // user picking end 

    ////////////////////////////////////////////////////////////////////////
    //ellipse ransac 

    //cv::Mat showCol;
    //cvtColor(edge, showCol, cv::COLOR_GRAY2RGB);
    //const int N = 1000;
    //const double minDistance = 100;
    //const float eps = 2;
    ////const float min_coverage = 0.9;
    //std::mt19937 rng(std::time(0));
    //std::uniform_int_distribution<unsigned long> distr(0, ps.size());
    //
    //for (int i = 0; i < N; i++) {
    //    //select 3 points randomly
    //    cout << i <<" :  iteration start" << endl; 
    //    std::vector<cv::Point> ellipPt(5);
    //    ellipPt[0] = ps[rand() % ps.size()];
    //    ellipPt[1] = ps[rand() % ps.size()];
    //    ellipPt[2] = ps[rand() % ps.size()];
    //    ellipPt[3] = ps[rand() % ps.size()];
    //    ellipPt[4] = ps[rand() % ps.size()];
    //    while (norm(ellipPt[0] - ellipPt[1]) < minDistance) ellipPt[1] = ps[distr(rng)];

    //    while (norm(ellipPt[0] - ellipPt[2]) < minDistance || 
    //           norm(ellipPt[1] - ellipPt[2]) < minDistance) ellipPt[2] = ps[distr(rng)];

    //    while (norm(ellipPt[0] - ellipPt[3]) < minDistance ||
    //        norm(ellipPt[1] - ellipPt[3]) < minDistance ||
    //        norm(ellipPt[2] - ellipPt[3]) < minDistance) ellipPt[3] = ps[distr(rng)];

    //    while (norm(ellipPt[0] - ellipPt[4]) < minDistance ||
    //        norm(ellipPt[1] - ellipPt[4]) < minDistance   ||
    //        norm(ellipPt[2] - ellipPt[4]) < minDistance   || 
    //        norm(ellipPt[3] - ellipPt[4]) < minDistance) ellipPt[4] = ps[distr(rng)];


    //    cv::RotatedRect myEllipse = cv::fitEllipse(ellipPt);

    //    ////count how many points on this circle
    //    int cnt = 0;
    //    float phi = myEllipse.angle;
    //    cv::Point2f cp = myEllipse.center;
    //    float width = myEllipse.size.width;
    //    float height = myEllipse.size.height;

    //    for (int i = 0; i < ps.size(); i++) {
    //        float error = (pow((ps[i].x - cp.x)*cos(phi) + (ps[i].y - cp.y)*sin(phi), 2) / pow(width, 2)) + pow((ps[i].x - cp.x)*sin(phi) - (ps[i].y - cp.y)*cos(phi), 2) / pow(height, 2) - 1;
    //        if (error <= eps) {
    //            cnt++;
    //        }
    //    }
    //    model_cnt mod{ myEllipse, cnt };
    //    models.push_back(mod);
    //}

    //std::sort(models.begin(), models.end(), compareByLength);
    ////int maxIndex = 0; 
    ////int max = models[0].counts;
    ////
    //for (int i = models.size(); i < models.size(); i++) {
    //    cv::ellipse(showCol, models[i].ellipse, cv::Scalar(0, 255, 0),2);
    //}
    //imshow("ellipse", showCol);
    //cv::waitKey(0);
/////////////////////////////////////////////////////////////////////////////////////////////////////////
   


    //for (;;) {
    //    cv::Mat imgNoLaser;
    //    cv::Mat imgWithLaser;
    //    cv::Mat gray;
    //    cv::Mat img;
    //    cv::Mat rImg;
    //    cv::Mat gaussian;
    //    cv::Mat cany;
    //    cv::Mat binary;

    //    //laser.laserON();
    //    //Sleep(200);
    //    //cap >> imgWithLaser;
    //    //laser.laserOFF();
    //    //Sleep(200);

    //    cap >> imgNoLaser;
    //    rImg = imgNoLaser(r);



    //    cv::resize(rImg, rImg, cv::Size(0, 0),2,2);

    //    cv::cvtColor(rImg, gray, cv::COLOR_RGB2GRAY);

    //           int numOfContour = 6;
    //    int contourSize = 20;
    //    int contourArea = 100;

    //    cv::Canny(gray, cany, cannyMin, cannyMax);
    //    imshow("canny", cany);
    //    cv::waitKey(1);
    //   
    //    cv::GaussianBlur(gray, gaussian,cv::Size(9,9),0,0);
    //    cv::adaptiveThreshold(gaussian, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV,9,2);
    //     vector<vector<cv::Point>> contour = laserProject::getContoursSortedExternal(cany);
    //    vector<int> indexes = laserProject::findCircleContoursIndexes(contour, numOfContour, contourSize, contourArea);

    //    //for (int i = 0; i < indexes.size(); i++) {
    //    //    laserProject::drawWithContourI(contour[indexes[i]], rImg, " ");
    //    //}

    //    int count = 0;
    //    for (int i = 0; i < contour.size(); i++) {
    //        if (contour[i].size() > 50) {
    //            count++;
    //            laserProject::drawWithContourI(contour[i], rImg, " ");
    //        }
    //    }
    //    cv::Mat binResult;
    //    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    //    //cv::erode(binary, binResult, element);
    //    //cout<<"number of contour" << contour.size() << endl;
    //    //cout << "contour size over 30 : " << count <<endl;


    //    //extract all points
    //    vector<cv::Point> ps;
    //    const int minConSize = 50;
    //    for (auto &con : contour) {
    //        if (con.size() < minConSize)
    //            continue;
    //        ps.insert(ps.end(), con.begin(), con.end());
    //    }

    //    //ransac
    //    //const int N = 10000;
    //    //const double min_dist = 5;
    //    //const float eps = 3;
    //    //const float min_coverage = 0.9;
    //    //for (int i = 0; i < N; i++) {
    //    //    //select 3 points randomly
    //    //    cv::Point p1, p2, p3;
    //    //    p1 = ps[rand() % ps.size()];
    //    //    p2 = ps[rand() % ps.size()];
    //    //    p3 = ps[rand() % ps.size()];
    //    //    while (true) {
    //    //        break;
    //    //        p2 = ps[rand() % ps.size()];
    //    //        p3 = ps[rand() % ps.size()];
    //    //        auto v1 = p2 - p1;
    //    //        auto v2 = p3 - p1;
    //    //        if (cv::norm(p1 - p2) > min_dist &&
    //    //            cv::norm(p1 - p3) > min_dist &&
    //    //            cv::norm(p2 - p3) > min_dist) {
    //    //            break;
    //    //        }
    //    //    }
    //    //    //compute circle 
    //    //    auto cir = findCircle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);

    //    //    //count how many points on this circle
    //    //    int cnt = std::count_if(ps.begin(), ps.end(), [&](cv::Point const& p)->bool {
    //    //        return std::abs(cv::norm(p - cir.center()) - cir.r) < eps;
    //    //    });

    //    //    float coverage = cnt / (2 * CV_PI*cir.r);
    //    //    if (coverage > min_coverage && coverage < 1) {
    //    //        //draw 
    //    //        cv::circle(rImg, cir.center(), cir.r, { 0,255,0 }, 1);
    //    //        std::cout <<"coverage : "<<coverage << endl;
    //    //        std::cout << "total Points : " << ps.size() << endl;
    //    //        std::cout << "count : " << cnt << endl;

    //    //    }
    //    //}

    //    cv::imshow("color", rImg);
    //    cv::waitKey(1);
    //    cv::imshow("bin", binary);
    //    //cv::imshow("eroded IMG", binResult);


    //    if (cv::waitKey(1) == 27) break;

    //}


}

#else

#endif
