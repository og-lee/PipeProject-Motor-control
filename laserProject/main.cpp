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


static int stepsX = 0;
static std::atomic<int> stepsY = 0;
static int absStepsX = 0;
static int absStepsY = 0;


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


struct Circle {
    float x, y, r;
    cv::Point center()const { return { (int)x,(int)y }; }
    cv::Point2f centerf()const { return { x,y }; }
};

// Function to find the circle on 
// which the given three points lie 
Circle findCircle(int x1, int y1, int x2, int y2, int x3, int y3)
{
    float x12 = x1 - x2;
    float x13 = x1 - x3;

    float y12 = y1 - y2;
    float y13 = y1 - y3;

    float y31 = y3 - y1;
    float y21 = y2 - y1;

    float x31 = x3 - x1;
    float x21 = x2 - x1;

    // x1^2 - x3^2 
    float sx13 = pow(x1, 2) - pow(x3, 2);

    // y1^2 - y3^2 
    float sy13 = pow(y1, 2) - pow(y3, 2);

    float sx21 = pow(x2, 2) - pow(x1, 2);
    float sy21 = pow(y2, 2) - pow(y1, 2);

    float f = ((sx13) * (x12)
        +(sy13) * (x12)
        +(sx21) * (x13)
        +(sy21) * (x13))
        / (2 * ((y31) * (x12)-(y21) * (x13)));
    float g = ((sx13) * (y12)
        +(sy13) * (y12)
        +(sx21) * (y13)
        +(sy21) * (y13))
        / (2 * ((x31) * (y12)-(x21) * (y13)));

    float c = -pow(x1, 2) - pow(y1, 2) - 2 * g * x1 - 2 * f * y1;

    // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0 
    // where centre is (h = -g, k = -f) and radius r 
    // as r^2 = h^2 + k^2 - c 
    float h = -g;
    float k = -f;
    float sqr_of_r = h * h + k * k - c;

    // r is the radius 
    float r = sqrt(sqr_of_r);

    //cout << "Centre = (" << h << ", " << k << ")" << endl;
    //cout << "Radius = " << r;
    return { h,k,r };
}



int main() {

    cv::VideoCapture cap;
    LaserRangeFinder laser;
    laser.open("\\\\.\\COM27");
    Sleep(10);
    if (!cap.open(0))
        return 0;
    cv::Mat img;
    cv::Mat roiImg;
    cap >> img;
    cv::Rect2d r = cv::selectROI(img);

    //laser.laserON();

    for (;;) {
        cv::Mat imgNoLaser;
        cv::Mat imgWithLaser;
        cv::Mat gray;
        cv::Mat img;
        cv::Mat rImg;
        cv::Mat gaussian;
        cv::Mat cany;

        //laser.laserON();
        //Sleep(200);
        //cap >> imgWithLaser;
        //cap >> imgWithLaser;
        //laser.laserOFF();
        //Sleep(200);
        //cap >> imgNoLaser;
        cap >> imgNoLaser;
        rImg = imgNoLaser(r);


        //imshow("no laser", imgNoLaser);
        //imshow("with laser", imgWithLaser);
        //cv::waitKey(1);

        cv::resize(rImg, rImg, cv::Size(0, 0), 3, 3);

        cv::cvtColor(rImg, gray, cv::COLOR_RGB2GRAY);

        cv::GaussianBlur(gray, gaussian, cv::Size(3, 3), 1);
        int cannyMin = 80;
        int cannyMax = 150;
        int numOfContour = 6;
        int contourSize = 20;
        int contourArea = 100;

        cv::Canny(gaussian, cany, cannyMin, cannyMax);
        //cv::adaptiveThreshold(gaussian, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY,5,5);
        vector<vector<cv::Point>> contour = laserProject::getContoursSortedExternal(cany);
        vector<int> indexes = laserProject::findCircleContoursIndexes(contour, numOfContour, contourSize, contourArea);
        if (indexes.size() != 6)
            continue;
        for (int i = 0; i < indexes.size(); i++) {
            laserProject::drawWithContourI(contour[indexes[i]], gray, " ");
        }


        //extract all points
        vector<cv::Point> ps;
        for (auto &con : contour) {
            ps.insert(ps.end(), con.begin(), con.end());
        }

        //ransac
        const int N = 1000;
        const double min_dist = 10;
        const float eps = 3;
        const float min_coverage = 0.8;
        for (int i = 0; i < N; i++) {
            //select 3 points randomly
            cv::Point p1, p2, p3;
            p1 = ps[rand() % ps.size()];
            while (true) {
                p2 = ps[rand() % ps.size()];
                p3 = ps[rand() % ps.size()];
                auto v1 = p2 - p1;
                auto v2 = p3 - p1;
                if (cv::norm(p1 - p2) > min_dist &&
                    cv::norm(p1 - p3) > min_dist &&
                    cv::norm(p2 - p3) > min_dist) {
                    break;
                }
            }
            //compute circle 
            auto cir = findCircle(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);

            //count how many points on this circle
            int cnt = std::count_if(ps.begin(), ps.end(), [&](cv::Point const& p)->bool {
                return std::abs(cv::norm(p - cir.center()) - cir.r) < eps;
            });

            float coverage = cnt / (2 * CV_PI*cir.r);
            if (coverage> min_coverage && coverage < 1) {
                //draw 
                cv::circle(rImg, cir.center(), cir.r, { 0,255,0 }, 1);
            }
        }

        cv::imshow("color", rImg);


        /*for (int i = 0; i < contour.size(); i++) {
            laserProject::drawWithContourI(contour[i], gray, " ");
        }*/
        imshow("blur", gaussian);
        imshow("gray", gray);
        imshow("cannyWithGaussian", cany);

        if (cv::waitKey(1) == 27) break;

    }

}

#else

static INAmessage msg_MOVEM0(SLAVE2, FUNC_WR, REMOTE_LOW_REG, 0x0008);
static INAmessage msg_MOVEM1(SLAVE2, FUNC_WR, REMOTE_LOW_REG, 0x0009);
static INAmessage msg_JOGFW(SLAVE2, FUNC_WR, REMOTE_LOW_REG, 0x1000);
static INAmessage msg_JOGBW(SLAVE2, FUNC_WR, REMOTE_LOW_REG, 0x2000);
static INAmessage msg_STOP(SLAVE2, FUNC_WR, REMOTE_LOW_REG, 0x0020);

static int moveCmd = 0;
// in the motor thread it keeps sending 
void motorThread(CSerialPort serial) {
    // check if the steps is - or + 
    // 
    while (1) {
        if (moveCmd == 1) {
            //move with M0 operation 
            sendMyMessage(msg_STOP, serial);
            std::this_thread::sleep_for(5ms);
            sendMyMessage(msg_MOVEM0, serial);
            std::this_thread::sleep_for(5ms);
        }
        else if (moveCmd == 2) {
            sendMyMessage(msg_STOP, serial);
            std::this_thread::sleep_for(5ms);
            sendMyMessage(msg_MOVEM1, serial);
            std::this_thread::sleep_for(5ms);
            //move with M1 operation
        }
        std::this_thread::sleep_for(10ms);
    }
}







int main(void)
{
    CSerialPort _serial;
    CSerialPort motorRs485;

    _serial.ClosePort();
    char portName[] = "\\\\.\\COM27";//"\\\\.\\COM27";
    char motorPort[] = "\\\\.\\COM26";

    if (motorRs485.OpenPort(motorPort)) {
        cout << "motor port opened" << endl;
        motorRs485.ConfigurePort(CBR_115200, 8, FALSE, EVENPARITY, ONESTOPBIT);
        motorRs485.SetCommunicationTimeouts(0, 0, 0, 0, 0);

    }
    if (_serial.OpenPort(portName))
    {
        cout << "port opened" << endl;
        _serial.ConfigurePort(CBR_19200, 8, FALSE, NOPARITY, ONESTOPBIT);
        // Timeout configuration
        _serial.SetCommunicationTimeouts(0, 0, 0, 0, 0);
        // memory buffer
        //BYTE* pByte = new BYTE[512];
        //string strData = ("");
        //string strTmp = ("");
        _serial.WriteByte('C');
    }
    else { cout << "failed to open" << endl; return 0; }


    cv::VideoCapture cap;

    cv::Mat originalImage = laserProject::takeImg(cap);
    cv::Mat originalGray;
    cv::Mat thresholdImg;                  // mat to store thresholded img
    cv::Mat blurredOriginal;               //blurred image before thresholding to remove noise 

    int pickPointNumber = 8;
    int minThresh = 100;
    int maxVal = 255;

    cv::cvtColor(originalImage, originalGray, cv::COLOR_BGR2GRAY);
    _serial.WriteByte('O');

    cv::GaussianBlur(originalGray, blurredOriginal, cv::Size(5, 5), 0, 0);
    cv::threshold(blurredOriginal, thresholdImg, minThresh, maxVal, cv::THRESH_BINARY);
    cv::imshow("thresh", thresholdImg);
    cv::waitKey(0);

    vector<vector<cv::Point>> contours;     //
    cv::findContours(thresholdImg, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    std::sort(contours.begin(), contours.end(), [](const vector<cv::Point> &a, const vector<cv::Point> &b) {return a.size() < b.size(); });


    int numOfContour = 6;
    int contourSize = 40;
    int contourArea = 100;
    double distance = 0;
    int currentPickedPoint = 0;
    int circleNum = 4;


    //get the indexes of contours found 
    vector<int> indexes = laserProject::findCircleContours(contours, originalImage, numOfContour, contourSize, contourArea);
    //sort the contours according to the size

    //ver contains the biggest ellipse points 
    std::vector<cv::Point2f> ver = laserProject::findBiggestEllipse(contours, indexes);

    cv::Point2f transformedRect[4] = { {0,0},{400,0},{400,400},{0,400} };
    cv::Mat transformMat = cv::getPerspectiveTransform(ver.data(), transformedRect);
    transformMat.convertTo(transformMat, CV_32F);

    //contour[i] is the original contour of ellipse 
    //4 circleContours Transformed with matrix T 

    std::vector<vector<cv::Point2f>> circleContours(circleNum);
    for (int i = 0; i < circleNum; i++) {
        circleContours[i] = laserProject::transform2Dpoints(contours[indexes[i]], transformMat);
    }
    laserProject::myCircle circleThroughCenters = laserProject::findCircleWithCenters(circleContours);


    //circPoin2f is the picked points to check distance 
    std::vector<cv::Point2f> circPoint2f = laserProject::returnCirclePoints(pickPointNumber, circleThroughCenters);
    std::vector<cv::Point2f>pickPointsOriginal = laserProject::transform2Dpoints(circPoint2f, transformMat.inv());



    if (!cap.isOpened()) {
        cap.open(0);
    }
    //distance from laser to the circle points 

    float kp1 = 1;
    float kp2 = 1;
    float ki1 = 0.05;
    float ki2 = 0.05;
    float int_errX = 0;
    float int_errY = 0;

    //thread 
    std::thread motorSerialThread(motorThread, motorRs485);

    //motorSerialThread.join();
    for (;;) {
        cv::Mat transformedImg;                //matrix for transformed image with 4 points 
        cv::Mat frame;
        cap >> frame;
        cv::Size size = frame.size();
        cv::Mat laserImage;
        cv::Mat laserGray;                      // grayscale of frame
        cv::Mat laserBinary;
        cv::cvtColor(frame, laserGray, cv::COLOR_BGR2GRAY);
        cv::Mat transformedImgRGB;
        int laserThresh = 50;
        //get the laser image //check the laser//laserGray (image with laser) 
        cv::subtract(laserGray, originalGray, laserImage);

        cv::threshold(laserImage, laserBinary, laserThresh, maxVal, cv::THRESH_BINARY);

        transformedImg.create(originalGray.size(), CV_8UC1);
        cv::warpPerspective(laserGray, transformedImg, transformMat, transformedImg.size());

        // find the laser contour if more than 1 contour found or none found continue to find 
        std::vector<std::vector<cv::Point>> laserContour;
        cv::findContours(laserBinary, laserContour, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
        if (laserContour.size() != 1 || laserContour.size() > 1)
            continue;

        std::vector<cv::Point2f> transformedPoints = laserProject::transform2Dpoints(laserContour[0], transformMat);

        cv::cvtColor(transformedImg, transformedImgRGB, cv::COLOR_GRAY2RGB);
        cv::circle(transformedImgRGB, circleThroughCenters.center, circleThroughCenters.radius, cv::Scalar(250, 0, 0), 2);


        cv::Moments cen = cv::moments(transformedPoints);
        float cx = cen.m10 / cen.m00;
        float cy = cen.m01 / cen.m00;


        distance = sqrt(pow(circPoint2f[currentPickedPoint].x - cx, 2) + pow((circPoint2f[currentPickedPoint].y - cy), 2));
        /*cout << "distance to the circle" << endl;
        cout << distance << endl;
*/
        float arr[3] = { cx,cy,1 };
        cv::Mat laserCenter(3, 1, CV_32F, arr);
        cv::Mat tInv = transformMat.inv();
        tInv.convertTo(tInv, CV_32F);
        cv::Mat laserCenterOriginal = tInv * laserCenter;


        cv::Point2f lpointCent;
        //laserCenterOrigin(2,0) already 1 so actually no need to divide
        lpointCent.x = laserCenterOriginal.at<float>(0, 0) / laserCenterOriginal.at<float>(2, 0);
        lpointCent.y = laserCenterOriginal.at<float>(1, 0) / laserCenterOriginal.at<float>(2, 0);

        cv::circle(transformedImgRGB, cv::Point2f(cx, cy), 1, cv::Scalar(0, 0, 250), 2);
        cv::circle(frame, lpointCent, 1, cv::Scalar(0, 0, 200), 2);
        laserProject::drawWithContour(pickPointsOriginal, frame, "original");
        laserProject::drawWithContour(circPoint2f, transformedImgRGB, "transformed img");


        // if found then laserContour[0] is the contour  
        // transform the contour with MATRIX T  and find the center of contour         

        float distX = lpointCent.x - pickPointsOriginal[0].x;
        float distY = lpointCent.y - pickPointsOriginal[0].y;
        int_errY += distY;

        float thresholdY = 1 / ki2;
        float thresholdX = 1 / ki1;

        if (int_errY > thresholdY)
            int_errY = thresholdY;
        else if (int_errY < -thresholdY)
            int_errY = -thresholdY;
        if (int_errX > thresholdX)
            int_errX = thresholdX;
        else if (int_errX < -thresholdX)
            int_errX = -thresholdX;


        stepsY = kp2 * distY + ki2 * int_errY;

        moveCmd = 0;
        if (stepsY > 0) {
            moveCmd = 2;
        }
        else if (stepsY < 0) {
            moveCmd = 1;
        }

        // make the motor move stepsY 0.01*steps degree 
        /*cout <<"steps to move Y : " <<stepsY << endl;
        cout << "distance in Y : " << distY << endl;
        cout << "int_Y : " << int_errY << endl;*/

        cout << "distance in Y : " << distY << endl;

        cout << "steps to move Y : " << stepsY << endl;
        cv::imshow("transformed img", transformedImgRGB);
        cv::imshow("original", frame);
        if (cv::waitKey(1) == 27) break;



    }

    motorSerialThread.join();

    _serial.ClosePort();

}

#endif
