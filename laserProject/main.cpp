// test branch
#include "laserProjectLIB.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Windows.h>
#include <stdio.h>
#include <string.h>
#include <tchar.h>
#include "SerialPort.h"
#include <atomic>
#include "MotorController.h"
#include "INAmessage.h"
#include "laserDevice.h"


static int stepsX = 0;
static std::atomic<int> stepsY = 0;
static int absStepsX = 0;
static int absStepsY = 0;



void main() {
    LaserRangeFinder laser_dev;
    MotorController motor1(SLAVE1);
    MotorController motor2(SLAVE2);
    cv::Mat img;
    cv::VideoCapture cap(0);
    cap >> img;
    cv::Rect2d r = cv::selectROI(img);

    //setup laser
    laser_dev.open("\\\\.\\COM27");
    //setup motor2
    motor1.open("\\\\.\\COM26");
    motor2.userPort(motor1);

    //std::vector<byte>returned = sendMyMessage(msg_MOVEM1, motor1.m_port);
    //std::vector<byte>returned = sendMyMessage(msg_READ_S1, motor1.m_port);
    while (1) {
        cv::Mat roiImg;
        cap >> img;
        roiImg=img(r);
        cv::resize(roiImg, roiImg, cv::Size(), 3, 3);
        cv::imshow("img",roiImg);
        cv::waitKey(1);
        double a = laser_dev.readDistance() * 1000;
        double horMotorPos = (double)readPosition(msg_READ_S1, motor1.m_port) / 100;
        double verMotorPos = (double)readPosition(msg_READ_S2, motor2.m_port) / 100;

         
        cout <<"laser distance is : "<< a << endl; 
        cout <<"motor horizontal position is : " << horMotorPos << endl; 
        cout << "motor verticle position is : " << verMotorPos << endl << endl;
        /*int deg = readPosition(msg_READ_S1, motor1.m_port);
        double val = deg / (double)100;
        cout << val << endl;
        Sleep(500);*/

        char key = cv::waitKey(10);
        if (key == 27) break;
    }
    
    

    /*cv::Mat img;
    cv::Mat roiImg;
    cv::Mat roiImgG;
    img = cv::imread("C:/Users/oggyu/Pictures/Camera Roll/1.jpg");
    resize(img, img,cv::Size() ,0.3, 0.3);
    cv::Rect2d r = cv::selectROI(img);
    roiImg = img(r);
    cv::resize(roiImg, roiImg, cv::Size(), 2, 2);
    cv::Canny(roiImg, roiImgG, 50, 100);


    cv::imshow("my edges", roiImg);
    cv::imshow("my canny", roiImgG);
    cv::waitKey(0);*/
    //laser_dev.laserOFF();

    
    return;
}
