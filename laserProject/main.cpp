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
  
    //setup laser
    laser_dev.open("\\\\.\\COM27");
    //setup motor2
    motor1.open("\\\\.\\COM26");
    motor2.userPort(motor1);
    laser_dev.laserON();
    cv::Mat img;
    cv::VideoCapture cap(0);
    cap >> img;
    cv::Rect2d r = cv::selectROI(img);
    std::vector<byte>returned;

    
    
   /* int curPos = readPosition(msg_READ_S1, motor1.m_port);

    int k = -5000 + curPos;
    changeM0_HOR(msg_DEG_HIGHM0, msg_DEG_LOWM0, k,motor1);
    moveM0_SLAVE1(motor1.m_port);*/



    //std::vector<byte>returned = sendMyMessage(msg_READ_S1, motor1.m_port);

    /*int horMotorPosition = readPosition(msg_READ_S1, motor1.m_port);
    int moveDegree = 1;
    while (1) {
        double distanceAfterMove = laser_dev.readDistance();
        cout << distanceAfterMove << endl;
        int destination = horMotorPosition + moveDegree;
        changeM0_HOR(msg_DEG_HIGHM0, msg_DEG_LOWM0, destination, motor1);
        moveM0_SLAVE1(motor1.m_port);

        horMotorPosition = destination;

        char key = cv::waitKey(10);
        if (key == 27) break;
    }*/


    while (1) {
        cv::Mat roiImg;
        cap >> img;
        roiImg=img(r);
        cv::resize(roiImg, roiImg, cv::Size(), 3, 3);
        cv::imshow("img",roiImg);
        cv::waitKey(1);
        char key = cv::waitKey(10);
        if (key == 27) break;
    }
    
    //positionData dat = laserProject::getLeftPoint(laser_dev, motor1);
    positionData dat1 = laserProject::getRightPoint(laser_dev, motor1);
    
    cout << dat1.dist << endl;
    cout << dat1.motorHor << endl;

    //cout << dat.dist << endl;
    //cout << dat.motorHor << endl;
    

        
    return;
}
