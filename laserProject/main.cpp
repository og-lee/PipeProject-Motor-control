// test branch
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
#include "laserDevice.h"


static int stepsX = 0;
static std::atomic<int> stepsY = 0;
static int absStepsX = 0;
static int absStepsY = 0;


// new things to update 
// test with not subtracting images // just the laser on , can I detect laser ?? 
// 


//Laser controller class
//class LaserRangeFinder {
//public:
//    ~LaserRangeFinder() { close(); }
//
//    void open(std::string const& portname) {
//        std::cout << "Setting up laser range finder..." << std::endl;
//        m_connected = m_port.OpenPort((char*)portname.c_str());
//        if (m_connected) {
//            std::cout << "Connected to " << portname << std::endl;
//            m_port.ConfigurePort(CBR_19200, 8, FALSE, NOPARITY, ONESTOPBIT);
//            m_port.SetCommunicationTimeouts(0, 0, 0, 0, 0);
//            laserOFF();
//        }
//        else {
//            std::cout << "Failed to connecte to port " << portname << std::endl;
//        }
//    }
//
//    //close device
//    void close() { if (m_connected) m_port.ClosePort(); }
//
//    //turn laser on
//    void laserON() {
//        if (m_connected) {
//            m_port.WriteByte('O');
//            unsigned char data[7];
//            for (int i = 0; i < 7; i++) {
//                m_port.ReadByte(data[i]);
//            }
//        }
//        else std::cout << "Laser range finder not connected" << std::endl;
//    }
//    //turn laser off
//    void laserOFF() {
//        if (m_connected) {
//            m_port.WriteByte('C');
//            unsigned char data[7];
//            for (int i = 0; i < 7; i++) {
//                m_port.ReadByte(data[i]);
//            }
//        }
//        else std::cout << "Laser range finder not connected" << std::endl;
//    }
//
//    //read laser distance
//    double readDistance() {
//        //your code here
//        //returned string is "D:xx.xxx"
//        //only retrieve 34.567
//        int size = 8;
//        std::vector<unsigned char> data(size);
//
//        if (m_connected) {
//            m_port.WriteByte('D');
//            for (int i = 0; i < size; i++) {
//                m_port.ReadByte(data[i]);
//            }
//        }
//        else std::cout << "Laser range finder not connected" << std::endl;
//
//        std::string distance = std::string(data.begin()+2, data.begin()+8);
//        double dis = std::stod(distance);
//        
//        cout << dis << endl; 
//        return dis; 
//    }
//
//    bool connected()const { return m_connected; }
//
//protected:
//    CSerialPort m_port;
//    bool m_connected = false;
//};
//


void main() {
    LaserRangeFinder laser_dev;
    MotorController motor1(SLAVE1);
    MotorController motor2(SLAVE2);

    //setup laser
    laser_dev.open("\\\\.\\COM27");
    //setup motor2
    motor1.open("\\\\.\\COM26");
    motor2.userPort(motor1);
    double a = laser_dev.readDistance() * 1000;



    //laser_dev.laserOFF();


    return;
}
