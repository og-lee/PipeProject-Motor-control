#pragma once
#include "SerialPort.h"
#include <iostream>
#include <string>
using namespace std;

class LaserRangeFinder {
public:
    ~LaserRangeFinder();

    void open(std::string const& portname);    //close device
    void close();
    //turn laser on
    void laserON();    //turn laser off
    void laserOFF();
    //read laser distance
    double readDistance();
    bool connected()const;

protected:
    CSerialPort m_port;
    bool m_connected = false;
};

