#pragma once
#include <stdio.h>
#include <string>
#include "SerialPort.h"
#include <thread>
#include <opencv2/core.hpp>

class MotorController {
public:
    MotorController(ushort slaveid) { m_slaveid = slaveid; }
    ~MotorController() { close(); }

    void open(std::string const& portname);
    void stop();
    void jogFW();
    void jogBW();
    void setPos(double angle);    //close device
    void close();
    bool connected()const { return m_connected; }
    void userPort(MotorController const & motorcon);


protected:
    CSerialPort m_port;
    ushort m_slaveid = 0;
    bool m_connected = false;
};
