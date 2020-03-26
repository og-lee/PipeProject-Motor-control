#include "MotorController.h"
#include "INAmessage.h"


    MotorController::MotorController(ushort slaveid) { m_slaveid = slaveid; }
    MotorController::~MotorController() { close(); }

    void MotorController::open(std::string const& portname) {
        std::cout << "Setting up motor controller " << m_slaveid << "..." << std::endl;
        m_connected = m_port.OpenPort((char*)portname.c_str());
        if (m_connected) {
            std::cout << "Connected to " << portname << std::endl;
            m_port.ConfigurePort(CBR_115200, 8, FALSE, EVENPARITY, ONESTOPBIT);
            m_port.SetCommunicationTimeouts(0, 0, 0, 0, 0);
        }
        else {
            std::cout << "Failed to connecte to port " << portname << std::endl;
        }
    }

    void MotorController::stop() {
        if (m_connected) {
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, REMOTE_LOW_REG, 0x0020,true), m_port);
            std::this_thread::sleep_for(5ms);
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }
    void MotorController::jogFW() {
        if (m_connected) {
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, REMOTE_LOW_REG, 0x0020,true), m_port);
            std::this_thread::sleep_for(5ms);
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, REMOTE_LOW_REG, 0x1000,true), m_port);
            std::this_thread::sleep_for(5ms);
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }

    //INAmessage inside sendmessage() ? every time i call  
    void MotorController::jogBW() {
        if (m_connected) {
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, REMOTE_LOW_REG, 0x0020,true), m_port);
            std::this_thread::sleep_for(5ms);
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, REMOTE_LOW_REG, 0x2000,true), m_port);
            std::this_thread::sleep_for(5ms);
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }


    void MotorController::setPos(double angle) {
        //ex 123.17 ==> 12317 
        if (m_connected) {
            int angleINT = (int)(angle * 100 + 0.5);
            //change M0 operation absolute degree;
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, M0_DEG_HIGH, (ushort)(angleINT >> 16),true), m_port);
            std::this_thread::sleep_for(5ms);
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, M0_DEG_LOW, (ushort)(angleINT & 0xffff),true), m_port);
            std::this_thread::sleep_for(5ms);
            // now operate M0 operation 
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, REMOTE_LOW_REG, 0x0020,true), m_port);
            std::this_thread::sleep_for(5ms);
            sendMyMessage(INAmessage(m_slaveid, FUNC_WR, REMOTE_LOW_REG, 0x0008,true), m_port);
            std::this_thread::sleep_for(5ms);
        }
        else std::cout << "port not connected" << endl;
    }
    
    //close device
    void MotorController::close() { if (m_connected) m_port.ClosePort(); }

    bool MotorController::connected()const { return m_connected; }

    void MotorController::userPort(MotorController const & motorcon) {
        m_port = motorcon.m_port;
        m_connected = motorcon.m_connected;
    }


    double MotorController::readPos() {
        if (m_connected) {


        }
        else std::cout << "port not connected" << endl;
        return 0;
    }
