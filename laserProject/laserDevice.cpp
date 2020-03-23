#include "laserDevice.h"

    LaserRangeFinder::~LaserRangeFinder() { close(); }

    void LaserRangeFinder::open(std::string const& portname) {
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
    void LaserRangeFinder::close() { if (m_connected) m_port.ClosePort(); }

    //turn laser on
    void LaserRangeFinder::laserON() {
        if (m_connected) {
            m_port.WriteByte('O');
            unsigned char data[7];
            for (int i = 0; i < 7; i++) {
                m_port.ReadByte(data[i]);
            }
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }
    //turn laser off
    void LaserRangeFinder::laserOFF() {
        if (m_connected) {
            m_port.WriteByte('C');
            unsigned char data[7];
            for (int i = 0; i < 7; i++) {
                m_port.ReadByte(data[i]);
            }
        }
        else std::cout << "Laser range finder not connected" << std::endl;
    }

    //read laser distance
    double LaserRangeFinder::readDistance() {
        //your code here
        //returned string is "D:xx.xxx"
        //only retrieve 34.567
        int size = 8;
        std::vector<unsigned char> data(size);

        if (m_connected) {
            m_port.WriteByte('D');
            for (int i = 0; i < size; i++) {
                m_port.ReadByte(data[i]);
            }
        }
        else std::cout << "Laser range finder not connected" << std::endl;

        std::string distance = std::string(data.begin() + 2, data.begin() + 8);
        double dis = std::stod(distance);

        cout << dis << endl;
        return dis;
    }

    bool LaserRangeFinder::connected()const { return m_connected; }

