#pragma once
#include <vector>
#include <windows.h>
#include "SerialPort.h"

static const byte FUNC_RD = 0x03;
static const byte FUNC_WR = 0x06;
static const byte FUNC_DIAG = 0x07;
static const byte FUNC_WR_MULTIPLE = 0x10;
static const byte FUNC_RDWR_MULTIPLE = 0x17;
static const byte SLAVE1 = 0x01;
static const byte SLAVE2 = 0x02;
static const byte REMOTE_LOW_REG = 0x7D;
static const unsigned short M0_DEG_HIGH = 0x1802;
static const unsigned short M0_DEG_LOW = 0x1803;
static const unsigned short M1_DEG_HIGH = 0x1842;
static const unsigned short M1_DEG_LOW = 0x1843;

class INAmessage {
public:
    INAmessage() {}
    INAmessage(const byte addr, const byte func, const unsigned short reg_add, const unsigned short reg_data);
    INAmessage(const INAmessage& other) {}
    ////~INAmessage() {}

    int degreeY;
    int degreeX;
    unsigned int Length;
    byte Address;
    byte Function;
    std::vector<byte> data;
    std::vector<byte> source;
    unsigned short CRC16;
    byte CRC16L;
    byte CRC16H;
    unsigned short getCRC16(const std::vector<byte>& sources);
};

void sendMyMessage(const INAmessage& msg, CSerialPort& _serial);
void changeM0(INAmessage& msgHigh, INAmessage& msgLow, int stepsX, int stepsY);
