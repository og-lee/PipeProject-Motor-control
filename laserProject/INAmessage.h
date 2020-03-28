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
static const byte READ_POS_REG_HIGH = 0x00;
static const byte READ_POS_REG_LOW = 0xc6;
static const unsigned short M0_DEG_HIGH = 0x1802;
static const unsigned short M0_DEG_LOW = 0x1803;
static const unsigned short M1_DEG_HIGH = 0x1842;
static const unsigned short M1_DEG_LOW = 0x1843;

class INAmessage {
public:
    INAmessage() {}
    INAmessage(const byte addr,
        const byte func,
        const unsigned short reg_add,
        const unsigned short reg_data,
        bool write);

    INAmessage(const INAmessage& other) {}
    ////~INAmessage() {}

    bool isWrite;
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

std::vector<byte> sendMyMessage(const INAmessage& msg, CSerialPort& _serial);
void changeM0(INAmessage& msgHigh, INAmessage& msgLow, int stepsX, int stepsY);
int readPosition(INAmessage& msg, CSerialPort& _serial);


//S1 is hor motor 
static INAmessage msg_MOVEM0(SLAVE1, FUNC_WR, REMOTE_LOW_REG, 0x0008,true);
static INAmessage msg_MOVEM1(SLAVE1, FUNC_WR, REMOTE_LOW_REG, 0x0009,true);


static INAmessage msg_M0(SLAVE1, FUNC_WR, REMOTE_LOW_REG, 0x0009,true);

static INAmessage msg_JOGFW(SLAVE2, FUNC_WR, REMOTE_LOW_REG, 0x1000,true);
static INAmessage msg_JOGBW(SLAVE2, FUNC_WR, REMOTE_LOW_REG, 0x2000,true);
static INAmessage msg_STOP(SLAVE2, FUNC_WR, REMOTE_LOW_REG, 0x0020,true);
static INAmessage msg_READ_S2(SLAVE2, FUNC_RD, READ_POS_REG_LOW,0x02,false);
static INAmessage msg_READ_S1(SLAVE1, FUNC_RD, READ_POS_REG_LOW,0x02,false);

