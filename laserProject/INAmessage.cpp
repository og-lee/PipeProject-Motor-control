#include "INAmessage.h"

INAmessage::INAmessage(const byte addr, 
    const byte func, 
    const unsigned short reg_add, 
    const unsigned short reg_data,
    bool write) {

    // if is WriteByte , then slave,add,regAdd,regData,CRC4byte 
    // if is ReadByte  is   slave,add,regAdd,
    isWrite = write;
    Address = addr;
    Function = func;
    data.resize(4);
    data[0] = (byte)(reg_add >> 8);
    data[1] = (byte)(reg_add & 0xff);
    data[2] = (byte)(reg_data >> 8);
    data[3] = (byte)(reg_data & 0xff);
    //std::vector<byte>sources{ Address,Function,data[0],data[1],data[2],data[3] };
    source = { Address,Function,data[0],data[1],data[2],data[3] };
    CRC16 = getCRC16(source);
    CRC16H = (byte)(CRC16 >> 8);
    CRC16L = (byte)(CRC16 & 0x00ff);
    if (write == true) {
        Length = ((data.size() + 4)*2);
    }
    else {
        Length = (data.size() + 4)*2+1;
    }
    // addr,func,reg_add,reg_data,crc
    //  1    1    (data[0 1 2 3])  2  
}

unsigned short INAmessage::getCRC16(const std::vector<byte>& sources) {
    unsigned short temp;
    unsigned short initial = 0xffff;
    byte resultLow;
    byte resultHigh;
    for (int j = 0; j < sources.size(); j++)
    {
        byte compareByte = sources[j];
        for (int i = 0; i < 8; i++)
        {

            if (i == 0 && j == 0)          // only the first time xor with the byte 
            {
                resultLow = (byte)(initial ^ compareByte);
                resultHigh = (byte)((initial >> 8) ^ (compareByte >> 8));
            }


            if (i == 0 && j != 0)
            {
                resultLow = (byte)(resultLow ^ compareByte);
                resultHigh = (byte)(resultHigh ^ (compareByte >> 8));

            }

            byte checkLSB = (byte)(resultLow & 0x01);

            temp = resultLow | (resultHigh << 8);
            //temp = BitConverter.ToUInt16(new byte[2]{ resultLow, resultHigh }, 0);
            temp = (unsigned short)(temp >> 1);

            if (checkLSB == 1)
            {
                resultLow = (byte)(temp ^ 0x01);
                resultHigh = (byte)((temp >> 8) ^ 0xA0);
            }
            else
            {
                resultLow = (byte)(temp);
                resultHigh = (byte)(temp >> 8);
            }
        }
    }
    unsigned short final = resultLow | (resultHigh << 8);
    return final;
}

std::vector<byte> sendMyMessage(const INAmessage& msg, CSerialPort& _serial) {
    std::vector<byte> response(msg.Length);
    _serial.WriteByte(msg.Address);
    _serial.WriteByte(msg.Function);
    for (int i = 0; i < msg.data.size(); i++) {
        _serial.WriteByte(msg.data[i]);
    }
    _serial.WriteByte(msg.CRC16L);
    _serial.WriteByte(msg.CRC16H);
    Sleep(4);
    for (int i = 0; i < msg.Length; i++) {
        _serial.ReadByte(response[i]);
    }
    Sleep(4);
    return response;
}


void changeM0_HOR(INAmessage& msgHigh,
    INAmessage& msgLow,
    int position,
    MotorController &motor1) {
    //this will change the register data[2] data[3] before sendMyMessage() is called 
    unsigned short stepXHigh = (unsigned short)(position >> 16);
    unsigned short stepXLow = (unsigned short)(position & 0xffff);
    msgLow.data[2] = (byte)(stepXLow >> 8);
    msgLow.data[3] = (byte)(stepXLow & 0xff);

    msgLow.source = {msgLow.Address,msgLow.Function,msgLow.data[0],msgLow.data[1],msgLow.data[2],msgLow.data[3] };
    msgLow.CRC16 = msgLow.getCRC16(msgLow.source);
    msgLow.CRC16H = (byte)(msgLow.CRC16 >> 8);
    msgLow.CRC16L = (byte)(msgLow.CRC16 & 0x00ff);

    msgHigh.data[2] = (byte)(stepXHigh >> 8);
    msgHigh.data[3] = (byte)(stepXHigh & 0xff);
    
    msgHigh.source = { msgHigh.Address,msgHigh.Function,msgHigh.data[0],msgHigh.data[1],msgHigh.data[2],msgHigh.data[3] };
    msgHigh.CRC16 = msgHigh.getCRC16(msgHigh.source);
    msgHigh.CRC16H = (byte)(msgHigh.CRC16 >> 8);
    msgHigh.CRC16L = (byte)(msgHigh.CRC16 & 0x00ff);

    std::vector<byte>ret = sendMyMessage(msgHigh, motor1.m_port);
    //sendMyMessage(msg_STOP_SLAVE1, motor1.m_port);
    sendMyMessage(msgLow, motor1.m_port);
}

int readPosition(INAmessage& msg, CSerialPort& _serial) {
    std::vector<byte> response = sendMyMessage(msg, _serial);
    // length is 8 + 9 = 17 
    // data part is 11 12 13 14 
    int degree = 0;            // degree is 32 bit integer
    degree = degree | response[11] << 24; 
    degree = degree | response[12] << 16; 
    degree = degree | response[13] << 8;
    degree = degree | response[14] ;
    return degree;
}


void moveM0_SLAVE1(CSerialPort& _serial) {
    sendMyMessage(msg_STOP_SLAVE1, _serial);
    sendMyMessage(msg_MOVEM0_SLAVE1, _serial);
}


bool isSlave1MotorMoving(CSerialPort& _serial) {
    std::vector<byte> response = sendMyMessage(msg_READ_MOVE_SLAVE1, _serial);
    int a = (response[13] >> 5) & 1;
    if (a) { 
        cout << "motor moving" << endl;
        return true; 
    }
    cout << "motor stopped" << endl;
    return false;
}
