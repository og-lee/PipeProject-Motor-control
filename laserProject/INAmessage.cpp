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


void changeM0(INAmessage& msgHigh, INAmessage& msgLow, int stepsX, int stepsY) {
    unsigned short stepYHigh = (unsigned short)(stepsY >> 16);
    unsigned short stepYLow = (unsigned short)(stepsY & 0xffff);
    unsigned short stepXHigh = (unsigned short)(stepsX >> 16);
    unsigned short stepXLow = (unsigned short)(stepsX & 0xffff);
}

double readPosition(INAmessage& msg, CSerialPort& _serial) {
    std::vector<byte> response = sendMyMessage(msg, _serial);
    // length is 8 + 9 = 17 
    // data part is 11 12 13 14 
    

}
