#pragma once

#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

class MotorListener : public itas109::CSerialPortListener
{
public:
    MotorListener(itas109::CSerialPort* sp) : p_sp(sp) {};

    void onReadEvent(const char* portName, unsigned int readBufferLen);

private:
    itas109::CSerialPort* p_sp;

    int countRead = 0;
};
