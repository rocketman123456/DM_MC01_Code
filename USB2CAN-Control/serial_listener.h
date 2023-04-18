#pragma once

#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

#include "leg_message.h"

class MotorListener : public itas109::CSerialPortListener
{
public:
    MotorListener(itas109::CSerialPort* sp) : p_sp(sp) {};

    void onReadEvent(const char* portName, unsigned int readBufferLen);

private:
    itas109::CSerialPort* p_sp;

public:
    joint_control control;
    leg_state     state;

private:
    uint8_t hex[8];
    int     countRead = 0;
};
