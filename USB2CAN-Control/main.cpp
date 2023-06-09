// main.cpp

#include "serial_listener.h"

#include "message_ops.h"

#ifdef _WIN32
#include <Windows.h>
#define imsleep(microsecond) Sleep(microsecond) // ms
#else
#include <unistd.h>
#define imsleep(microsecond) usleep(1000 * microsecond) // ms
#endif

#include <iostream>

using namespace itas109;

int main()
{
    CSerialPort sp;
    std::cout << "Version: " << sp.getVersion() << std::endl << std::endl;

    MotorListener listener(&sp);

    std::vector<SerialPortInfo> m_availablePortsList = CSerialPortInfo::availablePortInfos();

    std::cout << "availableFriendlyPorts: " << std::endl;

    for (size_t i = 1; i <= m_availablePortsList.size(); ++i)
    {
        SerialPortInfo serialPortInfo = m_availablePortsList[i - 1];
        std::cout << i << " - " << serialPortInfo.portName << " " << serialPortInfo.description << " " << serialPortInfo.hardwareId << std::endl;
    }

    if (m_availablePortsList.size() == 0)
    {
        std::cout << "No valid port" << std::endl;
    }
    else
    {
        std::cout << std::endl;

        int input = -1;
        do
        {
            std::cout << "Please Input The Index Of Port(1 - " << m_availablePortsList.size() << ")" << std::endl;

            std::cin >> input;

            if (input >= 1 && input <= m_availablePortsList.size())
            {
                break;
            }
        } while (true);

        const char* portName = m_availablePortsList[input - 1].portName;
        std::cout << "Port Name: " << portName << std::endl;

        sp.init(portName,                // windows:COM1 Linux:/dev/ttyS0
                itas109::BaudRate115200, // baudrate
                itas109::ParityNone,     // parity
                itas109::DataBits8,      // data bit
                itas109::StopOne,        // stop bit
                itas109::FlowNone,       // flow
                4096                     // read buffer size
        );
        sp.setReadIntervalTimeout(0); // read interval timeout 0ms

        sp.open();
        std::cout << "Open " << portName << (sp.isOpen() ? " Success" : " Failed") << std::endl;

        // connect for read
        sp.connectReadEvent(&listener);

        // enter motor mode
        uint8_t hex[8];
        std::cout << "enter motor mode" << std::endl;
        enter_motor_mode(hex, 8);
        sp.writeData(hex, sizeof(hex));

        imsleep(1);

        listener.control.p_des = 0;
        listener.control.v_des = 0;
        listener.control.t_ff = 0;
        listener.control.kp = 15.0;
        listener.control.kd = 0;

        pack_cmd(hex, 8, listener.control);

        auto hex_str = char2hexstr((char*)hex, 8);
        std::cout << hex_str << std::endl;

        sp.writeData(hex, sizeof(hex));

        for (;;)
        {
            imsleep(1);
        }
    }

    return 0;
}
