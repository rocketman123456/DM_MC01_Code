#include "serial_listener.h"
#include "message_ops.h"

#include <iostream>

using namespace itas109;

void MotorListener::onReadEvent(const char* portName, unsigned int readBufferLen)
{
    if (readBufferLen > 0)
    {
        char* data = new char[readBufferLen + 1]; // '\0'

        if (data)
        {
            // read
            int recLen = p_sp->readData(data, readBufferLen);

            if (recLen > 0)
            {
                data[recLen] = '\0';
                std::cout << portName << " - Count: " << ++countRead << ", Len: " << recLen << ", Str: " << data
                          << ", Hex: " << char2hexstr(data, recLen).c_str() << std::endl;

                // get motor state
                unpack_reply((uint8_t*)data, recLen, &state);
                std::cout << "Pos: " << state.a.p << "Vel: " << state.a.v << "Tau: " << state.a.t << std::endl;
                // send next control
                pack_cmd(hex, 8, control);
                p_sp->writeData(hex, 8);
            }

            delete[] data;
            data = NULL;
        }
    }
};
