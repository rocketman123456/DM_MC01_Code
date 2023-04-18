#include "serial_listener.h"

#include <iostream>

using namespace itas109;

std::string char2hexstr(const char* str, int len)
{
    static const char hexTable[17] = "0123456789ABCDEF";

    std::string result;
    for (int i = 0; i < len; ++i)
    {
        result += "0x";
        result += hexTable[(unsigned char)str[i] / 16];
        result += hexTable[(unsigned char)str[i] % 16];
        result += " ";
    }
    return result;
}

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
                std::cout << portName << " - Count: " << ++countRead << ", Length: " << recLen << ", Str: " << data
                          << ", Hex: " << char2hexstr(data, recLen).c_str() << std::endl;

                // return receive data
                // p_sp->writeData(data, recLen);
            }

            delete[] data;
            data = NULL;
        }
    }
};