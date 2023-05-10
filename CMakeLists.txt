cmake_minimum_required(VERSION 3.15)

project(QuadrupedControl)

include_directories(
    3rdParty/CSerialPort/include
)

add_subdirectory(3rdParty)
add_subdirectory(ConvertCAN)
add_subdirectory(USB2CAN-Control)
