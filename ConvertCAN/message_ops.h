#pragma once

#include "leg_message.h"

#include <string>

std::string char2hexstr(const char* str, int len);

void pack_cmd(uint8_t* msg, int32_t len, joint_control& joint);
void zero_msg(uint8_t* msg, int32_t len);
void enter_motor_mode(uint8_t* msg, int32_t len);
void exit_motor_mode(uint8_t* msg, int32_t len);
