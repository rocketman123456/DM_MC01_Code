#pragma once

#include "leg_message.h"

void pack_cmd(uint8_t* msg, int32_t len, joint_control& joint);
void unpack_reply(uint8_t* msg, int32_t len, leg_state* leg);
void zero_msg(uint8_t* msg, int32_t len);
void enter_motor_mode(uint8_t* msg, int32_t len);
void exit_motor_mode(uint8_t* msg, int32_t len);
