#pragma once

#include "motor_control.h"

void pack_cmd(uint8_t* msg, motor_cmd_t joint);
void unpack_reply(uint8_t* msg, leg_state_t* leg);
