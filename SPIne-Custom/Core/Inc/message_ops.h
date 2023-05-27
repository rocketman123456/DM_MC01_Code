#pragma once
#include "motor_control.h"

void pack_cmd(uint8_t* msg, motor_cmd_t* joint);
uint16_t unpack_reply(uint8_t* msg, leg_state_t* leg);

void zero(uint8_t* msg);
void enter_motor_mode(uint8_t* msg);
void exit_motor_mode(uint8_t* msg);
