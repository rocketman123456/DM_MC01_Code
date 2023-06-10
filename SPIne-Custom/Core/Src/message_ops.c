#include "message_ops.h"
#include "math_ops.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/// Value Limits ///
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

/// Joint Soft Stops ///
#define A_LIM_P 1.5f
#define A_LIM_N -1.5f
#define H_LIM_P 5.0f
#define H_LIM_N -5.0f
#define K_LIM_P 0.2f
#define K_LIM_N 7.7f
#define KP_SOFTSTOP 100.0f
#define KD_SOFTSTOP 0.4f;

/// CAN Command Packet Structure ///
/// 16 bit position command, between -4*pi and 4*pi
/// 12 bit velocity command, between -30 and + 30 rad/s
/// 12 bit kp, between 0 and 500 N-m/rad
/// 12 bit kd, between 0 and 100 N-m*s/rad
/// 12 bit feed forward torque, between -18 and 18 N-m
/// CAN Packet is 8 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], kp[11-8]]
/// 4: [kp[7-0]]
/// 5: [kd[11-4]]
/// 6: [kd[3-0], torque[11-8]]
/// 7: [torque[7-0]]
void pack_cmd(uint8_t* msg, motor_cmd_t* joint)
{
    /// limit data to be within bounds ///
    float p_des = fminf_(fmaxf_(P_MIN, joint->p_des), P_MAX);
    float v_des = fminf_(fmaxf_(V_MIN, joint->v_des), V_MAX);
    float kp    = fminf_(fmaxf_(KP_MIN, joint->kp), KP_MAX);
    float kd    = fminf_(fmaxf_(KD_MIN, joint->kd), KD_MAX);
    float t_ff  = fminf_(fmaxf_(T_MIN, joint->t_ff), T_MAX);
    /// convert floats to unsigned ints ///
    uint16_t p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    uint16_t v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    /// pack ints into the can buffer ///
    msg[0] = p_int >> 8;
    msg[1] = p_int & 0xFF;
    msg[2] = v_int >> 4;
    msg[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg[4] = kp_int & 0xFF;
    msg[5] = kd_int >> 4;
    msg[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg[7] = t_int & 0xff;
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [id]
/// 1: [position[15-8]]
/// 2: [position[7-0]]
/// 3: [velocity[11-4]]
/// 4: [velocity[3-0], current[11-8]]
/// 5: [current[7-0]]
void unpack_reply(uint8_t* msg, leg_state_t* leg)
{
    /// unpack ints from can buffer ///
    uint16_t id    = msg[0];
    uint16_t p_int = ((uint16_t)(msg[1]) << 8) | (uint16_t)(msg[2]);
    uint16_t v_int = ((uint16_t)(msg[3]) << 4) | (uint16_t)(msg[4] >> 4);
    uint16_t i_int = ((uint16_t)(msg[4] & 0xF) << 8) | (uint16_t)(msg[5]);
    /// convert uints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

    if(id <= 3 && id > 0)
    {
        leg->motor[id - 1].p = p;
        leg->motor[id - 1].v = v;
        leg->motor[id - 1].t = t;
    }
}

void zero(uint8_t* msg)
{
    msg[0] = 0xFF;
    msg[1] = 0xFF;
    msg[2] = 0xFF;
    msg[3] = 0xFF;
    msg[4] = 0xFF;
    msg[5] = 0xFF;
    msg[6] = 0xFF;
    msg[7] = 0xFE;
}

void enter_motor_mode(uint8_t* msg)
{
    msg[0] = 0xFF;
    msg[1] = 0xFF;
    msg[2] = 0xFF;
    msg[3] = 0xFF;
    msg[4] = 0xFF;
    msg[5] = 0xFF;
    msg[6] = 0xFF;
    msg[7] = 0xFC;
}

void exit_motor_mode(uint8_t* msg)
{
    msg[0] = 0xFF;
    msg[1] = 0xFF;
    msg[2] = 0xFF;
    msg[3] = 0xFF;
    msg[4] = 0xFF;
    msg[5] = 0xFF;
    msg[6] = 0xFF;
    msg[7] = 0xFD;
}
