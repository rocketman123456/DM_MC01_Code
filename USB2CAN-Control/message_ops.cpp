#include "leg_message.h"

#include "math_ops.h"

void zero_msg(uint8_t* msg, int32_t len)
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

void enter_motor_mode(uint8_t* msg, int32_t len)
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

void exit_motor_mode(uint8_t* msg, int32_t len)
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

// CAN Command Packet Structure ///
// 16 bit position command, between -4*pi and 4*pi
// 12 bit velocity command, between -30 and + 30 rad/s
// 12 bit kp, between 0 and 500 N-m/rad
// 12 bit kd, between 0 and 100 N-m*s/rad
// 12 bit feed forward torque, between -18 and 18 N-m
// CAN Packet is 8 8-bit words
// Formatted as follows.  For each quantity, bit 0 is LSB
// 0: [position[15-8]]
// 1: [position[7-0]]
// 2: [velocity[11-4]]
// 3: [velocity[3-0], kp[11-8]]
// 4: [kp[7-0]]
// 5: [kd[11-4]]
// 6: [kd[3-0], torque[11-8]]
// 7: [torque[7-0]]
void pack_cmd(uint8_t* msg, int32_t len, joint_control& joint)
{
    /// limit data to be within bounds ///
    float p_des = fminf(fmaxf(P_MIN, joint.p_des), P_MAX);
    float v_des = fminf(fmaxf(V_MIN, joint.v_des), V_MAX);
    float kp    = fminf(fmaxf(KP_MIN, joint.kp), KP_MAX);
    float kd    = fminf(fmaxf(KD_MIN, joint.kd), KD_MAX);
    float t_ff  = fminf(fmaxf(T_MIN, joint.t_ff), T_MAX);
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

// CAN Reply Packet Structure ///
// 16 bit position, between -4*pi and 4*pi
// 12 bit velocity, between -30 and + 30 rad/s
// 12 bit current, between -40 and 40;
// CAN Packet is 5 8-bit words
// Formatted as follows.  For each quantity, bit 0 is LSB
// 0: [position[15-8]]
// 1: [position[7-0]]
// 2: [velocity[11-4]]
// 3: [velocity[3-0], current[11-8]]
// 4: [current[7-0]]
void unpack_reply(uint8_t* msg, int32_t len, leg_state* leg)
{
    /// unpack ints from can buffer ///
    uint16_t id    = msg[0];
    uint16_t p_int = (msg[1] << 8) | msg[2];
    uint16_t v_int = (msg[3] << 4) | (msg[4] >> 4);
    uint16_t i_int = ((msg[4] & 0xF) << 8) | msg[5];
    /// convert uints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

    if (id == 1)
    {
        leg->a.p = p;
        leg->a.v = v;
        leg->a.t = t;
    }
    else if (id == 2)
    {
        leg->h.p = p;
        leg->h.v = v;
        leg->h.t = t;
    }
    else if (id == 3)
    {
        leg->k.p = p;
        leg->k.v = v;
        leg->k.t = t;
    }
}
