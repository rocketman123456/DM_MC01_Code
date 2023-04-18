#pragma once

// length of receive/transmit buffers
#define RX_LEN 66
#define TX_LEN 66

// length of outgoing/incoming messages
#define DATA_LEN 30
#define CMD_LEN 66

// Master CAN ID ///
#define CAN_ID 0x0

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

#define ENABLE_CMD 0xFFFF
#define DISABLE_CMD 0x1F1F

#include <stdint.h>

// 60 bytes
// 30 16-bit words
struct spi_data_t
{
    float   q_abad[2];
    float   q_hip[2];
    float   q_knee[2];
    float   qd_abad[2];
    float   qd_hip[2];
    float   qd_knee[2];
    int32_t flags[2];
    int32_t checksum;
};

// 132 bytes
// 66 16-bit words
struct spi_command_t
{
    float   q_des_abad[2];
    float   q_des_hip[2];
    float   q_des_knee[2];
    float   qd_des_abad[2];
    float   qd_des_hip[2];
    float   qd_des_knee[2];
    float   kp_abad[2];
    float   kp_hip[2];
    float   kp_knee[2];
    float   kd_abad[2];
    float   kd_hip[2];
    float   kd_knee[2];
    float   tau_abad_ff[2];
    float   tau_hip_ff[2];
    float   tau_knee_ff[2];
    int32_t flags[2];
    int32_t checksum;
};

struct joint_control
{
    float p_des, v_des, kp, kd, t_ff;
};

struct joint_state
{
    float p, v, t;
};

struct leg_state
{
    joint_state a, h, k;
};

struct leg_control
{
    joint_control a, h, k;
};


