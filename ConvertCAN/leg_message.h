#pragma once

// length of receive/transmit buffers
#define RX_LEN 66
#define TX_LEN 66

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

struct joint_control
{
    float p_des, v_des, kp, kd, t_ff;
};

struct joint_state
{
    float p, v, t;
};
