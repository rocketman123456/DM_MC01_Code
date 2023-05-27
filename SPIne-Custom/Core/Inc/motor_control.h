#pragma once

#include <stdint.h>

#define ABAD 0
#define HIP 1
#define KNEE 2

#define RF 0
#define LF 1
#define RH 2
#define LH 3

#pragma pack(1)

// 24 byte
typedef struct
{
    float p_des;
    float v_des;
    float kp;
    float kd;
    float t_ff;
} motor_cmd_t;

// 12 byte
typedef struct 
{
    float p;
    float v;
    float t;
} motor_data_t;

// 64 byte
typedef struct
{
    motor_cmd_t motor[3];
    int32_t flag;
} leg_cmd_t;

// 40 byte
typedef struct
{
    motor_data_t motor[3];
    int32_t flag;
} leg_state_t;

// 132 byte
typedef struct
{
    leg_cmd_t leg[2];
    uint32_t crc;
} spine_cmd_t;

// 84 byte
typedef struct
{
    leg_state_t leg[2];
    uint32_t crc;
} spine_state_t;

#pragma pack()

// typedef struct
// {
//     leg_cmd_t cmd[4];
// } robot_cmd_t;

// typedef struct
// {
//     leg_state_t state[4];
// } robot_state_t;

void control();
void fake_control();
