#include "leg_message.h"
#include "math_ops.h"
#include "mbed.h"
#include <cstring>

// length of receive/transmit buffers
#define RX_LEN 66
#define TX_LEN 66

// length of outgoing/incoming messages
//#define DATA_LEN 30
//#define CMD_LEN 66

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

spine_state_t spi_data;    // data from spine to up
spine_cmd_t   spi_command; // data from up to spine

// spi buffers
uint16_t rx_buff[RX_LEN];
uint16_t tx_buff[TX_LEN];

DigitalOut led(PC_5);

Serial pc(PA_2, PA_3);
CAN    can1(PB_12, PB_13, 1000000); // CAN Rx pin name, CAN Tx pin name
CAN    can2(PB_8, PB_9, 1000000);   // CAN Rx pin name, CAN Tx pin name

CANMessage rxMsg1, rxMsg2;
CANMessage txMsg1, txMsg2;
CANMessage a1_can, a2_can;
CANMessage h1_can, h2_can;
CANMessage k1_can, k2_can;

// int    ledState;
// Ticker sendCAN;
int    counter = 0;
// Ticker loop;

InterruptIn cs(PA_4);
DigitalIn   estop(PB_15);

leg_state_t l1_state, l2_state;
leg_cmd_t   l1_control, l2_control;

uint16_t x        = 0;
uint16_t count    = 0;
uint16_t counter2 = 0;

int control_mode = 1;
int is_standing  = 0;
int enabled      = 0;

static const uint8_t REFLECT_BIT_ORDER_TABLE[256] = {
    0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98,
    0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 0x0C, 0x8C, 0x4C, 0xCC,
    0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2,
    0x72, 0xF2, 0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA, 0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6,
    0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE, 0x01, 0x81,
    0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1, 0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9,
    0x39, 0xB9, 0x79, 0xF9, 0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5, 0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD,
    0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD, 0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
    0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB, 0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97,
    0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF};

// generates fake spi data from spi command
void test_control();
void control();

static uint32_t polynomial = 0x04C11DB7;
static uint32_t init_value = 0xFFFFFFFF;
static uint32_t xor_output = 0xFFFFFFFF;
static uint8_t  reflect_input = 1;
static uint8_t  reflect_output = 1;
static uint8_t  use_lut = 0;

uint32_t reflect(uint32_t value)
{
    uint32_t reflected = 0;
    for (int i = 0; i < 32; i++)
    {
        if (value & 0x01)
            reflected |= (1 << ((32 - 1) - i));
        value = (value >> 1);
    }
    return reflected;
}

uint32_t calculate(const uint8_t* data, const int length)
{
    uint32_t crc  = init_value;
    uint8_t  byte = 0;

    for (size_t i = 0; i < length; i++)
    {
        byte = reflect_input ? REFLECT_BIT_ORDER_TABLE[data[i]] : data[i];
        crc ^= (byte << 24);
        for (int j = 0; j < 8; j++)
        {
            crc = crc & 0x80000000 ? (crc << 1) ^ polynomial : crc << 1;
        }
    }

    if (reflect_output)
    {
        crc = reflect(crc);
    }

    crc = (crc ^ xor_output) & 0xFFFFFFFF;
    return crc;
}

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
void pack_cmd(CANMessage* msg, motor_cmd_t& joint)
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
    msg->data[0] = p_int >> 8;
    msg->data[1] = p_int & 0xFF;
    msg->data[2] = v_int >> 4;
    msg->data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg->data[4] = kp_int & 0xFF;
    msg->data[5] = kd_int >> 4;
    msg->data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg->data[7] = t_int & 0xff;
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void unpack_reply(CANMessage msg, leg_state_t* leg)
{
    /// unpack ints from can buffer ///
    uint16_t id    = msg.data[0];
    uint16_t p_int = (msg.data[1] << 8) | msg.data[2];
    uint16_t v_int = (msg.data[3] << 4) | (msg.data[4] >> 4);
    uint16_t i_int = ((msg.data[4] & 0xF) << 8) | msg.data[5];
    /// convert uints to floats ///
    float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

    if (id >= 1 && id <= 3)
    {
        leg->motor[id - 1].p = p;
        leg->motor[id - 1].v = v;
        leg->motor[id - 1].t = t;
    }
}

uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for (int i = 0; i < len; i++)
        t = t ^ data[i];
    return t;
}

void PackAll()
{
    pack_cmd(&a1_can, l1_control.motor[0]);
    pack_cmd(&a2_can, l2_control.motor[0]);
    pack_cmd(&h1_can, l1_control.motor[1]);
    pack_cmd(&h2_can, l2_control.motor[1]);
    pack_cmd(&k1_can, l1_control.motor[2]);
    pack_cmd(&k2_can, l2_control.motor[2]);
}

void WriteAll()
{
    // toggle = 1;
    can1.write(a1_can);
    wait(.00002);
    can2.write(a2_can);
    wait(.00002);
    can1.write(h1_can);
    wait(.00002);
    can2.write(h2_can);
    wait(.00002);
    can1.write(k1_can);
    wait(.00002);
    can2.write(k2_can);
    wait(.00002);
    // toggle = 0;
}

void Zero(CANMessage* msg)
{
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFE;
    // WriteAll();
}

void EnterMotorMode(CANMessage* msg)
{
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFC;
    // WriteAll();
}

void ExitMotorMode(CANMessage* msg)
{
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFD;
    // WriteAll();
}

void serial_isr()
{
    /// handle keyboard commands from the serial terminal ///
    while (pc.readable())
    {
        char c = pc.getc();
        // led = !led;
        switch (c)
        {
            case (27):
                // loop.detach();
                pc.printf("\n\r exiting motor mode \n\r");
                ExitMotorMode(&a1_can);
                ExitMotorMode(&a2_can);
                ExitMotorMode(&h1_can);
                ExitMotorMode(&h2_can);
                ExitMotorMode(&k1_can);
                ExitMotorMode(&k2_can);
                enabled = 0;
                break;
            case ('m'):
                pc.printf("\n\r entering motor mode \n\r");
                EnterMotorMode(&a1_can);
                EnterMotorMode(&a2_can);
                EnterMotorMode(&h1_can);
                EnterMotorMode(&h2_can);
                EnterMotorMode(&k1_can);
                EnterMotorMode(&k2_can);
                wait(.5);
                enabled = 1;
                break;
            case ('s'):
                pc.printf("\n\r standing \n\r");
                counter2    = 0;
                is_standing = 1;
                // stand();
                break;
            case ('z'):
                pc.printf("\n\r zeroing \n\r");
                Zero(&a1_can);
                Zero(&a2_can);
                Zero(&h1_can);
                Zero(&h2_can);
                Zero(&k1_can);
                Zero(&k2_can);
                break;
        }
    }
    WriteAll();
}

void spi_isr(void)
{
    led = !led;
    //pc.printf("\n\r enter SPI_ISR\n\r");

    GPIOC->ODR |= (1 << 8);
    GPIOC->ODR &= ~(1 << 8);

    int bytecount = 0;
    SPI1->DR      = tx_buff[0];
    while (cs == 0)
    {
        if (SPI1->SR & 0x1)
        {
            rx_buff[bytecount] = SPI1->DR;
            bytecount++;
            if (bytecount < TX_LEN)
            {
                SPI1->DR = tx_buff[bytecount];
            }
            // pc.printf("\n\r  SPI_ISR read \n\r");
        }
    }

    //pc.printf("read SPI_ISR finish \n");

    // after reading, save data into spi_command
    // should probably check checksum first!
    uint32_t calc_checksum = calculate((uint8_t*)rx_buff, sizeof(spine_cmd_t) - 4);
    memcpy(&spi_command, rx_buff, sizeof(spine_cmd_t));

    pc.write((uint8_t*)tx_buff, sizeof(spine_state_t), NULL);
    pc.write((uint8_t*)rx_buff, sizeof(spine_cmd_t), NULL);

    // run control, which fills in tx_buff for the next iteration
    if (calc_checksum != spi_command.crc)
    {
        spi_data.leg[0].flag = 0xdead;
        spi_data.leg[1].flag = 0xdead;
        pc.printf("checksum error SPI_ISR\n");
        return;
    }

    // test_control();
    control();
    PackAll();
    WriteAll();

    pc.printf("exit SPI_ISR\n");
}

int softstop_joint(motor_data_t& state, motor_cmd_t* control, float limit_p, float limit_n)
{
    if ((state.p) >= limit_p)
    {
        // control->p_des = limit_p;
        control->v_des = 0.0f;
        control->kp    = 0;
        control->kd    = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP * (limit_p - state.p);
        return 1;
    }
    else if ((state.p) <= limit_n)
    {
        // control->p_des = limit_n;
        control->v_des = 0.0f;
        control->kp    = 0;
        control->kd    = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP * (limit_n - state.p);
        return 1;
    }
    return 0;
}

void control()
{
    if (((spi_command.leg[0].flag & 0x1) == 1) && (enabled == 0))
    {
        enabled = 1;
        EnterMotorMode(&a1_can);
        can1.write(a1_can);
        EnterMotorMode(&a2_can);
        can2.write(a2_can);
        EnterMotorMode(&k1_can);
        can1.write(k1_can);
        EnterMotorMode(&k2_can);
        can2.write(k2_can);
        EnterMotorMode(&h1_can);
        can1.write(h1_can);
        EnterMotorMode(&h2_can);
        can2.write(h2_can);
        printf("enter motor mode\n");
        return;
    }
    else if ((((spi_command.leg[1].flag & 0x1)) == 0) && (enabled == 1))
    {
        enabled = 0;
        ExitMotorMode(&a1_can);
        can1.write(a1_can);
        ExitMotorMode(&a2_can);
        can2.write(a2_can);
        ExitMotorMode(&h1_can);
        can1.write(h1_can);
        ExitMotorMode(&h2_can);
        can2.write(h2_can);
        ExitMotorMode(&k1_can);
        can1.write(k1_can);
        ExitMotorMode(&k2_can);
        can2.write(k2_can);
        printf("exit motor mode\n");
        return;
    }

    spi_data.leg[0].motor[0].p = l1_state.motor[0].p;
    spi_data.leg[0].motor[1].p = l1_state.motor[1].p;
    spi_data.leg[0].motor[2].p = l1_state.motor[2].p;
    spi_data.leg[0].motor[0].v = l1_state.motor[0].v;
    spi_data.leg[0].motor[1].v = l1_state.motor[1].v;
    spi_data.leg[0].motor[2].v = l1_state.motor[2].v;
    spi_data.leg[0].motor[0].t = l1_state.motor[0].t;
    spi_data.leg[0].motor[1].t = l1_state.motor[1].t;
    spi_data.leg[0].motor[2].t = l1_state.motor[2].t;

    spi_data.leg[1].motor[0].p = l1_state.motor[0].p;
    spi_data.leg[1].motor[1].p = l1_state.motor[1].p;
    spi_data.leg[1].motor[2].p = l1_state.motor[2].p;
    spi_data.leg[1].motor[0].v = l1_state.motor[0].v;
    spi_data.leg[1].motor[1].v = l1_state.motor[1].v;
    spi_data.leg[1].motor[2].v = l1_state.motor[2].v;
    spi_data.leg[1].motor[0].t = l1_state.motor[0].t;
    spi_data.leg[1].motor[1].t = l1_state.motor[1].t;
    spi_data.leg[1].motor[2].t = l1_state.motor[2].t;

    if (estop == 0)
    {
        printf("estopped!!!!\n\r");
        memset(&l1_control, 0, sizeof(l1_control));
        memset(&l2_control, 0, sizeof(l2_control));
        spi_data.leg[0].flag = 0xdead;
        spi_data.leg[0].flag = 0xdead;
    }
    else
    {
        memset(&l1_control, 0, sizeof(l1_control));
        memset(&l2_control, 0, sizeof(l2_control));

        for(int i = 0; i < 3; ++i)
        {
            l1_control.motor[i].p_des = spi_command.leg[0].motor[i].p_des;
            l1_control.motor[i].v_des = spi_command.leg[0].motor[i].v_des;
            l1_control.motor[i].kp = spi_command.leg[0].motor[i].kp;
            l1_control.motor[i].kd = spi_command.leg[0].motor[i].kd;
            l1_control.motor[i].t_ff = spi_command.leg[0].motor[i].t_ff;
        }

        for(int i = 0; i < 3; ++i)
        {
            l2_control.motor[i].p_des = spi_command.leg[0].motor[i].p_des;
            l2_control.motor[i].v_des = spi_command.leg[0].motor[i].v_des;
            l2_control.motor[i].kp = spi_command.leg[0].motor[i].kp;
            l2_control.motor[i].kd = spi_command.leg[0].motor[i].kd;
            l2_control.motor[i].t_ff = spi_command.leg[0].motor[i].t_ff;
        }

        spi_data.leg[0].flag = 0;
        spi_data.leg[1].flag = 0;

        spi_data.leg[0].flag |= softstop_joint(l1_state.motor[0], &l1_control.motor[0], A_LIM_P, A_LIM_N);
        spi_data.leg[0].flag |= (softstop_joint(l1_state.motor[1], &l1_control.motor[1], H_LIM_P, H_LIM_N)) << 1;
        // spi_data.leg[0].flag |= (softstop_joint(l1_state.k, &l1_control.k, K_LIM_P, K_LIM_N))<<2;
        spi_data.leg[1].flag |= softstop_joint(l2_state.motor[0], &l2_control.motor[0], A_LIM_P, A_LIM_N);
        spi_data.leg[1].flag |= (softstop_joint(l2_state.motor[1], &l2_control.motor[1], H_LIM_P, H_LIM_N)) << 1;
        // spi_data.leg[1].flag |= (softstop_joint(l2_state.k, &l2_control.k, K_LIM_P, K_LIM_N))<<2;
    }

    spi_data.crc = calculate((uint8_t*)&spi_data, sizeof(spine_state_t) - 4);
    memcpy(tx_buff, &spi_data, sizeof(spine_state_t));
}

void test_control()
{
    for (int i = 0; i < 2; i++)
    {
        spi_data.leg[i].motor[0].p = spi_command.leg[i].motor[0].p_des + 1.f;
        spi_data.leg[i].motor[1].p = spi_command.leg[i].motor[1].p_des + 1.f;
        spi_data.leg[i].motor[2].p = spi_command.leg[i].motor[2].p_des + 1.f;

        spi_data.leg[i].motor[0].v = spi_command.leg[i].motor[0].v_des + 1.f;
        spi_data.leg[i].motor[1].v = spi_command.leg[i].motor[1].v_des + 1.f;
        spi_data.leg[i].motor[2].v = spi_command.leg[i].motor[2].v_des + 1.f;
    }

    spi_data.leg[0].flag = 0xdead;
    spi_data.leg[0].flag = 0xbeef;

    // only do first 56 bytes of message.
    spi_data.crc = calculate((uint8_t*)&spi_data, sizeof(spine_state_t) - 4);
    memcpy(tx_buff, &spi_data, sizeof(spine_state_t));
}

void init_spi(void)
{
    pc.printf("go to SPI Init \n");
    SPISlave* spi = new SPISlave(PA_7, PA_6, PA_5, PA_4);
    spi->format(16, 0);
    spi->frequency(6000000);
    spi->reply(0xff);
    cs.fall(&spi_isr);
    pc.printf("SPI Init done\n");
}

int main()
{
    wait(1);
    // led = 1;
    pc.baud(921600);
    pc.attach(&serial_isr);
    estop.mode(PullUp);

    // can1.frequency(1000000);                     // set bit rate to 1Mbps
    can1.filter(CAN_ID << 21, 0xFFE00004, CANStandard, 0); // set up can filter
    // can2.frequency(1000000);                     // set bit rate to 1Mbps
    can2.filter(CAN_ID << 21, 0xFFE00004, CANStandard, 0); // set up can filter

    memset(&tx_buff, 0, sizeof(spine_cmd_t));
    memset(&spi_data, 0, sizeof(spine_state_t));
    memset(&spi_command, 0, sizeof(spine_cmd_t));

    NVIC_SetPriority(TIM5_IRQn, 1);
    // NVIC_SetPriority(CAN1_RX0_IRQn, 3);
    // NVIC_SetPriority(CAN2_RX0_IRQn, 3);

    pc.printf("SPIne\n");
    // pc.printf("%d\n\r", RX_ID << 18);

    a1_can.len = 8; // transmit 8 bytes
    a2_can.len = 8; // transmit 8 bytes
    h1_can.len = 8;
    h2_can.len = 8;
    k1_can.len = 8;
    k2_can.len = 8;
    rxMsg1.len = 6; // receive 6 bytes
    rxMsg2.len = 6; // receive 6 bytes

    a1_can.id = 0x1;
    a2_can.id = 0x1;
    h1_can.id = 0x2;
    h2_can.id = 0x2;
    k1_can.id = 0x3;
    k2_can.id = 0x3;

    pack_cmd(&a1_can, l1_control.motor[0]);
    pack_cmd(&a2_can, l2_control.motor[0]);
    pack_cmd(&h1_can, l1_control.motor[1]);
    pack_cmd(&h2_can, l2_control.motor[1]);
    pack_cmd(&k1_can, l1_control.motor[2]);
    pack_cmd(&k2_can, l2_control.motor[2]);
    WriteAll();

    // SPI doesn't work if enabled while the CS pin is pulled low
    // Wait for CS to not be low, then enable SPI
    init_spi();

    led = 1;

    while (1)
    {
        // pc.printf("\n\r SPI_ISR\n\r");
        counter++;
        can2.read(rxMsg2);
        unpack_reply(rxMsg2, &l2_state);
        can1.read(rxMsg1); // read message into Rx message storage
        unpack_reply(rxMsg1, &l1_state);
        wait_us(10);
    }
}
