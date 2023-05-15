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

spi_data_t    spi_data;    // data from spine to up
spi_command_t spi_command; // data from up to spine

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

int    ledState;
Ticker sendCAN;
int    counter = 0;
Ticker loop;

InterruptIn cs(PA_4);
DigitalIn   estop(PB_15);

leg_state l1_state, l2_state;
leg_control l1_control, l2_control;

uint16_t x        = 0;
uint16_t count    = 0;
uint16_t counter2 = 0;

int control_mode = 1;
int is_standing  = 0;
int enabled      = 0;

// generates fake spi data from spi command
void test_control();
void control();

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
void pack_cmd(CANMessage* msg, joint_control joint)
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
void unpack_reply(CANMessage msg, leg_state* leg)
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

uint32_t xor_checksum(uint32_t* data, size_t len)
{
    uint32_t t = 0;
    for (int i = 0; i < len; i++)
        t = t ^ data[i];
    return t;
}

void rxISR1()
{
    can1.read(rxMsg1); // read message into Rx message storage
    unpack_reply(rxMsg1, &l1_state);
}

void rxISR2()
{
    can2.read(rxMsg2);
    unpack_reply(rxMsg2, &l2_state);
}

void PackAll()
{
    pack_cmd(&a1_can, l1_control.a);
    pack_cmd(&a2_can, l2_control.a);
    pack_cmd(&h1_can, l1_control.h);
    pack_cmd(&h2_can, l2_control.h);
    pack_cmd(&k1_can, l1_control.k);
    pack_cmd(&k2_can, l2_control.k);
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

void sendCMD()
{
    counter++;

    PackAll();

    if (counter > 100)
    {
        printf("%.3f %.3f %.3f   %.3f %.3f %.3f\n\r", l1_state.a.p, l1_state.h.p, l1_state.k.p, l2_state.a.p, l2_state.h.p, l2_state.k.p);
        counter = 0;
    }

    WriteAll();
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
                // loop.attach(&sendCMD, .001);
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
    pc.printf("\n\r enter SPI_ISR\n\r");

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

    pc.printf("\n\r read SPI_ISR finish \n\r");

    // after reading, save data into spi_command
    // should probably check checksum first!
    uint32_t calc_checksum = xor_checksum((uint32_t*)rx_buff, sizeof(spi_command_t) / 2 - 1);
    memcpy(&spi_command, rx_buff, sizeof(spi_command_t));
    //for (int i = 0; i < CMD_LEN; i++)
    //{
    //    ((uint16_t*)(&spi_command))[i] = rx_buff[i];
    //}

    // run control, which fills in tx_buff for the next iteration
    if (calc_checksum != spi_command.checksum)
    {
        spi_data.flags[0] = 0xdead;
        spi_data.flags[1] = 0xdead;
    }

    test_control();
    // control();
    // PackAll();
    // WriteAll();

    pc.printf("\n\r exit SPI_ISR\n\r");
}

int softstop_joint(joint_state state, joint_control* control, float limit_p, float limit_n)
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
    if (((spi_command.flags[0] & 0x1) == 1) && (enabled == 0))
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
        printf("enter motor mode\n\r");
        return;
    }
    else if ((((spi_command.flags[0] & 0x1)) == 0) && (enabled == 1))
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
        printf("exit motor mode\n\r");
        return;
    }

    spi_data.q_abad[0]  = l1_state.a.p;
    spi_data.q_hip[0]   = l1_state.h.p;
    spi_data.q_knee[0]  = l1_state.k.p;
    spi_data.qd_abad[0] = l1_state.a.v;
    spi_data.qd_hip[0]  = l1_state.h.v;
    spi_data.qd_knee[0] = l1_state.k.v;
    spi_data.tau_abad[0] = l1_state.a.t;
    spi_data.tau_hip[0]  = l1_state.h.t;
    spi_data.tau_knee[0] = l1_state.k.t;

    spi_data.q_abad[1]  = l2_state.a.p;
    spi_data.q_hip[1]   = l2_state.h.p;
    spi_data.q_knee[1]  = l2_state.k.p;
    spi_data.qd_abad[1] = l2_state.a.v;
    spi_data.qd_hip[1]  = l2_state.h.v;
    spi_data.qd_knee[1] = l2_state.k.v;
    spi_data.tau_abad[1] = l1_state.a.t;
    spi_data.tau_hip[1]  = l1_state.h.t;
    spi_data.tau_knee[1] = l1_state.k.t;

    if (estop == 0)
    {
        printf("estopped!!!!\n\r");
        memset(&l1_control, 0, sizeof(l1_control));
        memset(&l2_control, 0, sizeof(l2_control));
        spi_data.flags[0] = 0xdead;
        spi_data.flags[1] = 0xdead;
        //led               = 1;
    }
    else
    {
        //led = 0;

        memset(&l1_control, 0, sizeof(l1_control));
        memset(&l2_control, 0, sizeof(l2_control));

        l1_control.a.p_des = spi_command.q_des_abad[0];
        l1_control.a.v_des = spi_command.qd_des_abad[0];
        l1_control.a.kp    = spi_command.kp_abad[0];
        l1_control.a.kd    = spi_command.kd_abad[0];
        l1_control.a.t_ff  = spi_command.tau_abad_ff[0];

        l1_control.h.p_des = spi_command.q_des_hip[0];
        l1_control.h.v_des = spi_command.qd_des_hip[0];
        l1_control.h.kp    = spi_command.kp_hip[0];
        l1_control.h.kd    = spi_command.kd_hip[0];
        l1_control.h.t_ff  = spi_command.tau_hip_ff[0];

        l1_control.k.p_des = spi_command.q_des_knee[0];
        l1_control.k.v_des = spi_command.qd_des_knee[0];
        l1_control.k.kp    = spi_command.kp_knee[0];
        l1_control.k.kd    = spi_command.kd_knee[0];
        l1_control.k.t_ff  = spi_command.tau_knee_ff[0];

        l2_control.a.p_des = spi_command.q_des_abad[1];
        l2_control.a.v_des = spi_command.qd_des_abad[1];
        l2_control.a.kp    = spi_command.kp_abad[1];
        l2_control.a.kd    = spi_command.kd_abad[1];
        l2_control.a.t_ff  = spi_command.tau_abad_ff[1];

        l2_control.h.p_des = spi_command.q_des_hip[1];
        l2_control.h.v_des = spi_command.qd_des_hip[1];
        l2_control.h.kp    = spi_command.kp_hip[1];
        l2_control.h.kd    = spi_command.kd_hip[1];
        l2_control.h.t_ff  = spi_command.tau_hip_ff[1];

        l2_control.k.p_des = spi_command.q_des_knee[1];
        l2_control.k.v_des = spi_command.qd_des_knee[1];
        l2_control.k.kp    = spi_command.kp_knee[1];
        l2_control.k.kd    = spi_command.kd_knee[1];
        l2_control.k.t_ff  = spi_command.tau_knee_ff[1];

        spi_data.flags[0] = 0;
        spi_data.flags[1] = 0;
        spi_data.flags[0] |= softstop_joint(l1_state.a, &l1_control.a, A_LIM_P, A_LIM_N);
        spi_data.flags[0] |= (softstop_joint(l1_state.h, &l1_control.h, H_LIM_P, H_LIM_N)) << 1;
        // spi_data.flags[0] |= (softstop_joint(l1_state.k, &l1_control.k, K_LIM_P, K_LIM_N))<<2;
        spi_data.flags[1] |= softstop_joint(l2_state.a, &l2_control.a, A_LIM_P, A_LIM_N);
        spi_data.flags[1] |= (softstop_joint(l2_state.h, &l2_control.h, H_LIM_P, H_LIM_N)) << 1;
        // spi_data.flags[1] |= (softstop_joint(l2_state.k, &l2_control.k, K_LIM_P, K_LIM_N))<<2;

        // spi_data.flags[0] = 0xbeef;
        // spi_data.flags[1] = 0xbeef;
        // PackAll();
        // WriteAll();
    }

    spi_data.checksum = xor_checksum((uint32_t*)&spi_data, sizeof(spi_data_t) / 2 - 1);
    memcpy(tx_buff, &spi_data, sizeof(spi_data_t));
    //for (int i = 0; i < DATA_LEN; i++)
    //{
    //    tx_buff[i] = ((uint16_t*)(&spi_data))[i];
    //}
}

void test_control()
{
    for (int i = 0; i < 2; i++)
    {
        spi_data.q_abad[i] = spi_command.q_des_abad[i] + 1.f;
        spi_data.q_knee[i] = spi_command.q_des_knee[i] + 1.f;
        spi_data.q_hip[i]  = spi_command.q_des_hip[i] + 1.f;

        spi_data.qd_abad[i] = spi_command.qd_des_abad[i] + 1.f;
        spi_data.qd_knee[i] = spi_command.qd_des_knee[i] + 1.f;
        spi_data.qd_hip[i]  = spi_command.qd_des_hip[i] + 1.f;
    }

    spi_data.flags[0] = 0xdead;
    spi_data.flags[1] = 0xbeef;

    // only do first 56 bytes of message.
    spi_data.checksum = xor_checksum((uint32_t*)&spi_data, sizeof(spi_data_t) / 2 - 1);
    memcpy(tx_buff, &spi_data, sizeof(spi_data_t));
    //for (int i = 0; i < DATA_LEN; i++)
    //    tx_buff[i] = ((uint16_t*)(&spi_data))[i];
}

void init_spi(void)
{
    pc.printf("go to SPI Init \n\r");
    SPISlave* spi = new SPISlave(PA_7, PA_6, PA_5, PA_4);
    spi->format(16, 0);
    spi->frequency(6000000);
    spi->reply(0xff);
    cs.fall(&spi_isr);
    pc.printf("SPI Init done\n\r");
}

int main()
{
    wait(1);
    // led = 1;
    pc.baud(921600);
    pc.attach(&serial_isr);
    estop.mode(PullUp);
    // spi.format(16, 0);
    // spi.frequency(1000000);
    // spi.reply(0x0);
    // cs.fall(&spi_isr);

    // can1.frequency(1000000);                     // set bit rate to 1Mbps
    // can1.attach(&rxISR1);                 // attach 'CAN receive-complete' interrupt handler
    can1.filter(CAN_ID << 21, 0xFFE00004, CANStandard, 0); // set up can filter
    // can2.frequency(1000000);                     // set bit rate to 1Mbps
    // can2.attach(&rxISR2);                 // attach 'CAN receive-complete' interrupt handler
    can2.filter(CAN_ID << 21, 0xFFE00004, CANStandard, 0); // set up can filter

    memset(&tx_buff, 0, TX_LEN * sizeof(uint16_t));
    memset(&spi_data, 0, sizeof(spi_data_t));
    memset(&spi_command, 0, sizeof(spi_command_t));

    NVIC_SetPriority(TIM5_IRQn, 1);
    // NVIC_SetPriority(CAN1_RX0_IRQn, 3);
    // NVIC_SetPriority(CAN2_RX0_IRQn, 3);

    pc.printf("\n\r SPIne\n\r");
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

    pack_cmd(&a1_can, l1_control.a);
    pack_cmd(&a2_can, l2_control.a);
    pack_cmd(&h1_can, l1_control.h);
    pack_cmd(&h2_can, l2_control.h);
    pack_cmd(&k1_can, l1_control.k);
    pack_cmd(&k2_can, l2_control.k);
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
