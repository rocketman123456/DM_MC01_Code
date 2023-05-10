#include "message_ops.h"

#include <iostream>
#include <string>

using namespace std;

int main()
{
    joint_control j;
    float p, v, kp, kd, t_ff;
    uint8_t can[8];
    while (cin >> p >> v >> kp >> kd >> t_ff)
    {
        j.p_des = p;
        j.v_des = v;
        j.kp = kp;
        j.kd = kd;
        j.t_ff = t_ff;
        pack_cmd(can, 8, j);

        auto str = char2hexstr((char*)&can, 8);

        cout << str << endl;
    }
    return 0;
}