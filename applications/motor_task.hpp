#ifndef APPLICATIONS_MOTOR_TASK_HPP
#define APPLICATIONS_MOTOR_TASK_HPP

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 电机配置：motor_a/b 为 GM6020（ID 1/2，电流控制帧 0x1FE），
// motor_c 为 C610 驱动的 M2006（ID 3，控制帧 0x200）。
// 单位均为输出轴侧：角度 rad、转速 rad/s、转矩 N·m、温度 ℃；
// M2006 已按 36:1 减速比换算。

// 指令输入：其他模块写入，motor_task 每 1 ms 下发一次。初值 0，上电不会输出转矩。
// 量程：motor_a/b 为 ±2.223 N·m，motor_c 为 ±1.8 N·m，超出部分由驱动自动限幅。
extern volatile float g_motor_a_cmd;
extern volatile float g_motor_b_cmd;
extern volatile float g_motor_c_cmd;

// 状态输出：由 motor_task 刷新。
extern volatile float g_motor_a_angle;
extern volatile float g_motor_b_angle;
extern volatile float g_motor_c_angle;
extern volatile float g_motor_a_speed;
extern volatile float g_motor_b_speed;
extern volatile float g_motor_c_speed;
extern volatile float g_motor_a_torque;
extern volatile float g_motor_b_torque;
extern volatile float g_motor_c_torque;
extern volatile uint8_t g_motor_a_temperature;
extern volatile uint8_t g_motor_b_temperature;
extern volatile uint8_t g_motor_c_temperature;
extern volatile bool g_motor_a_online;
extern volatile bool g_motor_b_online;
extern volatile bool g_motor_c_online;

void motor_task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_MOTOR_TASK_HPP
