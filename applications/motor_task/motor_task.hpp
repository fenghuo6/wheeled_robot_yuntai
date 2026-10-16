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
// 注意：g_motor_pid_enable 为 1 时，三个电机的指令均由环路（速度环或角度双环）写入，
// 此时不要再从外部写这三个量。
extern volatile float g_motor_a_cmd;
extern volatile float g_motor_b_cmd;
extern volatile float g_motor_c_cmd;

// 电机编号：控制层用于区分三个电机（如 motor_control.cpp 里的辅助函数）
typedef enum
{
  MOTOR_A = 0,
  MOTOR_B = 1,
  MOTOR_C = 2,
} MotorId;

// 控制模式：g_motor_*_mode 的取值
typedef enum
{
  MOTOR_MODE_SPEED = 0,  // 单速度环：跟踪 g_motor_*_speed_set
  MOTOR_MODE_ANGLE = 1,  // 双环（角度外环 + 速度内环）：转到并保持在 g_motor_*_angle_set
} MotorControlMode;

// 电机速度环使能：1 时各电机按各自模式跟踪指定目标，0 时停止输出转矩；默认 0。
extern volatile bool g_motor_pid_enable;

// 速度环目标转速（rad/s，输出轴侧）：速度模式下生效，默认 0。
// 例：120 rpm = 120 × 2π / 60 ≈ 12.57 rad/s。
extern volatile float g_motor_a_speed_set;
extern volatile float g_motor_b_speed_set;
extern volatile float g_motor_c_speed_set;

// 控制模式（取值见 MotorControlMode）：默认 MOTOR_MODE_SPEED。
extern volatile uint8_t g_motor_a_mode;
extern volatile uint8_t g_motor_b_mode;
extern volatile uint8_t g_motor_c_mode;

// 位置环目标角度（rad，输出轴侧）：角度模式下生效，默认 0。
// 角度基准为电机编码器零点；要相对当前位置转固定角，写 当前角度 + 增量。
extern volatile float g_motor_a_angle_set;
extern volatile float g_motor_b_angle_set;
extern volatile float g_motor_c_angle_set;

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
