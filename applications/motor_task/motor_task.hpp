#ifndef APPLICATIONS_MOTOR_TASK_HPP
#define APPLICATIONS_MOTOR_TASK_HPP

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 电机任务对外接口
// ----------------------------------------------------------------------------
// 硬件：motor_a/b 为 GM6020（控制帧 0x1FE，a=ID2 底座、b=ID1 折叠），
//       motor_c 为 C610 驱动的 M2006（ID 3，控制帧 0x200）。
// 角色：a = yaw，b = 折叠，c = pitch（与 sp::Gimbal 无关，折叠轴单独保持）。
// 单位：角度 rad、转速 rad/s、转矩 N·m、温度 ℃，均为输出轴侧；
//       M2006 已按 36:1 减速比换算。
//
// 分层：motor_task.cpp 是唯一操作 CAN 与 PID 的模块；
//       motor_control.cpp 只写本文件的"使能/目标"全局量，不碰底层。
// ============================================================================

// 电机编号：控制层用于区分三个电机（如 motor_control.cpp 里的辅助函数）
typedef enum
{
  MOTOR_A = 0,
  MOTOR_B = 1,
  MOTOR_C = 2,
} MotorId;

// ─────────────────────── 1. 使能与模式 ───────────────────────
// 角度环使能：1 时三个电机跟踪各自目标，0 时停止输出转矩；默认 0。
extern volatile bool g_motor_pid_enable;

// 轨迹跟随模式：1 时 a(yaw)/c(pitch) 以 IMU 的世界系姿态为反馈，
// 跟踪下面的 g_gimbal_*_set；0 时按 g_motor_*_angle_set 以电机编码器为反馈。
// 折叠轴 b 不受本开关影响，始终锁位保持。
extern volatile bool g_gimbal_traj_enable;

// ─────────────────────── 2. 目标值 ───────────────────────
// 轨迹跟随的世界系目标角（rad），由 trajectory_task 的曲线给出。
extern volatile float g_gimbal_yaw_set;
extern volatile float g_gimbal_pitch_set;

// 相对本次使能位置的转角（rad），默认 0 表示保持使能时的姿态。
// 零点由 motor_task 在每次使能后第一次拿到有效反馈时自动锁存，外部无需回零。
// 轨迹跟随开启时 a/c 忽略本组值，改用 g_gimbal_*_set。
// b(折叠)始终使用本值：改它就能驱动折叠/展开（如 motor_angle(MOTOR_B, deg)）。
extern volatile float g_motor_a_angle_set;
extern volatile float g_motor_b_angle_set;
extern volatile float g_motor_c_angle_set;

// ─────────────────────── 3. 指令（只读，供观察） ───────────────────────
// 本周期下发的转矩指令，由 motor_task 每 1 ms 刷新。
// 量程：motor_a/b 为 ±2.223 N·m，motor_c 为 ±1.8 N·m，超出部分由驱动自动限幅。
// 注意：g_motor_pid_enable 为 1 时这三个量由角度环写入，此时不要再从外部写。
extern volatile float g_motor_a_cmd;
extern volatile float g_motor_b_cmd;
extern volatile float g_motor_c_cmd;

// ─────────────────────── 4. 反馈状态 ───────────────────────
// 角度 rad、转速 rad/s、转矩 N·m、温度 ℃、在线状态，均由 motor_task 刷新。
extern volatile float g_motor_a_angle;
extern volatile float g_motor_b_angle;
extern volatile float g_motor_c_angle;
extern volatile float g_motor_a_speed;
extern volatile float g_motor_b_speed;
extern volatile float g_motor_c_speed;

// 相对本次使能位置的转角(rad)，只读，用于观察/调参。
// 与 g_motor_*_angle_set 同一基准，因此"目标 vs 实际"可以直接对比：
//   目标 - 本值 = 角度误差。
// 未使能或零点尚未锁存（还没收到第一帧反馈）时为 0。
extern volatile float g_motor_a_angle_rel;
extern volatile float g_motor_b_angle_rel;
extern volatile float g_motor_c_angle_rel;
extern volatile float g_motor_a_torque;
extern volatile float g_motor_b_torque;
extern volatile float g_motor_c_torque;
extern volatile uint8_t g_motor_a_temperature;
extern volatile uint8_t g_motor_b_temperature;
extern volatile uint8_t g_motor_c_temperature;
extern volatile bool g_motor_a_online;
extern volatile bool g_motor_b_online;
extern volatile bool g_motor_c_online;

// ─────────────────── 4.1 CAN1 接收诊断（只读） ───────────────────
// 用于确认反馈帧是否真的到达、其 ID 是多少。
//   g_can1_rx_frames  : 收到标准帧总数（递增即说明 CAN 接收正常）
//   g_can1_last_rx_id : 最近一帧的 ID
//   g_can1_a_frames   : 匹配 motor_a.rx_id 的帧数（为 0 说明 ID 不匹配）
extern volatile uint32_t g_can1_rx_frames;
extern volatile uint32_t g_can1_last_rx_id;
extern volatile uint32_t g_can1_a_frames;
extern volatile uint32_t g_can1_b_frames;
// CAN ID 扫描表（见 motor_task.cpp 说明），slot=0 表示空槽。
extern volatile uint32_t g_can1_id_slot[4];
extern volatile uint32_t g_can1_id_count[4];

// ─────────────────────── 4.2 折叠轴重力前馈（读写） ───────────────────────
// 折叠轴在竖直平面内摆动, 重力矩随角度变化: τ(θ) = m·g·r·cos(θ - θ0)。
// 用等价形式 τ(θ) = P·cosθ + Q·sinθ（θ 为编码器绝对角 rad）:
//   P = A·cosθ0, Q = A·sinθ0, A = m·g·r 为力臂水平时的最大重力矩。
// 写成 P/Q 的好处是可以由若干组 (角度, 所需力矩) 数据直接线性最小二乘拟合。
// 前馈叠加在 PID 输出之外, 因此 PID 只需要处理残差(摩擦、动态误差)。
extern volatile float g_fold_ff_p;  // cos 分量(N·m)
extern volatile float g_fold_ff_q;  // sin 分量(N·m)
// 标定结果(只读, 供观察/抄写):
extern volatile float g_fold_cal_amp;    // 拟合出的 A(N·m)
extern volatile float g_fold_cal_phase;  // 拟合出的 θ0(rad)
extern volatile uint32_t g_fold_cal_state;  // 0 空闲 / 1 采集中 / 2 已完成
extern volatile float g_motor_b_pid_out;    // 不含前馈的 PID 输出(N·m)

// ─────────────────────── 5. 任务入口 ───────────────────────
void motor_task(void const * argument);

// 反馈帧统一入口：CAN1(中断) 与 CAN2(轮询) 都从这里进来。
extern "C" void motor_process_can_frame(uint32_t id, const uint8_t * data, bool extended);

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_MOTOR_TASK_HPP
