#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "laser_task.hpp"
#include "motor_task/motor_task.hpp"
#include "tools/mahony/mahony.hpp"
#include "trajectory_task/trajectory_task.hpp"

extern sp::Mahony imu;

sp::Plotter plotter(&huart1);

// 通道分配（上位机按顺序显示）:
//   1~5    yaw / 实际 yaw / yaw_vel / 实际 yaw 速度 / 【电机 a 目标角(°)】
//   6~10   pitch / 实际 pitch / pitch_vel / 实际 pitch 速度 / 【电机 c 目标角(°)】
//   11~18  实际 roll / 【电机 a 实际角(°)】 / 【电机 b 实际角(°)】 / 【电机 a 转矩指令】 /
//          【电机 b 目标角(°)】 / 【电机 c 转矩指令】 / 【电机 c 实际角(°)】 / GY-56 距离
//
// 标【】的是调参通道，取代了原来的 yaw_acc / pitch_acc / 电机 b 转速 / 编码器绝对角 /
// 电机 a、c 转速：
//   角度单位统一为"度"，基准是本次使能时的位置（零点自动锁存），所以：
//     目标给 20°，曲线就从 0 走到 20；目标 - 实际 = 角度误差；误差不收敛/来回摆就是 PID 问题。
//   转矩指令单位 N·m，用于判断"力够不够"：
//     一直贴着 ±上限（a: 0.7，c: 0.5）= 力矩受限，再调大 kp/ki 也没用；
//     稳态时停在某个小正值（如 pitch 0.15）= 积分项在顶重力，属正常。
//   目标与实际成对，但不相邻，画图时请这样配对：
//     yaw(a)   : 5  目标 / 12 实际 / 14 转矩指令
//     pitch(c) : 10 目标 / 17 实际 / 16 转矩指令
//     折叠(b)  : 15 目标 / 13 实际（b 的转矩指令未占通道，需要时再换）
//   轨迹跟随阶段的目标看 1/6（世界系），5/10 此时不被使用，读数为 0 属正常；
//   12/13/17 是编码器测到的真实转角，两种模式下都有效，可直接看"电机到底动没动"。
// 期望与实际的量两两相邻, 便于直接对比: 1~2 位置、3~4 速度、6~7 位置、8~9 速度。
// 激光诊断量(g_gy56_valid_frames / g_gy56_checksum_errors)、电机转矩、指令、温度及
// 在线状态仍在 laser_task.hpp、motor_task.hpp 中发布, 需要时可替换任一通道。
extern "C" void plotter_task()
{
  constexpr float k_rad_to_deg = 57.29578f;

  while (1)
  {
    plotter.plot(
        yaw, imu.yaw, yaw_vel, imu.vyaw, g_motor_a_angle_set * k_rad_to_deg, pitch, imu.pitch,
        pitch_vel, imu.vpitch, g_motor_c_angle_set * k_rad_to_deg, imu.roll,
        g_motor_a_angle_rel * k_rad_to_deg, g_motor_b_angle_rel * k_rad_to_deg, g_motor_a_cmd,
        g_motor_b_angle_set * k_rad_to_deg, g_motor_c_cmd, g_motor_c_angle_rel * k_rad_to_deg,
        static_cast<float>(g_gy56_distance_cm));
    osDelay(10);
  }
}