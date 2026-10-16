#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "laser_task.hpp"
#include "motor_task.hpp"
#include "tools/mahony/mahony.hpp"
#include "trajectory_task/trajectory_task.hpp"

extern sp::Mahony imu;

sp::Plotter plotter(&huart1);

// 通道分配（上位机按顺序显示）:
//   1~5    yaw / 实际 yaw / yaw_vel / 实际 yaw 速度 / yaw_acc
//   6~10   pitch / 实际 pitch / pitch_vel / 实际 pitch 速度 / pitch_acc
//   11~18  实际 roll / 电机 a 角度 / 电机 b 角度 / 电机 a 转速 / 电机 b 转速 /
//          电机 c 转速 / M2006 角度 / GY-56 距离
// 期望与实际的量两两相邻, 便于直接对比: 1~2 位置、3~4 速度、6~7 位置、8~9 速度。
// 激光诊断量(g_gy56_valid_frames / g_gy56_checksum_errors)、电机转矩、指令、温度及
// 在线状态仍在 laser_task.hpp、motor_task.hpp 中发布, 需要时可替换任一通道。
extern "C" void plotter_task()
{
  while (1) {
    plotter.plot(
      yaw, imu.yaw, yaw_vel, imu.vyaw, yaw_acc, pitch, imu.pitch, pitch_vel, imu.vpitch, pitch_acc,
      imu.roll, g_motor_a_angle, g_motor_b_angle, g_motor_a_speed, g_motor_b_speed,
      g_motor_c_speed, g_motor_c_angle, static_cast<float>(g_gy56_distance_cm));
    osDelay(10);
  }
}