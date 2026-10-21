#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/plotter/plotter.hpp"
#include "laser_task.hpp"
#include "motor_task/motor_task.hpp"
#include "trajectory_task/trajectory_task.hpp"

// 阻塞发送(use_dma = false)：一帧 75 字节 @921600 约 0.8ms，100Hz 下占用很小。
// 不走 DMA 是避免上一帧未发完时 HAL 返回 HAL_BUSY 而整帧被静默丢弃，导致上位机一条数据都没有。
sp::Plotter plotter(&huart1, false);

// ============================================================================
// 通道分配
//   1 a 实际角(°) | 2 b 实际角(°) | 3 c 实际角(°)
//   4 a 目标角(°) | 5 b 目标角(°) | 6 c 目标角(°)
//   7 a 目标力矩(N·m) | 8 b 目标力矩(N·m) | 9 c 目标力矩(N·m)
//   10 GY-56 距离(cm) | 11 GY-56 在线(1=500ms 内有有效帧)
//   12 轨迹 yaw(°) | 13 轨迹 pitch(°)
//   14 实际 yaw(°)(IMU) | 15 实际 pitch(°)(IMU)
//
// 实际角与目标角同基准（都相对本次使能时的零点），可直接相减得到误差。
// a/c 是否出力由各自的开关决定（motor_axis_enable）；即使不出力，实际角照常随手动转动变化。
// 12/13 与 14/15 同为世界(IMU)系角度，两两相减即为轨迹跟随误差。
// ============================================================================
extern "C" void plotter_task(void const *argument)
{
  (void)argument;
  constexpr float k_rad_to_deg = 57.29578f;

  while (1)
  {
    // 一次取一组自洽的轨迹值（内部有临界区保护），避免 yaw/pitch 来自不同周期。
    const TrajectoryPoint traj = trajectory_read();

    plotter.plot(
      g_motor_a_angle_rel * k_rad_to_deg, g_motor_b_angle_rel * k_rad_to_deg,
      g_motor_c_angle_rel * k_rad_to_deg, g_motor_a_angle_set * k_rad_to_deg,
      g_motor_b_angle_set * k_rad_to_deg, g_motor_c_angle_set * k_rad_to_deg, g_motor_a_cmd,
      g_motor_b_cmd, g_motor_c_cmd, static_cast<float>(g_gy56_distance_cm),
      static_cast<float>(g_gy56_online), traj.yaw * k_rad_to_deg, traj.pitch * k_rad_to_deg,
      g_yaw_rad * k_rad_to_deg, g_pitch_rad * k_rad_to_deg);
    osDelay(10);
  }
}
