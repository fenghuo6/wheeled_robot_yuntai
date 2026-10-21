#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/plotter/plotter.hpp"
#include "laser_task.hpp"
#include "motor_task/motor_task.hpp"

// 阻塞发送(use_dma = false)：一帧 75 字节 @921600 约 0.8ms，100Hz 下占用很小。
// 不走 DMA 是避免上一帧未发完时 HAL 返回 HAL_BUSY 而整帧被静默丢弃，导致上位机一条数据都没有。
sp::Plotter plotter(&huart1, false);

// ============================================================================
// 通道分配
//   1 a 实际角(°) | 2 b 实际角(°) | 3 c 实际角(°)
//   4 a 目标角(°) | 5 b 目标角(°) | 6 c 目标角(°)
//   7 a 目标力矩(N·m) | 8 b 目标力矩(N·m) | 9 c 目标力矩(N·m)
//   10 世界系 yaw 误差(°) | 11 世界系 pitch 误差(°)
//   12 世界系目标 yaw(°) | 13 世界系目标 pitch(°)
//   14 实际 yaw(°)(IMU) | 15 实际 pitch(°)(IMU)
//   16 a 在线(1/0) | 17 c 在线(1/0) | 18 轨迹阶段(0空闲 1对齐 2跟随)
//
// 实际角与目标角同基准（都相对本次使能时的零点），可直接相减得到误差。
// a/c 是否出力由各自的开关决定（motor_axis_enable）；即使不出力，实际角照常随手动转动变化。
// 10/11 是 目标−实际 的误差，跟踪好时应该贴着 0；用它判断滞后和过冲最直接。
// 12/13 是电机真正在追的世界系目标（g_gimbal_*_set），14/15 是 IMU 实际姿态。
// 16~18 是排查用诊断量：目标不动且误差单调漂移时，先用它们确认轴有没有被使能、流程走到哪一步。
// （GY-56 暂不打印，排查完再换回来。）
// ============================================================================
extern "C" void plotter_task(void const *argument)
{
  (void)argument;
  constexpr float k_rad_to_deg = 57.29578f;

  while (1)
  {
    const float yaw_err = (g_gimbal_yaw_set - g_yaw_rad) * k_rad_to_deg;
    const float pitch_err = (g_gimbal_pitch_set - g_pitch_rad) * k_rad_to_deg;

    // 12/13 恢复为世界系目标角（16~18 仍保留诊断）。
    plotter.plot(
      g_motor_a_angle_rel * k_rad_to_deg, g_motor_b_angle_rel * k_rad_to_deg,
      g_motor_c_angle_rel * k_rad_to_deg, g_motor_a_angle_set * k_rad_to_deg,
      g_motor_b_angle_set * k_rad_to_deg, g_motor_c_angle_set * k_rad_to_deg, g_motor_a_cmd,
      g_motor_b_cmd, g_motor_c_cmd, yaw_err, pitch_err, g_gimbal_yaw_set * k_rad_to_deg,
      g_gimbal_pitch_set * k_rad_to_deg, g_yaw_rad * k_rad_to_deg, g_pitch_rad * k_rad_to_deg,
      static_cast<float>(g_motor_a_online), static_cast<float>(g_motor_c_online),
      static_cast<float>(g_traj_phase_code));
    osDelay(10);
  }
}
