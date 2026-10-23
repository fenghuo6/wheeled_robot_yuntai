#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/plotter/plotter.hpp"
#include "laser_task.hpp"
#include "motor_task/motor_task.hpp"
#include "score_detect.hpp"
#include "trajectory_task/trajectory_task.hpp"

// 阻塞发送(use_dma = false)：一帧 10 通道 = 42 字节 @921600 约 0.5ms，100Hz 下占用很小。
// 不走 DMA 是避免上一帧未发完时 HAL 返回 HAL_BUSY 而整帧被静默丢弃，导致上位机一条数据都没有。
sp::Plotter plotter(&huart1, false);

// ============================================================================
// 通道分配（10 通道）
//   1 a 实际角(°) | 2 b 实际角(°) | 3 c 实际角(°)
//   4 IMU 实际 yaw(°) | 5 IMU 实际 pitch(°)
//   6 轨迹 yaw(°) | 7 轨迹 pitch(°)
//   8 yaw 命中率(0~1,比赛计分) | 9 pitch 命中率(0~1,比赛计分)
//   10 红外距离(cm)
//
// 实际角是相对本次使能位置的角（使能瞬间为 0），折叠时通道 2 从 0 升到
//   g_cfg_fold_deg 再回 0。
// 轨迹是 trajectory_task 输出的世界系角度，未加 yaw 平移偏置；
//   电机实际在追的目标 = 该值 + g_yaw_offset，所以与 IMU 读数可能差一个常量。
// 命中率是比赛计分程序的输出：|误差| < 容差(0.5°/1.5°) 的时间占比，
//   只在轨迹跟随阶段累积，非跟随阶段读数为 0。
// ============================================================================
namespace
{
constexpr float k_rad_to_deg = 57.29578f;
constexpr uint32_t k_sample_period_ms = 10U;
} // namespace

extern "C" void plotter_task(void const *argument)
{
  (void)argument;

  while (1)
  {
    // 一次取一组自洽的轨迹值（内部有临界区保护），避免 yaw/pitch 来自不同周期。
    const TrajectoryPoint traj = trajectory_read();
    plotter.plot(
      g_motor_a_angle_rel * k_rad_to_deg, g_motor_b_angle_rel * k_rad_to_deg,
      g_motor_c_angle_rel * k_rad_to_deg, g_yaw_rad * k_rad_to_deg, g_pitch_rad * k_rad_to_deg,
      traj.yaw * k_rad_to_deg, traj.pitch * k_rad_to_deg, g_yaw_fire_ratio, g_pitch_fire_ratio,
      static_cast<float>(g_gy56_distance_cm));
    osDelay(k_sample_period_ms);
  }
}
