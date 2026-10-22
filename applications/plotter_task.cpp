#include <cmath>

#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/plotter/plotter.hpp"
#include "laser_task.hpp"
#include "motor_task/motor_task.hpp"
#include "score_detect.hpp"
#include "trajectory_task/trajectory_task.hpp"
#include "trajectory_task/trajectory_task.hpp"

// 阻塞发送(use_dma = false)：一帧 75 字节 @921600 约 0.8ms，100Hz 下占用很小。
// 不走 DMA 是避免上一帧未发完时 HAL 返回 HAL_BUSY 而整帧被静默丢弃，导致上位机一条数据都没有。
sp::Plotter plotter(&huart1, false);

// ============================================================================
// 通道分配
//   1 a 实际角(°) | 2 b 实际角(°) | 3 c 实际角(°)
//   4 a 目标力矩(N·m) | 5 b 目标力矩(N·m) | 6 c 目标力矩(N·m)
//   7 yaw 误差(°) | 8 pitch 误差(°)
//   9 轨迹 yaw(°) | 10 轨迹 pitch(°)
//   11 实际 yaw(°)(IMU) | 12 实际 pitch(°)(IMU)
//   13 yaw 误差 RMS(°) | 14 pitch 误差 RMS(°)
//   15 yaw 命中率(0~1,比赛计分) | 16 pitch 命中率(0~1,比赛计分)
//
// 实际角与目标角同基准（都相对本次使能时的零点），可直接相减得到误差。
// a/c 是否出力由各自的开关决定（motor_axis_enable）；即使不出力，实际角照常随手动转动变化。
// 7/8 是 目标−实际 的误差，跟踪好时应该贴着 0；用它判断滞后和过冲最直接。
// 9/10 是 trajectory_task 输出的轨迹本身（世界系角度），未加 yaw 平移偏置；
//   电机实际在追的目标 = 该值 + g_yaw_offset。
// 13/14 是误差均方根，窗口正好一个完整轨迹周期（随 TRAJECTORY_PERIOD_MS 自动变化），
//   只在轨迹跟随阶段累积。用它做跨参数、跨速度的客观比较，不要看曲线高矮（GUI 自动缩放）。
// 15/16 是比赛计分程序的输出：|误差| < 容差(0.5°/1.5°) 的时间占比，1 ms 更新、时间常数 1 s。
// （GY-56 暂不打印，排查完再换回来。）
// ============================================================================
extern "C" void plotter_task(void const *argument)
{
  (void)argument;
  constexpr float k_rad_to_deg = 57.29578f;
  // 滚动窗口, 长度正好一个轨迹周期: 10ms 一拍 → 周期/10 个样本。
  // 若窗口短于一个周期, 读数会随轨迹相位大幅跳(实测 1 秒窗口能跳 0.97~1.73), 跨参数/跨速度就不可比。
  // 跟着 TRAJECTORY_PERIOD_MS 自动变化, 换周期不必改这里。
  constexpr uint32_t k_sample_period_ms = 10U;
  constexpr uint32_t k_rms_window = TRAJECTORY_PERIOD_MS / k_sample_period_ms;

  static float yaw_sq[k_rms_window];
  static float pitch_sq[k_rms_window];
  uint32_t idx = 0U;
  uint32_t filled = 0U;
  float yaw_sum = 0.0f;
  float pitch_sum = 0.0f;
  float yaw_rms = 0.0f;
  float pitch_rms = 0.0f;

  while (1)
  {
    const float yaw_err = (g_gimbal_yaw_set - g_yaw_rad) * k_rad_to_deg;
    const float pitch_err = (g_gimbal_pitch_set - g_pitch_rad) * k_rad_to_deg;
    // 一次取一组自洽的轨迹值（内部有临界区保护），避免 yaw/pitch 来自不同周期。
    const TrajectoryPoint traj = trajectory_read();

    // 只统计真正跟随阶段: 对齐阶段目标是内部斜坡, 误差没有意义。
    if (g_traj_following)
    {
      const float ys = yaw_err * yaw_err;
      const float ps = pitch_err * pitch_err;
      if (filled == k_rms_window) {
        yaw_sum -= yaw_sq[idx];
        pitch_sum -= pitch_sq[idx];
      }
      yaw_sq[idx] = ys;
      pitch_sq[idx] = ps;
      yaw_sum += ys;
      pitch_sum += ps;
      idx = (idx + 1U) % k_rms_window;
      if (filled < k_rms_window) ++filled;

      // 每满一个周期重算一次和, 避免长时间增量累加的浮点漂移。
      if (idx == 0U && filled == k_rms_window) {
        yaw_sum = 0.0f;
        pitch_sum = 0.0f;
        for (uint32_t i = 0U; i < k_rms_window; ++i) {
          yaw_sum += yaw_sq[i];
          pitch_sum += pitch_sq[i];
        }
      }
      yaw_rms = std::sqrt(yaw_sum / static_cast<float>(filled));
      pitch_rms = std::sqrt(pitch_sum / static_cast<float>(filled));
    }
    else
    {
      idx = 0U;
      filled = 0U;
      yaw_sum = 0.0f;
      pitch_sum = 0.0f;
      yaw_rms = 0.0f;
      pitch_rms = 0.0f;
    }

    plotter.plot(
      g_motor_a_angle_rel * k_rad_to_deg, g_motor_b_angle_rel * k_rad_to_deg,
      g_motor_c_angle_rel * k_rad_to_deg, g_motor_a_cmd, g_motor_b_cmd, g_motor_c_cmd, yaw_err,
      pitch_err, traj.yaw * k_rad_to_deg, traj.pitch * k_rad_to_deg, g_yaw_rad * k_rad_to_deg,
      g_pitch_rad * k_rad_to_deg, yaw_rms, pitch_rms, g_yaw_fire_ratio, g_pitch_fire_ratio);
    osDelay(10);
  }
}
