#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/plotter/plotter.hpp"
#include "laser_task.hpp"
#include "motor_task/motor_task.hpp"
#include "score_detect.hpp"

// 阻塞发送(use_dma = false)：一帧 7 通道 = 31 字节 @921600 约 0.3ms，100Hz 下占用很小。
// 不走 DMA 是避免上一帧未发完时 HAL 返回 HAL_BUSY 而整帧被静默丢弃，导致上位机一条数据都没有。
sp::Plotter plotter(&huart1, false);

// ============================================================================
// 通道分配（7 通道）
//   1 yaw 命中率(0~1,比赛计分) | 2 pitch 命中率(0~1,比赛计分)
//   3 yaw 目标值(°)            | 4 pitch 目标值(°)
//   5 yaw 实际值(°, IMU)       | 6 pitch 实际值(°, IMU)
//   7 红外距离(cm)
//
// 目标值取 g_gimbal_yaw_set / g_gimbal_pitch_set —— 这是云台"真正在追"的角度
//   （对齐阶段是限速斜坡、跟随时是轨迹点），命中率也是拿它和实际值比较的，
//   所以 3/5 与 1、4/6 与 2 可以直接对照着看：目标与实际贴合得越紧，命中率越高。
// 实际值取 IMU 世界系姿态（g_yaw_rad / g_pitch_rad）。
// 命中率只在轨迹跟随阶段累积（对齐阶段误差恒 0，不统计），非跟随阶段读数停在最后一次。
// 红外距离单位 cm（该模块实测按整数厘米上报），离线时保持上一次的有效值。
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
    plotter.plot(
      g_yaw_fire_ratio, g_pitch_fire_ratio, g_gimbal_yaw_set * k_rad_to_deg,
      g_gimbal_pitch_set * k_rad_to_deg, g_yaw_rad * k_rad_to_deg, g_pitch_rad * k_rad_to_deg,
      static_cast<float>(g_gy56_distance_cm));
    osDelay(k_sample_period_ms);
  }
}
