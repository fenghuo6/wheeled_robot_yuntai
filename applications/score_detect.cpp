#include "score_detect.hpp"

#include <cmath>

namespace
{
// 1 ms 调用周期下的滤波系数，时间常数 ≈ 1/α = 1000 次 = 1 s。
constexpr float k_alpha = 0.001f;
constexpr float k_pi = 3.14159265359f;
constexpr float k_two_pi = 6.28318530718f;
constexpr float k_rad_to_deg = 57.29578f;

// 角度差回绕到 [-π, π]。
// IMU 的 yaw 由 atan2 算出，范围 (-π, π]；直接相减时，云台在 +179°、目标在 −179°
// 这类情形的实际误差只有 2°，相减却得到 358° → 永远不命中。必须先回绕。
float wrap_diff(float a, float b)
{
  float d = a - b;
  while (d > k_pi) d -= k_two_pi;
  while (d < -k_pi) d += k_two_pi;
  return d;
}

void detect(float gimbal_angle, float plan_angle, float threshold, float *fire_ratio)
{
  if (fire_ratio == nullptr) return;
  const float fire =
      (std::fabs(wrap_diff(gimbal_angle, plan_angle)) < threshold) ? 1.0f : 0.0f;
  *fire_ratio = k_alpha * fire + (1.0f - k_alpha) * (*fire_ratio);
}

// 状态放在文件作用域，保证初值为 0（调用方传的就是这两个变量）。
float s_yaw_ratio = 0.0f;
float s_pitch_ratio = 0.0f;
}  // namespace

volatile float g_yaw_fire_ratio = 0.0f;
volatile float g_pitch_fire_ratio = 0.0f;

void yaw_detect(float gimbal_angle, float plan_angle, float *fire_ratio)
{
  detect(gimbal_angle, plan_angle, 0.5f / k_rad_to_deg, fire_ratio);
}

void pitch_detect(float gimbal_angle, float plan_angle, float *fire_ratio)
{
  detect(gimbal_angle, plan_angle, 1.5f / k_rad_to_deg, fire_ratio);
}

void score_detect_update(
    bool active, float gimbal_yaw, float plan_yaw, float gimbal_pitch, float plan_pitch)
{
  // 非跟随阶段（对齐中/未启动）直接清零，避免"目标=当前朝向、误差恒 0"把命中率抬高。
  if (!active) {
    s_yaw_ratio = 0.0f;
    s_pitch_ratio = 0.0f;
    g_yaw_fire_ratio = 0.0f;
    g_pitch_fire_ratio = 0.0f;
    return;
  }
  yaw_detect(gimbal_yaw, plan_yaw, &s_yaw_ratio);
  pitch_detect(gimbal_pitch, plan_pitch, &s_pitch_ratio);
  g_yaw_fire_ratio = s_yaw_ratio;
  g_pitch_fire_ratio = s_pitch_ratio;
}
