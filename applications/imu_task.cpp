#include <cmath>

#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);

// 全局变量给其他任务读
volatile float g_yaw_rad = 0.0f;
volatile float g_pitch_rad = 0.0f;
volatile float g_roll_rad = 0.0f;
volatile uint32_t g_imu_update_ms = 0U;
volatile bool g_imu_bias_ready = false;
volatile float g_imu_bias_z = 0.0f;

namespace
{
// ─────────────── 陀螺零偏标定 ───────────────
// yaw 完全靠陀螺积分（无磁力计），零偏会直接变成漂移速率；而 yaw 环路为了"修正"
// 这个假的误差，会持续单方向驱动机构，最终表现为线缆单向缠绕。
// 上电后先估计零偏并从后续测量中扣除（典型 0.1~0.5°/s → 0.01~0.05°/s）。
//
// 只采集"静止"样本，所以运动不会被误当成零偏。
// 标定期间 A/C 由 motor_control 门控保持不使能（见 g_imu_bias_ready），机构静止；
// 标定完成后才放行轨迹跟随。
constexpr int k_bias_samples = 1000;         // 1000 个静止样本 ≈ 1 s
constexpr float k_static_threshold = 0.05f;  // rad/s(≈2.9°/s)，三轴都小于它才算静止
constexpr uint32_t k_cal_timeout_ms = 3000U; // 超时兜底：迟迟采不到足够静止样本就放弃标定

float s_sum[3] = {0.0f, 0.0f, 0.0f};
float s_bias[3] = {0.0f, 0.0f, 0.0f};
int s_count = 0;
bool s_ready = false;
float s_yaw_offset = 0.0f;   // 标定完成瞬间冻结的 yaw 读数
uint32_t s_start_ms = 0U;
bool s_started = false;
}  // namespace

extern "C" void imu_task()
{
  bmi088.init();

  while (true)
  {
    bmi088.update();

    float gyro[3] = {bmi088.gyro[0], bmi088.gyro[1], bmi088.gyro[2]};
    const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());
    if (!s_started) {
      s_start_ms = now_ms;
      s_started = true;
    }

    if (!s_ready)
    {
      const bool is_static = std::fabs(gyro[0]) < k_static_threshold &&
                             std::fabs(gyro[1]) < k_static_threshold &&
                             std::fabs(gyro[2]) < k_static_threshold;
      if (is_static)
      {
        s_sum[0] += gyro[0];
        s_sum[1] += gyro[1];
        s_sum[2] += gyro[2];
        if (++s_count >= k_bias_samples)
        {
          s_bias[0] = s_sum[0] / static_cast<float>(s_count);
          s_bias[1] = s_sum[1] / static_cast<float>(s_count);
          s_bias[2] = s_sum[2] / static_cast<float>(s_count);
          // 标定期间机构静止，物理位置与上电时相同，因此把当前 yaw 冻结为新零点：
          // 既正确，又顺带消掉了这段静止时间内由零偏造成的漂移。
          s_yaw_offset = imu.yaw;
          s_ready = true;
          g_imu_bias_ready = true;
          g_imu_bias_z = s_bias[2];
        }
      }

      // 超时兜底：采样期间机构一直被打扰(有人碰/在动)，就放弃标定继续用 0 零偏，
      // 不能因为标定完不成就永远不放行轨迹。此时不做 yaw 重新归零（位置不可信）。
      if (!s_ready && (now_ms - s_start_ms) > k_cal_timeout_ms) {
        if (s_count > 0) {
          s_bias[0] = s_sum[0] / static_cast<float>(s_count);
          s_bias[1] = s_sum[1] / static_cast<float>(s_count);
          s_bias[2] = s_sum[2] / static_cast<float>(s_count);
        }
        s_ready = true;
        g_imu_bias_ready = true;
        g_imu_bias_z = s_bias[2];
      }
    }
    else
    {
      gyro[0] -= s_bias[0];
      gyro[1] -= s_bias[1];
      gyro[2] -= s_bias[2];
    }

    imu.update(bmi088.acc, gyro);

    g_yaw_rad = imu.yaw - s_yaw_offset;
    g_pitch_rad = imu.pitch;
    g_roll_rad = imu.roll;
    g_imu_update_ms = now_ms;

    osDelay(1);
  }
}