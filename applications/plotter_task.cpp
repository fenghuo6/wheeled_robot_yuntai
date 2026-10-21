#include <cmath>

#include "cmsis_os.h"
#include "can.h"
#include "io/plotter/plotter.hpp"
#include "laser_task.hpp"
#include "motor_task/motor_task.hpp"
#include "tools/mahony/mahony.hpp"
#include "trajectory_task/trajectory_task.hpp"

extern sp::Mahony imu;

// 阻塞发送(use_dma = false)：一帧 75 字节 @921600 约 0.8ms，100Hz 下占用很小。
// 不走 DMA 是避免上一帧未发完时 HAL 返回 HAL_BUSY 而整帧被静默丢弃，导致上位机一条数据都没有。
sp::Plotter plotter(&huart1, false);

// ============================================================================
// 通道分配（当前：折叠轴 b(ID1) 重力前馈标定 + 折叠动作）
//   1 b 目标角(°) | 2 b 实际角(°) | 3 b 转矩指令(含前馈,N·m) | 4 b 误差(°)
//   5 b 在线 | 6 b 绝对角(°) | 7 PID 输出(不含前馈,N·m) | 8 前馈值(N·m)
//   9 标定状态(0空闲/1采集中/2完成) | 10 拟合幅值A(N·m) | 11 拟合相位θ0(°)
//   12 a 在线 | 13 c 在线 | 14 总帧数/10ms | 15 IMU yaw(°) | 16 IMU pitch(°)
//   17 IMU roll(°) | 18 温度(℃,b)
//
// 标定判读：
//   9 在 1 期间折叠轴会在 0→15→30→45° 依次停留，结束后变 2
//   标定完成瞬间 10/11 给出 A 和 θ0；前馈(8)随即开始随角度变化
//   标定后再看折叠动作：3(含前馈)在到位后应明显比 7(PID 输出)小 —— 说明重力已被前馈承担
// ============================================================================
extern "C" void plotter_task(void const *argument)
{
  (void)argument;
  constexpr float k_rad_to_deg = 57.29578f;

  uint32_t prev_frames = 0U;

  while (1)
  {
    const uint32_t frames = g_can1_rx_frames;
    const float frame_rate = static_cast<float>(frames - prev_frames);
    prev_frames = frames;

    const float ff =
        g_fold_ff_p * std::cos(g_motor_b_angle) + g_fold_ff_q * std::sin(g_motor_b_angle);

    plotter.plot(
        g_motor_b_angle_set * k_rad_to_deg, g_motor_b_angle_rel * k_rad_to_deg, g_motor_b_cmd,
        (g_motor_b_angle_set - g_motor_b_angle_rel) * k_rad_to_deg,
        g_motor_b_online ? 1.0f : 0.0f, g_motor_b_angle * k_rad_to_deg, g_motor_b_pid_out, ff,
        static_cast<float>(g_fold_cal_state), g_fold_cal_amp,
        g_fold_cal_phase * k_rad_to_deg, g_motor_a_online ? 1.0f : 0.0f,
        g_motor_c_online ? 1.0f : 0.0f, frame_rate, imu.yaw * k_rad_to_deg,
        imu.pitch * k_rad_to_deg, imu.roll * k_rad_to_deg,
        static_cast<float>(g_motor_b_temperature));
    osDelay(10);
  }
}
