#include <cstdint>
#include "cmsis_os.h"
#include "imu_task.hpp"
#include "motor_task.hpp"
#include "trajectory_task/trajectory_task.hpp"

// ============================================================================
// 电机控制层
// ----------------------------------------------------------------------------
// 职责：决定三个电机"做什么"——写目标角度、开关使能。
//       不碰 CAN，不碰 PID（那些都在 motor_task.cpp 内部）。
// 调用：由 uart_task 每 10 ms 调用一次 motor_control_update()。
// 分层约定：本层只写 motor_task.hpp 暴露的全局量，参数一律用"度"，
//           换算成 rad 后再下发。
//
// 当前模式：motor_a(yaw) 与 motor_c(pitch) 以 IMU 世界系姿态为反馈，
//           跟随 trajectory_task 的曲线；motor_b(折叠) 由 motor_task 上电锁位保持，
//           本层不写它的目标。
// ============================================================================
namespace
{
  // ───────────────────── 1. 按编号取全局量 ─────────────────────
  // 三个电机共用同一套控制函数，靠下面两个函数把编号映射到具体变量。
  volatile float &angle_set_of(uint8_t motor)
  {
    switch (motor)
    {
    case MOTOR_B:
      return g_motor_b_angle_set;
    case MOTOR_C:
      return g_motor_c_angle_set;
    default:
      return g_motor_a_angle_set;
    }
  }

  // ─────────────────────── 2. 单位换算 ───────────────────────
  // 控制层对外一律用"度"，底层 rad 只在转换后写入 g_motor_*_angle_set。
  constexpr float k_deg_to_rad = 3.14159265f / 180.0f;

  // ───────────────────── 3. 基础控制函数 ─────────────────────
  // 以下函数当前只被示例使用，加了 unused 属性避免 -Wall 报警；
  // 你启用任一示例后，该属性可以删掉。

  // 相对使能时的位置转 deg 度（正为正向）。0 表示原地保持。
  // 零点由 motor_task 在每次使能后自动锁存，这里不需要关心编码器绝对值。
  __attribute__((unused)) void motor_angle(uint8_t motor, float deg)
  {
    angle_set_of(motor) = deg * k_deg_to_rad;
  }

  // 总使能：true 时三个电机输出转矩，false 时全部停止输出。
  __attribute__((unused)) void motor_enable(bool enable) { g_motor_pid_enable = enable; }

  // ─────────────────────── 4. 折叠轴控制 ───────────────────────
  // 折叠轴(b)同样用 motor_angle(MOTOR_B, deg) 驱动，deg 是相对使能位置的转角。
  // 使能后默认目标 0，即锁在原位不动（防止云台倒下）；改写它即可折叠/展开。
  // 方向不对就把度数取负；不要一次给到 90°，先小角度确认方向后再加大。
  // 例：motor_angle(MOTOR_B, 45.0f);   // 从使能位置折下 45°

  // ─────────────────────── 5. 轨迹跟随 ───────────────────────
  // 目标为世界系角度，motor_task 以 IMU 姿态作为反馈闭环。
  // 轨迹本身就在世界系，因此这里直接下发，不做任何偏置换算。
  // 某个轴转向相反时不要去改这里，改 motor_task.cpp 的 k_imu_dir_*。
  //
  // 启动流程：轨迹上电停在起点，云台先限速对齐到"轨迹开始位置"，到位后再放行轨迹。
  // 这样使能瞬间不会从当前姿态直冲轨迹值。
  constexpr float k_align_rate = 1.0f;  // 对齐阶段的角速度上限 rad/s
  constexpr float k_align_dt = 0.01f;   // 与控制层调用周期(10ms)一致
  constexpr float k_align_tolerance = 0.0174533f;  // 到位判定阈值，1°
  constexpr uint32_t k_align_hold_ms = 200U;       // 需要连续到位的时长
  constexpr uint32_t k_control_period_ms = 10U;    // 本函数的调用周期

  enum class TrajPhase
  {
    kIdle,    // 未启动，轨迹停在起点
    kAlign,   // 限速趋近轨迹起点，轨迹不放行
    kFollow,  // 正常跟随
  };
  TrajPhase g_traj_phase = TrajPhase::kIdle;
  float g_align_yaw = 0.0f;    // 对齐阶段实际下发的目标（已限速）
  float g_align_pitch = 0.0f;
  uint32_t g_in_position_ms = 0U;

  float step_toward(float value, float target, float max_step)
  {
    const float diff = target - value;
    if (diff > max_step) return value + max_step;
    if (diff < -max_step) return value - max_step;
    return target;
  }

  bool within(float value, float target, float tolerance)
  {
    const float diff = value - target;
    return (diff < 0.0f ? -diff : diff) <= tolerance;
  }

  // 停止跟随：轨迹回到起点，状态复位，下次使能重新对齐。
  void trajectory_follow_abort()
  {
    trajectory_pause();
    g_traj_phase = TrajPhase::kIdle;
    g_in_position_ms = 0U;
  }

  // 每 10 ms 推进一次启动流程。
  // imu_ok 为假（IMU 数据过期）时不推进对齐判定，目标维持当前读数使偏差为 0。
  void trajectory_follow_update(bool imu_ok)
  {
    // 一旦进入跟随模式就切到 IMU 反馈，避免退回编码器模式后目标变成 0 而抢动。
    g_gimbal_traj_enable = true;

    if (g_traj_phase == TrajPhase::kIdle) {
      if (!imu_ok) {
        g_gimbal_yaw_set = g_yaw_rad;
        g_gimbal_pitch_set = g_pitch_rad;
        return;
      }
      // 以当前姿态为对齐起点，轨迹此刻仍停在起点。
      g_align_yaw = g_yaw_rad;
      g_align_pitch = g_pitch_rad;
      g_in_position_ms = 0U;
      g_traj_phase = TrajPhase::kAlign;
    }

    // 未放行时 trajectory_read() 恒为轨迹起点，即"轨迹开始位置"。
    const TrajectoryPoint point = trajectory_read();

    if (g_traj_phase == TrajPhase::kAlign) {
      if (imu_ok) {
        g_align_yaw = step_toward(g_align_yaw, point.yaw, k_align_rate * k_align_dt);
        g_align_pitch = step_toward(g_align_pitch, point.pitch, k_align_rate * k_align_dt);
        g_gimbal_yaw_set = g_align_yaw;
        g_gimbal_pitch_set = g_align_pitch;

        const bool in_position = within(g_yaw_rad, point.yaw, k_align_tolerance) &&
                                 within(g_pitch_rad, point.pitch, k_align_tolerance);
        g_in_position_ms = in_position ? (g_in_position_ms + k_control_period_ms) : 0U;
      }
      if (g_in_position_ms >= k_align_hold_ms) {
        // 放行前目标已与起点重合，切换瞬间不会跳变。
        trajectory_start();
        g_traj_phase = TrajPhase::kFollow;
      }
      return;
    }

    g_gimbal_yaw_set = point.yaw;
    g_gimbal_pitch_set = point.pitch;
  }
}  // namespace

// ============================================================================
// 对外接口：由 uart_task 每 10 ms 调用一次
// ----------------------------------------------------------------------------
// 执行顺序固定为：① 安全检查 → ② 启动流程（对齐/放行/跟随） → ③ 使能。
// 安全检查必须在前，否则掉线/过热时仍会先下发一次目标再被停。
//
// 可用的控制函数（第一个参数用 MOTOR_A / MOTOR_B / MOTOR_C）：
//   motor_angle(motor, deg)            相对使能位置转 deg 度并保持（0 = 原地保持）
//   motor_enable(true / false)         总使能：false 时三个电机都停止输出转矩
//
// 轨迹跟随的启动流程由 trajectory_follow_update() 内部推进：
//   IDLE →（限速对齐到轨迹起点）→ ALIGN →（连续到位 200ms）→ 放行轨迹 → FOLLOW
//
// 两条必须记住的规则：
//   1. 角度基准是"本次使能时的位置"，由 motor_task 自动锁存，不需要手动回零。
//      所以给 0 就是保持不动；给 90 就是从使能位置转 90°。
//   2. g_motor_pid_enable 初值是 false，所以上电默认不出力，必须调用 motor_enable(true)。
//
// 可读回的状态（用于判断与保护）：
//   g_motor_a_angle / _speed / _torque / _temperature / _online（b、c 同理）
//   注意：这些底层全局量是 rad、rad/s（不是度），角度为编码器绝对值、不含使能零点。
//   转矩 N·m、温度 ℃，均为输出轴侧。
// ============================================================================
extern "C" void motor_control_update(void)
{
  const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());
  const bool imu_ok = static_cast<uint32_t>(now_ms - g_imu_update_ms) < IMU_STALE_TIMEOUT_MS;

  // ---- ① 安全检查 ----
  // 轨迹跟随的两个轴必须反馈正常，任一掉线或过热立即停（折叠轴由 motor_task 自行处理）。
  if (!g_motor_a_online || !g_motor_c_online) {
    motor_enable(false);       // 反馈掉了（>100ms 无 CAN 帧），立即停
    trajectory_follow_abort(); // 轨迹回到起点，下次重新对齐
    return;
  }
  if (g_motor_a_temperature > 80U || g_motor_c_temperature > 80U) {
    motor_enable(false);       // 过热保护
    trajectory_follow_abort();
    return;
  }

  // ---- ② 启动流程：对齐 → 放行 → 跟随 ----
  trajectory_follow_update(imu_ok);

  // ---- ③ 使能 ----
  motor_enable(true);

  // ==========================================================================
  // 可选测试示例（默认全部注释，不影响当前行为）
  // 用法：去掉对应示例的注释符，并把上面的 trajectory_follow_update() 注释掉。
  // ==========================================================================

  // ── 示例 1：motor_a 从使能位置转 +90° 并保持
  //   motor_angle(MOTOR_A, 90.0f);
  //   motor_enable(true);

  // ── 示例 2：什么都不给 = 三个轴都保持使能时的姿势（motor_angle 默认 0）
  //   motor_enable(true);

  // ── 示例 3：停止（立即不再输出转矩；PID 内部状态由 motor_task 自动复位）
  //   motor_enable(false);

  // ── 示例 4：yaw 与 pitch 各自不同目标，同时运行
  //   motor_angle(MOTOR_A, 90.0f);        // a 从使能位置转 +90°
  //   motor_angle(MOTOR_C, -45.0f);       // c 从使能位置转 -45°
  //   motor_enable(true);
  //   motor_b(折叠)默认保持使能位置，要折叠就再加一行 motor_angle(MOTOR_B, deg)。

  // ── 示例 5：自动往复测试（不用遥控器）——每 2 s 在 0 与 +90° 之间切换
  //   static uint32_t last_toggle_ms = 0U;
  //   static bool at_zero = true;
  //   const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());
  //   if (now_ms - last_toggle_ms >= 2000U) {
  //     last_toggle_ms = now_ms;
  //     at_zero = !at_zero;
  //     motor_angle(MOTOR_A, at_zero ? 0.0f : 90.0f);
  //   }
  //   motor_enable(true);

  // ── 示例 6：角度正弦扫频（调角度环 PID 用）——目标角在 ±45° 间按 0.25 Hz 摆动
  //   （需要 #include <cmath>）
  //   static const uint32_t t0_ms = static_cast<uint32_t>(osKernelSysTick());
  //   const float t_s = static_cast<float>(osKernelSysTick() - t0_ms) * 1e-3f;
  //   const float amp_deg = 45.0f;
  //   const float omega = 2.0f * 3.14159265f * 0.25f;
  //   motor_angle(MOTOR_A, amp_deg * std::sin(omega * t_s));   // 需 #include <cmath>
  //   motor_enable(true);

  // ── 示例 7：带安全检查的模板——先确认反馈正常再使能，异常就停
  //   if (!g_motor_a_online) {
  //     motor_enable(false);          // 反馈掉了（>100ms 无 CAN 帧），立即停
  //     return;
  //   }
  //   if (g_motor_a_temperature > 80U) {
  //     motor_enable(false);          // 过热保护
  //     return;
  //   }
  //   motor_angle(MOTOR_A, 90.0f);
  //   motor_enable(true);

  // ── 示例 8：上电自动跑一次（等三个电机反馈都正常后再启动，避免盲跑）
  //   static bool started = false;
  //   if (!started) {
  //     const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());
  //     if (!g_motor_a_online || !g_motor_b_online || !g_motor_c_online) return;
  //     motor_angle(MOTOR_A, 90.0f);
  //     motor_enable(true);
  //     started = true;
  //   }
}