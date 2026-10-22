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
// 单位：本层对外一律用"度"，写进 g_motor_*_angle_set 前才换算成 rad。
// 基准：目标角都是"相对本次使能位置"，零点由 motor_task 在使能后自动锁存：
//         给 0  → 保持使能时的姿势；
//         给 90 → 从使能位置转 90°。
// 使能：g_motor_pid_enable 上电初值是 false，必须调用 motor_enable(true) 才出力。
//
// 本文件只有三部分：
//   1. 基础控制函数  motor_angle / motor_enable / motor_axis_enable
//   2. 轨迹跟随      a/c 以 IMU 世界系姿态为反馈，跟随 trajectory_task 的曲线
//   3. 入口          安全检查 → 折叠轴使能 → C 轴角度环方波自检
//
// 当前生效：折叠轴 b 使能自锁（顶住云台重力，前馈承担重力矩）；
//           A(yaw) 已调好（kp=2, ki=0.01），自检代码保留但已注释；
//           C(pitch) 正在单独调，纯 P 起步（k_pitch_kp=2, 其余 0）。
// 每个轴独立开关：motor_axis_enable(MOTOR_A / MOTOR_C, true|false)。
// ============================================================================
namespace
{
  // ─────────────────────── 1. 基础控制函数 ───────────────────────

  // 控制层对外一律用"度"，底层 rad 只在写入 g_motor_*_angle_set 时换算。
  constexpr float k_deg_to_rad = 3.14159265f / 180.0f;

  // 三个电机共用同一套函数，靠本函数把编号映射到具体目标变量。
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

  // 相对使能位置转 deg 度并保持（0 = 原地保持，负值 = 反向）。
  void motor_angle(uint8_t motor, float deg) { angle_set_of(motor) = deg * k_deg_to_rad; }

  // 折叠轴使能：true = b 自锁保持目标角；false = b 停止输出。
  void motor_enable(bool enable) { g_motor_pid_enable = enable; }

  // 单轴角度环开关：true = 该轴出力跟踪目标；false = 该轴不出力、可被外力自由转动。
  // a(yaw)/c(pitch) 默认都是关的，调好哪一轴就单独打开哪一轴。
  void motor_axis_enable(uint8_t motor, bool enable)
  {
    switch (motor)
    {
    case MOTOR_B:
      g_motor_pid_enable = enable;   // 折叠轴同时受总使能控制
      break;
    case MOTOR_C:
      g_motor_c_loop_enable = enable;
      break;
    default:
      g_motor_a_loop_enable = enable;
      break;
    }
  }

  // ─────────────────────── 2. 轨迹跟随 ───────────────────────
  // 目标为世界系角度，motor_task 直接以 IMU 姿态作为反馈闭环。
  // 轨迹本身就在世界系，因此这里直接下发，不做任何偏置换算。
  // 某个轴转向相反时不要去改这里，改 motor_task.cpp 的 k_imu_dir_*。
  //
  // yaw 与 pitch 都按"绝对角度"对齐：IMU 的 yaw/pitch 零点直接与轨迹角度对应，
  // 不做整体平移。这样计分程序用 plan_angle 与 imu.yaw 直接比较才有意义；
  // 若做平移，云台会稳定地差一个常量偏置，判分永远是 0。
  // 代价：上电后云台要转回轨迹起点（yaw 可能转较大角度），对齐阶段限速 23°/s。
  //
  // 启动流程：轨迹上电停在起点，云台先限速对齐到"轨迹开始位置"，到位后再放行轨迹，
  // 这样使能瞬间不会从当前姿态直冲轨迹值。
  constexpr float k_align_rate = 0.4f;             // 对齐阶段的角速度上限 rad/s(23°/s)
  // 本函数现由 motor_task 每 1 ms 调用一次（原来挂在 10 ms 的 uart_task 上，
  // 目标值是 10 ms 的阶梯，实测滞后 8 ms 与之一致）。下面两个常量必须跟着周期改。
  constexpr float k_align_dt = 0.001f;             // 与控制层调用周期(1 ms)一致
  // 容差 2°→3°: 任何残余力矩缺口(重力/摩擦)都会让误差停在 1~3° 上。
  // 超时 10s→4s: 起点目标是轨迹表的绝对角度(如 pitch −30°), 机械若到不了就永远进不了
  // 窗口, 只能靠超时兜底。4 秒足够走完对齐斜坡, 再等下去没有意义。
  constexpr float k_align_tolerance = 0.0523599f;  // 到位判定阈值，3°
  constexpr uint32_t k_align_hold_ms = 200U;       // 需要连续到位的时长
  constexpr uint32_t k_align_timeout_ms = 4000U;
  constexpr uint32_t k_control_period_ms = 1U;     // 本函数的调用周期(ms)

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
  uint32_t g_align_ms = 0U;    // 对齐已累计的时间，用于超时兜底

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
    g_align_ms = 0U;
  }

  // 每 1 ms 推进一次启动流程。
  // imu_ok 为假（IMU 数据过期）时不推进对齐判定，目标维持当前读数使偏差为 0。
  void trajectory_follow_update(bool imu_ok)
  {
    // 一旦进入跟随模式就切到 IMU 反馈，避免退回编码器模式后目标变成 0 而抢动。
    g_gimbal_traj_enable = true;

    // 未放行时 trajectory_read() 恒为轨迹起点，即"轨迹开始位置"。
    const TrajectoryPoint point = trajectory_read();

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
      g_align_ms = 0U;
      g_traj_phase = TrajPhase::kAlign;
    }

    if (g_traj_phase == TrajPhase::kAlign) {
      if (imu_ok) {
        g_align_yaw = step_toward(g_align_yaw, point.yaw, k_align_rate * k_align_dt);
        g_align_pitch = step_toward(g_align_pitch, point.pitch, k_align_rate * k_align_dt);
        g_gimbal_yaw_set = g_align_yaw;
        g_gimbal_pitch_set = g_align_pitch;

        const bool in_position = within(g_yaw_rad, point.yaw, k_align_tolerance) &&
                                 within(g_pitch_rad, point.pitch, k_align_tolerance);
        g_in_position_ms = in_position ? (g_in_position_ms + k_control_period_ms) : 0U;
        g_align_ms += k_control_period_ms;
      }
      if (g_in_position_ms >= k_align_hold_ms || g_align_ms >= k_align_timeout_ms) {
        // 放行前目标已与起点重合，切换瞬间不会跳变。
        // 超时也放行：此时目标早已停在起点，误差只有几度，继续等下去没有意义。
        trajectory_start();
        g_traj_phase = TrajPhase::kFollow;
      }
      return;
    }

    g_gimbal_yaw_set = point.yaw;
    g_gimbal_pitch_set = point.pitch;
  }
}  // namespace

// 轨迹是否处于正常跟随阶段（见 motor_task.hpp 说明）。
volatile bool g_traj_following = false;

// ============================================================================
// 入口：由 motor_task 每 1 ms 调用一次
// ----------------------------------------------------------------------------
// 调用周期必须是 1 ms：
//   ① 对齐斜坡的步长(k_align_dt)和各种 ms 计时(k_control_period_ms)都按 1 ms 设计；
//   ② 目标值的目标就是 1 ms 更新。以前挂在 10 ms 的 uart_task 上，g_gimbal_*_set
//      是 10 ms 的阶梯信号，而电机每 1 ms 都在追它，等于人为引入 0~10 ms 的额外滞后
//      （实测 yaw 滞后 8 ms 与之一致）。
// 顺序固定：① 安全检查 → ② 下发目标与使能。
// 安全检查必须在前，否则掉线/过热时仍会先下发一次目标再被停。
// ============================================================================
extern "C" void motor_control_update(void)
{
  const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());

  // ---- ① 安全检查 ----
  // 单轴掉线由 motor_task 内部处理：run_motor_loop() 发现该轴 is_alive() 为假就把它的
  // 转矩指令清零，不影响其他轴。这里不能"任一轴掉线就整机停"，否则单电机台架测试
  // （只接 motor_a）会因为没接的 motor_c 一直离线而永远不出力矩。
  // 只有轨迹跟随必须 a/c 两轴都在线：目标在世界系，缺反馈时误差不再变化，环路会持续死推。
  if (g_gimbal_traj_enable && (!g_motor_a_online || !g_motor_c_online)) {
    // 反馈掉了(>100ms 无 CAN 帧)：只停 yaw/pitch 两个轴，折叠轴继续自锁，别让机械臂掉下来。
    motor_axis_enable(MOTOR_A, false);
    motor_axis_enable(MOTOR_C, false);
    trajectory_follow_abort();  // 轨迹回到起点，下次重新对齐
    return;
  }

  if (g_motor_a_temperature > 80U || g_motor_b_temperature > 80U ||
      g_motor_c_temperature > 80U) {
    motor_enable(false);        // 过热保护
    trajectory_follow_abort();
    return;
  }

  // ---- ② 折叠轴使能 / 自锁 ----
  // 折叠轴 b 出力：使能后它保持在本次使能位置，重力矩由 motor_task 的前馈承担。
  motor_angle(MOTOR_B, 0.0f);  // b 保持使能位置
  motor_enable(true);

  // ---- ③ A 轴(yaw)角度环自检：已通过（0 ↔ 15° 方波，过冲约 40%、振荡≈0.3 s，属 kp=2 的固有特性）
  //   换实验时可重新放开。看 plotter 通道 1(实际角) / 4(目标角) / 7(目标力矩)。
  //   constexpr float k_a_test_deg = 15.0f;
  //   constexpr uint32_t k_a_test_half_ms = 1000U;
  //   const bool a_test_high = ((now_ms / k_a_test_half_ms) & 1U) != 0U;
  //   motor_angle(MOTOR_A, a_test_high ? k_a_test_deg : 0.0f);
  //   motor_axis_enable(MOTOR_A, true);

  // ---- ③ 轨迹跟随（IMU 世界系反馈，符号已在"世界系保持"自检中确认）----
  // 启动流程由 trajectory_follow_update() 内部管理：
  //   kIdle  → 以当前 IMU 姿态为起点
  //   kAlign → 以 k_align_rate 限速爬向轨迹起点，连续到位 200 ms 才放行
  //   kFollow→ 目标 = 轨迹当前点
  const bool imu_ok =
      static_cast<uint32_t>(now_ms - g_imu_update_ms) < IMU_STALE_TIMEOUT_MS;
  // 三路反馈都在线才进世界系闭环：IMU + yaw(a) + pitch(c) + 折叠(b)。
  // b 掉线时它不出力、折叠臂会缓慢下坠，而 IMU 装在下游，下坠会让"世界系姿态"自己转起来。
  // 这时跑轨迹跟随毫无意义（目标冻结、误差单调增大、永远对不齐），所以 b 离线就整体不启动。
  const bool traj_ready =
      imu_ok && g_motor_b_online && g_motor_a_online && g_motor_c_online;

  if (traj_ready) {
    motor_axis_enable(MOTOR_A, true);
    motor_axis_enable(MOTOR_C, true);
    trajectory_follow_update(true);   // 内部会置 g_gimbal_traj_enable = true
  }
  else {
    // IMU 或电机反馈不在线：退出世界系闭环、松开这两个轴（折叠轴 b 不受影响）。
    g_gimbal_traj_enable = false;
    motor_axis_enable(MOTOR_A, false);
    motor_axis_enable(MOTOR_C, false);
    trajectory_follow_abort();
  }
  g_traj_following = (g_traj_phase == TrajPhase::kFollow);

  // ==========================================================================
  // 示例（默认注释）。换实验时把上面的 ②③ 注释掉，放开其中一个示例即可。
  // ⚠ a/c 的角度环开关是 motor_axis_enable(MOTOR_A/C, true)；
  //   motor_angle(MOTOR_A/C, ...) → 写目标角，但必须同时打开该轴开关才会出力。
  // ==========================================================================

  // ── 示例 1：a/c 轨迹跟随——跟随世界系轨迹（需要 IMU 在线），b 仍自锁
  //   const bool imu_ok =
  //     static_cast<uint32_t>(now_ms - g_imu_update_ms) < IMU_STALE_TIMEOUT_MS;
  //   trajectory_follow_update(imu_ok);   // 内部写 g_gimbal_yaw_set / g_gimbal_pitch_set
  //   motor_axis_enable(MOTOR_A, true);
  //   motor_axis_enable(MOTOR_C, true);
  //   g_gimbal_traj_enable = true;        // 切到世界系反馈
  //   motor_angle(MOTOR_B, 0.0f);
  //   motor_enable(true);

  // ── 示例 2：停止——折叠轴立即不再输出转矩（PID 内部状态由 motor_task 自动复位）
  //   motor_enable(false);

  // ── 示例 3：B 轴限速折叠——目标 -30°，最快 40°/s，约 0.75 s 走完
  //   直接给 -30° 是阶跃，P 项会瞬间出 0.52 N·m（折叠臂惯量大、无阻尼，实测过冲 60%），
  //   所以用 b_ref_deg 做斜坡，每个 10 ms 周期最多走 rate × 0.01 = 0.4°。
  //   b_ref_deg 必须是 static：斜坡状态要跨周期保留。
  //   方向或角度按实际改 k_b_target_deg；第一次先给 ±5° 确认方向。
  //   constexpr float k_b_target_deg = -30.0f;   // 目标角(°)，相对使能位置
  //   constexpr float k_b_rate_deg_s = 40.0f;    // 速度上限(°/s)
  //   static float b_ref_deg = 0.0f;             // 限速后的目标角(°)
  //   const float b_step_deg = k_b_rate_deg_s * 0.01f;
  //   if (b_ref_deg < k_b_target_deg - b_step_deg) {
  //     b_ref_deg += b_step_deg;
  //   }
  //   else if (b_ref_deg > k_b_target_deg + b_step_deg) {
  //     b_ref_deg -= b_step_deg;
  //   }
  //   else {
  //     b_ref_deg = k_b_target_deg;
  //   }
  //   motor_angle(MOTOR_B, b_ref_deg);
  //   motor_enable(true);
}
