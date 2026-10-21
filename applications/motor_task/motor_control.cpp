#include <cmath>
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
  // 单轴掉线由 motor_task 内部处理：run_motor_loop() 发现该轴 is_alive() 为假就把它的
  // 转矩指令清零，不影响其他轴。这里不能"任一轴掉线就整机停"，否则单电机台架测试
  // （只接 motor_a）会因为没接的 motor_c 一直离线而永远不出力矩。
  // 只有轨迹跟随必须 a/c 两轴都在线：目标在世界系，缺反馈时误差不再变化，环路会持续死推。
  if (g_gimbal_traj_enable && (!g_motor_a_online || !g_motor_c_online)) {
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
 // trajectory_follow_update(imu_ok);

  // ---- ③ 使能 ----
  //motor_enable(true);

  // ==========================================================================
  // 可选测试示例（默认全部注释，不影响当前行为）
  // 用法：去掉对应示例的注释符，并把上面的 trajectory_follow_update() 注释掉。
  // ==========================================================================

  // ── 示例 0：开环力矩测试（排查"电机到底收不收指令"，完全不经过 PID）
  //   把示例 1 的 motor_enable(true) 换成下面两行。此时 g_motor_a_cmd 由本函数直接写，
  //   motor_task 只负责原样下发。0.3N·m 起步逐步加大(0.5 / 1.0)，电机应持续同向转动：
  //   转了 → 电机与 CAN 通路没问题，问题在 PID 参数；不转 → 是模式/ID/接线问题。
  //   注意：开环不会因电机堵转而自动降力，电机会一直加速直到撞限位/电源保护。
  //         上面那次"冲一下然后停"就是开环的正常表现，测完必须立刻注释掉。
  //   motor_enable(false);
  //   g_motor_a_cmd = 0.5f;

  // ── 折叠轴重力前馈标定 ────────────────────────────────────────────────
  //   原理: 折叠轴在竖直平面内摆动, 重力矩 τ(θ)=m·g·r·cos(θ-θ0)=P·cosθ+Q·sinθ。
  //   做法: 让折叠轴依次停在若干角度, 每个角度等到稳态后采样"绝对角 θ / 稳态指令 τ",
  //         最后对 (θ,τ) 做线性最小二乘解出 P、Q, 直接写入前馈, 前馈立即生效。
  //   结果同时发布到 g_fold_cal_amp / g_fold_cal_phase, 可抄进常量长期使用。
  //   ⚠ 标定会主动移动折叠轴, 请先确认整个角度范围(0~45°)内没有障碍物。
  //   ⚠ 标定值已固化在 motor_task.cpp 的 g_fold_ff_p / g_fold_ff_q 中,
  //     正常使用时保持 false(上电不扫角度); 更换负载后再临时置 true 重标一次。
  constexpr bool k_fold_cal_enable = false;
  constexpr float k_cal_angles[] = {0.0f, 15.0f, 30.0f, 45.0f};
  constexpr uint32_t k_cal_points = 4U;
  constexpr uint32_t k_cal_settle_ms = 2000U;  // 每点先稳定 2s 再采样
  constexpr uint32_t k_cal_sample_ms = 400U;   // 采样窗 0.4s(40 个控制周期)

  struct FoldCal
  {
    uint32_t point = 0U;
    uint32_t t_ms = 0U;
    float sum_angle = 0.0f;
    float sum_cmd = 0.0f;
    uint32_t n = 0U;
    float cost[k_cal_points] = {};  // Σcos²θ / Σcosθsinθ / Σsin²θ / Στcosθ / Στsinθ
    float sx = 0.0f, sxy = 0.0f, sy = 0.0f, sxz = 0.0f, syz = 0.0f;
  };
  static FoldCal cal;

  if (k_fold_cal_enable && g_fold_cal_state != 2U) {
    // 阶段 A: 切换到本标定点
    if (cal.n == 0U && cal.t_ms == 0U) {
      cal.t_ms = now_ms;
      cal.point = 0U;
      g_fold_cal_state = 1U;
    }
    const float target_deg = k_cal_angles[cal.point];
    motor_angle(MOTOR_B, target_deg);
    motor_enable(true);

    const uint32_t elapsed = now_ms - cal.t_ms;
    if (elapsed >= k_cal_settle_ms && elapsed < k_cal_settle_ms + k_cal_sample_ms) {
      // 阶段 B: 采样。用不含前馈的 PID 输出, 它正好等于维持该姿态所需的力矩。
      cal.sum_angle += g_motor_b_angle;
      cal.sum_cmd += g_motor_b_pid_out;
      ++cal.n;
    }
    else if (elapsed >= k_cal_settle_ms + k_cal_sample_ms) {
      // 阶段 C: 收下本点数据(40 点平均), 换下一个角度
      if (cal.n > 0U) {
        const float theta = cal.sum_angle / static_cast<float>(cal.n);
        const float tau = cal.sum_cmd / static_cast<float>(cal.n);
        const float c = std::cos(theta);
        const float s = std::sin(theta);
        cal.sx += c * c;
        cal.sxy += c * s;
        cal.sy += s * s;
        cal.sxz += tau * c;
        cal.syz += tau * s;
      }
      cal.sum_angle = 0.0f;
      cal.sum_cmd = 0.0f;
      cal.n = 0U;
      cal.t_ms = now_ms;

      if (cal.point + 1U >= k_cal_points) {
        // 阶段 D: 最小二乘解 [sx sxy; sxy sy]·[P;Q] = [sxz; syz]
        const float det = cal.sx * cal.sy - cal.sxy * cal.sxy;
        if (std::fabs(det) > 1e-6f) {
          g_fold_ff_p = (cal.sxz * cal.sy - cal.syz * cal.sxy) / det;
          g_fold_ff_q = (cal.syz * cal.sx - cal.sxz * cal.sxy) / det;
          g_fold_cal_amp = std::sqrt(g_fold_ff_p * g_fold_ff_p + g_fold_ff_q * g_fold_ff_q);
          g_fold_cal_phase = std::atan2(g_fold_ff_q, g_fold_ff_p);
        }
        g_fold_cal_state = 2U;
      }
      else {
        ++cal.point;
      }
    }
  }
  else {
    // 正常折叠动作: 保持 → 折叠30° → 保持 → 复位，每段 3s
    // 目标用限速斜坡而不是瞬间阶跃: 30° 一步跳变会让无阻尼的大惯量手臂冲过头(实测过冲 60%)。
    // 限速后加速度小, 过冲几乎消失, 而且不依赖 D 项(不会放大编码器噪声)。
    static uint32_t phase_start_ms = 0U;
    static uint8_t phase = 0U;
    static float fold_ref = 0.0f;  // 限速后的目标角(度)
    constexpr uint32_t k_phase_ms = 3000U;
    constexpr float k_fold_angle = 30.0f;
    constexpr float k_fold_slew_deg_per_s = 40.0f;  // 折叠速度上限, 30° 约 0.75s 走完
    constexpr float k_control_dt = 0.01f;           // 本函数调用周期

    if (now_ms - phase_start_ms >= k_phase_ms) {
      phase_start_ms = now_ms;
      phase = static_cast<uint8_t>((phase + 1U) % 4U);
    }
    const bool folded = (phase == 1U) || (phase == 2U);
    const float goal = folded ? k_fold_angle : 0.0f;
    const float max_step = k_fold_slew_deg_per_s * k_control_dt;
    if (fold_ref < goal - max_step) {
      fold_ref += max_step;
    }
    else if (fold_ref > goal + max_step) {
      fold_ref -= max_step;
    }
    else {
      fold_ref = goal;
    }

    motor_angle(MOTOR_A, 0.0f);
    motor_angle(MOTOR_B, fold_ref);
    motor_enable(true);
  }

  // ── 调参用：15° 阶跃往复测试（每 4 s 在 0 与 +15° 之间切换）
  // static uint32_t last_toggle_ms = 0U;
  // static bool at_zero = true;
  // if (now_ms - last_toggle_ms >= 4000U) {
  //   last_toggle_ms = now_ms;
  //   at_zero = !at_zero;
  // }
  // motor_angle(MOTOR_A, at_zero ? 0.0f : 15.0f);
  // motor_enable(true);

  // ── 示例 1：motor_a 从使能位置转 +90° 并保持
  //   motor_angle(MOTOR_A, 30.0f);
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
    //motor_enable(true);
   //  static uint32_t last_toggle_ms = 0U;
   //  static bool at_zero = true;
   // if (now_ms - last_toggle_ms >= 2000U) {
   //    last_toggle_ms = now_ms;
    //   at_zero = !at_zero;
   //    motor_angle(MOTOR_A, at_zero ? 0.0f : 90.0f);
   //  }

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