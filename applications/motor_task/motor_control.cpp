#include <cstdint>
#include "cmsis_os.h"
#include "imu_task.hpp"
#include "motor_control.hpp"
#include "motor_task.hpp"
#include "trajectory_task/trajectory_task.hpp"
#include "laser_task.hpp"
// ============================================================================
// 电机控制层
// ----------------------------------------------------------------------------
// 职责：决定三个电机"做什么"——写目标角度、开关使能。
//       不碰 CAN，不碰 PID（那些都在 motor_task.cpp 内部）。
// 调用：由 motor_task 每 1 ms 调用一次 motor_control_update()。
// 单位：本层对外一律用"度"，写进 g_motor_*_angle_set 前才换算成 rad。
// 基准：目标角都是"相对本次使能位置"，零点由 motor_task 在使能后自动锁存：
//         给 0  → 保持使能时的姿势；
//         给 90 → 从使能位置转 90°。
//
// 对外只暴露三个动作（见 motor_control.hpp），优先级 失能 > 折叠 > 跟随：
//   motor_disable(true)      三个轴全松，云台因重力下垂
//   motorB_fold_enable(b)    b 折叠 / 复位，带限速与可调停留时间
//   motorA_track / motorC_track / motor_all_track
//                            a/c 按 IMU 世界系姿态跟随 trajectory_task
//
// 每个轴的三种归宿（由请求 + 在线状态共同决定）：
//   跟随  请求了跟随且该轴可用 → IMU 闭环跟轨迹
//   顶住  未跟随但需要顶住     → 编码器模式，目标 0，顶在使能位置
//   松手  其余情况             → 环路关闭，可被外力自由转动
// 折叠期间 a/c 一律不跟随（IMU 装在折叠关节下游，b 一动世界系就失效），
// 改为顶住或松手，由 g_cfg_fold_hold_axes 决定。
// ============================================================================

// ============================================================================
// 用法示例
// ----------------------------------------------------------------------------
// 本层只负责"把某个动作做出来"，不判断"什么时候该做"。触发条件（红外距离、
// 底盘 CAN 命令、遥控器开关）由上层决定，判断完调用下面的接口即可。
// 三个动作互相独立，同时请求时按优先级仲裁：失能 > 折叠 > 跟随。
// 所有接口都只是写一个 bool 请求量，可在任意任务/中断里调用（上电初值全为 0，
// 即"不动作"），真正的执行在 motor_control_update() 里每 1 ms 做一次。
//
// ─────────────────────────── 功能 1：折叠 ───────────────────────────
// 场景：红外测到障碍 → b 向后折叠避让 → 停留一会儿 → 自动复位。
//
// 用法 A（推荐）：一次调用跑完全套，停留时间由 g_cfg_fold_hold_ms 决定
//   if (红外触发 && !已折叠) {
//     motorB_fold_enable(true);        // 开始：折叠 1.5s + 停留 T + 复位 1.5s
//     已折叠 = true;
//   }
//   // 复位完成后再清标志，避免重复触发
//   if (已折叠 && g_fold_phase == 4U) {  // 4 = 复位完成（只在 hold_ms > 0 时才到得了）
//     motorB_fold_enable(false);         // 请求拉低，回到待命
//     已折叠 = false;
//   }
//
// 用法 B：自己控制停留时间（g_cfg_fold_hold_ms = 0 时必须用这种方式）
//   motorB_fold_enable(true);          // 折叠到 g_cfg_fold_deg 并一直保持
//   ...自己计时 T 毫秒...
//   motorB_fold_enable(false);         // 复位回 0°
//
// 折叠过程中随时调 false 都能立刻从半路开始复位（已验证 32° → 0° 正常）。
// 折叠/复位期间 a/c 不跟随轨迹，是否顶住由 g_cfg_fold_hold_axes 决定。
// 斜坡会自动走到目标，不要为了"让它继续走"而反复调用 true。
//
// ─────────────────────────── 功能 2：轨迹跟随 ───────────────────────────
// 场景：云台到达指定区域后开始跟随 trajectory_task 的曲线。
//
//   motor_all_track(true);             // a(yaw) + c(pitch) 两轴一起跟随
//   ...比赛阶段...
//   motor_all_track(false);            // 两轴一起退出
//
// 也可以只跟一个轴（测单轴时用）：
//   motorA_track(true);                // 只跟 yaw
//   motorC_track(true);                // 只跟 pitch
// 两个请求是独立的电平量，可以一真一假；motor_all_track(x) 等价于同时调用
// 上面两个，绝不会"只跟一个"。
// 不跟随的那个轴默认松手（可自由转动）；想让它在原地顶住，把
// g_cfg_untracked_hold_axes 设为 true。
//
// 首次调用不会立刻摆动：先限速对齐到轨迹起点（g_cfg_align_rate_dps），
// 对齐的轴都进 g_cfg_align_tolerance_deg 并连续保持 200 ms 才放行轨迹；
// 超过 g_cfg_align_timeout_ms 也强制放行，避免机械到不了目标时永久卡住。
// 跟随阶段可以用 g_traj_following（bool）判断"已经真正在跟了"，
// 计分/RMS 统计都要用它做门控，否则对齐阶段（误差恒 0）会把读数刷高。
//
// 注意：折叠期间调用会被拒绝并记住，折叠流程走完（g_fold_phase == 4）后自动开始。
//       理由：IMU 装在折叠关节下游，b 一动世界系姿态就整体变化，此时跟随没有意义。
//       单轴掉线不影响另一轴：track_a / track_c 是分开判定的。
//
// ─────────────────────────── 功能 3：总失能 ───────────────────────────
// 场景：遥控器失能（遥控器接在底盘 C 板上，通过 CAN 传过来）。
//
//   motor_disable(true);               // 三个轴全部停止输出（b 也松）
//   motor_disable(false);              // 解除：b 立即重新自锁
//
// 失能是最高优先级，任何时候调用都立刻生效，并会：
//   · a/b/c 三个轴全部停止输出——三个关节都没有抱闸，云台会因重力下垂，
//     这是有意的（避免堵转发热），确认机械上有缓冲/限位；
//   · 丢弃 b 的折叠流程（重新使能后不会自动再折一次）；
//   · 保留跟随请求——解除失能后若之前请求过跟随，会重新对齐再开始。
// 失能/过热导致的停机期间，g_motor_pid_enable 为 false，可直接在调试器里读。
//
// ─────────────────────────── 参数与状态观察 ───────────────────────────
// 全部参数都是 volatile 全局量（见 motor_control.hpp），调试器里直接改，立刻生效：
//   ── 折叠 ──
//   g_cfg_fold_deg           折叠角(°)          默认 60
//   g_cfg_fold_rate_dps      折叠限速(°/s)      默认 40
//   g_cfg_unfold_rate_dps    复位限速(°/s)      默认 40
//   g_cfg_fold_hold_ms       ★停留时间(ms)     默认 3000，0 = 不自动复位
//   g_cfg_fold_hold_axes     折叠时 a/c 顶住     默认 true
//   ── 红外触发 ──
//   g_cfg_ir_trigger_cm      ★距离阈值(cm)     默认 30，0 = 关闭红外触发
//   ── 轨迹跟随 ──
//   g_cfg_untracked_hold_axes 不跟随时 a/c 顶住  默认 false
//   g_cfg_align_rate_dps     对齐限速(°/s)      默认 23
//   g_cfg_align_tolerance_deg 对齐容差(°)       默认 3
//   g_cfg_align_timeout_ms   对齐超时(ms)       默认 4000
//
// 红外原始数据在 laser_task 里（不是参数，是传感器读数）：
//   g_gy56_distance_cm   最近一帧的距离(cm)
//   g_gy56_online        1 = 500ms 内收到过有效帧
//   g_gy56_mode / g_gy56_temperature_c   测量模式 / 模块温度
//
// g_fold_phase：折叠阶段 0=停在0° 1=折叠中 2=停留中 3=复位中 4=复位完成
// g_traj_status_bits（也就是 plotter 通道 18 的值）：
//   低 4 位 bit0 零偏标定完成 | bit1 a就绪 | bit2 c就绪 | bit3 跟随中
//   高 4 位 = g_fold_phase
//   例：0x0F(=15) → 零偏+a+c 全就绪且正在跟随（高 4 位 0 = 停在 0°）
//       0x27(=39) → 停留折叠中，a/c 就绪但不跟随（折叠期间强制退出跟随）
//       0x43(=67) → 复位完成，c 掉线，仅 a 就绪
//
// ─────────────────────────── 当前实验与切换方式 ───────────────────────────
// 本文件里"什么时候该做什么"的实验代码集中在 motor_control_update()：
//   · 函数开头 ⓪ 那段与函数末尾「示例 1~9」默认**全部注释**，
//     放开其中一段（去掉行首的 "//   "）就能切换实验，其余保持注释；
//   · 一段都不放开 = 上电不动作，只剩 b 自锁顶住云台重力（与原来的行为一致）。
// 正式比赛时改为由红外距离 / 底盘 CAN 命令触发（见示例 8 与示例 9）。
// ============================================================================
namespace
{
  // ─────────────────────── 1. 基础控制函数 ───────────────────────

  // 控制层对外一律用"度"，底层 rad 只在写入 g_motor_*_angle_set 时换算。
  constexpr float k_deg_to_rad = 3.14159265f / 180.0f;

  // 控制层内部按 1 ms 一拍推进，斜坡步长都用 0.001 s。
  constexpr float k_control_dt = 0.001f;

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

  // 总使能：true = 三个轴按各自开关出力；false = 全部停止输出。
  void motor_enable(bool enable) { g_motor_pid_enable = enable; }

  // ─────────────────────── 2. 请求（由 API 写，update 读） ───────────────────────
  // 全部是"电平"语义：调用者给什么值就保持什么值，update 每 1 ms 按它决策。
  bool g_req_disable = false; // 总失能请求
  bool g_req_fold = false;    // 折叠请求（true=折叠，false=复位）
  bool g_req_track_a = false; // a(yaw) 跟随请求
  bool g_req_track_c = false; // c(pitch) 跟随请求

  // ─────────────────────── 3. 折叠状态机 ───────────────────────
  enum class FoldPhase : uint8_t
  {
    kAtZero = 0,    // 停在 0°，等待折叠请求
    kFolding = 1,   // 0° → 目标角
    kHolding = 2,   // 到位后停留
    kUnfolding = 3, // 目标角 → 0°
    kDone = 4,      // 复位完成，锁存在 0°
  };

  FoldPhase g_fold_ph = FoldPhase::kAtZero;
  float g_fold_ref_deg = 0.0f;        // 限速后的折叠目标角(°)，跨周期保留
  uint32_t g_fold_hold_start_ms = 0U; // 进入停留阶段的时刻
  // 本次请求是否已经跑过一遍。跑过就锁住，避免"请求一直是 true"时反复折叠：
  // 上层若每周期都调 motorB_fold_enable(true)，没有这个锁存就会
  // 折叠→停留→复位→再折叠…无限循环。想再折一次必须先调 false 把它解锁。
  bool g_fold_ran = false;

  bool fold_busy()
  {
    return g_fold_ph == FoldPhase::kFolding || g_fold_ph == FoldPhase::kHolding ||
           g_fold_ph == FoldPhase::kUnfolding;
  }

  // 折叠流程清零（总失能时用），同时放弃当前折叠请求，
  // 否则重新使能的瞬间会从倒下的姿势再折一次。
  void fold_reset()
  {
    g_fold_ph = FoldPhase::kAtZero;
    g_fold_ref_deg = 0.0f;
    g_fold_hold_start_ms = 0U;
    g_fold_ran = false;
    g_req_fold = false;
    g_fold_phase = static_cast<uint8_t>(FoldPhase::kAtZero);
  }

  // 每 1 ms 推进一次折叠斜坡。
  // 单向限速：折叠用 g_cfg_fold_rate_dps，复位用 g_cfg_unfold_rate_dps。
  // 直接给大角度阶跃会让 P 项瞬间打出 0.5 N·m 以上，折叠臂惯量大且无阻尼，
  // 实测过冲 60%，所以必须限速。
  void fold_update(uint32_t now_ms)
  {
    const float step_fold = g_cfg_fold_rate_dps * k_control_dt;
    const float step_unfold = g_cfg_unfold_rate_dps * k_control_dt;

    // 请求拉低 = 解锁，允许下一次折叠。想再折一次必须先调 motorB_fold_enable(false)。
    if (!g_req_fold)
      g_fold_ran = false;

    switch (g_fold_ph)
    {
    case FoldPhase::kAtZero:
    case FoldPhase::kDone:
      g_fold_ref_deg = 0.0f;
      // g_fold_ran 保证整条序列只跑一遍：请求一直保持 true 也不会折第二次。
      if (g_req_fold && !g_fold_ran)
      {
        g_fold_ran = true;
        g_fold_ph = FoldPhase::kFolding;
      }
      break;

    case FoldPhase::kFolding:
      if (!g_req_fold)
      {
        // 外部中途取消：立刻转入复位，不要停在半路
        g_fold_ph = FoldPhase::kUnfolding;
        break;
      }
      g_fold_ref_deg += step_fold;
      if (g_fold_ref_deg >= g_cfg_fold_deg)
      {
        g_fold_ref_deg = g_cfg_fold_deg;
        g_fold_ph = FoldPhase::kHolding;
        g_fold_hold_start_ms = now_ms;
      }
      break;

    case FoldPhase::kHolding:
      if (!g_req_fold)
      {
        g_fold_ph = FoldPhase::kUnfolding;
        break;
      }
      // g_cfg_fold_hold_ms = 0 表示不自动复位，一直保持到外部拉低请求。
      // 自动复位时不清请求：防重复由 g_fold_ran 负责，请求保持原样更好预测。
      if (g_cfg_fold_hold_ms > 0U && (now_ms - g_fold_hold_start_ms) >= g_cfg_fold_hold_ms)
      {
        g_fold_ph = FoldPhase::kUnfolding;
      }
      break;

    case FoldPhase::kUnfolding:
      g_fold_ref_deg -= step_unfold;
      if (g_fold_ref_deg <= 0.0f)
      {
        g_fold_ref_deg = 0.0f;
        g_fold_ph = FoldPhase::kDone;
      }
      break;
    }

    motor_angle(MOTOR_B, g_fold_ref_deg);
    g_fold_phase = static_cast<uint8_t>(g_fold_ph);
  }

  // ─────────────────────── 4. 轨迹跟随 ───────────────────────
  // 目标为世界系角度，motor_task 直接以 IMU 姿态作为反馈闭环。
  // 轨迹本身就在世界系，因此这里直接下发，不做任何偏置换算。
  // 某个轴转向相反时不要去改这里，改 motor_task.cpp 的 k_imu_dir_*。
  //
  // yaw 与 pitch 都按"绝对角度"对齐：IMU 的 yaw/pitch 零点直接与轨迹角度对应，
  // 不做整体平移。这样计分程序用 plan_angle 与 imu.yaw 直接比较才有意义；
  // 若做平移，云台会稳定地差一个常量偏置，判分永远是 0。
  // 代价：上电后云台要转回轨迹起点（yaw 可能转较大角度），对齐阶段限速。
  //
  // 启动流程：轨迹上电停在起点，云台先限速对齐到"轨迹开始位置"，到位后再放行轨迹，
  // 这样使能瞬间不会从当前姿态直冲轨迹值。
  constexpr uint32_t k_align_hold_ms = 200U; // 需要连续到位的时长

  enum class TrajPhase
  {
    kIdle,   // 未启动，轨迹停在起点
    kAlign,  // 限速趋近轨迹起点，轨迹不放行
    kFollow, // 正常跟随
  };
  TrajPhase g_traj_phase = TrajPhase::kIdle;
  float g_align_yaw = 0.0f; // 对齐阶段实际下发的目标（已限速）
  float g_align_pitch = 0.0f;
  uint32_t g_in_position_ms = 0U;
  uint32_t g_align_ms = 0U; // 对齐已累计的时间，用于超时兜底

  float step_toward(float value, float target, float max_step)
  {
    const float diff = target - value;
    if (diff > max_step)
      return value + max_step;
    if (diff < -max_step)
      return value - max_step;
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

  // 三轴全松 + 轨迹复位。失能与过热保护共用。
  // trajectory_pause() 里有临界区，只在真正需要退出跟随时调一次。
  void all_off()
  {
    motor_enable(false);
    g_motor_a_loop_enable = false;
    g_motor_c_loop_enable = false;
    g_gimbal_traj_a = false;
    g_gimbal_traj_c = false;
    if (g_traj_phase != TrajPhase::kIdle)
      trajectory_follow_abort();
    g_traj_following = false;
  }

  // 每 1 ms 推进一次启动流程。
  // imu_ok 为假（IMU 数据过期）时不推进对齐判定，目标维持当前读数使偏差为 0。
  // yaw_ok / pitch_ok：该轴是否在跟随（请求了跟随且反馈在线）。
  //   不在跟随的轴不参与对齐判定，否则一个轴掉线会让另一个轴永远进不了窗口、只能等超时。
  void trajectory_follow_update(bool imu_ok, bool yaw_ok, bool pitch_ok)
  {
    // 未放行时 trajectory_read() 恒为轨迹起点，即"轨迹开始位置"。
    const TrajectoryPoint point = trajectory_read();

    if (g_traj_phase == TrajPhase::kIdle)
    {
      if (!imu_ok)
      {
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

    if (g_traj_phase == TrajPhase::kAlign)
    {
      if (imu_ok)
      {
        const float align_step = g_cfg_align_rate_dps * k_deg_to_rad * k_control_dt;
        g_align_yaw = step_toward(g_align_yaw, point.yaw, align_step);
        g_align_pitch = step_toward(g_align_pitch, point.pitch, align_step);
        g_gimbal_yaw_set = g_align_yaw;
        g_gimbal_pitch_set = g_align_pitch;

        // 只检查"在跟随"的轴，不跟随的轴不拖累另一个轴的对齐。
        const float tol = g_cfg_align_tolerance_deg * k_deg_to_rad;
        const bool yaw_in = !yaw_ok || within(g_yaw_rad, point.yaw, tol);
        const bool pitch_in = !pitch_ok || within(g_pitch_rad, point.pitch, tol);
        g_in_position_ms = (yaw_in && pitch_in) ? (g_in_position_ms + 1U) : 0U;
        g_align_ms += 1U;
      }
      if (g_in_position_ms >= k_align_hold_ms || g_align_ms >= g_cfg_align_timeout_ms)
      {
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
} // namespace

// 轨迹是否处于正常跟随阶段（见 motor_task.hpp 说明）。
volatile bool g_traj_following = false;
// 启动条件与阶段位图（见 motor_task.hpp 说明）。
volatile uint8_t g_traj_status_bits = 0U;
// ILC 当前叠加的补偿力矩（见 motor_task.hpp 说明）。
volatile float g_yaw_ilc_out = 0.0f;

// ─────────────────────────── 可调参数（默认值） ───────────────────────────
volatile float g_cfg_fold_deg = 60.0f;
volatile float g_cfg_fold_rate_dps = 40.0f;
volatile float g_cfg_unfold_rate_dps = 40.0f;
volatile uint32_t g_cfg_fold_hold_ms = 3000U;
volatile bool g_cfg_fold_hold_axes = true;
volatile bool g_cfg_untracked_hold_axes = false;
// 红外触发阈值(cm)，0 = 关闭。实际使用见 motor_control_update() 的示例 8。
volatile uint16_t g_cfg_ir_trigger_cm = 30U;
volatile float g_cfg_align_rate_dps = 23.0f;
volatile float g_cfg_align_tolerance_deg = 3.0f;
volatile uint32_t g_cfg_align_timeout_ms = 4000U;

// 折叠阶段（诊断）。
volatile uint8_t g_fold_phase = 0U;

// ============================================================================
// 对外动作接口
// ----------------------------------------------------------------------------
// 用法（含完整调用示例、参数说明、状态位解码）见文件顶部的「用法示例」。
// 函数签名与语义见 motor_control.hpp。
// ============================================================================

void motor_disable(bool disable) { g_req_disable = disable; }

void motorB_fold_enable(bool fold) { g_req_fold = fold; }

void motorA_track(bool track) { g_req_track_a = track; }

void motorC_track(bool track) { g_req_track_c = track; }

// 两轴一起跟随 = 同时调用两个单轴接口，绝不可能是"只跟一个"。
void motor_all_track(bool track)
{
  motorA_track(track);
  motorC_track(track);
}

// ============================================================================
// 入口：由 motor_task 每 1 ms 调用一次
// ----------------------------------------------------------------------------
// 调用周期必须是 1 ms：
//   ① 对齐斜坡的步长和各种 ms 计时都按 1 ms 设计；
//   ② 目标值本身就要 1 ms 更新。以前挂在 10 ms 的 uart_task 上，g_gimbal_*_set
//      是 10 ms 的阶梯信号，而电机每 1 ms 都在追它，等于人为引入 0~10 ms 的额外滞后
//      （实测 yaw 滞后 8 ms 与之一致）。
// 顺序固定：① 失能 → ② 过热 → ③ 折叠 → ④ 跟随 → ⑤ 汇总使能。
// 安全检查必须在前，否则掉线/过热时仍会先下发一次目标再被停。
// ============================================================================
void motor_control_update(void)
{
  const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());

  // ---- ① 总失能：最高优先级，三个轴全松 ----
  // 失能就是彻底不出力，避免堵转发热。代价是云台会因重力下垂（三个关节都没有抱闸）。
  if (g_req_disable)
  {
    all_off();
    fold_reset();
    g_traj_status_bits = static_cast<uint8_t>(g_fold_phase << 4);
    return;
  }

  // ---- ② 过热保护：等价于失能 ----
  if (g_motor_a_temperature > 80U || g_motor_b_temperature > 80U ||
      g_motor_c_temperature > 80U)
  {
    all_off();
    g_traj_status_bits = static_cast<uint8_t>(g_fold_phase << 4);
    return;
  }

  // ---- ③ 折叠轴：先算目标角，再决定 a/c 用哪种反馈 ----
  // b 恒随总使能自锁，重力矩由 motor_task 的前馈承担。
  fold_update(now_ms);
  motor_enable(true);

  // ---- ④ 轨迹跟随 ----
  const bool imu_ok =
      static_cast<uint32_t>(now_ms - g_imu_update_ms) < IMU_STALE_TIMEOUT_MS;
  // 公共前提：IMU 可用 + 零偏标定完成 + 折叠轴 b 在线。
  //   b 必须在线：它掉线时不出力、折叠臂会缓慢下坠，而 IMU 装在折叠关节下游，
  //   下坠会让"世界系姿态"自己转起来 —— 此时 a/c 的世界系闭环都失去意义。
  // 零偏标定必须在开始运动之前完成，否则永远采不到静止样本，
  //   而且零偏会一直存在 → yaw 单向漂移 → 环路持续单方向驱动 → 线缆缠绕。
  const bool base_ready = imu_ok && g_imu_bias_ready && g_motor_b_online;
  const bool a_online = base_ready && g_motor_a_online;
  const bool c_online = base_ready && g_motor_c_online;

  // 折叠期间不允许跟随：IMU 在折叠关节下游，b 一动世界系姿态就整体变化，
  // 此时跟随等于让 a/c 去追一个被折叠动作污染掉的参考。
  const bool busy = fold_busy();
  const bool want_a = g_req_track_a && !busy;
  const bool want_c = g_req_track_c && !busy;
  const bool track_a = want_a && a_online;
  const bool track_c = want_c && c_online;

  g_gimbal_traj_a = track_a;
  g_gimbal_traj_c = track_c;

  if (track_a || track_c)
  {
    // 轨迹推进只要有轴在跟随就走；不在跟随的轴已切回编码器模式，不受影响。
    trajectory_follow_update(imu_ok, track_a, track_c);
  }
  else if (g_traj_phase != TrajPhase::kIdle)
  {
    // 只在真正退出跟随时复位一次。trajectory_task 在未放行时自己会持续发布起点值，
    // 不必每 1 ms 调 trajectory_pause()（那里面是临界区，会拖累 1 ms 实时循环）。
    trajectory_follow_abort();
  }

  // 未跟随的轴：折叠期间看 g_cfg_fold_hold_axes，平时看 g_cfg_untracked_hold_axes。
  // 顶住 = 保持使能 + 编码器模式目标 0（motor_task 会把目标算成使能位置）。
  const bool hold_cfg = busy ? g_cfg_fold_hold_axes : g_cfg_untracked_hold_axes;
  const bool hold_a = !track_a && hold_cfg && a_online;
  const bool hold_c = !track_c && hold_cfg && c_online;

  g_motor_a_loop_enable = track_a || hold_a;
  g_motor_c_loop_enable = track_c || hold_c;
  if (!track_a)
    motor_angle(MOTOR_A, 0.0f);
  if (!track_c)
    motor_angle(MOTOR_C, 0.0f);

  g_traj_following = (g_traj_phase == TrajPhase::kFollow);
  g_traj_status_bits = static_cast<uint8_t>((g_imu_bias_ready ? 1U : 0U) |
                                            (a_online ? 2U : 0U) | (c_online ? 4U : 0U) |
                                            (g_traj_following ? 8U : 0U) |
                                            (g_fold_phase << 4));

  // ==========================================================================
  // 示例（默认全部注释）。换实验时放开其中一段（去掉行首的 "//   "），其余保持注释。
  // 一段都不放开 = 上电不动作，只剩 b 自锁顶住云台重力。
  // 下面几段也可以放到函数开头 ⓪ 的位置，效果相同；放在末尾是为了不打乱主流程。
  // --------------------------------------------------------------------------
  // 三个动作接口都是"电平"语义：调用一次之后一直保持，不必每周期重复调用。
  // 折叠例外：整条序列只跑一遍，请求保持 true 也不会反复折（内部 g_fold_ran 锁存）。
  // ==========================================================================

  // ---- ⓪ 当前实验（全部注释）----
  //
  // 上电自动跑一次折叠：折叠 → 停留 g_cfg_fold_hold_ms → 复位 → 停在 0°。
  // 每毫秒调用也没关系：内部有 g_fold_ran 锁存，整条序列只跑一遍，不会反复折叠。
  // 想再折一次必须先 motorB_fold_enable(false) 解锁。
  //   motorB_fold_enable(true);
  //

  // ── 示例 1：轨迹跟随——两轴一起（首次会先限速对齐轨迹起点，不会立刻摆动）
  //    motor_all_track(true);
  //   退出：motor_all_track(false);

  // ── 示例 2：轨迹跟随——只跟 yaw，pitch 不跟
  //   不跟随的轴默认松手（可自由转动）；想让它在原地顶住就把
  //   g_cfg_untracked_hold_axes 设为 true。
  //  motorA_track(true);
  //   退出：motorA_track(false);

  // ── 示例 3：轨迹跟随——只跟 pitch
  // motorC_track(true);
  //   退出：motorC_track(false);

  // ── 示例 4：折叠——停在折叠位不自动复位（先把 g_cfg_fold_hold_ms 设为 0）
  //   g_cfg_fold_hold_ms = 0U 时折叠到位后会一直保持 60°，不会自己往回走。
  //   想复位时再调用一次 motorB_fold_enable(false)。
  //  motorB_fold_enable(true);

  // ── 示例 5：折叠——中途反悔，立刻从当前位置复位（不需要等停留结束）
  //   motorB_fold_enable(false);
  //   已验证：从 32° 处开始复位，1.6 s 后回到 0°，无异常。

  // ── 示例 6：折叠完成后再自动开始跟随（串起功能 1 和功能 2）
  //   第 1 行只在阶段 0 时触发一次，避免反复重折；第 2 行等复位完成后放行跟随。
  //   if (g_fold_phase == 0U) motorB_fold_enable(true);
  //   if (g_fold_phase == 4U) motor_all_track(true);

  // ── 示例 7：总失能——三个轴全松（含 b）
  //   三个关节都没有抱闸，云台会因重力下垂，先确认机械上有缓冲/限位。
  //   失能会丢弃折叠流程；跟随请求会保留，解除后重新对齐。
  //   motor_disable(true);
  //   解除：motor_disable(false);

  // ── 示例 8：接红外测距（GY-56 修好后，需要 #include "laser_task.hpp"）
  //   阈值在 g_cfg_ir_trigger_cm（默认 30cm，0 = 关闭红外触发），调试器里可改。
  //   必须判 g_gy56_online：掉线时 g_gy56_distance_cm 会保留最后一个值，
  //   不判 online 会在传感器拔掉后一直误触发折叠。
  //   if (g_cfg_ir_trigger_cm > 0U && g_gy56_online &&
  //       g_gy56_distance_cm < g_cfg_ir_trigger_cm) {
  //     motorB_fold_enable(true);                  // 检测到障碍 → 折叠（只折一次）
  //   }
  //   if (g_fold_phase == 4U) motor_all_track(true);   // 复位完成后开始跟随

  // ── 示例 9：底盘 CAN 遥控器失能（协议定下来之后，需要 chassis_comm 模块）
  //   协议待定，这里只示意结构：收到失能位就调用，清掉再调用 false。
  //   motor_disable(g_chassis_disable_cmd);
  //   if (g_chassis_disable_cmd) return;           // 失能期间不再下发其它动作
}