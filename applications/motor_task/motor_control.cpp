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
// 本层只负责"把某个动作做出来"，不判断"什么时候该做"。触发条件（底盘 CAN 命令、
// 遥控器开关）由上层决定，判断完调用下面的接口即可。（红外距离触发是例外：
// 已在控制层内置，见 2.1 与示例 8，应用层无需介入。）
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
// 底盘触发（已实现）：底盘上档自动行驶 3.5 m 跑完且车轮停稳后，0x124 载荷的 bit1 置 1，
// 本文件的 ⓪ 在电平跳变时自动调用 motor_all_track(true)；离开上档后该位清 0，
// ⓪ 随之调用 motor_all_track(false) 退出跟随。应用层不需要再写触发逻辑。
//
// ─────────────────────────── 功能 3：总失能 ───────────────────────────
// 场景：遥控器失能（遥控器接在底盘 C 板上，通过 CAN 传过来）。
// 两条通路都已在 motor_task 里接好，本层 update 直接把结果并进失能判定，
// 无需应用层调用：
//   0x123（下档/断线时 200 Hz，快路径）+ 0x124.bit0 → g_chassis_disable_cmd；
//   0x124（所有状态 100 Hz）                        → g_chassis_online，丢失即判不在线。
// 链路丢失是否失能由 g_cfg_chassis_link 决定（0=不保护 1=收到过心跳后武装 2=始终武装）。
// 需要软件自行失能时仍可调用本接口：
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
//   g_cfg_fold_deg           折叠角(°)          默认 100
//   g_cfg_fold_rate_dps      折叠限速(°/s)      默认 150（调试器里可再快）
//   g_cfg_fold_soft_land_deg 折叠软着陆段(°)    默认 12，0 = 关闭（过冲会变大）
//   g_cfg_unfold_rate_dps    复位限速(°/s)      默认 40
//   g_cfg_fold_hold_ms       ★停留时间(ms)     默认 3000，0 = 不自动复位
//   g_cfg_fold_hold_axes     折叠时 a/c 顶住     默认 true
//   g_cfg_fold_settle_tol_deg 复位到位容差(°)   默认 1.0（进不了就一直压住 b 轴）
//   g_cfg_fold_settle_timeout_ms 复位settle超时 默认 3000
//   g_fold_reset_err_deg     复位完成时实际偏差(°)（诊断，读它就知道差多少）
//   g_fold_settling          1 = 复位微调中（诊断；此期间 b 轴积分分离被关掉）
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
  constexpr float k_rad_to_deg = 57.29578f; // 读反馈量（rad）做判断时用

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
  // 复位斜坡第一次走到 0 的时刻。0 = 本次复位还没到 0。
  // 用来给"目标到 0 但机械没到"的等待限时（见 kUnfolding）。
  uint32_t g_fold_zero_ms = 0U;
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
    g_fold_zero_ms = 0U;
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

    // 每次进入复位段都重新计时（中途取消折叠也会转进复位）。
    if (g_fold_ph != FoldPhase::kUnfolding)
      g_fold_zero_ms = 0U;

    // 复位微调标志：默认 0，只在 kUnfolding 的"目标已到 0 但实际没到"时置 1，
    // motor_task 据此关掉折叠轴的积分分离，让积分把静摩擦残差顶平。
    g_fold_settling = 0.0f;

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
      {
        // 接近目标时线性减速（软着陆）：高速斜坡突然停住会把速度差全部变成过冲，
        // 实测 40°/s 时都过冲 60%。减速段内保留 15% 的最低速，避免最后几度磨太久。
        float step = step_fold;
        if (g_cfg_fold_soft_land_deg > 0.0f)
        {
          const float remain = g_cfg_fold_deg - g_fold_ref_deg;
          if (remain < g_cfg_fold_soft_land_deg)
          {
            step *= remain / g_cfg_fold_soft_land_deg;
            const float min_step = step_fold * 0.15f;
            if (step < min_step)
              step = min_step;
          }
        }
        g_fold_ref_deg += step;
      }
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
    {
      // 斜坡把"目标"降到 0°。
      if (g_fold_ref_deg > 0.0f)
      {
        g_fold_ref_deg -= step_unfold;
        if (g_fold_ref_deg < 0.0f)
          g_fold_ref_deg = 0.0f;
      }
      if (g_fold_ref_deg > 0.0f)
        break;

      // 目标到 0 不代表机械到 0：限速斜坡一停，摩擦/线缆/重力会让 b 轴停在
      // 差几度的位置（实测约 2~5°）。以前到这里立刻判 kDone，上层就以为已复位。
      // 现在的做法是继续以 0° 为目标压住 b 轴（等于再"调"一段），
      // 直到实际角度进入 g_cfg_fold_settle_tol_deg 才算复位完成；
      // 始终进不去（静摩擦卡住）则超过 g_cfg_fold_settle_timeout_ms 后放行，
      // 并把残差记到 g_fold_reset_err_deg 供调试观察。
      if (g_fold_zero_ms == 0U)
        g_fold_zero_ms = now_ms;

      const float err_deg = g_motor_b_angle_rel * k_rad_to_deg;
      const float abs_err_deg = (err_deg >= 0.0f) ? err_deg : -err_deg;
      const bool settled = abs_err_deg <= g_cfg_fold_settle_tol_deg;
      const bool timeout = (now_ms - g_fold_zero_ms) >= g_cfg_fold_settle_timeout_ms;
      if (settled || timeout)
      {
        g_fold_reset_err_deg = err_deg;
        g_fold_ph = FoldPhase::kDone;
      }
      else
      {
        // 告诉 motor_task：现在在"压住 b 轴做微调"，积分不要暂停，
        // 否则静摩擦造成的几度残差会被积分分离冻住，永远回不到 0°。
        g_fold_settling = 1.0f;
      }
      break;
    }
    }

    motor_angle(MOTOR_B, g_fold_ref_deg);
    g_fold_phase = static_cast<uint8_t>(g_fold_ph);
  }

  // ─────────────────────── 3.1 红外遮挡 → 折叠并复位一次 ───────────────────────
  // 触发条件：GY-56 在线、测距状态可靠、距离 < g_cfg_ir_trigger_cm（0 = 关闭）。
  // 一次序列 = 折叠 → 停留 g_cfg_fold_hold_ms → 复位，全部由上面的状态机完成；
  // 本函数只负责"发起请求"和"序列结束后解锁"，不需要应用层再写触发代码。
  // 序列结束后不自动开始轨迹跟随（跟随请求由应用层/示例区决定）。
  // 上电不会自己折叠：唯一的触发源是红外（或应用层直接调 motorB_fold_enable）。
  // 要求 g_cfg_fold_hold_ms > 0（默认 3000），否则折叠后不会自动复位，序列停在保持位。
  //
  // 重新武装有回差：必须离开"阈值 + k_ir_release_cm"才允许下一次触发，
  // 否则目标停在 30cm 附近时，会在阈值上下反复折叠、反复触发。
  constexpr uint16_t k_ir_release_cm = 5U;
  bool g_ir_armed = true;    // true = 允许触发下一次
  bool g_ir_folding = false; // true = 已发起折叠序列，等它复位完成

  // 距离是否可信：离线或测距状态非"可靠"时，g_gy56_distance_cm 可能是上一次的旧值
  // （例如超量程、信号弱），此时不能拿它做触发判断。
  bool ir_distance_valid() { return g_gy56_online && g_gy56_range_status == 0U; }

  void ir_fold_update()
  {
    // 已发起：等整条序列跑完（kDone = 复位完成）再解锁请求、重新武装。
    if (g_ir_folding)
    {
      if (!g_req_fold)
      {
        // 请求已被 fold_reset() 撤销（总失能、过热等）：放弃本次序列。
        // 不直接重新武装，要求先离开遮挡区，避免解除失能后立即又折一次。
        g_ir_folding = false;
        g_ir_armed = false;
      }
      else if (g_fold_ph == FoldPhase::kDone)
      {
        motorB_fold_enable(false); // 复位完成 → 解锁，允许下一次折叠
        g_ir_folding = false;
        g_ir_armed = false;
        ++g_ir_fold_count;
      }
      return;
    }

    // 未触发：等障碍离开后再武装。
    // 注意"离开"有两种表现：距离变大（> 阈值+回差），或者根本测不到（超量程/信号弱/
    // 模块离线）。后者是把手/障碍移开时最常见的情况 —— 若要求"必须测到 >35cm 才武装"，
    // 空旷场景下会永远武装不上，表现为"只折一次就不再触发"。
    // 触发侧仍要求数据可信，所以这里放宽不会导致误触发。
    if (!g_ir_armed)
    {
      if (!ir_distance_valid() || g_gy56_distance_cm > g_cfg_ir_trigger_cm + k_ir_release_cm)
        g_ir_armed = true;
      return;
    }

    if (g_cfg_ir_trigger_cm == 0U || !ir_distance_valid())
      return;
    if (g_gy56_distance_cm >= g_cfg_ir_trigger_cm)
      return;
    if (fold_busy())
      return; // 已有折叠流程在跑（应用层自己发起的），等它结束再触发

    motorB_fold_enable(true);
    g_ir_folding = true;
    g_ir_armed = false;
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
volatile float g_cfg_fold_deg = 100.0f;
volatile float g_cfg_fold_rate_dps = 150.0f;
volatile float g_cfg_unfold_rate_dps = 40.0f;
volatile uint32_t g_cfg_fold_hold_ms = 3000U;
// 折叠最后这一段(°)内按剩余角度线性减速，避免高速斜坡突然停住造成过冲；0 = 关闭。
volatile float g_cfg_fold_soft_land_deg = 12.0f;
volatile bool g_cfg_fold_hold_axes = true;
// 复位 settling：斜坡到 0 后继续压住 b 轴，等实际角度进容差才算复位完成。
volatile float g_cfg_fold_settle_tol_deg = 1.0f;
volatile uint32_t g_cfg_fold_settle_timeout_ms = 3000U;
// 诊断：复位判完成时 b 轴的实际偏差(°)。
volatile float g_fold_reset_err_deg = 0.0f;
volatile bool g_cfg_untracked_hold_axes = false;
// 红外触发阈值(cm)，0 = 关闭。实际使用见 motor_control_update() 的示例 8。
volatile uint16_t g_cfg_ir_trigger_cm = 30U;
volatile float g_cfg_align_rate_dps = 23.0f;
volatile float g_cfg_align_tolerance_deg = 3.0f;
volatile uint32_t g_cfg_align_timeout_ms = 4000U;
// 底盘心跳（CAN 0x124）丢失的处理策略，三档。默认 0。
//   0 = 不做链路保护，云台完全按自己的状态工作（与加这套功能之前一致，单板可用）。
//   1 = 上电后收到过心跳才武装（g_chassis_heartbeat_seen）；此后心跳丢失 → 失能。
//       底盘没接/没上电/没刷心跳固件时不会把云台锁死，日常调试与比赛都适用。
//   2 = 始终武装：收不到心跳就失能，包括"开机前线就断了"。
//       比赛用这一档；代价是云台单独上电时 b 轴不顶，云台臂会因重力下垂。
volatile uint8_t g_cfg_chassis_link = 0U;

// 红外遮挡触发的完成次数（诊断）：每走完一次"遮挡→折叠→停留→复位"序列 +1。
volatile uint32_t g_ir_fold_count = 0U;

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
// 顺序固定：⓪ 底盘跑完触发 → ① 失能 → ② 过热 → ③ 折叠 → ④ 跟随 → ⑤ 汇总使能。
// 安全检查必须在前，否则掉线/过热时仍会先下发一次目标再被停。
// ⓪ 只记录"跟随请求"，真正的执行仍在 ④ 之后，所以放在最前面不会绕过安全逻辑。
// ============================================================================
// 调试开关：false = 三个轴一律不出力（失能），用于单独调传感器/上层逻辑时排除电机。
// true = 正常工作。关闭时连同折叠、跟随、底盘触发一起跳过，本文件不再写任何使能/目标。
// 注意：三个关节都没有抱闸，失能后云台会因重力下垂，先把机械撑住。
constexpr bool kMotorOutputEnable = true;

void motor_control_update(void)
{
  const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());

  if (!kMotorOutputEnable)
  {
    all_off();
    return;
  }

  // ---- ⓪ 底盘"上档跑完"状态位 → 开始 / 停止跟随 ----
  // g_chassis_follow_cmd 是 0x124 载荷里的 bit1（电平），底盘每 10 ms 刷新一次：
  //   上档跑完且车轮停稳 → 1（开始跟随）；离开上档 → 0（退出跟随）。
  // 只在电平变化时动作一次：
  //   · 电平语义：丢帧不会漏掉，重复帧也不会重复触发；
  //   · 边沿语义：不会把上层之后主动 motor_all_track(true/false) 的意图每 10 ms 覆盖回去。
  // 放在 ① 之前：失能期间收到的信号也照常记成"跟随请求"（与 motor_disable 的语义一致，
  // 解除失能后重新对齐再开始跟随），不会被安全逻辑吞掉。
  static bool follow_cmd_prev = false;
  if (g_chassis_follow_cmd != follow_cmd_prev)
  {
    motor_all_track(g_chassis_follow_cmd);
    follow_cmd_prev = g_chassis_follow_cmd;
  }

  // ---- ① 总失能：最高优先级，三个轴全松 ----
  // 失能就是彻底不出力，避免堵转发热。代价是云台会因重力下垂（三个关节都没有抱闸）。
  // 三个来源并联，任一为真即失能：
  //   g_req_disable            本层 API（motor_disable）
  //   g_chassis_disable_cmd    底盘遥控器下档，由 motor_task 从 CAN 0x123 还原（100 ms 超时）
  //   g_cfg_chassis_link == 1 && 曾收到过心跳 && !g_chassis_online
  //   g_cfg_chassis_link == 2 && !g_chassis_online
  //                            底盘心跳（CAN 0x124）丢失 → fail-safe 失能
  // 【为什么必须有心跳这一路】0x123 只在失能时发，所以"收不到 0x123"既可能是
  //   "底盘正常行驶"，也可能是"C 板没接上/线断了"。只有常发的 0x124 能把这两种情况分开；
  //   否则线一断，遥控器下档就静默失效，操作手按了也不会松手。
  // 【为什么分三档】云台物理上无法区分"底盘正常但没说话"和"底盘不在"，只能由配置决定：
  //   0 = 不保护（单板调试，与加这套功能之前完全一致）；
  //   1 = 上电收到过心跳之后才武装（日常；底盘没接也不会把云台锁死在失能）；
  //   2 = 始终武装（比赛；开机前线就断也保护，代价是"没接底盘 = 云台不出力"）。
  const bool link_lost =
      (g_cfg_chassis_link == 2U) ? !g_chassis_online
      : (g_cfg_chassis_link == 1U) ? (g_chassis_heartbeat_seen && !g_chassis_online)
                                   : false;
  if (g_req_disable || g_chassis_disable_cmd || link_lost)
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
  // 红外触发只写折叠请求，由下面的状态机推进，所以放在 fold_update 之前。
  ir_fold_update();
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
  //   本条保持注释。红外触发只负责折叠，不会自动放行跟随；
  //   需要跟随时在这里放开 motor_all_track(true)（两轴）或单轴接口。
  //   motorC_track(true);
  //   退出：motorC_track(false);

  // ── p示例 4：折叠——停在折叠位不自动复位（先把 g_cfg_fold_hold_ms 设为 0）
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

  // ── 示例 8：红外测距触发折叠（已实现，无需再写代码）
  //   阈值 g_cfg_ir_trigger_cm（默认 30cm，0 = 关闭），调试器里可改。
  //   距离小于阈值就自动跑一次"折叠 → 停留 g_cfg_fold_hold_ms → 复位"，
  //   逻辑在 ③ 前的 ir_fold_update()：序列完成才解锁，障碍离开后重新武装，
  //   所以每次"靠近 → 离开 → 再靠近"都会再折一次（g_ir_fold_count 可看次数）。
  //   "离开"= 距离 > 阈值+5cm 或 测不到（超量程/信号弱/离线）。
  //   只折叠不跟随：需要跟随时在示例 1/2/3 里放开对应行。
  //   前提：g_gy56_online 且测距状态可靠（避免掉线/超量程时用旧值误触发）。

  // ── 示例 9：底盘 CAN 遥控器失能（已实现，无需再写代码）
  //   协议：底盘在遥控器下档（失能）期间每 1 ms 发标准帧 0x123，解除后停发。
  //   motor_task 收到后把 g_chassis_disable_cmd 置 true，100 ms 无帧自动清 false，
  //   本函数开头的 ① 已经把该标志并进总失能判定（失能 > 折叠 > 跟随），
  //   所以失能期间这里根本不会执行到，无需再额外 return。

  // ── 示例 10：底盘上档跑完 → 开始跟随 / 离开上档 → 停止跟随（已实现）
  //   跑完 3.5 m 且车轮停稳后，0x124 载荷 bit1 置 1；离开上档自动清 0。
  //   motor_task 把它解成 g_chassis_follow_cmd，本函数开头的 ⓪ 在跳变时调用
  //   motor_all_track(true/false)，开始跟随后照 ④ 的流程先对齐再跟随。

  // ── 示例 11：上电自动折叠 —— 已移除
  //   上电不再自己折叠，唯一触发源是红外（见示例 8）。
  //   需要一次性折叠时用示例 6 的两行写法，把 g_cfg_fold_hold_ms 设成需要的停留时间。
}