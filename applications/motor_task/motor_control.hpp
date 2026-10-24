#ifndef APPLICATIONS_MOTOR_TASK_MOTOR_CONTROL_HPP
#define APPLICATIONS_MOTOR_TASK_MOTOR_CONTROL_HPP

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 云台控制层对外接口
// ----------------------------------------------------------------------------
// 本层决定三个电机"做什么"，只写 motor_task.hpp 里的使能/目标全局量，
// 不碰 CAN、不碰 PID。由 motor_task 每 1 ms 调用一次 motor_control_update()。
//
// 三层动作，优先级从高到低：
//   ① motor_disable(true)     总失能：任何情况下最高优先，直接把三个轴全松
//   ② motorB_fold_enable()    折叠轴 b 折叠 / 复位
//   ③ motor*_track()          轨迹跟随（按轴独立）
//
// 上电默认（谁都没调用时）：b 自锁顶住云台重力，a/c 松开不出力。
// ============================================================================

// ───────────────────────────── 1. 动作接口 ─────────────────────────────

// 总失能。true = 三个轴全部停止输出（a/b/c 都松，云台会因重力下垂）；
// false = 恢复正常，b 立即重新自锁。
// 失能期间折叠流程被清零（重新使能后不会自动再折一次），轨迹跟随请求保留。
// 遥控器右下开关拨到底的失能走另一条通路（DBus → g_chassis_disable_cmd），
// 与本接口并联，任一个为真即失能，无需应用层再调用本接口去复刻那条通路。
void motor_disable(bool disable);

// 折叠轴 b。true = 折叠（限速斜坡到 g_cfg_fold_deg）；false = 复位（限速回到 0°）。
// 停留时间：g_cfg_fold_hold_ms > 0 时，折叠到位后自动停留该时长再自动复位，
//           走完一遍停在 0° 且 g_fold_phase 变为 4（应用层据此判"复位完成"）。
//           g_cfg_fold_hold_ms = 0 时一直保持折叠，直到外部调用 false。
//
// 【只折一次】整条序列跑完一遍后，请求即使仍为 true 也不会再折第二次。
//   想再折一次必须先调 motorB_fold_enable(false)，再调 true。
//   这样上层每周期无条件调用 motorB_fold_enable(true) 也不会反复折叠。
void motorB_fold_enable(bool fold);

// 折叠唯一的自动触发源是红外（见 2.1 与 g_cfg_ir_trigger_cm）：上电不会自己折叠。
// 需要一次性折叠时仍在应用层调用 motorB_fold_enable(true)，或用 motor_control.cpp 示例 6。

// 轨迹跟随。true = 该轴切到 IMU 世界系反馈跟随 trajectory_task 的曲线；
// false = 该轴退出跟随。两者可独立设置，互不影响。
// 进入跟随后先限速对齐到轨迹起点（g_cfg_align_rate_dps），到位才放行轨迹。
void motorA_track(bool track);
void motorC_track(bool track);
void motor_all_track(bool track);

// ───────────────────────────── 2. 可调参数 ─────────────────────────────
// 全部是 volatile 全局量：调试器里直接改，立刻生效，不用重新编译。
// 每个量下面写明单位与默认值。

// 折叠角(°)，正值为向后折叠（实测方向）。默认 60。
extern volatile float g_cfg_fold_deg;
// 折叠限速(°/s)。折叠臂惯量大且无阻尼，直接给阶跃实测过冲 60%，必须限速。默认 150。
extern volatile float g_cfg_fold_rate_dps;
// 折叠软着陆段(°)：进入最后这么长的角度内线性减速，避免高速斜坡突然停住造成过冲。
// 默认 12；设 0 = 关闭（纯匀速限速，最快但过冲最大）。
extern volatile float g_cfg_fold_soft_land_deg;
// 复位限速(°/s)。默认 40。
extern volatile float g_cfg_unfold_rate_dps;
// ★ 折叠到位后停留多久再自动复位(ms)。默认 3000。
//   0 = 不自动复位（纯电平控制，外部调 motorB_fold_enable(false) 才回）。
extern volatile uint32_t g_cfg_fold_hold_ms;
// 折叠/复位期间 a/c 是否保持使能位置（true = 顶住，false = 松手）。
// 默认 true：pitch 无抱闸且有重力矩，松手会直接砸到限位。
extern volatile bool g_cfg_fold_hold_axes;
// 复位到位容差(°)：复位斜坡降到 0° 后继续压住 b 轴，实际角度进入该容差才算复位完成。
// 默认 1.0。摩擦/线缆导致"复位不到位"时看这个值与小一个数量级的效果。
extern volatile float g_cfg_fold_settle_tol_deg;
// 复位 settling 超时(ms)：实际角度始终进不了容差时，超过该时间仍判复位完成（避免卡死）。
// 默认 3000。残差会记在 g_fold_reset_err_deg 里；若每次都超时且残差偏大，
// 说明 b 轴出力不够 —— 去 motor_task.hpp 的 4.3 调 g_cfg_b_kp / g_cfg_b_ki。
extern volatile uint32_t g_cfg_fold_settle_timeout_ms;
// 诊断：复位判完成时 b 轴的实际偏差(°)。正 = 还差一点没回到 0；接近 0 = 真到位。
extern volatile float g_fold_reset_err_deg;
// 未跟随轨迹的轴是否保持使能位置。默认 false（与上电行为一致）。
// 想测单轴、同时不让 pitch 下垂时设为 true。
extern volatile bool g_cfg_untracked_hold_axes;

// ───────────────────────────── 2.1 红外触发 ─────────────────────────────
// 红外距离触发阈值(cm)：距离小于它就开始折叠。默认 50。
// 设为 0 = 关闭红外触发（调试期最安全，避免手靠近时误触发折叠）。
// 阈值只在 g_gy56_online 为 1 时才生效：GY-56 掉线时 g_gy56_distance_cm
// 会保留最后一个值，不加 online 判断会在传感器拔掉后一直误触发。
//
// 【已内置】小于阈值时自动执行一次完整序列：折叠 → 停留 g_cfg_fold_hold_ms → 复位，
// 不需要应用层再调用 motorB_fold_enable；序列结束后内部自动解锁并重新武装，
// 所以每次"障碍靠近 → 离开 → 再靠近"都会再折一次。只折叠，不自动开始轨迹跟随。
// 【跟随时关闭】只要存在跟随请求（自动档/测试档），红外不再触发新的折叠；
// 正在跑的折叠序列会正常走完，退出跟随后折叠功能立即恢复。
// 重新武装的条件：距离回到 阈值+5cm 以外，或者根本测不到（超量程/信号弱/离线）——
// 后者是障碍移开时的常见表现，两者都算"离开"，避免空旷场景下只折一次就不再触发。
// 前提：g_cfg_fold_hold_ms > 0（默认 3000），否则折到位后不自动复位。
// 注意：启用后由本层独占折叠请求，应用层不要再直接调 motorB_fold_enable()。
extern volatile uint16_t g_cfg_ir_trigger_cm;
// 红外触发完成的序列次数（诊断）：每走完一次折叠+复位 +1。
extern volatile uint32_t g_ir_fold_count;

// ───────────────────────────── 2.2 遥控器档位 ─────────────────────────────
// 右开关三档（由 uart_task 每 10 ms 解析，写在 motor_task.hpp 的 g_chassis_* 里）：
//   UP   测试档：立刻开始 yaw+pitch 跟随，用于验证整条链路是否完整。
//   MID  自动档：等"折叠复位完成"（g_fold_phase == 4）后，延时下面的
//        g_cfg_auto_follow_delay_ms 再开始跟随；没折过就不跟随。
//        再折一次会重新计时（折叠期间本来也不跟随）。
//   DOWN 失能档：三轴全松，折叠流程清零，优先级最高。
// 跟随启动后仍按 g_cfg_align_rate_dps 限速对齐轨迹起点。

// 中档自动跟随的延时(ms)：折叠完成后等这么久再跟随。默认 3000。
extern volatile uint32_t g_cfg_auto_follow_delay_ms;

// 对齐阶段角速度上限(°/s)。默认 23。
extern volatile float g_cfg_align_rate_dps;
// 对齐到位容差(°)。默认 3。
extern volatile float g_cfg_align_tolerance_deg;
// 对齐超时(ms)，超时后不再等、直接放行轨迹。默认 4000。
extern volatile uint32_t g_cfg_align_timeout_ms;

// ───────────────────────────── 2.3 接收机链路 ─────────────────────────────
// 遥控接收机（DBus）掉线时的处理策略，三档，默认 0。
//   0 = 不做链路保护，云台完全按自己的状态工作（与加这套功能之前一致，单板可用）。
//   1 = 上电后收到过至少一帧 DBus 才武装（g_chassis_heartbeat_seen），此后丢失 → 失能。
//       没插接收机也不会把云台锁死，日常推荐。
//   2 = 始终武装，包括"开机前线就断了"。比赛用；代价是云台单独上电时 b 轴不顶、
//       云台臂会因重力下垂（三个关节都没有抱闸）。
// 为什么需要这一层：失能信号来自接收机，接收机一掉线就收不到开关状态，
//   不保护的话线一断、操作手拨到底也不会松手。
extern volatile uint8_t g_cfg_chassis_link;

// ───────────────────────────── 3. 诊断 ─────────────────────────────
// 折叠流程阶段：0=停在 0° 1=折叠中 2=停留中 3=复位中 4=复位完成(锁存到下次折叠)。
// 同时放在 g_traj_status_bits 的高 4 位，可直接在 plotter 通道 18 上看到。
extern volatile uint8_t g_fold_phase;

// ───────────────────────────── 4. 任务入口 ─────────────────────────────
// 由 motor_task 每 1 ms 调用一次，调用周期必须保持 1 ms（对齐斜坡按它算步长）。
void motor_control_update(void);

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_MOTOR_TASK_MOTOR_CONTROL_HPP
