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
// 折叠限速(°/s)。折叠臂惯量大且无阻尼，直接给阶跃实测过冲 60%，必须限速。默认 40。
extern volatile float g_cfg_fold_rate_dps;
// 复位限速(°/s)。默认 40。
extern volatile float g_cfg_unfold_rate_dps;
// ★ 折叠到位后停留多久再自动复位(ms)。默认 3000。
//   0 = 不自动复位（纯电平控制，外部调 motorB_fold_enable(false) 才回）。
extern volatile uint32_t g_cfg_fold_hold_ms;
// 折叠/复位期间 a/c 是否保持使能位置（true = 顶住，false = 松手）。
// 默认 true：pitch 无抱闸且有重力矩，松手会直接砸到限位。
extern volatile bool g_cfg_fold_hold_axes;
// 未跟随轨迹的轴是否保持使能位置。默认 false（与上电行为一致）。
// 想测单轴、同时不让 pitch 下垂时设为 true。
extern volatile bool g_cfg_untracked_hold_axes;

// ───────────────────────────── 2.1 红外触发 ─────────────────────────────
// 红外距离触发阈值(cm)：距离小于它就开始折叠。默认 30。
// 设为 0 = 关闭红外触发（调试期最安全，避免手靠近时误触发折叠）。
// 阈值只在 g_gy56_online 为 1 时才生效：GY-56 掉线时 g_gy56_distance_cm
// 会保留最后一个值，不加 online 判断会在传感器拔掉后一直误触发。
extern volatile uint16_t g_cfg_ir_trigger_cm;

// 对齐阶段角速度上限(°/s)。默认 23。
extern volatile float g_cfg_align_rate_dps;
// 对齐到位容差(°)。默认 3。
extern volatile float g_cfg_align_tolerance_deg;
// 对齐超时(ms)，超时后不再等、直接放行轨迹。默认 4000。
extern volatile uint32_t g_cfg_align_timeout_ms;

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
