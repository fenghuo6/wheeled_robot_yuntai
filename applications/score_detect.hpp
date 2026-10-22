#ifndef APPLICATIONS_SCORE_DETECT_HPP
#define APPLICATIONS_SCORE_DETECT_HPP

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 命中率统计（比赛计分用）。
//   fire_ratio = 指数滑动平均( |实际 − 期望| < 阈值 ? 1 : 0 )
//   α = 0.001，配合 1 ms 调用周期，时间常数约 1 s，
//   稳态值约等于"误差落在容差窗口内的时间占比"。
//   ⚠ 调用周期必须真的是 1 ms；放到 10 ms 任务里时间常数会变成 10 s，分数不可比。
//
// 容差：yaw 0.5°、pitch 1.5°（与比赛给定的计分程序一致）。
void yaw_detect(float gimbal_angle, float plan_angle, float *fire_ratio);
void pitch_detect(float gimbal_angle, float plan_angle, float *fire_ratio);

// 一次算两轴并缓存结果，供 plotter 读取。由 motor_task 每 1 ms 调用。
// active=false 时不累积并清零：对齐阶段目标是"当前朝向"，误差恒为 0，
//   若让它参与统计，命中率会被抬到接近 1（实测出现过 0.9 的虚高），掩盖真实跟随水平。
// gimbal_* 传 IMU 实际姿态，plan_* 传云台真正在追的目标（世界系，rad）。
void score_detect_update(
    bool active, float gimbal_yaw, float plan_yaw, float gimbal_pitch, float plan_pitch);

// 最近一次算出的命中率，0~1。
extern volatile float g_yaw_fire_ratio;
extern volatile float g_pitch_fire_ratio;

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_SCORE_DETECT_HPP
