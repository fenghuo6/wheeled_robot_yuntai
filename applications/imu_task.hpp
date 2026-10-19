#ifndef APPLICATIONS_IMU_TASK_HPP
#define APPLICATIONS_IMU_TASK_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 云台 IMU 解算结果（世界系欧拉角），单位 rad，由 imu_task 每 1 ms 刷新。
extern volatile float g_yaw_rad;
extern volatile float g_pitch_rad;
extern volatile float g_roll_rad;

// 最近一次解算的时刻（ms）。闭环用 IMU 作反馈前必须先判它是否还在更新，
// 否则数据冻结时误差会一直不变，环路会把电机往错误方向死推。
extern volatile uint32_t g_imu_update_ms;

// 超过该时间没有更新就认为 IMU 数据不可信（ms）。
#define IMU_STALE_TIMEOUT_MS 50U

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_IMU_TASK_HPP
