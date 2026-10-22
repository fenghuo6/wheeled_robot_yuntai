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

// 陀螺零偏标定是否完成。
// 上电后累计 1 秒"静止"样本（三轴角速度都小于阈值）估计零偏，之后从测量中扣除，
// 并把当前的 yaw 重新冻结为零点 —— 标定期间云台静止，物理位置与上电时相同，
// 所以重新归零既正确，又顺带消掉了这段时间积累的漂移误差。
// 完成后才能保证 yaw 的长期漂移足够小（不标定时典型 0.1~0.5°/s，标定后 0.01~0.05°/s）。
extern volatile bool g_imu_bias_ready;

// 陀螺 Z 轴零偏估计值（rad/s），标定完成前为 0。
// 用途：验证标定是否真的采到了样本；同时可看出零偏的量级（不标定时典型 0.1~0.5°/s）。
extern volatile float g_imu_bias_z;

// 超过该时间没有更新就认为 IMU 数据不可信（ms）。
#define IMU_STALE_TIMEOUT_MS 50U

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_IMU_TASK_HPP
