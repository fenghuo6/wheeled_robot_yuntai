#ifndef TRAJECTORY_TASK_HPP
#define TRAJECTORY_TASK_HPP

#include <stdint.h>

// 单位：角度 rad、速度 rad/s、加速度 rad/s^2。

// 第一版接入先把轨迹做得又慢又小，确认"跟得上、没滞后"之后再放大：
//   原参数 680ms + yaw45/pitch60 对应峰值 yaw 410°/s、pitch 476°/s，
//   远超实测能力(yaw≈60°/s, pitch≈60°/s)，会全程饱和。
//   现在: 5000ms + 20/20  → 峰值 yaw 25°/s、pitch 22°/s，行程 ±10°。
//   确认机械能到 ±10° 且跟踪良好后，再把两个 RANGE 改回 45/60、PERIOD 逐步减小。
#define TRAJECTORY_YAW_RANGE_DEG 20.0f
#define TRAJECTORY_PITCH_RANGE_DEG 20.0f

#define TRAJECTORY_SAMPLE_INTERVAL_MS 1U
// 第一版接入时先把轨迹放慢：原 680ms 一个周期时 yaw 峰值 410°/s、pitch 峰值 476°/s，
// 远超两条轴实测能力(yaw≈60°/s, pitch≈60°/s)，会全程饱和、跟踪严重滞后。
// 5000ms: 峰值 yaw 25.6°/s、pitch 22.2°/s，实测误差 yaw ±3°、pitch ±6°，平滑不发散。
// 【不要再往下放慢】试过 10000ms(峰值减半)：误差反而变大且出现周期性"蹭动"振荡 ——
// 低速时轴频繁落入静摩擦区，走走停停叠加积分卷绕。也就是说精度瓶颈不是带宽，是摩擦。
#define TRAJECTORY_PERIOD_MS 5000U
#define TRAJECTORY_YAW_MAX_ACCELERATION_RAD_S2 110.0f
#define TRAJECTORY_PITCH_MAX_ACCELERATION_RAD_S2 70.0f
#define TRAJECTORY_UPDATE_INTERVAL_MS 1U

#ifdef __cplusplus
extern "C" {
#endif

extern volatile float yaw;
extern volatile float yaw_vel;
extern volatile float yaw_acc;
extern volatile float pitch;
extern volatile float pitch_vel;
extern volatile float pitch_acc;

// 轨迹推进使能：0 时停在起点(t=0)并把起点值持续发布，1 时正常推进。
// 上电默认 0，即轨迹先不跑；由控制层在对齐完成后调用 trajectory_start() 放行。
extern volatile uint8_t g_trajectory_running;

typedef struct
{
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
} TrajectoryPoint;

void trajectory_task(void const * argument);

TrajectoryPoint trajectory_read(void);

TrajectoryPoint trajectory_sample_at_ms(uint32_t elapsed_ms);

// 从起点放行：相位归零后开始推进。
void trajectory_start(void);

// 停住并回到起点；之后 trajectory_read() 恒返回起点值。
void trajectory_pause(void);

#ifdef __cplusplus
}
#endif

#endif  // TRAJECTORY_TASK_HPP
