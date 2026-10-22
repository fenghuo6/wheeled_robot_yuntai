#ifndef TRAJECTORY_TASK_HPP
#define TRAJECTORY_TASK_HPP

#include <stdint.h>

// 单位：角度 rad、速度 rad/s、加速度 rad/s^2。

// 行程按初始设计值：yaw ±22.5°、pitch ±30°。
// 【解耦实验结论】pitch 行程临时缩到 ±0.5°(只动 yaw)时，yaw RMS 由 4.3° 降到 3.2~3.5°:
//   pitch 的反作用力矩只占 yaw 误差的约 1°/23%, 其余 ~3.3° 是 yaw 自身的能力极限。
// yaw 的带宽核算: ωn = √(kp/J) = √(2/0.0063) ≈ 17.8rad/s ≈ 2.8Hz,
//   而 680ms 周期的基频 1.47Hz, 只有约 2 倍余量 => 跟踪误差必然大。
//   要把误差压到 0.5° 需 ωn 提高约 5 倍, 即 kp ≈ 50N·m/rad, 远超稳定性边界。
//   => 680ms/±22.5° 这个工况下 yaw 的命中率存在物理上限, 靠调参无法达标。
// ⚠ 调参临时状态: 只调 pitch, yaw 基本不摆动。
//   YAW_RANGE 取 0.02°(static_assert 要求 >0), 等效静止, 保持对齐后的位置不动。
//   yaw 调完之后把 YAW_RANGE 改回 45.0f。
#define TRAJECTORY_YAW_RANGE_DEG 0.02f
#define TRAJECTORY_PITCH_RANGE_DEG 60.0f

#define TRAJECTORY_SAMPLE_INTERVAL_MS 1U
// 【周期按初始设计值 680ms】
// 之前为了调参和测能力边界，曾把周期放到 5000/3000/2000ms。现在恢复初始值。
// 实测过的速度-精度关系（完整轨迹周期的误差 RMS，通道 16/17），供判断本次是否够用：
//   5000ms → yaw 25.6°/s, pitch 22.2°/s : yaw 1.58°, pitch 0.68°
//   3000ms → yaw 42.7°/s, pitch 37.0°/s : yaw 1.50°, pitch 0.90°   ← yaw 平坦段末尾
//   2000ms → yaw 64.0°/s, pitch 55.5°/s : yaw 2.30°, pitch 0.90°   ← yaw 开始饱和
// 结论: yaw 的拐点在 43~64°/s（与早期阶跃测试的"极限约 60°/s"吻合）；
//       pitch 到 55°/s 仍平坦，瓶颈是与速度无关的常数项(重力矩+减速箱摩擦)而非带宽。
// 注意: 不要再"放慢"到 10000ms —— 试过，误差反而变大且出现"蹭动"振荡
//       (低速时轴频繁落入静摩擦区，走走停停叠加积分卷绕)。瓶颈不是带宽，是摩擦。
#define TRAJECTORY_PERIOD_MS 680U
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
