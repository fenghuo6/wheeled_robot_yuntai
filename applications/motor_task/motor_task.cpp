#include "cmsis_os.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "imu_task.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_task.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"
#include "trajectory_task/trajectory_task.hpp"

// ============================================================================
// CAN1 接收诊断量（只读，供 plotter 观察）
// ----------------------------------------------------------------------------
// 用来确认"反馈到底有没有进来、进来的 ID 是多少"：
//   g_can1_rx_frames  : 收到的标准帧总数（单调递增）
//   g_can1_last_rx_id : 最近一帧的 ID
//   g_can1_a_frames   : 与 motor_a.rx_id 匹配上的帧数
// 三个量一起看就能区分"没有反馈"和"反馈 ID 不匹配"。
// ============================================================================
volatile std::uint32_t g_can1_rx_frames = 0;
volatile std::uint32_t g_can1_last_rx_id = 0;
volatile std::uint32_t g_can1_a_frames = 0;
volatile std::uint32_t g_can1_b_frames = 0;
// CAN ID 扫描表：记录总线上出现过的不同反馈 ID 及其帧数（最多 4 个槽位）。
// slot 为 0 表示空槽；用来确认每个电机实际的反馈报文 ID 是多少。
volatile std::uint32_t g_can1_id_slot[4] = {0U, 0U, 0U, 0U};
volatile std::uint32_t g_can1_id_count[4] = {0U, 0U, 0U, 0U};

// ============================================================================
// 电机任务
// ----------------------------------------------------------------------------
// 职责：唯一直接操作 CAN 与 PID 的模块。
//   1. 接收 RM 电机反馈帧，维护各电机的角度/转速/转矩/温度/在线状态；
//   2. 每 1 ms 跑一次角度环，把目标角度换算成转矩指令；
//   3. 按控制帧打包下发（两台 GM6020 共用 0x1FE，M2006 用 0x200）。
// 数据来源：目标由 motor_control.cpp 写入全局量，反馈由 CAN 中断写入电机对象。
// 单位约定：角度 rad、转速 rad/s、转矩 N·m、温度 ℃，均为输出轴侧。
// ============================================================================
namespace
{
  // ─────────────────────────── 1. 电机对象 ───────────────────────────
  // 角色（已用"只让 b 动 20°"实测确认）：a(ID2)=底座 yaw，b(ID1)=折叠，c(ID3)=pitch。
  // a/b 为 GM6020（直驱、绝对编码器，共用控制帧 0x1FE 的槽 1/槽 0），
  // c 为 C610 驱动的 M2006（控制帧 0x200）。
  sp::RM_Motor motor_a(2, sp::RM_Motors::GM6020);
  sp::RM_Motor motor_b(1, sp::RM_Motors::GM6020);
  // M2006 带 36:1 减速箱，传入减速比后角度、转速、指令均为输出轴侧单位。
  sp::RM_Motor motor_c(3, sp::RM_Motors::M2006, sp::M2006_P36);

  // ─────────────────────── 2. 反馈解析（CAN 回调） ───────────────────────
  // 按反馈帧 ID 更新对应电机状态；RM 电机反馈使用标准帧。
  // 同时统计实际收到的 CAN ID，用于确认 GM6020 反馈报文 ID 是否与 rm_motor 的假设一致。
  void process_motor_feedback(std::uint32_t id, const std::uint8_t *data, bool extended)
  {
    if (extended)
      return;

    ++g_can1_rx_frames;
    g_can1_last_rx_id = id;

    // 扫描表：新 ID 占一个空槽，已知 ID 累加计数（中断上下文，只做简单线性查找）
    for (int i = 0; i < 4; ++i)
    {
      if (g_can1_id_slot[i] == id)
      {
        ++g_can1_id_count[i];
        break;
      }
      if (g_can1_id_slot[i] == 0U)
      {
        g_can1_id_slot[i] = id;
        ++g_can1_id_count[i];
        break;
      }
    }

    const std::uint32_t stamp_ms = static_cast<std::uint32_t>(osKernelSysTick());
    if (id == motor_a.rx_id)
    {
      ++g_can1_a_frames;
      motor_a.read(data, stamp_ms);
    }
    if (id == motor_b.rx_id)
    {
      ++g_can1_b_frames;
      motor_b.read(data, stamp_ms);
    }
    if (id == motor_c.rx_id)
      motor_c.read(data, stamp_ms);
  }

  // ─────────────────────────── 3. 环路参数 ───────────────────────────
  // 角度环：角度误差经 PID 直接换算成转矩，单环控制，没有速度内环。
  // 输出量纲为 N·m，限幅即该电机的转矩上限。
  constexpr float k_pid_dt = 1e-3f;        // 控制周期 1ms, 与任务周期一致
  constexpr float k_d_filter_alpha = 0.3f; // D 项一阶低通, 抑制转速噪声

  // ---- 3.1 yaw(a) 角度环：GM6020，直驱 ----
  // 换算关系(由 sp::RM_Motor 内部完成, 这里只用于估算参数):
  //   GM6020      : ±16384 对应 ±3A, 力矩常数 0.741N·m/A, 1N·m → 7370 计数, 上限 2.22N·m
  //   C610+M2006  : ±10000 对应 ±10A, 36:1 减速, 输出侧上限约 1.8N·m
  // 所以 kp 的物理含义很直接: 1° 误差出力 = kp * 0.01745 N·m。
  constexpr float k_max_torque = 2.0f;        // GM6020 硬件上限 2.22N·m(3A), 留余量取 2.0(≈2.7A)
  constexpr float k_max_torque_m2006 = 1.2f;  // M2006 上限约 1.8N·m, 取 66%
  //
  // 【调参记录】(yaw 轴, 已调完)
  //   第 1 步(纯 P): kp=1→振荡3次/静差13°(0.23N·m 摩擦); kp=2→6次/静差7°;
  //                  kp=4→过冲约200%、4~5次慢衰减 => 实用临界 Ku≈4, Tu≈0.25s
  //   第 2 步(D): PID 内部 kd 会放大角度量化噪声(1LSB/ms≈0.77rad/s), 加 kd 反而更抖;
  //               速度阻尼(damp)方向异常, 会发散 => 两者均置 0, 不用 D。
  //   第 3 步(I): 见下方 k_angle_ki 处说明。
  // 单位: kp 为 N·m/rad, ki 为 N·m/(rad·s), kd 为 N·m/(rad/s)。
  constexpr float k_angle_kp = 2.0f;        // 已定, 不再改
  // ki 原为 0.01 —— 那是"只看阶跃静差"时留下的值, 太小了: 0.01×0.1rad ≈ 0.001N·m/s,
  // 要 200s 才能积出 0.2N·m 顶摩擦, 等于没有积分。
  // 轨迹跟随实测暴露了后果: yaw 误差 ±5° 且峰值出现在极值处(死区特征),
  // 与"摩擦/kp = 0.23/2 = 6.6°"的推算吻合。按 pitch 的同一量级改成 5。
  constexpr float k_angle_ki = 5.0f;        // 消摩擦死区, 与 pitch 同量级
  constexpr float k_angle_kd = 0.0f;        // 保持 0: 角度差分的 D 噪声太大
  constexpr float k_angle_damp = 0.0f;      // 保持 0: 速度阻尼会发散(转速符号待确认)
  constexpr float k_angle_max_iout = 0.5f;  // 积分上限要能覆盖摩擦(0.2~0.3N·m)

  // ---- 3.2 pitch(c) 角度环：C610 + M2006，36:1 减速 ----
  // 不能沿用 yaw 的增益：减速箱带来更大的输出侧惯量与摩擦，且转速反馈按 36:1 缩小，
  // 动态特性完全不同，必须单独调。
  // 起始值与 yaw 相同，按阶跃响应逐步往下调（顺序：先 P，再 I）。
  // 【pitch 调参记录】
  //   kp=1: 卡在误差 2.5° 处不动、目标反向时又冲过头 14° → 阶梯状(净力不足)
  //   kp=3: 持续大幅振荡、振幅还在增长 → 已越过 Ku
  //   => Ku 在 1~3 之间, 且这条轴阻尼极低, 纯 P 怎么取都不好, 必须加阻尼。
  //   关键差别: M2006 有 36:1 减速箱, 输出侧角度分辨率是 GM6020 的 36 倍
  //   (0.0012°/LSB, 1kHz 下差分噪声仅 0.02rad/s), 所以这里 kd 可用 ——
  //   yaw 用不了 kd 正是因为 0.044°/LSB 会被放大成 0.77rad/s 的假转速。
  //   kd 的物理含义就是阻尼系数: dout = kd × 转速, 单位 N·m/(rad/s)。
  constexpr float k_pitch_kp = 2.0f;        // 落在 1~3 之间的已知可动点
  // kp=2 + kd=0.2 实测: 平滑斜坡跟随、无过冲、无振荡, 力矩干净;
  // 但到位后静差约 2°(高侧到 13°, 低侧到 1.5°)。原因是 kp×2° = 0.11N·m 被摩擦吃掉。
  // 所以需要 I 慢慢积出这 0.11N·m:
  //   增量/秒 = ki × 误差 = 5 × 0.035 ≈ 0.18N·m/s, 约 0.6s 补足, 短于 1s 半周期。
  //   max_iout 必须 ≥ 摩擦(≈0.11), 取 0.6 留余量; 它同时限制大阶跃时的积分卷绕。
  // 若出现"到位后来回蹭"(积分过冲), 只降 ki: 5 → 3 → 2, 不要动 kp/kd。
  constexpr float k_pitch_ki = 5.0f;        // 消静差
  constexpr float k_pitch_kd = 0.2f;        // 阻尼; 由 J≈0.0045kg·m² 估算临界值≈0.19
  constexpr float k_pitch_damp = 0.0f;      // 用 PID 内部 kd 即可, 不再用电机转速阻尼
  constexpr float k_pitch_max_iout = 1.0f;  // 必须打开, 否则积分被限成 0 等于没加;
                                            // 0.6 在 pitch 绝对 -10° 的姿态下不够(实测被卡满后留 1.94° 固定误差),
                                            // 抬到 1.0, 仍小于 max_out 1.2

  // ---- 3.2 折叠轴(b)角度环 ----
  // 默认目标是"上电锁位"：首次收到有效反馈时把当时的角度写进 g_motor_b_angle_set，
  // 之后一直按该目标闭环，靠积分项顶住重力矩，云台不会因为没出力而倒下。
  // 控制层可以用 motor_angle(MOTOR_B, deg) 改写这个目标，驱动折叠/展开来躲避上方障碍。
  // 需要比 yaw/pitch 更大的力矩和积分上限，否则大负载时会被重力慢慢压下去。
  constexpr float k_fold_kp = 1.0f;          // N·m/rad
  constexpr float k_fold_ki = 0.05f;          // N·m/(rad·s)
  // kd 置 0: 与 yaw 轴同一个问题 —— PID 的 D 项靠 1kHz 角度差分, 1 个编码器 LSB/ms
  // 就相当于 0.77rad/s, kd 会把量化噪声放大成转矩脉冲, 保持工况下表现为高频抖动。
  constexpr float k_fold_kd = 0.0f;
  constexpr float k_fold_max_torque = 2.0f;  // 顶重力用: GM6020 上限 2.22N·m, 取 2.0(≈2.7A)
  constexpr float k_fold_max_iout = 1.5f;    // 积分上限: 静差需要它长期顶住重力矩
  // 积分分离阈值(N·m): P 项超过它时暂停积分, 用于大角度阶跃("折叠/展开")避免超调。
  //   折叠/展开动作: 取 0.2 —— 有了重力前馈后, 稳态力矩由前馈承担, 积分只需补摩擦;
  //                  阈值取小一些可避免动作过程中积分悄悄攒满造成过冲。
  //   ⚠ 纯自锁(不动作、只顶住)时置 0, 否则下垂到一定角度后积分被冻住、误差消不掉。
  constexpr float k_fold_integral_pause = 0.2f;

  // ---- 3.3 轨迹跟随的方向 ----
  // 电机输出正转矩就使该角度读数增大则为 +1.0f，否则 -1.0f。
  // 实测发现某个轴越纠越偏（正反馈发散）时，把对应符号取反。
  constexpr float k_imu_dir_yaw = 1.0f;
  constexpr float k_imu_dir_pitch = 1.0f;

  // ─────────────────────────── 4. PID 对象 ───────────────────────────
  sp::PID motor_a_angle_pid(
      k_pid_dt, k_angle_kp, k_angle_ki, k_angle_kd, k_max_torque, k_angle_max_iout, k_d_filter_alpha);
  sp::PID motor_c_angle_pid(
      k_pid_dt, k_pitch_kp, k_pitch_ki, k_pitch_kd, k_max_torque_m2006, k_pitch_max_iout,
      k_d_filter_alpha);
  sp::PID motor_b_angle_pid(
      k_pid_dt, k_fold_kp, k_fold_ki, k_fold_kd, k_fold_max_torque, k_fold_max_iout, k_d_filter_alpha);

  // ─────────────────────── 5. 使能零点 ───────────────────────
  // 每次使能后，各轴以"第一次拿到有效反馈时的角度"为原点，之后所有目标都相对它计算。
  // 因此 g_motor_*_angle_set 默认 0 就表示原地保持，电机使能后不会乱动。
  // 关使能时清零，下次使能重新取零点。
  struct AxisZero
  {
    float offset = 0.0f;
    bool latched = false;
  };

  // 反馈有效时锁存零点，每个使能周期只锁一次。
  void latch_zero(AxisZero &zero, const sp::RM_Motor &motor, uint32_t now_ms)
  {
    if (zero.latched || !motor.is_alive(now_ms))
      return;
    zero.offset = motor.angle;
    zero.latched = true;
  }

  AxisZero zero_a;
  AxisZero zero_b;
  AxisZero zero_c;

  // ─────────────────────── 6. 转速阻尼 ───────────────────────
  // 为什么不用 PID 内部对角度做微分的 D 项:
  //   1 kHz 下相邻两帧角度差 1 个编码器 LSB(0.044°) 就相当于 0.77 rad/s 的"转速"，
  //   kd=0.3 时每个 LSB 会打出 0.23N·m 的转矩脉冲 —— 和摩擦(0.23N·m)同量级，
  //   根本压不住振荡，反而喂出一个高频极限环(实测加 kd 后振荡更大)。
  // 改用电机自己上报的转速(分辨率 1rpm≈0.105rad/s，比差分细 7 倍)做阻尼，
  // 再做一阶低通，噪声只剩 kd×0.105 量级。
  struct DampFilter
  {
    float speed = 0.0f;
  };

  constexpr float k_damp_alpha = 0.2f; // 转速低通系数, 越小越平滑

  float damp_speed(DampFilter &filter, float speed)
  {
    filter.speed = k_damp_alpha * speed + (1.0f - k_damp_alpha) * filter.speed;
    return filter.speed;
  }

  DampFilter damp_a;
  DampFilter damp_b;
  DampFilter damp_c;

  // ───────────────────────── 7. 单个电机的环路 ─────────────────────────
  // 角度误差直接出转矩; 反馈超时则复位并停止输出。
  // sign 用于对齐"正转矩使反馈角增大"的方向, 不一致时取 -1.0f。
  // integral_pause > 0 时启用积分分离（P 项超过该值就暂停积分），用于大角度阶跃的折叠轴。
  // damp_gain 为转速阻尼系数(N·m 每 rad/s), 0 表示只用 PID 内部的 D 项。
  void run_motor_loop(
      sp::RM_Motor &motor, sp::PID &angle_pid, float angle_set, float angle_fdb, float sign,
      float integral_pause, float damp_gain, DampFilter &damp, volatile float &cmd,
      uint32_t now_ms)
  {
    if (!motor.is_alive(now_ms))
    {
      angle_pid.clear();
      damp.speed = 0.0f;
      cmd = 0.0f;
      return;
    }

    if (integral_pause > 0.0f)
    {
      angle_pid.calc(angle_set, angle_fdb, integral_pause);
    }
    else
    {
      angle_pid.calc(angle_set, angle_fdb);
    }

    // 阻尼作用在电机自身坐标系里(与 sign / 传感器坐标系无关), 永远与转动方向相反。
    cmd = sign * angle_pid.out - damp_gain * damp_speed(damp, motor.speed);
  }
} // namespace

// ============================================================================
// 对外状态与指令：其他模块只通过下面这些全局量与电机打交道。
// ============================================================================
// ─────────────────────────── 1. 指令与使能 ───────────────────────────
// 转矩指令(N·m)。使能时由角度环写入，此时不要再从外部写。
volatile float g_motor_a_cmd = 0.0f;
volatile float g_motor_b_cmd = 0.0f;
volatile float g_motor_c_cmd = 0.0f;
// 折叠轴(b)角度环使能：置 1 后 b 自锁保持目标角；初值 0，上电不输出转矩。
volatile bool g_motor_pid_enable = false;
// a(yaw)/c(pitch) 角度环使能：默认 0 = 不输出转矩、可自由转动；
// 置 1 后该轴按当前模式跟踪目标（角度模式用编码器反馈，轨迹跟随用 IMU 反馈）。
// 与 b 的区别：b 恒随总使能自锁，a/c 必须单独打开才会出力。
volatile bool g_motor_a_loop_enable = false;
volatile bool g_motor_c_loop_enable = false;
// 轨迹跟随模式：1 时 a(yaw)/c(pitch) 用 IMU 世界系姿态作反馈；初值 0。
volatile bool g_gimbal_traj_enable = false;

// ───────────────────────────── 2. 目标值 ─────────────────────────────
// 轨迹跟随的世界系目标角(rad)。
volatile float g_gimbal_yaw_set = 0.0f;
volatile float g_gimbal_pitch_set = 0.0f;
// 电机编码器坐标系下的目标角度(rad, 输出轴侧)：轨迹跟随关闭时生效；
// b(折叠)的目标由本任务上电锁位写入，外部写它会被覆盖。
volatile float g_motor_a_angle_set = 0.0f;
volatile float g_motor_b_angle_set = 0.0f;
volatile float g_motor_c_angle_set = 0.0f;

// ───────────────────────────── 3. 反馈状态 ─────────────────────────────
// 角度 rad（输出轴侧，b/c 已按减速比换算）。
volatile float g_motor_a_angle = 0.0f;
volatile float g_motor_b_angle = 0.0f;
volatile float g_motor_c_angle = 0.0f;

// 转速 rad/s。
volatile float g_motor_a_speed = 0.0f;
volatile float g_motor_b_speed = 0.0f;
volatile float g_motor_c_speed = 0.0f;

// 实际转矩 N·m。
volatile float g_motor_a_torque = 0.0f;
volatile float g_motor_b_torque = 0.0f;
volatile float g_motor_c_torque = 0.0f;

// 温度 ℃。
volatile uint8_t g_motor_a_temperature = 0U;
volatile uint8_t g_motor_b_temperature = 0U;
volatile uint8_t g_motor_c_temperature = 0U;

// 相对本次使能位置的转角(rad)：编码器角度减去锁存的零点，与 g_motor_*_angle_set 同基准。
// 仅供观察/调参，环路本身不读它。
volatile float g_motor_a_angle_rel = 0.0f;
volatile float g_motor_b_angle_rel = 0.0f;
volatile float g_motor_c_angle_rel = 0.0f;

// 在线状态：超过 100 ms 没有反馈帧即为离线。
volatile bool g_motor_a_online = false;
volatile bool g_motor_b_online = false;
volatile bool g_motor_c_online = false;

// ─────────────────── 4.1 折叠轴重力前馈 ───────────────────
// 前馈系数: τ_ff(θ) = g_fold_ff_p·cosθ + g_fold_ff_q·sinθ，θ = motor_b.angle。
// 由实测标定得到(见 motor_control.cpp 的标定流程)：
//   A = m·g·r = 0.345 N·m, θ0 = 8.4763° = 0.14794 rad
//   P = A·cosθ0 = 0.3412,  Q = A·sinθ0 = 0.05085
// 负载变化后需要重新标定：让 b 依次停在若干角度，采稳态 PID 输出 τ 与该角度 θ，
// 对 τ = P·cosθ + Q·sinθ 做最小二乘重算这两个值（旧版标定流程见 git 历史）。
volatile float g_fold_ff_p = 0.3412f;
volatile float g_fold_ff_q = 0.05085f;
volatile float g_fold_cal_amp = 0.345f;
volatile float g_fold_cal_phase = 0.14794f;
volatile uint32_t g_fold_cal_state = 0U;
volatile float g_motor_b_pid_out = 0.0f;

// ─────────────────────────── 4. 轨迹快照 ───────────────────────────
// 每周期由 trajectory_read() 取一组自洽的轨迹值（内部有临界区保护），
// 口径为 world(IMU) 系，单位 rad / rad/s / rad/s²。
// 这里只用于观察；真正的目标值由 motor_control.cpp 自行调用 trajectory_read() 读取。
TrajectoryPoint g_motor_traj_snapshot;

// ============================================================================
// 任务主体：每 1 ms 一轮，顺序固定为
//   ① 取轨迹快照 → ② 锁存使能零点 → ③ 跑角度环 → ④ 打包下发 → ⑤ 回写状态 → ⑥ 定周期
// 顺序不能乱：④ 必须用到 ③ 刚算出的指令，⑤ 必须在 ④ 之后才能反映本轮下发的结果。
// ============================================================================
extern "C" void motor_task(void const *argument)
{
  (void)argument;

  // ---- 初始化：CAN1 收发 ----
  // bus-off 后自动恢复，否则一次掉线/干扰就会永久停掉 CAN（现象：突然不响应、重启才恢复）
  hcan1.Init.AutoBusOff = ENABLE;
  HAL_CAN_Init(&hcan1);

  sp::can1.config();
  sp::can1.start();
  sp::can1.set_rx_callback(process_motor_feedback);

  bool pid_was_enabled = false;
  TickType_t wake_tick = xTaskGetTickCount();

  while (true)
  {
    const std::uint32_t now_ms = static_cast<std::uint32_t>(osKernelSysTick());

    // ---- ① 轨迹快照：仅供观察，不影响电机行为 ----
    g_motor_traj_snapshot = trajectory_read();

    // ---- ② 锁存使能零点 ----
    // 第一次拿到有效反馈时记录当时的编码器角度作为原点，之后的相对角都相对它。
    // 编码器上电读数即为机械绝对角，因此无需额外的回零动作。
    // a/c 各自独立使能，所以除了总使能，也要在它们自己的角度环打开时锁零，
    // 否则目标角会以编码器绝对零点为基准，使能瞬间会直接冲向该绝对角度。
    if (g_motor_pid_enable || g_motor_a_loop_enable)
      latch_zero(zero_a, motor_a, now_ms);
    if (g_motor_pid_enable)
      latch_zero(zero_b, motor_b, now_ms);
    if (g_motor_pid_enable || g_motor_c_loop_enable)
      latch_zero(zero_c, motor_c, now_ms);

    // ---- ③ 角度环：每 1 ms 计算一次，使能时接管指令 ----
    if (g_motor_pid_enable)
    {
      const bool imu_ok =
          static_cast<std::uint32_t>(now_ms - g_imu_update_ms) < IMU_STALE_TIMEOUT_MS;

      // a(yaw)/c(pitch) 是否投入由各自的使能开关决定（默认关）。
      // 关掉时清指令 + 复位 PID，电机不输出转矩、可被外力自由转动；
      // 打开时按当前模式跟踪目标（角度模式用编码器反馈，轨迹跟随模式用 IMU 反馈）。
      if (g_gimbal_traj_enable)
      {
        // 轨迹跟随: 目标在世界系, 反馈取 IMU 姿态, 与编码器零点无关。
        // IMU 数据过期时反馈会一直停在旧值, 误差不再变化, 环路会持续死推, 必须停止输出。
        if (imu_ok && g_motor_a_loop_enable)
          run_motor_loop(
              motor_a, motor_a_angle_pid, g_gimbal_yaw_set, g_yaw_rad, k_imu_dir_yaw, 0.0f,
              k_angle_damp, damp_a, g_motor_a_cmd, now_ms);
        else
        {
          motor_a_angle_pid.clear();
          g_motor_a_cmd = 0.0f;
        }

        if (imu_ok && g_motor_c_loop_enable)
          run_motor_loop(
              motor_c, motor_c_angle_pid, g_gimbal_pitch_set, g_pitch_rad, k_imu_dir_pitch, 0.0f,
              k_pitch_damp, damp_c, g_motor_c_cmd, now_ms);
        else
        {
          motor_c_angle_pid.clear();
          g_motor_c_cmd = 0.0f;
        }
      }
      else
      {
        // 角度模式: 目标相对使能零点, 默认 0 即保持不动。
        if (g_motor_a_loop_enable)
          run_motor_loop(
              motor_a, motor_a_angle_pid, zero_a.offset + g_motor_a_angle_set, motor_a.angle, 1.0f,
              0.0f, k_angle_damp, damp_a, g_motor_a_cmd, now_ms);
        else
        {
          motor_a_angle_pid.clear();
          g_motor_a_cmd = 0.0f;
        }

        if (g_motor_c_loop_enable)
          run_motor_loop(
              motor_c, motor_c_angle_pid, zero_c.offset + g_motor_c_angle_set, motor_c.angle, 1.0f,
              0.0f, k_pitch_damp, damp_c, g_motor_c_cmd, now_ms);
        else
        {
          motor_c_angle_pid.clear();
          g_motor_c_cmd = 0.0f;
        }
      }

      // 折叠轴与上面两种模式无关: 始终以使能零点为目标把云台顶在原位, 不让它倒下。
      // 折叠是大角度阶跃, 用积分分离避免到位过冲。
      run_motor_loop(
          motor_b, motor_b_angle_pid, zero_b.offset + g_motor_b_angle_set, motor_b.angle, 1.0f,
          k_fold_integral_pause, 0.0f, damp_b, g_motor_b_cmd, now_ms);

      // 重力前馈: PID 只需处理残差, 前馈承担随角度变化的重力矩 τ(θ)=P·cosθ+Q·sinθ。
      // 电机离线时 run_motor_loop 已把指令清零, 此处不能再叠加前馈。
      g_motor_b_pid_out = g_motor_b_cmd;
      if (motor_b.is_alive(now_ms)) {
        g_motor_b_cmd +=
          g_fold_ff_p * std::cos(motor_b.angle) + g_fold_ff_q * std::sin(motor_b.angle);
      }
      pid_was_enabled = true;
    }
    else if (pid_was_enabled)
    {
      // 关闭瞬间清零并复位 PID, 避免残留转矩; 同时丢弃零点, 下次使能重新取。
      g_motor_a_cmd = 0.0f;
      g_motor_b_cmd = 0.0f;
      g_motor_c_cmd = 0.0f;
      motor_a_angle_pid.clear();
      motor_b_angle_pid.clear();
      motor_c_angle_pid.clear();
      zero_a = AxisZero{};
      zero_b = AxisZero{};
      zero_c = AxisZero{};
      pid_was_enabled = false;
    }

    // ---- ④ 打包下发 ----
    // 取当前指令(环路输出或外部模块写入); 初值为 0，因此上电后电机保持不动。
    motor_a.cmd(g_motor_a_cmd);
    motor_b.cmd(g_motor_b_cmd);
    motor_c.cmd(g_motor_c_cmd);

    // 两台 GM6020 共用一个控制帧。
    std::memset(sp::can1.tx_data, 0, sizeof(sp::can1.tx_data));
    motor_a.write(sp::can1.tx_data);
    motor_b.write(sp::can1.tx_data);
    sp::can1.send(motor_a.tx_id);

    // M2006 使用独立控制帧。
    std::memset(sp::can1.tx_data, 0, sizeof(sp::can1.tx_data));
    motor_c.write(sp::can1.tx_data);
    sp::can1.send(motor_c.tx_id);

    // ---- ⑤ 回写状态：在线状态、角度、转速、转矩、温度 ----
    g_motor_a_online = motor_a.is_alive(now_ms);
    g_motor_b_online = motor_b.is_alive(now_ms);
    g_motor_c_online = motor_c.is_alive(now_ms);
    g_motor_a_angle = motor_a.angle;
    g_motor_b_angle = motor_b.angle;
    g_motor_c_angle = motor_c.angle;
    // 相对使能位置的转角；清零后零点未锁存，直接报 0（避免把上一个使能周期的偏移带出来）。
    g_motor_a_angle_rel = zero_a.latched ? (motor_a.angle - zero_a.offset) : 0.0f;
    g_motor_b_angle_rel = zero_b.latched ? (motor_b.angle - zero_b.offset) : 0.0f;
    g_motor_c_angle_rel = zero_c.latched ? (motor_c.angle - zero_c.offset) : 0.0f;
    g_motor_a_speed = motor_a.speed;
    g_motor_b_speed = motor_b.speed;
    g_motor_c_speed = motor_c.speed;
    g_motor_a_torque = motor_a.torque;
    g_motor_b_torque = motor_b.torque;
    g_motor_c_torque = motor_c.torque;
    g_motor_a_temperature = motor_a.temperature;
    g_motor_b_temperature = motor_b.temperature;
    g_motor_c_temperature = motor_c.temperature;

    // ---- ⑥ 定周期：固定 1 ms，保证 PID 的 dt 与 CAN 控制帧的节拍稳定 ----
    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(1));
  }
}