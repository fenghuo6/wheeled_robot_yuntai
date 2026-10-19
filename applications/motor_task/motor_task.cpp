#include "cmsis_os.h"

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
// 角色：a(ID1)=yaw，b(ID2)=折叠，c(ID3)=pitch。
// a/b 为 GM6020（直驱、绝对编码器），c 为 C610 驱动的 M2006。
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);
// M2006 带 36:1 减速箱，传入减速比后角度、转速、指令均为输出轴侧单位。
sp::RM_Motor motor_c(3, sp::RM_Motors::M2006, sp::M2006_P36);

// ─────────────────────── 2. 反馈解析（CAN 回调） ───────────────────────
// 按反馈帧 ID 更新对应电机状态；RM 电机反馈使用标准帧。
void process_motor_feedback(std::uint32_t id, const std::uint8_t * data, bool extended)
{
  if (extended) return;

  const std::uint32_t stamp_ms = static_cast<std::uint32_t>(osKernelSysTick());
  if (id == motor_a.rx_id) motor_a.read(data, stamp_ms);
  if (id == motor_b.rx_id) motor_b.read(data, stamp_ms);
  if (id == motor_c.rx_id) motor_c.read(data, stamp_ms);
}

// ─────────────────────────── 3. 环路参数 ───────────────────────────
// 角度环：角度误差经 PID 直接换算成转矩，单环控制，没有速度内环。
// 输出量纲为 N·m，限幅即该电机的转矩上限。
constexpr float k_pid_dt = 1e-3f;           // 控制周期 1ms, 与任务周期一致
constexpr float k_d_filter_alpha = 0.3f;    // D 项一阶低通, 抑制转速噪声

// ---- 3.1 yaw(a) / pitch(c) 角度环 ----
constexpr float k_max_torque = 0.7f;        // GM6020 转矩上限(参考值): 约 0.135A
constexpr float k_max_torque_m2006 = 0.5f;  // C610+M2006 转矩上限: 峰值 10A≈1.8N·m
// 增益由原双环等效换算: 外环 kp=6.0/ki=2.0 乘以内环 kp=0.3 -> kp=1.8/ki=0.6;
// 原内环对转速的比例作用(0.3N·m 每 rad/s)等效为本环的微分项 kd=0.3。
// 单位: kp 为 N·m/rad, ki 为 N·m/(rad·s), kd 为 N·m/(rad/s)。
constexpr float k_angle_kp = 1.8f;          // 角度误差 0.1rad -> 0.18N·m
constexpr float k_angle_ki = 0.6f;          // 消除角度静差, 顶住摩擦与重力
constexpr float k_angle_kd = 0.30f;         // 转速阻尼, 抑制过冲
constexpr float k_angle_max_iout = 0.15f;   // 积分上限(N·m), 等效原内环 0.5rad/s 的输出

// ---- 3.2 折叠轴(b)角度环 ----
// 默认目标是"上电锁位"：首次收到有效反馈时把当时的角度写进 g_motor_b_angle_set，
// 之后一直按该目标闭环，靠积分项顶住重力矩，云台不会因为没出力而倒下。
// 控制层可以用 motor_angle(MOTOR_B, deg) 改写这个目标，驱动折叠/展开来躲避上方障碍。
// 需要比 yaw/pitch 更大的力矩和积分上限，否则大负载时会被重力慢慢压下去。
constexpr float k_fold_kp = 3.0f;           // N·m/rad
constexpr float k_fold_ki = 2.0f;           // N·m/(rad·s)
constexpr float k_fold_kd = 0.30f;          // N·m/(rad/s)
constexpr float k_fold_max_torque = 1.2f;   // GM6020 额定约 2.2N·m, 取一半足够顶住且留余量
constexpr float k_fold_max_iout = 1.2f;     // 积分上限取满, 否则重力矩大时压不住
// 积分分离阈值(N·m): P 项超过它时暂停积分。
// 折叠是大角度阶跃, 不限住积分会在到位前攒满, 到目标后表现为过冲和来回摆。
constexpr float k_fold_integral_pause = 0.4f;

// ---- 3.3 轨迹跟随的方向 ----
// 电机输出正转矩就使该角度读数增大则为 +1.0f，否则 -1.0f。
// 实测发现某个轴越纠越偏（正反馈发散）时，把对应符号取反。
constexpr float k_imu_dir_yaw = 1.0f;
constexpr float k_imu_dir_pitch = 1.0f;

// ─────────────────────────── 4. PID 对象 ───────────────────────────
sp::PID motor_a_angle_pid(
  k_pid_dt, k_angle_kp, k_angle_ki, k_angle_kd, k_max_torque, k_angle_max_iout, k_d_filter_alpha);
sp::PID motor_c_angle_pid(
  k_pid_dt, k_angle_kp, k_angle_ki, k_angle_kd, k_max_torque_m2006, k_angle_max_iout,
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
void latch_zero(AxisZero & zero, const sp::RM_Motor & motor, uint32_t now_ms)
{
  if (zero.latched || !motor.is_alive(now_ms)) return;
  zero.offset = motor.angle;
  zero.latched = true;
}

AxisZero zero_a;
AxisZero zero_b;
AxisZero zero_c;

// ───────────────────────── 6. 单个电机的环路 ─────────────────────────
// 角度误差直接出转矩; 反馈超时则复位并停止输出。
// sign 用于对齐"正转矩使反馈角增大"的方向, 不一致时取 -1.0f。
// integral_pause > 0 时启用积分分离（P 项超过该值就暂停积分），用于大角度阶跃的折叠轴。
void run_motor_loop(
  sp::RM_Motor & motor, sp::PID & angle_pid, float angle_set, float angle_fdb, float sign,
  float integral_pause, volatile float & cmd, uint32_t now_ms)
{
  if (!motor.is_alive(now_ms)) {
    angle_pid.clear();
    cmd = 0.0f;
    return;
  }

  if (integral_pause > 0.0f) {
    angle_pid.calc(angle_set, angle_fdb, integral_pause);
  }
  else {
    angle_pid.calc(angle_set, angle_fdb);
  }
  cmd = sign * angle_pid.out;
}
}  // namespace

// ============================================================================
// 对外状态与指令：其他模块只通过下面这些全局量与电机打交道。
// ============================================================================
// ─────────────────────────── 1. 指令与使能 ───────────────────────────
// 转矩指令(N·m)。使能时由角度环写入，此时不要再从外部写。
volatile float g_motor_a_cmd = 0.0f;
volatile float g_motor_b_cmd = 0.0f;
volatile float g_motor_c_cmd = 0.0f;
// 角度环使能：置 1 后由角度环接管三个电机的指令；初值 0，上电不输出转矩。
volatile bool g_motor_pid_enable = false;
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
extern "C" void motor_task(void const * argument)
{
  (void)argument;

  // ---- 初始化：注册 CAN 反馈回调 ----
  sp::can1.set_rx_callback(process_motor_feedback);

  bool pid_was_enabled = false;
  TickType_t wake_tick = xTaskGetTickCount();

  while (true) {
    const std::uint32_t now_ms = static_cast<std::uint32_t>(osKernelSysTick());

    // ---- ① 轨迹快照：仅供观察，不影响电机行为 ----
    g_motor_traj_snapshot = trajectory_read();

    // ---- ② 锁存使能零点 ----
    // 第一次拿到有效反馈时记录当时的编码器角度作为原点，之后目标都是相对它。
    // 编码器上电读数即为机械绝对角，因此无需额外的回零动作。
    if (g_motor_pid_enable) {
      latch_zero(zero_a, motor_a, now_ms);
      latch_zero(zero_b, motor_b, now_ms);
      latch_zero(zero_c, motor_c, now_ms);
    }

    // ---- ③ 角度环：每 1 ms 计算一次，使能时接管指令 ----
    if (g_motor_pid_enable) {
      const bool imu_ok =
        static_cast<std::uint32_t>(now_ms - g_imu_update_ms) < IMU_STALE_TIMEOUT_MS;

      if (g_gimbal_traj_enable) {
        // 轨迹跟随: 目标在世界系, 反馈取 IMU 姿态, 与编码器零点无关。
        // IMU 数据过期时反馈会一直停在旧值, 误差不再变化, 环路会持续死推, 必须停止输出。
        if (imu_ok) {
          run_motor_loop(
            motor_a, motor_a_angle_pid, g_gimbal_yaw_set, g_yaw_rad, k_imu_dir_yaw, 0.0f,
            g_motor_a_cmd, now_ms);
          run_motor_loop(
            motor_c, motor_c_angle_pid, g_gimbal_pitch_set, g_pitch_rad, k_imu_dir_pitch, 0.0f,
            g_motor_c_cmd, now_ms);
        }
        else {
          motor_a_angle_pid.clear();
          motor_c_angle_pid.clear();
          g_motor_a_cmd = 0.0f;
          g_motor_c_cmd = 0.0f;
        }
      }
      else {
        // 角度模式: 目标相对使能零点, 默认 0 即保持不动。
        run_motor_loop(
          motor_a, motor_a_angle_pid, zero_a.offset + g_motor_a_angle_set, motor_a.angle, 1.0f,
          0.0f, g_motor_a_cmd, now_ms);
        run_motor_loop(
          motor_c, motor_c_angle_pid, zero_c.offset + g_motor_c_angle_set, motor_c.angle, 1.0f,
          0.0f, g_motor_c_cmd, now_ms);
      }

      // 折叠轴与上面两种模式无关: 始终以使能零点为目标把云台顶在原位, 不让它倒下。
      // 折叠是大角度阶跃, 用积分分离避免到位过冲。
      run_motor_loop(
        motor_b, motor_b_angle_pid, zero_b.offset + g_motor_b_angle_set, motor_b.angle, 1.0f,
        k_fold_integral_pause, g_motor_b_cmd, now_ms);
      pid_was_enabled = true;
    }
    else if (pid_was_enabled) {
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