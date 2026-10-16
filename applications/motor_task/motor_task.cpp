#include "cmsis_os.h"

#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_task.hpp"
#include "task.h"
#include "tools/pid/pid.hpp"
#include "trajectory_task/trajectory_task.hpp"

// 电机对象与反馈回调仅供本任务使用。
namespace
{
// 当前配置：两台 GM6020 和一台 C610 驱动的 M2006。
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);
// M2006 带 36:1 减速箱，传入减速比后角度、转速、指令均为输出轴侧单位。
sp::RM_Motor motor_c(3, sp::RM_Motors::M2006, sp::M2006_P36);

// 按反馈帧 ID 更新对应电机状态；RM 电机反馈使用标准帧。
void process_motor_feedback(std::uint32_t id, const std::uint8_t * data, bool extended)
{
  if (extended) return;

  const std::uint32_t stamp_ms = static_cast<std::uint32_t>(osKernelSysTick());
  if (id == motor_a.rx_id) motor_a.read(data, stamp_ms);
  if (id == motor_b.rx_id) motor_b.read(data, stamp_ms);
  if (id == motor_c.rx_id) motor_c.read(data, stamp_ms);
}

// 电机速度环：set 为指定转速, fdb 为该电机自身的反馈转速(输出轴侧, rad/s), 输出为转矩指令。
// 空载标定用, 不依赖 IMU 与机械安装, 正负号由电机自身定义。
constexpr float k_pid_dt = 1e-3f;            // 控制周期 1ms, 与任务周期一致
constexpr float k_max_torque =0.7f;         // 参考值(空载): 约 0.135A, 电机空载电流 78mA 约 0.058N·m
constexpr float k_max_torque_m2006 = 0.5f;   // C610+M2006: 峰值 10A≈1.8N·m, 空载标定先用 0.5N·m
constexpr float k_max_integral = 0.0f;       // 参考值: 积分关闭; 需消静差时与 ki 同时赋值
constexpr float k_d_filter_alpha = 0.3f;     // D 项一阶低通, 抑制转速噪声

// 增益取参考值 kp = 0.3, ki = kd = 0 (纯 P)。纯 P 必有稳态误差, 约等于 所需转矩/kp:
// 空载所需约 0.06N·m -> 约 0.2rad/s(目标 120rpm = 12.57rad/s 时约 1.6%)。
constexpr float k_motor_a_kp = 0.30f, k_motor_a_ki = 0.00f, k_motor_a_kd = 0.00f;
constexpr float k_motor_b_kp = 0.30f, k_motor_b_ki = 0.00f, k_motor_b_kd = 0.00f;
constexpr float k_motor_c_kp = 0.30f, k_motor_c_ki = 0.00f, k_motor_c_kd = 0.00f;

sp::PID motor_a_speed_pid(
  k_pid_dt, k_motor_a_kp, k_motor_a_ki, k_motor_a_kd, k_max_torque, k_max_integral,
  k_d_filter_alpha);
sp::PID motor_b_speed_pid(
  k_pid_dt, k_motor_b_kp, k_motor_b_ki, k_motor_b_kd, k_max_torque, k_max_integral,
  k_d_filter_alpha);
sp::PID motor_c_speed_pid(
  k_pid_dt, k_motor_c_kp, k_motor_c_ki, k_motor_c_kd, k_max_torque_m2006, k_max_integral,
  k_d_filter_alpha);

// 单个电机的位置/速度环路配置。
// 位置环(外环): 角度误差 -> 目标转速, 输出再由速度环(内环)换算成转矩, 构成双环。
constexpr float k_angle_max_speed = 1.0f;    // 外环输出的目标转速上限(rad/s, 约 57°/s)
constexpr float k_angle_max_iout = 0.5f;     // 外环积分上限(rad/s)
// 限速必须与转矩上限匹配: 0.1N·m 刹不住高速轴(重轴过冲), 故先取 1rad/s。
// 提高 k_max_torque 后可同步提高本值以加快转动。
constexpr float k_angle_kp = 6.0f;           // 单位 1/s: 角度误差 0.1rad -> 目标转速 0.6rad/s
constexpr float k_angle_ki = 2.0f;           // 消除角度静差: 靠它积累顶住摩擦所需的目标转速
constexpr float k_angle_kd = 0.30f;          // 角度微分(阻尼), 抑制过冲

sp::PID motor_a_angle_pid(
  k_pid_dt, k_angle_kp, k_angle_ki, k_angle_kd, k_angle_max_speed, k_angle_max_iout,
  k_d_filter_alpha);
sp::PID motor_b_angle_pid(
  k_pid_dt, k_angle_kp, k_angle_ki, k_angle_kd, k_angle_max_speed, k_angle_max_iout,
  k_d_filter_alpha);
sp::PID motor_c_angle_pid(
  k_pid_dt, k_angle_kp, k_angle_ki, k_angle_kd, k_angle_max_speed, k_angle_max_iout,
  k_d_filter_alpha);

// 单个电机的环路: 角度模式跑双环, 速度模式只跑速度环; 反馈超时或模式切换则复位。
void run_motor_loop(
  sp::RM_Motor & motor, sp::PID & angle_pid, sp::PID & speed_pid, float angle_set, float speed_set,
  uint8_t mode, volatile float & cmd, uint32_t now_ms, uint8_t & last_mode)
{
  if (!motor.is_alive(now_ms)) {
    angle_pid.clear();
    speed_pid.clear();
    cmd = 0.0f;
    return;
  }

  if (mode != last_mode) {
    angle_pid.clear();
    speed_pid.clear();
    last_mode = mode;
  }

  if (mode == MOTOR_MODE_ANGLE) {
    angle_pid.calc(angle_set, motor.angle);
    speed_pid.calc(angle_pid.out, motor.speed);
  }
  else {
    speed_pid.calc(speed_set, motor.speed);
  }
  cmd = speed_pid.out;
}
}

// 提供给其他模块读写的电机指令、角度、转速、转矩、温度和在线状态。
volatile float g_motor_a_cmd = 0.0f;
volatile float g_motor_b_cmd = 0.0f;
volatile float g_motor_c_cmd = 0.0f;
// 速度环使能：置 1 后由环路接管三个电机的指令；初值 0，上电不输出转矩。
volatile bool g_motor_pid_enable = false;
// 速度环目标转速(rad/s, 输出轴侧)：速度模式下生效；初值 0 表示保持静止。
volatile float g_motor_a_speed_set = 0.0f;
volatile float g_motor_b_speed_set = 0.0f;
volatile float g_motor_c_speed_set = 0.0f;
// 控制模式(取值见 MotorControlMode)：角度模式下跑双环，转到并保持在目标角度。
volatile uint8_t g_motor_a_mode = MOTOR_MODE_SPEED;
volatile uint8_t g_motor_b_mode = MOTOR_MODE_SPEED;
volatile uint8_t g_motor_c_mode = MOTOR_MODE_SPEED;
// 位置环目标角度(rad, 输出轴侧)：角度模式下生效；初值 0 表示回到编码器零点。
volatile float g_motor_a_angle_set = 0.0f;
volatile float g_motor_b_angle_set = 0.0f;
volatile float g_motor_c_angle_set = 0.0f;

volatile float g_motor_a_angle = 0.0f;
volatile float g_motor_b_angle = 0.0f;
volatile float g_motor_c_angle = 0.0f;

volatile float g_motor_a_speed = 0.0f;
volatile float g_motor_b_speed = 0.0f;
volatile float g_motor_c_speed = 0.0f;

volatile float g_motor_a_torque = 0.0f;
volatile float g_motor_b_torque = 0.0f;
volatile float g_motor_c_torque = 0.0f;

volatile uint8_t g_motor_a_temperature = 0U;
volatile uint8_t g_motor_b_temperature = 0U;
volatile uint8_t g_motor_c_temperature = 0U;

volatile bool g_motor_a_online = false;
volatile bool g_motor_b_online = false;
volatile bool g_motor_c_online = false;

// 轨迹快照：每周期由 trajectory_read() 取一组自洽的轨迹值（内部有临界区保护），
// 口径为 world(IMU) 系，单位 rad / rad/s / rad/s²。
// 目前仅用于观察与调试，尚未作为 PID 的目标值；当目标值用时再从这里取。
TrajectoryPoint g_motor_traj_snapshot;

// 电机任务：注册反馈处理、运行速度/位置环路、周期发送控制帧并刷新对外状态。
extern "C" void motor_task(void const * argument)
{
  (void)argument;
  sp::can1.set_rx_callback(process_motor_feedback);

  bool pid_was_enabled = false;
  uint8_t last_mode_a = MOTOR_MODE_SPEED;
  uint8_t last_mode_b = MOTOR_MODE_SPEED;
  uint8_t last_mode_c = MOTOR_MODE_SPEED;
  TickType_t wake_tick = xTaskGetTickCount();

  while (true) {
    const std::uint32_t now_ms = static_cast<std::uint32_t>(osKernelSysTick());

    // 每周期读一次轨迹快照, 便于观察/后续取用（当前不影响电机行为）。
    g_motor_traj_snapshot = trajectory_read();

    // 环路: 每 1 ms 计算一次, 使能时接管指令。
    if (g_motor_pid_enable) {
      run_motor_loop(
        motor_a, motor_a_angle_pid, motor_a_speed_pid, g_motor_a_angle_set, g_motor_a_speed_set,
        g_motor_a_mode, g_motor_a_cmd, now_ms, last_mode_a);
      run_motor_loop(
        motor_b, motor_b_angle_pid, motor_b_speed_pid, g_motor_b_angle_set, g_motor_b_speed_set,
        g_motor_b_mode, g_motor_b_cmd, now_ms, last_mode_b);
      run_motor_loop(
        motor_c, motor_c_angle_pid, motor_c_speed_pid, g_motor_c_angle_set, g_motor_c_speed_set,
        g_motor_c_mode, g_motor_c_cmd, now_ms, last_mode_c);
      pid_was_enabled = true;
    }
    else if (pid_was_enabled) {
      // 关闭瞬间清零并复位 PID, 避免残留转矩。
      g_motor_a_cmd = 0.0f;
      g_motor_b_cmd = 0.0f;
      g_motor_c_cmd = 0.0f;
      motor_a_angle_pid.clear();
      motor_b_angle_pid.clear();
      motor_c_angle_pid.clear();
      motor_a_speed_pid.clear();
      motor_b_speed_pid.clear();
      motor_c_speed_pid.clear();
      pid_was_enabled = false;
    }

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

    // 同步反馈在线状态及角度、转速、转矩、温度数据。
    g_motor_a_online = motor_a.is_alive(now_ms);
    g_motor_b_online = motor_b.is_alive(now_ms);
    g_motor_c_online = motor_c.is_alive(now_ms);
    g_motor_a_angle = motor_a.angle;
    g_motor_b_angle = motor_b.angle;
    g_motor_c_angle = motor_c.angle;
    g_motor_a_speed = motor_a.speed;
    g_motor_b_speed = motor_b.speed;
    g_motor_c_speed = motor_c.speed;
    g_motor_a_torque = motor_a.torque;
    g_motor_b_torque = motor_b.torque;
    g_motor_c_torque = motor_c.torque;
    g_motor_a_temperature = motor_a.temperature;
    g_motor_b_temperature = motor_b.temperature;
    g_motor_c_temperature = motor_c.temperature;

    // 固定 1 ms 周期, 保证 PID 的 dt 与 CAN 控制帧的节拍稳定。
    vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(1));
  }
}