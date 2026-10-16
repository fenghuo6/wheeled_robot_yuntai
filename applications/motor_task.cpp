#include "cmsis_os.h"

#include <cstdint>
#include <cstring>

#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor_task.hpp"

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
}

// 提供给其他模块读写的电机指令、角度、转速、转矩、温度和在线状态。
volatile float g_motor_a_cmd = 0.0f;
volatile float g_motor_b_cmd = 0.0f;
volatile float g_motor_c_cmd = 0.0f;
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

// 电机任务：注册反馈处理、周期发送控制帧并刷新对外状态。
extern "C" void motor_task(void const * argument)
{
  (void)argument;
  sp::can1.set_rx_callback(process_motor_feedback);

  while (true) {
    // 取外部模块下发的指令；初值为 0，因此上电后电机保持不动。
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
    const std::uint32_t now_ms = static_cast<std::uint32_t>(osKernelSysTick());
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

    osDelay(1);
  }
}