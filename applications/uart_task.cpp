#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "laser_task.hpp"
#include "motor_task/motor_task.hpp"

sp::DBus remote(&huart3);

// 电机控制逻辑（motor_control_update）已移到 motor_task 里按 1 ms 调用：
// 挂在 10 ms 任务上会让目标值变成 10 ms 阶梯，追它的电机因此多出 0~10 ms 滞后。
// 本任务负责：
//   1. 启动 DBus 接收（收数据走中断回调，任务主体只做 10 ms 一次的开关解析）；
//   2. 把遥控器右开关的档位翻译成云台指令：
//        右开关 UP   → g_chassis_follow_cmd = 1（测试档：立刻跟随 yaw+pitch）
//        右开关 MID  → g_chassis_auto_cmd   = 1（自动档：折叠完成后延时 3s 再跟随）
//        右开关 DOWN → g_chassis_disable_cmd= 1（失能，优先级最高）
// 三者都是电平量，档位切换后最迟 10 ms 生效；具体的边沿处理和延时在
// motor_control.cpp 的 ⓪ 里做，停在同一档不会反复重启对齐。

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());

    // 遥控器 → 云台指令（下档 = 失能，中档 = 自动跟随，上档 = 测试跟随）。
    // 只在接收机在线时采信开关：没插接收机/刚上电时不要把云台锁死在失能，
    // 真正需要"接收机掉线即失能"时把 g_cfg_chassis_link 设为 1 或 2。
    const bool rc_alive = remote.is_alive(now_ms);
    g_chassis_online = rc_alive;
    if (rc_alive) {
      g_chassis_heartbeat_seen = true;  // 自锁存：曾连上过接收机
      const bool sw_down = (remote.sw_r == sp::DBusSwitchMode::DOWN);
      const bool sw_up = (remote.sw_r == sp::DBusSwitchMode::UP);
      const bool sw_mid = (remote.sw_r == sp::DBusSwitchMode::MID);
      g_chassis_disable_cmd = sw_down;  // 下档：失能
      g_chassis_follow_cmd = sw_up;     // 上档：测试档，立刻跟随
      g_chassis_auto_cmd = sw_mid;      // 中档：自动档，折叠完成后延时跟随
    }

    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
  else if (huart == &huart6) {
    gy56_uart_receive_event(Size);
  }
}
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  else if (huart == &huart6) {
    gy56_uart_error();
  }
}