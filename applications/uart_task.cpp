#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "laser_task.hpp"

sp::DBus remote(&huart3);

// 电机控制逻辑（motor_control_update）已移到 motor_task 里按 1 ms 调用：
// 挂在 10 ms 任务上会让目标值变成 10 ms 阶梯，追它的电机因此多出 0~10 ms 滞后。
// 本任务只负责启动 DBus 接收；UART 收数据走中断回调，不需要任务主体轮询。

extern "C" void uart_task()
{
  remote.request();

  while (true) {
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