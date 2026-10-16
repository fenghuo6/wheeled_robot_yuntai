#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "laser_task.hpp"

sp::DBus remote(&huart3);

// 电机的控制逻辑在 motor_control.cpp 里实现
extern "C" void motor_control_update(void);

extern "C" void uart_task()
{
  remote.request();

  while (true) {
    motor_control_update();
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