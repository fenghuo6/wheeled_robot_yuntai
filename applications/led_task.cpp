#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task()
{
  led.start();

  while (true) {
    // 红
    led.set(1.0f, 0.0f, 0.0f);
    osDelay(200);

    // 绿
    led.set(0.0f, 1.0f, 0.0f);
    osDelay(200);

    // 蓝
    led.set(0.0f, 0.0f, 1.0f);
    osDelay(200);
  }
}