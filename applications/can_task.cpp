#include "cmsis_os.h"

#include "io/can/can.hpp"

extern "C" void can_task(void const * argument)
{
  (void)argument;
  sp::can1.config();
  sp::can1.start();

  while (true) {
    osDelay(1);
  }
}