#include "cmsis_os.h"
#include "imu_task.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);

// 全局变量给其他任务读
volatile float g_yaw_rad = 0.0f;
volatile float g_pitch_rad = 0.0f;
volatile float g_roll_rad = 0.0f;
volatile uint32_t g_imu_update_ms = 0U;

extern "C" void imu_task()
{
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);

    g_yaw_rad = imu.yaw;
    g_pitch_rad = imu.pitch;
    g_roll_rad = imu.roll;
    g_imu_update_ms = static_cast<uint32_t>(osKernelSysTick());

    osDelay(1);
  }
}