#include "cmsis_os.h"
#include "can.h"
#include "io/can/can.hpp"
#include "motor_task/motor_task.hpp"

namespace
{
// 自环测试开关
// ----------------------------------------------------------------------------
// 打开后 CAN1 工作在"静默自环"模式：控制器发出的帧由它自己内部接收，
// 既不上总线，也不会驱动电机，用来把两类问题分开：
//   收到帧数 ≈ 20/10ms → 固件、中断、过滤器、接收链路都正常，问题在物理层
//   仍为 0              → 固件侧（中断/过滤器）有问题
// 测完必须改回 false，否则电机收不到任何指令。
constexpr bool k_loopback_selftest = false;

void init_can1()
{
  // 总线关闭后自动恢复：否则一次干扰/掉线就会永久停掉 CAN，
  // 之后电机再也收不到指令，现象是"突然不响应、重启才恢复"。
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.Mode = k_loopback_selftest ? CAN_MODE_SILENT_LOOPBACK : CAN_MODE_NORMAL;

  // MX_CAN1_Init() 已初始化过一次，这里只覆盖模式与自动恢复，
  // 重入时 HAL 不会再跑一次 MspInit（GPIO/NVIC 已配好）。
  HAL_CAN_Init(&hcan1);
}

// CAN2（PB12/PB13）已经由 MX_CAN2_Init() 初始化，但默认既没有启动、
// 也没有配置过滤器，所以插在 CAN2 口上的电机完全收不到。
// 这里补上过滤器（bank 14 起，CAN1 用 0~13）并启动；
// 接收用轮询，避免改动 sp_middleware 里只处理 hcan1 的中断回调。
void init_can2()
{
  // 同样打开自动恢复，避免 CAN2 因偶尔无应答进入 bus-off 后永久失效
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  HAL_CAN_Init(&hcan2);

  CAN_FilterTypeDef filter = {0};
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;    // 全通过
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterBank = 14;          // CAN2 的过滤器从 bank 14 开始
  filter.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan2, &filter);  // 返回值故意忽略，与 sp::CAN 一致
  HAL_CAN_Start(&hcan2);
}

// 把 CAN2 接收队列里的帧全部取出，交给统一的解析入口。
void poll_can2()
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
    motor_process_can_frame(header.StdId, data, header.IDE == CAN_ID_EXT);
  }
}
}  // namespace

extern "C" void can_task(void const * argument)
{
  (void)argument;

  init_can1();
  sp::can1.config();
  sp::can1.start();

  init_can2();

  while (true) {
    poll_can2();
    osDelay(1);
  }
}
