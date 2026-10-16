#include "laser_task.hpp"
#include <cstdint>
#include "cmsis_os.h"
#include "usart.h"

// GY-56 UART 协议参数及接收状态。
// 模块须工作在串口模式（PS 焊点开放）且波特率与 USART6 一致（9600，即模块出厂默认）。
// 如需改用 115200，模块侧发送 0xA5 0x76 0x01 0x1C（0xA5 0x87 0x01 0x2D 掉电保存），
// 同时把 USART6、cboard.ioc 的波特率一并改掉，否则重新生成代码后会失配。
namespace
{
constexpr uint8_t kFrameHeader = 0x5AU;
constexpr uint8_t kDistanceFrameType = 0x15U;
constexpr uint8_t kHeaderLength = 0x02U;    // 0x5A 0x5A
constexpr uint8_t kPrefixLength = 0x04U;    // 帧头 2 字节 + 类型 1 字节 + 数据量 1 字节
constexpr uint8_t kChecksumLength = 0x01U;
constexpr uint8_t kMinFrameLength = 0x08U;  // 手册：每帧 8~13 字节
constexpr uint8_t kMaxFrameLength = 0x0DU;
constexpr uint32_t kRxBufferLength = 32U;
constexpr uint32_t kOnlineTimeoutMs = 500U;
constexpr uint32_t kTaskPeriodMs = 50U;

uint8_t rx_buffer[kRxBufferLength];
uint8_t frame[kMaxFrameLength];
uint8_t frame_size = 0U;
uint8_t frame_length = 0U;
volatile uint32_t last_valid_frame_ms = 0U;

// 逐字节同步 0x5A 0x5A 帧头，按 Byte3 给出的数据量确定帧长并校验，最后更新测距结果。
void parse_byte(uint8_t byte)
{
  if (frame_size < kHeaderLength) {
    if (byte == kFrameHeader) {
      frame[frame_size++] = byte;
    } else {
      frame_size = 0U;
    }
    return;
  }

  frame[frame_size++] = byte;

  if (frame_size < kPrefixLength) return;

  if (frame_size == kPrefixLength) {
    const uint16_t total_length =
      static_cast<uint16_t>(kPrefixLength) + frame[3] + kChecksumLength;
    if (total_length < kMinFrameLength || total_length > kMaxFrameLength) {
      frame_size = 0U;
      return;
    }
    frame_length = static_cast<uint8_t>(total_length);
    return;
  }

  if (frame_size < frame_length) return;

  const uint8_t length = frame_length;
  frame_size = 0U;
  frame_length = 0U;

  uint8_t checksum = 0U;
  for (uint8_t i = 0U; i < length - kChecksumLength; ++i) checksum += frame[i];
  if (checksum != frame[length - kChecksumLength]) {
    ++g_gy56_checksum_errors;
    return;
  }

  if (frame[2] != kDistanceFrameType) return;

  g_gy56_distance_cm = static_cast<uint16_t>(
    (static_cast<uint16_t>(frame[4]) << 8U) | frame[5]);
  g_gy56_mode = frame[6];
  if (frame[3] >= 0x04U) g_gy56_temperature_c = frame[7];
  last_valid_frame_ms = static_cast<uint32_t>(osKernelSysTick());
  ++g_gy56_valid_frames;
}
}

// 对外发布的测距结果、在线状态和通信诊断信息。
volatile uint16_t g_gy56_distance_cm = 0U;
volatile uint8_t g_gy56_mode = 0U;
volatile uint8_t g_gy56_temperature_c = 0U;
volatile uint8_t g_gy56_online = 0U;
volatile uint32_t g_gy56_valid_frames = 0U;
volatile uint32_t g_gy56_checksum_errors = 0U;
volatile uint8_t g_gy56_uart_error = HAL_OK;

// 启动 USART6 的 DMA 空闲接收。
void gy56_uart_start_receive(void)
{
  const HAL_StatusTypeDef status =
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, kRxBufferLength);
  g_gy56_uart_error = static_cast<uint8_t>(status);
  if (status == HAL_OK) __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
}

// 处理一批 DMA 收到的数据，并重新启动接收。
void gy56_uart_receive_event(uint16_t size)
{
  for (uint16_t i = 0U; i < size && i < kRxBufferLength; ++i) {
    parse_byte(rx_buffer[i]);
  }
  gy56_uart_start_receive();
}

// 记录 UART 错误，并尝试恢复接收。
void gy56_uart_error(void)
{
  g_gy56_uart_error = static_cast<uint8_t>(HAL_UART_GetError(&huart6));
  gy56_uart_start_receive();
}

// GY-56 任务入口：启动接收，并周期性更新传感器在线状态。
extern "C" void gy56_laser_task(void const * argument)
{
  (void)argument;
  gy56_uart_start_receive();

  for (;;) {
    const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());
    g_gy56_online =
      (g_gy56_valid_frames > 0U && now_ms - last_valid_frame_ms <= kOnlineTimeoutMs) ? 1U : 0U;
    osDelay(kTaskPeriodMs);
  }
}
