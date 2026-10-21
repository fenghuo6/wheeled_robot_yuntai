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
constexpr uint8_t k_frame_header = 0x5AU;         // 帧头标志，两字节均为该值
constexpr uint8_t k_distance_frame_type = 0x15U;  // Byte2 帧类型：距离数据帧
constexpr uint8_t k_header_length = 0x02U;        // 帧头长度（字节），内容为 0x5A 0x5A
constexpr uint8_t k_prefix_length = 0x04U;        // 帧头 2 字节 + 类型 1 字节 + 数据量 1 字节
constexpr uint8_t k_checksum_length = 0x01U;      // 帧尾校验和长度（字节）
constexpr uint8_t k_min_frame_length = 0x08U;     // 帧长下限（字节），手册：每帧 8~13 字节
constexpr uint8_t k_max_frame_length = 0x0DU;     // 帧长上限（字节）
constexpr uint32_t k_rx_buffer_length = 32U;      // DMA 接收缓冲区长度（字节）
constexpr uint32_t k_online_timeout_ms = 500U;    // 超过该时间未收到有效帧即判离线（ms）
constexpr uint32_t k_task_period_ms = 50U;        // 在线状态刷新周期（ms）

uint8_t rx_buffer[k_rx_buffer_length];       // DMA 写入的原始字节
uint8_t frame[k_max_frame_length];           // 逐字节组帧缓冲
uint8_t frame_size = 0U;                     // 组帧进度：已写入 frame 的字节数
uint8_t frame_length = 0U;                   // 当前帧总长，由 Byte3 算出，组帧过程中有效
uint32_t last_valid_frame_ms = 0U;           // 最近一帧通过校验的时刻（ms），用于判在线
bool has_valid_frame = false;                // 是否收到过有效帧

// 逐字节同步 0x5A 0x5A 帧头，按 Byte3 给出的数据量确定帧长并校验，最后更新测距结果。
void parse_byte(uint8_t byte)
{
  if (frame_size < k_header_length) {
    if (byte == k_frame_header) {
      frame[frame_size++] = byte;
    } else {
      frame_size = 0U;
    }
    return;
  }

  frame[frame_size++] = byte;

  if (frame_size < k_prefix_length) return;

  if (frame_size == k_prefix_length) {
    const uint16_t total_length =
      static_cast<uint16_t>(k_prefix_length) + frame[3] + k_checksum_length;
    if (total_length < k_min_frame_length || total_length > k_max_frame_length) {
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
  for (uint8_t i = 0U; i < length - k_checksum_length; ++i) checksum += frame[i];
  if (checksum != frame[length - k_checksum_length]) return;

  if (frame[2] != k_distance_frame_type) return;

  g_gy56_distance_cm = static_cast<uint16_t>(
    (static_cast<uint16_t>(frame[4]) << 8U) | frame[5]);
  g_gy56_mode = frame[6];
  if (frame[3] >= 0x04U) g_gy56_temperature_c = frame[7];
  last_valid_frame_ms = static_cast<uint32_t>(osKernelSysTick());
  has_valid_frame = true;
}
}

// 对外发布的测距结果与在线状态。
volatile uint16_t g_gy56_distance_cm = 0U;       // 最近一帧的距离（cm）
volatile uint8_t g_gy56_mode = 0U;               // 测量模式：0 长距离 / 1 快速 / 2 高精度 / 3 一般
volatile uint8_t g_gy56_temperature_c = 0U;      // 模块 MCU 温度（℃）
volatile uint8_t g_gy56_online = 0U;             // 1 表示 k_online_timeout_ms 内收到过有效帧

// 启动 USART6 的 DMA 空闲接收。
void gy56_uart_start_receive(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx_buffer, k_rx_buffer_length) == HAL_OK) {
    __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  }
}

// 处理一批 DMA 收到的数据，并重新启动接收。
void gy56_uart_receive_event(uint16_t size)
{
  for (uint16_t i = 0U; i < size && i < k_rx_buffer_length; ++i) {
    parse_byte(rx_buffer[i]);
  }
  gy56_uart_start_receive();
}

// UART 出错后重新启动接收，让通信自行恢复。
void gy56_uart_error(void) { gy56_uart_start_receive(); }

// GY-56 任务入口：启动接收，并周期性更新传感器在线状态。
extern "C" void gy56_laser_task(void const * argument)
{
  (void)argument;
  gy56_uart_start_receive();

  for (;;) {
    const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());
    g_gy56_online =
      (has_valid_frame && now_ms - last_valid_frame_ms <= k_online_timeout_ms) ? 1U : 0U;
    osDelay(k_task_period_ms);
  }
}
