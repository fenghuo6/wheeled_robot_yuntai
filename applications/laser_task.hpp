#ifndef APPLICATIONS_LASER_TASK_HPP
#define APPLICATIONS_LASER_TASK_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint16_t g_gy56_distance_cm;      // 距离（cm）
extern volatile uint8_t g_gy56_mode;              // 测量模式：0 长距离 / 1 快速 / 2 高精度 / 3 一般
extern volatile uint8_t g_gy56_temperature_c;     // 模块 MCU 温度（℃）
extern volatile uint8_t g_gy56_online;            // 1: 在线（500ms 内有有效帧）；0: 离线
extern volatile uint32_t g_gy56_valid_frames;     // 通过校验的帧累计数，通信诊断用
extern volatile uint32_t g_gy56_checksum_errors;  // 校验失败累计数，通信诊断用
extern volatile uint8_t g_gy56_uart_error;        // 最近 UART 状态/错误码

void gy56_laser_task(void const * argument);   // 任务入口：启动接收并周期刷新在线状态
void gy56_uart_start_receive(void);            // 启动/重启 USART6 的 DMA 空闲接收
void gy56_uart_receive_event(uint16_t size);   // UART 回调传入本批字节数，解析并重启接收
void gy56_uart_error(void);                    // UART 错误回调：记录错误码并重启接收

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_LASER_TASK_HPP
