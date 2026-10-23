#ifndef APPLICATIONS_LASER_TASK_HPP
#define APPLICATIONS_LASER_TASK_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint16_t g_gy56_distance_cm; // 距离（cm）：模块直接上报的整数厘米，1cm 分辨率
extern volatile uint8_t g_gy56_mode;         // 模式：1 短距(40~1300mm) / 2 中距(40~3000mm) / 3 长距(40~4000mm)
// 测距状态：0 可靠；1 环境光；2 信号弱；4 超量程；5 硬件故障；7 噪声；8 溢出；14 无效；
// 0xFF 表示尚未收到有效帧
extern volatile uint8_t g_gy56_range_status;
extern volatile uint8_t g_gy56_online; // 1: 在线（500ms 内有有效帧）；0: 离线

void gy56_laser_task(void const * argument);  // 任务入口：启动接收并周期刷新在线状态
void gy56_uart_start_receive(void);           // 启动/重启 USART6 的 DMA 空闲接收
void gy56_uart_receive_event(uint16_t size);  // UART 回调传入本批字节数，解析并重启接收
void gy56_uart_error(void);                   // UART 错误回调：重启接收

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_LASER_TASK_HPP
