#ifndef APPLICATIONS_LASER_TASK_HPP
#define APPLICATIONS_LASER_TASK_HPP

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint16_t g_gy56_distance_cm;
extern volatile uint8_t g_gy56_mode;
extern volatile uint8_t g_gy56_temperature_c;
extern volatile uint8_t g_gy56_online;
extern volatile uint32_t g_gy56_valid_frames;
extern volatile uint32_t g_gy56_checksum_errors;
extern volatile uint8_t g_gy56_uart_error;

void gy56_laser_task(void const * argument);
void gy56_uart_start_receive(void);
void gy56_uart_receive_event(uint16_t size);
void gy56_uart_error(void);

#ifdef __cplusplus
}
#endif

#endif  // APPLICATIONS_LASER_TASK_HPP
