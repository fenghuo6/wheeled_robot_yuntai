#include <cstdint>
#include "cmsis_os.h"
#include "motor_task.hpp"

// 电机控制层：只写 motor_task 暴露的"模式/目标/使能"全局量，
// 不直接碰 CAN 与 PID（那些都在 motor_task 内部）。
// 由 uart_task 每 10 ms 调用一次 motor_control_update()。
namespace
{
// 按电机编号取对应的全局量，三个电机共用下面同一套处理。
volatile uint8_t & mode_of(uint8_t motor)
{
  switch (motor) {
    case MOTOR_B:
      return g_motor_b_mode;
    case MOTOR_C:
      return g_motor_c_mode;
    default:
      return g_motor_a_mode;
  }
}

volatile float & speed_set_of(uint8_t motor)
{
  switch (motor) {
    case MOTOR_B:
      return g_motor_b_speed_set;
    case MOTOR_C:
      return g_motor_c_speed_set;
    default:
      return g_motor_a_speed_set;
  }
}

volatile float & angle_set_of(uint8_t motor)
{
  switch (motor) {
    case MOTOR_B:
      return g_motor_b_angle_set;
    case MOTOR_C:
      return g_motor_c_angle_set;
    default:
      return g_motor_a_angle_set;
  }
}

volatile float & angle_of(uint8_t motor)
{
  switch (motor) {
    case MOTOR_B:
      return g_motor_b_angle;
    case MOTOR_C:
      return g_motor_c_angle;
    default:
      return g_motor_a_angle;
  }
}

// 包装函数：把"先写目标、再切模式"的顺序固定下来，
// 否则切模式瞬间目标还是旧值，电机会朝旧目标抢动一下。
// 当前函数体里没有启用示例，所以这四个函数暂时没被调用，加了 unused 属性
// 避免 -Wall 报警；你启用任一示例后，该属性可以删掉。

// 单位换算：控制层对外一律用"度"，底层 rad 只在转换后写入 g_motor_*_set。
constexpr float k_deg_to_rad = 3.14159265f / 180.0f;
constexpr float k_rad_to_deg = 180.0f / 3.14159265f;

// 速度模式：目标转速 单位 度/秒（输出轴侧，正负表示方向），内部换算成 rad/s 下发。
__attribute__((unused)) void motor_speed(uint8_t motor, float deg_per_s)
{
  speed_set_of(motor) = deg_per_s * k_deg_to_rad;
  mode_of(motor) = MOTOR_MODE_SPEED;
}

// 角度模式：转到指定角度 单位 度 并保持（基准是电机编码器零点），内部换算成 rad 下发。
__attribute__((unused)) void motor_angle(uint8_t motor, float deg)
{
  angle_set_of(motor) = deg * k_deg_to_rad;
  mode_of(motor) = MOTOR_MODE_ANGLE;
}

// 角度模式：相对当前位置转 delta_deg 度（正为正向），内部换算成 rad 下发。
__attribute__((unused)) void motor_angle_add(uint8_t motor, float delta_deg)
{
  motor_angle(motor, angle_of(motor) * k_rad_to_deg + delta_deg);
}

// 总使能：true 时三个电机按各自模式输出转矩，false 时全部停止输出。
__attribute__((unused)) void motor_enable(bool enable) { g_motor_pid_enable = enable; }
}  // namespace

// ===== 控制逻辑：由 uart_task 每 10 ms 调用一次 =====
//
// 单位：下面这些函数的参数一律用"度"（角速度用 度/秒），内部自动换算成 rad 下发。
//
// 可用的控制函数（三个电机通用，第一个参数用 MOTOR_A / MOTOR_B / MOTOR_C）：
//   motor_speed(motor, deg_per_s)      速度模式：以 deg_per_s 度/秒 持续转动
//   motor_angle(motor, deg)            角度模式：转到绝对角度 deg 度并保持
//   motor_angle_add(motor, delta_deg)  角度模式：相对当前位置转 delta_deg 度
//   motor_enable(true / false)         总使能：false 时三个电机都停止输出转矩
//
// 三条必须记住的规则：
//   1. 改模式时目标必须一起给（上面三个函数已经帮你按正确顺序写好：先写目标再切模式）。
//      若手动分开写，务必先写 *_set 再写 *_mode，否则切模式瞬间目标还是旧值，电机会抢动。
//   2. 角度基准是电机编码器零点，即"上电时电机所在的位置"就是 0。
//      想相对现在转 90° 要用 motor_angle_add(motor, 90.0f)，不要用 motor_angle(motor, 90.0f)。
//   3. g_motor_pid_enable 初值是 false，所以上电默认不出力，必须调用 motor_enable(true)。
//
// 可读回的状态（用于判断与保护）：
//   g_motor_a_angle / _speed / _torque / _temperature / _online（b、c 同理）
//   注意：这些底层全局量仍是 rad、rad/s（不是度）；要看度数乘 k_rad_to_deg。
//   转矩 N·m、温度 ℃，均为输出轴侧。
extern "C" void motor_control_update(void)
{
    
    // 带安全检查的模板——先确认反馈正常再使能，异常就停
     if (!g_motor_a_online) {
       motor_enable(false);          // 反馈掉了（>100ms 无 CAN 帧），立即停
       return;
     }
     if (g_motor_a_temperature > 80U) {
       motor_enable(false);          // 过热保护
       return;
     }
     motor_angle(MOTOR_A, 90.0f);
     motor_enable(true);

    
  // ── 示例 1：最简——motor_a 以 180 度/秒 持续转，负值即反转
  //   motor_speed(MOTOR_A, 180.0f);
  //   motor_enable(true);

  // ── 示例 2：最简——motor_a 转到编码器零点 +90° 并停在那里保持
  //   motor_angle(MOTOR_A, 90.0f);
  //   motor_enable(true);

  // ── 示例 3：相对当前位置转 90°（每次调用累加，只调一次别放在循环里反复调）
  //   motor_angle_add(MOTOR_A, 90.0f);
  //   motor_enable(true);

  // ── 示例 4：停止（立即不再输出转矩；PID 内部状态由 motor_task 自动复位）
  //   motor_enable(false);

  // ── 示例 5：三个电机各自不同目标，同时运行
  //   motor_angle(MOTOR_A, 90.0f);        // a 停在 +90°
  //   motor_speed(MOTOR_B, 120.0f);       // b 以 120 度/秒 转
  //   motor_angle(MOTOR_C, -45.0f);       // c 停在 -45°
  //   motor_enable(true);

  // ── 示例 6：自动往复测试（不用遥控器）——每 2 s 在 0 与 +90° 之间切换
  //   static uint32_t last_toggle_ms = 0U;
  //   static bool at_zero = true;
  //   const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());
  //   if (now_ms - last_toggle_ms >= 2000U) {
  //     last_toggle_ms = now_ms;
  //     at_zero = !at_zero;
  //     motor_angle(MOTOR_A, at_zero ? 0.0f : 90.0f);
  //   }
  //   motor_enable(true);

  // ── 示例 7：角度正弦扫频（调外环 PID 用）——目标角在 ±45° 间按 0.25 Hz 摆动
  //   static const uint32_t t0_ms = static_cast<uint32_t>(osKernelSysTick());
  //   const float t_s = static_cast<float>(osKernelSysTick() - t0_ms) * 1e-3f;
  //   const float amp_deg = 45.0f;
  //   const float omega = 2.0f * 3.14159265f * 0.25f;
  //   motor_angle(MOTOR_A, amp_deg * std::sin(omega * t_s));   // 需 #include <cmath>
  //   motor_enable(true);

  // ── 示例 8：带安全检查的模板——先确认反馈正常再使能，异常就停
  //   if (!g_motor_a_online) {
  //     motor_enable(false);          // 反馈掉了（>100ms 无 CAN 帧），立即停
  //     return;
  //   }
  //   if (g_motor_a_temperature > 80U) {
  //     motor_enable(false);          // 过热保护
  //     return;
  //   }
  //   motor_angle(MOTOR_A, 90.0f);
  //   motor_enable(true);

  // ── 示例 9：上电自动跑一次（等三个电机反馈都正常后再启动，避免盲跑）
  //   static bool started = false;
  //   if (!started) {
  //     const uint32_t now_ms = static_cast<uint32_t>(osKernelSysTick());
  //     if (!g_motor_a_online || !g_motor_b_online || !g_motor_c_online) return;
  //     motor_angle(MOTOR_A, 90.0f);
  //     motor_enable(true);
  //     started = true;
  //   }

  // 当前不实现任何控制逻辑：不输出转矩，电机保持不动。
  // 想启用哪个示例，把它下面的注释符去掉即可（示例 7 需要 #include <cmath>）。
}