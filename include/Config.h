/**
 * 系统配置文件
 * 集中管理所有常量和配置参数
 */

/*
腿高150 速度[P=0.31,I=0.000,D=0.005] 角度[P=1.2,I=0.000,D=0.005]
腿高100 速度[P=0.250,I=0.000,D=0.005] 角度[P=1.0,I=0.000,D=0.005]
腿高80  速度[P=0.20,I=0.000,D=0.005] 角度[P=0.7,I=0.000,D=0.005]
*/

#ifndef CONFIG_H
#define CONFIG_H

// ==================== 系统信息 ====================
#define SYSTEM_NAME "MiniWheel 轮腿机器人"
#define SYSTEM_VERSION "v2.1"

// ==================== 硬件引脚配置 ====================
// I2C 引脚 (主总线 - 用于IMU和第一个AS5600)
#define I2C_SDA 17
#define I2C_SCL 18
#define I2C_CLOCK_SPEED 400000  // 400kHz - 提高I2C速度减少通信延迟

// I2C1 引脚 (第二总线 - 用于第二个AS5600)
#define I2C1_SDA 9
#define I2C1_SCL 8
#define I2C1_CLOCK_SPEED 400000  // 100kHz

// 舵机引脚
#define SERVO1_PIN 12  // 左腿舵机1 (270度)
#define SERVO2_PIN 13  // 左腿舵机2 (180度)
#define SERVO3_PIN 15  // 右腿舵机1 (270度)
#define SERVO4_PIN 16  // 右腿舵机2 (180度)

// LED引脚
#define LED_PIN 48     // ESP32-S3板载WS2812 LED引脚
#define LED_COUNT 1    // LED数量

// BLDC电机驱动引脚
// 电机1 (左轮)
#define MOTOR1_PWM_A 4
#define MOTOR1_PWM_B 5
#define MOTOR1_PWM_C 6
#define MOTOR1_ENABLE 7

// 电机2 (右轮)
#define MOTOR2_PWM_A 35
#define MOTOR2_PWM_B 36
#define MOTOR2_PWM_C 37
#define MOTOR2_ENABLE 38

// ==================== IMU 配置 ====================
#define IMU_UPDATE_RATE_HZ 100  // 降低到100Hz (10ms间隔)
#define IMU_ADDRESS 0x68  // MPU6050默认I2C地址

// ==================== BLE 配置 ====================
#define BLE_DEVICE_NAME "MiniWheel-Robot"
#define BLE_SERVICE_UUID "00001000-0000-1000-8000-00805f9b34fb"
#define BLE_CHAR_UUID "00001001-0000-1000-8000-00805f9b34fb"
#define BLE_STATUS_CHAR1_UUID "00001002-0000-1000-8000-00805f9b34fb"  // 状态数据1（电机+平衡）
#define BLE_STATUS_CHAR2_UUID "00001003-0000-1000-8000-00805f9b34fb"  // 状态数据2（速度+姿态）

// ==================== FreeRTOS 配置 ====================
#define CMD_QUEUE_SIZE 10

// 任务优先级 (数字越大优先级越高)
#define TASK_PRIORITY_SERIAL 2
#define TASK_PRIORITY_MOTOR 3
#define TASK_PRIORITY_IMU 4  // 降低IMU任务优先级，避免过度抢占FOC任务

// 任务堆栈大小
#define TASK_STACK_SERIAL 4096
#define TASK_STACK_MOTOR 4096
#define TASK_STACK_IMU 3072

// CPU核心分配
#define TASK_CORE_SERIAL 0
#define TASK_CORE_MOTOR 1
#define TASK_CORE_IMU 0

// ==================== 机器人物理参数 ====================
// 轮腿初始位置
#define INITIAL_LEG_X 24.0f      // 初始X坐标 (mm)
#define INITIAL_LEG_HEIGHT 80.0f // 初始高度 (mm)

// 平衡控制基础角度偏移
#define BASE_ANGLE_OFFSET -174.0f // 基础角度偏移量 (度)

// SimpleFOC PID参数
// 速度环PID（外环）- 控制目标角度
#define BALANCE_SPEED_PID_P 0.2f
#define BALANCE_SPEED_PID_I 0.0f
#define BALANCE_SPEED_PID_D 0.005f

// 角度环PID（内环）- 控制电机扭矩
#define BALANCE_ANGLE_PID_P 0.7f
#define BALANCE_ANGLE_PID_I 0.0f
#define BALANCE_ANGLE_PID_D 0.005f

// 转向环PID - 控制转向力矩
#define STEERING_PID_P 0.03f
#define STEERING_PID_I 0.0f
#define STEERING_PID_D 0.0f

// 基础角度偏移修正PID
#define OFFSET_PID_P 0.001f
#define OFFSET_PID_I 0.0f
#define OFFSET_PID_D 0.01f

// 滚转角PID - 控制滚转偏移角
#define ROLL_PID_P 0.1f
#define ROLL_PID_I 0.0f
#define ROLL_PID_D 0.001f

// ==================== LQR 风格多 PID 控制参数（参考 3.Software 实现） ====================
// 俯仰角 P 控制（对应 LQR 中的 k1）
#define LQR_ANGLE_PID_P   1.0f
#define LQR_ANGLE_PID_I   0.0f
#define LQR_ANGLE_PID_D   0.0f

// 俯仰角速度 P 控制（对应 LQR 中的 k2）
#define LQR_GYRO_PID_P    0.06f
#define LQR_GYRO_PID_I    0.0f
#define LQR_GYRO_PID_D    0.0f

// 前后位移 P 控制（对应 LQR 中的 k3）
#define LQR_DISTANCE_PID_P 0.5f
#define LQR_DISTANCE_PID_I 0.0f
#define LQR_DISTANCE_PID_D 0.0f

// 线速度 P 控制（对应 LQR 中的 k4）
#define LQR_SPEED_PID_P   0.7f
#define LQR_SPEED_PID_I   0.0f
#define LQR_SPEED_PID_D   0.0f

// 小扭矩非线性补偿（对 LQR 总输出做 PI）
#define LQR_U_PID_P       1.0f
#define LQR_U_PID_I       0.0f
#define LQR_U_PID_D       0.0f

// 重心自适应（缓慢调整直立基准角）
#define LQR_ZERO_PID_P    0.002f
#define LQR_ZERO_PID_I    0.0f
#define LQR_ZERO_PID_D    0.0f

// 滚转控制参数
#define ROLL_ANGLE_OFFSET_LIMIT 70.0f  // 滚转偏移角限制 (度)
#define ROLL_BASE_EXPECTED_ANGLE 180.0f  // 基础期望IMU_Y角度 (度)

// 自动偏移修正参数
#define SPEED_DEAD_ZONE 1.0f  // 速度死区范围 (RPM)

// 平衡控制输出限制
#define BALANCE_OUTPUT_LIMIT 1000.0f

// ==================== LQR 增强控制配置 ====================
// 轮子半径 (单位: m) - 对齐 LQR_Practise 工程中的设置
#define WHEEL_RADIUS_M 0.03f
// LQR 位移积分上限 (单位: m)
#define LQR_POSITION_LIMIT 0.10f
// LQR 输出与原PID平衡输出的混合比例 (0~1，可根据实际调节)
#define LQR_MIX_K 0.1f

// ==================== 电机配置 ====================
// AS5600编码器I2C地址
#define AS5600_ADDRESS 0x36

// 电机方向配置（使用扭矩反转方式）
#define MOTOR1_REVERSE true   // 电机1反向（左轮）
#define MOTOR2_REVERSE true   // 电机2反向（右轮）

// BLDC电机参数
#define MOTOR_POLE_PAIRS 7
#define MOTOR_VOLTAGE_LIMIT 12.0f
#define MOTOR_VELOCITY_LIMIT 20.0f
#define MOTOR_VOLTAGE_POWER_SUPPLY 12.0f

// 电机PID参数
#define MOTOR_PID_P 0.1f
#define MOTOR_PID_I 0.0f
#define MOTOR_PID_D 0.0f

// 速度低通滤波时间常数
#define MOTOR_LPF_TF 0.07f

// ==================== 调试配置 ====================
#define DEBUG_MODE true
#define SERIAL_BAUD_RATE 115200

#endif // CONFIG_H

