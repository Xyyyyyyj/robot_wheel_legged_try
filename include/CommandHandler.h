/**
 * 命令处理器
 * 统一管理命令定义、队列和处理逻辑
 */

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include "Config.h"

// 命令类型枚举
enum CommandType {
    CMD_SERVO_ANGLE,        // 设置舵机角度
    CMD_LEFT_LEG_POS,       // 设置左腿位置
    CMD_RIGHT_LEG_POS,      // 设置右腿位置
    CMD_RESET,              // 重置
    CMD_SYSTEM_STATUS,      // 显示状态
    CMD_SET_HEIGHT,         // 设置腿高
    CMD_IMU_STATUS,         // IMU状态
    CMD_MOTOR_STATUS,       // 电机状态
    CMD_I2C_SCAN,           // I2C总线扫描
    CMD_AS5600_TEST,        // AS5600通信测试
    CMD_I2C_CHECK,          // I2C总线健康检查
    CMD_BALANCE_LIMIT,      // 设置平衡控制输出限制
    CMD_TARGET_SPEED,       // 设置目标速度
    CMD_BALANCE_STATUS,     // 显示平衡控制状态
    CMD_IMU_RATE,           // 显示IMU更新频率
    CMD_SPEED_PID,          // 设置速度环PID参数
    CMD_ANGLE_PID,          // 设置角度环PID参数
    CMD_STEERING_PID,       // 设置转向环PID参数
    CMD_TARGET_YAW_RATE,    // 设置目标转向速度
    CMD_OFFSET_PID,         // 设置偏移修正PID参数
    CMD_AUTO_OFFSET,        // 设置自动偏移修正使能
    CMD_SPEED_DEAD_ZONE,    // 设置速度死区
    CMD_BASE_ANGLE_OFFSET,  // 设置基础角度偏移量
    CMD_ROLL_ANGLE,         // 设置滚转角
    CMD_ROLL_PID,           // 设置滚转PID参数
    CMD_ADAPTIVE_PID,       // 设置变PID使能
    CMD_CALIBRATION_POINT,  // 设置变PID校准点
    CMD_BLE_CONTROL_DATA    // BLE控制数据（期望高度、滚转角、线速度、角速度）
};

// 命令结构体
struct Command {
    CommandType type;
    union {
        struct { int servoNum; float angle; } servoAngle;
        struct { float x, y; } position;
        struct { float height; } legHeight;
        struct { float limit; } balanceLimit;
        struct { float speed; } targetSpeed;
        struct { float yawRate; } targetYawRate;
        struct { float P, I, D; } pidParams;
        struct { bool enabled; } autoOffset;
        struct { float deadZone; } speedDeadZone;
        struct { float offset; } baseAngleOffset;
        struct { float angle; } rollAngle;
        struct { bool enabled; } adaptivePID;
        struct { int index; float height; float speedP, speedI, speedD; float angleP, angleI, angleD; } calibrationPoint;
        struct { float height, rollAngle, linearVelocity, angularVelocity; } bleControlData;
    } data;
};

class CommandHandler {
private:
    QueueHandle_t cmdQueue;
    
public:
    CommandHandler();
    
    // 初始化命令队列
    bool begin();
    
    // 发送命令到队列
    bool sendCommand(const Command& cmd, TickType_t timeout = pdMS_TO_TICKS(100));
    
    // 从队列接收命令
    bool receiveCommand(Command& cmd, TickType_t timeout = pdMS_TO_TICKS(50));
    
    // 解析串口命令
    bool parseSerialCommand(const String& input, Command& cmd);
    
    // 解析BLE数据
    void parseBLEData(const uint8_t* data, size_t len);
    
    // 获取命令队列句柄
    QueueHandle_t getQueue() { return cmdQueue; }
};

#endif // COMMAND_HANDLER_H

