#ifndef WHEELLEG_H
#define WHEELLEG_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <SimpleFOC.h>
#include "Config.h"

// 五连杆参数结构体
struct FiveLinkageParams {
    float L1;  // 连杆1长度 (mm)
    float L2;  // 连杆2长度 (mm)
    float L3;  // 连杆3长度 (mm)
    float L4;  // 连杆4长度 (mm)
    float L5;  // 连杆5长度 (mm)
};

// 末端位置结构体
struct EndEffectorPos {
    float x;
    float y;
};

class WheelLeg {
private:
    // 舵机对象
    Servo servo1;  // 舵机1 (270度舵机) - 左腿舵机1
    Servo servo2;  // 舵机2 (180度舵机) - 左腿舵机2
    Servo servo3;  // 舵机3 (270度舵机) - 右腿舵机1
    Servo servo4;  // 舵机4 (180度舵机) - 右腿舵机2
    
    // 舵机引脚
    int servo1Pin;
    int servo2Pin;
    int servo3Pin;
    int servo4Pin;
    
    // 舵机当前角度存储 (相对角度)
    float servoAngles[4];
    
    // 舵机零位角度
    int zeroPositions[4];
    
    // 舵机类型 (true: 270度, false: 180度)
    bool is270Servo[4];
    
    // 五连杆参数
    FiveLinkageParams linkageParams;
    
    // 初始高度设置
    float initialHeight;  // 初始高度（轮子末端Y坐标的绝对值）
    float initialX;       // 初始X坐标
    
    // 高度限制
    static constexpr float MIN_HEIGHT = 80.0f;   // 最小高度（mm）
    static constexpr float MAX_HEIGHT = 150.0f;  // 最大高度（mm）
    
    // 平衡控制参数
    float targetSpeed;       // 目标速度
    float targetSpeedFiltered; // 经过斜坡滤波后的目标速度
    float speedRampRate;     // 速度斜坡变化率 (RPM/s)
    unsigned long lastSpeedUpdateTime; // 上次速度更新时间
    float currentSpeed;      // 当前速度
    bool balanceEnabled;     // 平衡控制使能标志
    float balanceOutputLimit; // 平衡控制输出限制
    float baseAngleOffset;   // 基础角度偏移量
    
    // 转向控制参数
    float targetYawRate;     // 目标转向速度（z轴角速度）
    float currentYawRate;    // 当前转向速度（z轴角速度）
    bool steeringEnabled;    // 转向控制使能标志
    
    // 滚转控制参数
    float targetRollAngle;   // 目标滚转角（度）
    float rollAngleOffset;   // 滚转偏移角（度）
    bool rollControlEnabled; // 滚转控制使能标志
    
    // 基础角度偏移自动修正参数
    float baseAngleOffsetCorrection; // 基础角度偏移修正值
    bool autoOffsetEnabled;  // 自动偏移修正使能标志
    float speedDeadZone;     // 速度死区范围
    
    // 基础角度偏移冻结控制
    bool offsetFrozen;
    float lastHeightForOffset;
    float lastBaseAngleOffsetCorrection;
    unsigned long lastOffsetChangeTime;

    static constexpr float HEIGHT_CHANGE_THRESHOLD = 1.0f;
    static constexpr float OFFSET_STABLE_THRESHOLD = 0.01f;
    static constexpr unsigned long OFFSET_STABLE_TIME_MS = 5000;
    
    // 变PID控制参数
    bool adaptivePIDEnabled;  // 变PID使能标志
    
    // 变PID参考点（腿高 -> PID参数）
    struct PIDCalibrationPoint {
        float height;      // 腿高 (mm)
        float speedP;      // 速度环P
        float speedI;      // 速度环I
        float speedD;      // 速度环D
        float angleP;      // 角度环P
        float angleI;      // 角度环I
        float angleD;      // 角度环D
    };
    static constexpr int NUM_CALIBRATION_POINTS = 3;
    PIDCalibrationPoint calibrationPoints[NUM_CALIBRATION_POINTS];
    
    // PID平滑过渡参数
    bool pidTransitioning;           // PID参数是否正在过渡
    unsigned long pidTransitionStartTime;  // 过渡开始时间
    float pidTransitionDuration;     // 过渡持续时间（毫秒）
    float targetSpeedP, targetSpeedI, targetSpeedD;  // 目标速度环PID
    float targetAngleP, targetAngleI, targetAngleD;  // 目标角度环PID
    float startSpeedP, startSpeedI, startSpeedD;     // 起始速度环PID
    float startAngleP, startAngleI, startAngleD;     // 起始角度环PID
    
    // 私有辅助函数：根据腿高插值计算PID参数
    void updatePIDForHeight(float height);
    
    // 私有辅助函数：更新PID平滑过渡
    void updatePIDTransition();
    
    // SimpleFOC PID控制器
    PIDController speedPID;   // 速度环PID控制器（外环）
    PIDController anglePID;   // 角度环PID控制器（内环）
    PIDController steeringPID; // 转向PID控制器
    PIDController offsetPID;  // 基础角度偏移修正PID控制器（增量式）
    PIDController rollPID;    // 滚转角PID控制器（增量式）
    
    // 私有辅助函数：五连杆正运动学（左腿）
    // 输入：舵机1和舵机2的相对角度（度）
    //   theta1: 舵机1的相对角度，0度=水平向左，逆时针为正
    //           -90度=竖直向下，90度=竖直向上
    //   theta2: 舵机2的相对角度，0度=水平向右，顺时针为正
    //           90度=竖直向下，-90度=竖直向上
    // 输出：末端位置（x, y）
    // 全局坐标系：舵机1在原点(0,0)，X轴向右为正，Y轴向上为正
    //            角度定义：水平向右为0°，逆时针为正
    // 角度转换：舵机1全局角度 = 180° + theta1
    //          舵机2全局角度 = -theta2
    EndEffectorPos forwardKinematics(float theta1_deg, float theta2_deg);
    
    // 私有辅助函数：五连杆逆运动学（左腿）
    // 输入：目标末端位置（x, y，相对舵机1）
    // 输出：舵机相对角度（度）
    //   theta1: 舵机1相对角度，0度=水平向左，逆时针为正
    //   theta2: 舵机2相对角度，0度=水平向右，顺时针为正
    // 返回值：是否成功求解
    // 全局坐标系：舵机1在原点(0,0)，X轴向右为正，Y轴向上为正
    //            角度定义：水平向右为0°，逆时针为正
    bool inverseKinematics(float x, float y, float &theta1_deg, float &theta2_deg);
    
    // 私有辅助函数：五连杆正运动学（右腿）
    // 输入：舵机3和舵机4的相对角度（度）
    //   theta3: 舵机3的相对角度，0度=水平向右（机器人右侧），顺时针为正
    //           90度=竖直向下，-90度=竖直向上
    //   theta4: 舵机4的相对角度，0度=水平向左（机器人左侧），逆时针为正
    //           90度=竖直向上，-90度=竖直向下
    // 输出：末端位置（x, y）
    // 全局坐标系：舵机3在原点(0,0)，舵机4在(L5,0)
    //            X轴向后为正，Y轴向上为正
    //            角度定义：向后为0°，逆时针为正
    // 角度转换：舵机3全局角度 = 180° + theta3（与舵机1相同）
    //          舵机4全局角度 = -theta4（与舵机2相同）
    EndEffectorPos forwardKinematicsRight(float theta3_deg, float theta4_deg);
    
    // 私有辅助函数：五连杆逆运动学（右腿）
    // 输入：目标末端位置（x, y，相对舵机3）
    // 输出：舵机相对角度（度）
    //   theta3: 舵机3相对角度，0度=水平向右（机器人右侧），顺时针为正
    //   theta4: 舵机4相对角度，0度=水平向左（机器人左侧），逆时针为正
    // 返回值：是否成功求解
    // 全局坐标系：舵机3在原点(0,0)，舵机4在(L5,0)
    //            X轴向后为正，Y轴向上为正
    //            角度定义：向后为0°，逆时针为正
    // 角度转换：舵机3全局角度 = 180° + theta3（与舵机1相同）
    //          舵机4全局角度 = -theta4（与舵机2相同）
    bool inverseKinematicsRight(float x, float y, float &theta3_deg, float &theta4_deg);

public:
    // 构造函数
    WheelLeg(int pin1, int pin2, int pin3, int pin4);
    
    // 配置五连杆参数
    void setLinkageParams(float L1, float L2, float L3, float L4, float L5);
    
    // 设置初始高度和X坐标
    void setInitialPosition(float x, float height);
    
    // 获取初始高度
    float getInitialHeight() { return initialHeight; }
    
    // 获取初始X坐标
    float getInitialX() { return initialX; }
    
    // 初始化所有舵机
    void begin();
    
    // 设置单个舵机角度（输入相对角度，自动根据零位计算实际角度）
    // 舵机1: 输入a → 实际控制 零位角度+a
    // 舵机2: 输入a → 实际控制 零位角度-a
    // 舵机3: 输入a → 实际控制 零位角度-a
    // 舵机4: 输入a → 实际控制 零位角度+a
    bool setServoAngle(int servoNum, float relativeAngle);
    
    // 获取单个舵机当前角度
    float getServoAngle(int servoNum);
    
    // 设置左腿末端位置 (舵机1和2)
    bool setLeftLegPosition(float x, float y);
    
    // 设置右腿末端位置 (舵机3和4)
    bool setRightLegPosition(float x, float y);
    
    // 分阶段设置两条腿位置（先设置舵机1、3，延迟后设置舵机2、4）
    bool setBothLegsPositionStaged(float x, float y, int delayMs = 50);
    
    // 获取左腿末端位置
    EndEffectorPos getLeftLegPosition();
    
    // 获取右腿末端位置
    EndEffectorPos getRightLegPosition();
    
    // 获取舵机零位
    int getZeroPosition(int servoNum);
    
    // 获取舵机最大角度
    int getMaxAngle(int servoNum);
    
    // 重置所有舵机到0度
    void reset();
    
    // 打印当前状态
    void printStatus();
    
    // 打印使用帮助
    void printHelp();
    
    // 平衡控制相关方法
    
    // 设置平衡控制输出限制
    void setBalanceOutputLimit(float limit);
    
    // 设置SimpleFOC PID参数
    void setSpeedPID(float P, float I, float D);
    void setAnglePID(float P, float I, float D);
    void setSteeringPID(float P, float I, float D);
    void setOffsetPID(float P, float I, float D);
    void setRollPID(float P, float I, float D);
    
    // 设置目标速度和转向速度
    void setTargetSpeed(float speed);
    void setTargetYawRate(float yawRate);
    void setSpeedRampRate(float rampRate);  // 设置速度斜坡变化率
    
    // 滚转控制
    void setTargetRollAngle(float rollAngle);  // 设置目标滚转角
    void setRollControlEnabled(bool enabled);  // 启用/禁用滚转控制
    
    // 自动偏移修正控制
    void setAutoOffsetEnabled(bool enabled);
    void setSpeedDeadZone(float deadZone);
    void unlockOffsetFreeze();  // 解锁平衡偏移角冻结状态
    
    // 变PID控制
    void setAdaptivePIDEnabled(bool enabled);  // 启用/禁用变PID
    void setCalibrationPoint(int index, float height, float speedP, float speedI, float speedD, 
                            float angleP, float angleI, float angleD);  // 设置校准点
    void setPIDTransitionDuration(float durationMs);  // 设置PID平滑过渡时间（毫秒）
    bool isAdaptivePIDEnabled() const { return adaptivePIDEnabled; }
    bool isPIDTransitioning() const { return pidTransitioning; }
    
    // 设置基础角度偏移量
    void setBaseAngleOffset(float offset);
    
    // 计算平衡控制输出 - 包含转向控制和滚转控制
    // 输入：当前IMU的X角度（度），Y角度（度），Z轴角速度（度/秒），电机1 RPM，电机2 RPM
    // 输出：电机扭矩输出（左右轮差速）
    void calculateBalanceOutput(float currentAngleX, float currentAngleY, float currentYawRate, float motor1RPM, float motor2RPM, float& leftOutput, float& rightOutput);
    
    // 获取平衡控制状态
    bool isBalanceEnabled() const { return balanceEnabled; }
    float getTargetSpeed() const { return targetSpeed; }
    float getCurrentSpeed() const { return currentSpeed; }
    float getBalanceOutputLimit() const { return balanceOutputLimit; }
    float getBaseAngleOffset() const { return baseAngleOffset; }
    
    // 获取转向控制状态
    bool isSteeringEnabled() const { return steeringEnabled; }
    float getTargetYawRate() const { return targetYawRate; }
    float getCurrentYawRate() const { return currentYawRate; }
    
    // 获取滚转控制状态
    bool isRollControlEnabled() const { return rollControlEnabled; }
    float getTargetRollAngle() const { return targetRollAngle; }
    float getRollAngleOffset() const { return rollAngleOffset; }
    
    // 获取自动偏移修正状态
    bool isAutoOffsetEnabled() const { return autoOffsetEnabled; }
    float getBaseAngleOffsetCorrection() const { return baseAngleOffsetCorrection; }
    float getSpeedDeadZone() const { return speedDeadZone; }
};

#endif // WHEELLEG_H
