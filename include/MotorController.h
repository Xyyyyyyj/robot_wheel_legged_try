/**
 * MotorController.h
 * BLDC电机控制器 - 使用SimpleFOC库和AS5600编码器
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include "Config.h"

class MotorController {
public:
    /**
     * 构造函数
     * @param pwmA PWM A相引脚
     * @param pwmB PWM B相引脚  
     * @param pwmC PWM C相引脚
     * @param enable 使能引脚
     * @param i2cBus I2C总线指针 (默认使用Wire)
     * @param reverse 是否反转电机方向 (默认为false)
     */
    MotorController(int pwmA, int pwmB, int pwmC, int enable, TwoWire* i2cBus = &Wire, bool reverse = false);
    
    /**
     * 初始化电机和编码器
     * @return 初始化是否成功
     */
    bool begin();
    
    /**
     * FOC循环 - 需要在主循环中高频调用
     */
    void loopFOC();
    
    /**
     * 电机运动控制 - 在loopFOC之后调用
     */
    void move();
    
    /**
     * 设置目标扭矩
     * @param torque 目标扭矩值
     */
    void setTarget(float torque);
    
    /**
     * 获取电机角度（度）
     * @return 角度值（度）
     */
    float getAngleDeg();
    
    /**
     * 获取电机速度（RPM）
     * @return 速度值（RPM）
     */
    float getSpeedRPM();
    
    /**
     * 获取电机角度（弧度）
     * @return 角度值（弧度）
     */
    float getAngleRad();
    
    /**
     * 获取当前目标扭矩
     * @return 目标扭矩值
     */
    float getTargetTorque();
    
    /**
     * 打印电机状态
     */
    void printStatus();
    
    /**
     * 检查电机是否就绪
     * @return 是否就绪
     */
    bool isReady() const { return _isInitialized; }

private:
    BLDCMotor _motor;
    BLDCDriver3PWM _driver;
    MagneticSensorI2C _sensor;
    TwoWire* _i2cBus;
    bool _reverse;
    
    bool _isInitialized;
    
    float _currentAngleDeg;
    float _currentSpeedRPM;
};

#endif // MOTOR_CONTROLLER_H
