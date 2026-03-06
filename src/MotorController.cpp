/**
 * MotorController.cpp
 * BLDC电机控制器实现
 */

#include "MotorController.h"

MotorController::MotorController(int pwmA, int pwmB, int pwmC, int enable, TwoWire* i2cBus, bool reverse) 
    : _motor(MOTOR_POLE_PAIRS),
      _driver(pwmA, pwmB, pwmC, enable),
      _sensor(AS5600_ADDRESS, 12, 0x0C, 4),
      _i2cBus(i2cBus),
      _reverse(reverse),
      _isInitialized(false),
      _currentAngleDeg(0.0f),
      _currentSpeedRPM(0.0f) {
}

bool MotorController::begin() {
    Serial.printf("[电机] 初始化BLDC电机控制器...\n");
    
    // 检查AS5600编码器是否存在
    _i2cBus->beginTransmission(AS5600_ADDRESS);
    byte error = _i2cBus->endTransmission();
    
    if (error != 0) {
        Serial.printf("[电机] 错误: AS5600编码器未找到 (地址:0x%02X, 错误:%d)\n", AS5600_ADDRESS, error);
        Serial.printf("[电机] 请检查:\n");
        Serial.printf("  - AS5600编码器连接 (I2C地址:0x%02X)\n", AS5600_ADDRESS);
        Serial.printf("  - I2C总线连接 (SDA:%d, SCL:%d)\n", I2C_SDA, I2C_SCL);
        Serial.printf("  - 编码器供电 (3.3V或5V)\n");
        Serial.printf("  - I2C上拉电阻 (通常4.7kΩ)\n");
        _isInitialized = false;
        return false;
    }
    
    // 初始化AS5600编码器
    _sensor.init(_i2cBus);
    Serial.printf("[电机] AS5600编码器初始化完成\n");
    
    // 将编码器链接到电机
    _motor.linkSensor(&_sensor);
    
    // 配置驱动器
    _driver.voltage_power_supply = MOTOR_VOLTAGE_POWER_SUPPLY;
    _driver.init();
    _motor.linkDriver(&_driver);
    Serial.printf("[电机] 驱动器初始化完成\n");
    
    // 设置电机控制模式为扭矩控制
    _motor.torque_controller = TorqueControlType::voltage;
    _motor.controller = MotionControlType::torque;
    
    // 显示电机反转状态
    Serial.printf("[电机] 方向反转: %s\n", _reverse ? "启用" : "禁用");
    
    // 配置电机参数
    _motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    _motor.velocity_limit = MOTOR_VELOCITY_LIMIT;
    _motor.LPF_velocity.Tf = MOTOR_LPF_TF;
    
    // 配置PID参数
    _motor.PID_velocity.P = MOTOR_PID_P;
    _motor.PID_velocity.I = MOTOR_PID_I;
    _motor.PID_velocity.D = MOTOR_PID_D;
    
    // 初始化电机
    _motor.init();
    Serial.printf("[电机] 电机初始化完成\n");
    
    // 初始化FOC
    _motor.initFOC();
    Serial.printf("[电机] FOC初始化完成\n");
    
    // 设置初始目标值
    _motor.target = 0;
    
    _isInitialized = true;
    Serial.printf("[电机] BLDC电机控制器初始化成功 ✓\n");
    
    return true;
}

void MotorController::loopFOC() {
    if (!_isInitialized) return;
    _motor.loopFOC();
}

void MotorController::move() {
    if (!_isInitialized) return;
    
    // 更新电机状态
    _motor.move();
    
    // 读取当前角度和速度
    float angleRad = _sensor.getAngle();
    _currentAngleDeg = angleRad * 180.0f / PI;
    _currentSpeedRPM = _motor.shaft_velocity;
}

void MotorController::setTarget(float torque) {
    if (!_isInitialized) return;
    // 如果启用反转，则反转扭矩符号
    _motor.target = _reverse ? -torque : torque;
}

float MotorController::getAngleDeg() {
    return _currentAngleDeg;
}

float MotorController::getSpeedRPM() {
    return _currentSpeedRPM;
}

float MotorController::getAngleRad() {
    if (!_isInitialized) return 0.0f;
    return _sensor.getAngle();
}

float MotorController::getTargetTorque() {
    if (!_isInitialized) return 0.0f;
    // 返回原始扭矩值（如果有反转则需要反转回来）
    return _reverse ? -_motor.target : _motor.target;
}

void MotorController::printStatus() {
    if (!_isInitialized) {
        Serial.printf("[电机] 未初始化\n");
        return;
    }
    
    Serial.printf("========== 电机状态 ==========\n");
    Serial.printf("[电机] 角度: %.2f° (%.3f rad)\n", _currentAngleDeg, getAngleRad());
    Serial.printf("[电机] 速度: %.2f RPM\n", _currentSpeedRPM);
    Serial.printf("[电机] 目标扭矩: %.2f\n", _motor.target);
    Serial.printf("[电机] 电压限制: %.2f V\n", _motor.voltage_limit);
    Serial.printf("[电机] 速度限制: %.2f rad/s\n", _motor.velocity_limit);
    Serial.printf("[电机] 控制模式: 扭矩控制\n");
    Serial.printf("[电机] 初始化状态: ✓\n");
    Serial.printf("============================\n");
}
