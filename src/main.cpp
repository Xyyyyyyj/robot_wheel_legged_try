/**
 * MiniWheel 轮腿机器人主程序
 * 重构版本 - 模块化设计
 */

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include "Config.h"
#include "WheelLeg.h"
#include "IMUManager.h"
#include "BLEController.h"
#include "CommandHandler.h"
#include "LEDIndicator.h"
#include "MotorController.h"
#include "I2CScanner.h"

// ==================== 全局对象 ====================
WheelLeg wheelLeg(SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN);
IMUManager imuManager;
BLEController bleController;
CommandHandler cmdHandler;
LEDIndicator ledIndicator(LED_PIN, LED_COUNT);
MotorController motorController1(MOTOR1_PWM_A, MOTOR1_PWM_B, MOTOR1_PWM_C, MOTOR1_ENABLE, &Wire, MOTOR1_REVERSE);
MotorController motorController2(MOTOR2_PWM_A, MOTOR2_PWM_B, MOTOR2_PWM_C, MOTOR2_ENABLE, &Wire1, MOTOR2_REVERSE);

// ==================== FreeRTOS 对象 ====================
TaskHandle_t taskSerialHandle = NULL;
TaskHandle_t taskMotorControlHandle = NULL;
TaskHandle_t taskIMUHandle = NULL;
TaskHandle_t taskMotorFOCHandle = NULL;

SemaphoreHandle_t xMutexWheelLeg = NULL;
SemaphoreHandle_t xMutexMotor1 = NULL;
SemaphoreHandle_t xMutexMotor2 = NULL;

// ==================== 全局状态变量 ====================
float currentRollAngle = 0.0f;  // 当前滚转角（度）

// ==================== 辅助函数 ====================

// 打印帮助信息
void printHelp() {
    wheelLeg.printHelp();
}

// BLE数据回调处理
void onBLEDataReceived(const uint8_t* data, size_t len) {
    cmdHandler.parseBLEData(data, len);
}

// ==================== FreeRTOS 任务 ====================

// 任务1: 串口命令处理
void taskSerialCommand(void *pvParameters) {
    for(;;) {
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n');
            Command cmd;
            if (cmdHandler.parseSerialCommand(input, cmd)) {
                cmdHandler.sendCommand(cmd);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 任务2: 电机控制
void taskMotorControl(void *pvParameters) {
    Command cmd;
    
    for(;;) {
        if (cmdHandler.receiveCommand(cmd)) {
            switch(cmd.type) {
                case CMD_SERVO_ANGLE:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setServoAngle(cmd.data.servoAngle.servoNum, 
                                              cmd.data.servoAngle.angle);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_LEFT_LEG_POS:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setLeftLegPosition(cmd.data.position.x, cmd.data.position.y);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_RIGHT_LEG_POS:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setRightLegPosition(cmd.data.position.x, cmd.data.position.y);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_RESET:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.reset();
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_SYSTEM_STATUS:
                    Serial.printf("========== 系统状态 ==========\n");
                    Serial.printf("[DEBUG] 尝试获取轮腿互斥锁...\n");
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        Serial.printf("[DEBUG] 轮腿互斥锁获取成功\n");
                        wheelLeg.printStatus();
                        xSemaphoreGive(xMutexWheelLeg);
                    } else {
                        Serial.printf("[错误] 无法获取轮腿互斥锁\n");
                    }
                    
                    
                    Serial.printf("[DEBUG] 电机1状态: %s\n", motorController1.isReady() ? "就绪" : "未就绪");
                    Serial.printf("[DEBUG] 电机2状态: %s\n", motorController2.isReady() ? "就绪" : "未就绪");
                    Serial.printf("============================\n");
                    break;
                    
                case CMD_SET_HEIGHT:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(200)) == pdTRUE) {
                        float height = cmd.data.legHeight.height;
                        float x = wheelLeg.getInitialX();
                        
                        wheelLeg.setInitialPosition(x, height);
                        bool success = wheelLeg.setLeftLegPosition(x, -height) &&
                                      wheelLeg.setRightLegPosition(x, -height);
                        
                        Serial.printf("[腿高] %.2f mm %s\n", height, 
                                     success ? "✓" : "✗");
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_IMU_STATUS:
                    imuManager.printStatus();
                    break;
                    
                case CMD_MOTOR_STATUS:
                    // 显示两个电机的状态
                    Serial.printf("========== 双电机状态 ==========\n");
                    if (xSemaphoreTake(xMutexMotor1, pdMS_TO_TICKS(100)) == pdTRUE) {
                        Serial.printf("--- 电机1 (左轮) ---\n");
                        motorController1.printStatus();
                        xSemaphoreGive(xMutexMotor1);
                    } else {
                        Serial.printf("[错误] 无法获取电机1互斥锁\n");
                    }
                    
                    if (xSemaphoreTake(xMutexMotor2, pdMS_TO_TICKS(100)) == pdTRUE) {
                        Serial.printf("--- 电机2 (右轮) ---\n");
                        motorController2.printStatus();
                        xSemaphoreGive(xMutexMotor2);
                    } else {
                        Serial.printf("[错误] 无法获取电机2互斥锁\n");
                    }
                    Serial.printf("============================\n");
                    break;
                    
                case CMD_I2C_SCAN:
                    scanI2CDevices();
                    break;
                    
                case CMD_AS5600_TEST:
                    testAS5600Communication();
                    break;
                    
                case CMD_I2C_CHECK:
                    checkI2CBusHealth();
                    break;
                    
                case CMD_TARGET_SPEED:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setTargetSpeed(cmd.data.targetSpeed.speed);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_BALANCE_LIMIT:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setBalanceOutputLimit(cmd.data.balanceLimit.limit);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_BALANCE_STATUS:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        Serial.printf("========== 平衡控制状态 ==========\n");
                        Serial.printf("控制状态: 启用 (默认)\n");
                        Serial.printf("\nSimpleFOC PID控制器:\n");
                        Serial.printf("  速度环PID: P=%.3f, I=%.3f, D=%.3f (限制:±10.0°)\n", 
                                     BALANCE_SPEED_PID_P, BALANCE_SPEED_PID_I, BALANCE_SPEED_PID_D);
                        Serial.printf("  角度环PID: P=%.3f, I=%.3f, D=%.3f (限制:±%.1f)\n", 
                                     BALANCE_ANGLE_PID_P, BALANCE_ANGLE_PID_I, BALANCE_ANGLE_PID_D, wheelLeg.getBalanceOutputLimit());
                        Serial.printf("\n速度信息:\n");
                        Serial.printf("  目标速度: %.2f\n", wheelLeg.getTargetSpeed());
                        Serial.printf("  当前速度: %.2f\n", wheelLeg.getCurrentSpeed());
                        float speedError = wheelLeg.getTargetSpeed() - wheelLeg.getCurrentSpeed();
                        Serial.printf("  速度误差: %.2f\n", speedError);
                        Serial.printf("\n角度参数:\n");
                        Serial.printf("  基础角度偏移: %.2f°\n", wheelLeg.getBaseAngleOffset());
                        Serial.printf("\n控制算法:\n");
                        Serial.printf("  外环: 目标角度 = speedPID(速度误差)\n");
                        Serial.printf("  内环: 电机输出 = anglePID(角度误差)\n");
                        Serial.printf("  使用SimpleFOC优化PID算法，自动积分限幅和微分滤波\n");
                        Serial.printf("  (需要IMU角度数据计算最终输出)\n");
                        Serial.printf("============================\n");
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_IMU_RATE:
                    {
                        float rate = imuManager.getUpdateRate();
                        Serial.printf("========== IMU更新频率 ==========\n");
                        Serial.printf("配置频率: %d Hz\n", IMU_UPDATE_RATE_HZ);
                        if (rate > 0.0f) {
                            Serial.printf("实际频率: %.1f Hz\n", rate);
                            float efficiency = (rate / IMU_UPDATE_RATE_HZ) * 100.0f;
                            Serial.printf("效率: %.1f%%\n", efficiency);
                        } else {
                            Serial.printf("实际频率: 计算中...\n");
                        }
                        Serial.printf("I2C速度: %d Hz\n", I2C_CLOCK_SPEED);
                        Serial.printf("任务优先级: %d\n", TASK_PRIORITY_IMU);
                        Serial.printf("============================\n");
                    }
                    break;
                    
                case CMD_SPEED_PID:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setSpeedPID(cmd.data.pidParams.P, cmd.data.pidParams.I, cmd.data.pidParams.D);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_ANGLE_PID:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setAnglePID(cmd.data.pidParams.P, cmd.data.pidParams.I, cmd.data.pidParams.D);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_STEERING_PID:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setSteeringPID(cmd.data.pidParams.P, cmd.data.pidParams.I, cmd.data.pidParams.D);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_TARGET_YAW_RATE:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setTargetYawRate(cmd.data.targetYawRate.yawRate);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_OFFSET_PID:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setOffsetPID(cmd.data.pidParams.P, cmd.data.pidParams.I, cmd.data.pidParams.D);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_ROLL_PID:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setRollPID(cmd.data.pidParams.P, cmd.data.pidParams.I, cmd.data.pidParams.D);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_AUTO_OFFSET:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setAutoOffsetEnabled(cmd.data.autoOffset.enabled);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_SPEED_DEAD_ZONE:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setSpeedDeadZone(cmd.data.speedDeadZone.deadZone);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_BASE_ANGLE_OFFSET:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setBaseAngleOffset(cmd.data.baseAngleOffset.offset);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_ROLL_ANGLE:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        // 更新全局滚转角
                        currentRollAngle = cmd.data.rollAngle.angle;
                        
                        // 解锁平衡偏移角冻结状态
                        wheelLeg.unlockOffsetFreeze();
                        
                        // 获取当前腿高
                        float baseHeight = wheelLeg.getInitialHeight();
                        float x = wheelLeg.getInitialX();
                        
                        // 获取轮距（L5参数）
                        float wheelDistance = 47.0f;  // 轮距 (mm)
                        
                        // 将滚转角转换为弧度
                        float rollAngleRad = currentRollAngle * PI / 180.0f;
                        
                        // 计算左右腿高度调整量
                        float heightAdjustment = sin(rollAngleRad) * wheelDistance / 2.0f;
                        
                        // 左腿高 = height + sin(滚转角) * 轮距/2
                        float leftHeight = baseHeight + heightAdjustment;
                        // 右腿高 = height - sin(滚转角) * 轮距/2
                        float rightHeight = baseHeight - heightAdjustment;
                        
                        // 分别设置左右腿高度
                        bool success = wheelLeg.setLeftLegPosition(x, -leftHeight) &&
                                      wheelLeg.setRightLegPosition(x, -rightHeight);
                        
                        if (success) {
                            Serial.printf("[滚转角] %.2f° -> 左腿: %.2f mm, 右腿: %.2f mm %s\n", 
                                         currentRollAngle, leftHeight, rightHeight, "✓");
                        } else {
                            Serial.printf("[滚转角] %.2f° 设置失败 ✗\n", currentRollAngle);
                        }
                        
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_ADAPTIVE_PID:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setAdaptivePIDEnabled(cmd.data.adaptivePID.enabled);
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_CALIBRATION_POINT:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        wheelLeg.setCalibrationPoint(
                            cmd.data.calibrationPoint.index,
                            cmd.data.calibrationPoint.height,
                            cmd.data.calibrationPoint.speedP,
                            cmd.data.calibrationPoint.speedI,
                            cmd.data.calibrationPoint.speedD,
                            cmd.data.calibrationPoint.angleP,
                            cmd.data.calibrationPoint.angleI,
                            cmd.data.calibrationPoint.angleD
                        );
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
                    
                case CMD_BLE_CONTROL_DATA:
                    if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(100)) == pdTRUE) {
                        // 1. 更新目标速度（使用线速度）
                        wheelLeg.setTargetSpeed(cmd.data.bleControlData.linearVelocity);
                        
                        // 2. 更新腿高（如果有效值）
                        if (cmd.data.bleControlData.height > 0) {
                            float x = wheelLeg.getInitialX();
                            float baseHeight = cmd.data.bleControlData.height;
                            
                            // 设置初始位置（使用基础高度）
                            wheelLeg.setInitialPosition(x, baseHeight);
                        }
                        
                        // 3. 设置目标滚转角（闭环控制）
                        wheelLeg.setTargetRollAngle(cmd.data.bleControlData.rollAngle);
                        
                        // 4. 处理角速度（设置目标转向速度）
                        wheelLeg.setTargetYawRate(cmd.data.bleControlData.angularVelocity);
                        
                        Serial.printf("[BLE控制] 线速度: %.2f, 角速度: %.2f, 腿高: %.2f mm, 目标滚转角: %.2f°\n", 
                                     cmd.data.bleControlData.linearVelocity,
                                     cmd.data.bleControlData.angularVelocity,
                                     cmd.data.bleControlData.height,
                                     cmd.data.bleControlData.rollAngle);
                        
                        xSemaphoreGive(xMutexWheelLeg);
                    }
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 任务3: IMU数据读取和平衡控制
void taskIMU(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / IMU_UPDATE_RATE_HZ);
    
    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        if (imuManager.isReady()) {
            imuManager.update();
            
            // 执行平衡控制
            IMUData imuData;
            if (imuManager.getData(imuData, pdMS_TO_TICKS(5))) {
                // 获取轮腿控制器访问权限 - 减少超时时间
                if (xSemaphoreTake(xMutexWheelLeg, pdMS_TO_TICKS(3)) == pdTRUE) {
                    // 获取电机RPM数据 - 使用短超时确保能获取到数据
                    static float lastMotor1Speed = 0.0f, lastMotor2Speed = 0.0f;
                    float motor1Speed = 0.0f, motor2Speed = 0.0f;
                    
                    if (xSemaphoreTake(xMutexMotor1, pdMS_TO_TICKS(1)) == pdTRUE) {  // 1ms超时
                        motor1Speed = motorController1.getSpeedRPM();
                        lastMotor1Speed = motor1Speed;  // 更新上次的值
                        xSemaphoreGive(xMutexMotor1);
                    } else {
                        // 如果获取失败，使用上次的值
                        motor1Speed = lastMotor1Speed;
                    }
                    
                    if (xSemaphoreTake(xMutexMotor2, pdMS_TO_TICKS(1)) == pdTRUE) {  // 1ms超时
                        motor2Speed = motorController2.getSpeedRPM();
                        lastMotor2Speed = motor2Speed;  // 更新上次的值
                        xSemaphoreGive(xMutexMotor2);
                    } else {
                        // 如果获取失败，使用上次的值
                        motor2Speed = lastMotor2Speed;
                    }
                    
                    // 计算平衡控制输出 - 包含转向控制和滚转控制
                    float leftOutput, rightOutput;
                    wheelLeg.calculateBalanceOutput(imuData.angleX, imuData.angleY, imuData.gyroZ, motor1Speed, motor2Speed, leftOutput, rightOutput);
                    
                    // 将差速控制输出应用到电机
                    // 获取电机控制权限并设置扭矩 - 使用非阻塞方式
                    if (xSemaphoreTake(xMutexMotor1, 0) == pdTRUE) {  // 非阻塞
                        motorController1.setTarget(leftOutput);  // 电机1为左轮
                        xSemaphoreGive(xMutexMotor1);
                    }
                    if (xSemaphoreTake(xMutexMotor2, 0) == pdTRUE) {  // 非阻塞
                        motorController2.setTarget(rightOutput); // 电机2为右轮
                        xSemaphoreGive(xMutexMotor2);
                    }
                    
                    // 更新BLE状态数据（每100ms更新一次，降低BLE负载）
                    static unsigned long lastBLEUpdate = 0;
                    unsigned long currentTime = millis();
                    if (currentTime - lastBLEUpdate >= 100) {
                        if (bleController.isConnected()) {
                            // 状态数据1：电机+平衡（16字节）
                            BLEStatusData1 statusData1;
                            statusData1.motor1RPM = motor1Speed;
                            statusData1.motor2RPM = motor2Speed;
                            float baseOffset = wheelLeg.getBaseAngleOffset();
                            float autoCorrection = wheelLeg.getBaseAngleOffsetCorrection();
                            statusData1.balanceAngleOffset = baseOffset + autoCorrection;
                            statusData1.leftOutput = leftOutput;
                            
                            // 调试输出（每1秒输出一次）
                            static unsigned long lastDebugPrint = 0;
                            if (currentTime - lastDebugPrint >= 1000) {
                                Serial.printf("[BLE] 基础偏移=%.3f°, 自动修正=%.3f°, 总偏移=%.3f°\n", 
                                             baseOffset, autoCorrection, statusData1.balanceAngleOffset);
                                lastDebugPrint = currentTime;
                            }
                            
                            // 状态数据2：速度+姿态（16字节）
                            BLEStatusData2 statusData2;
                            statusData2.rightOutput = rightOutput;
                            statusData2.currentSpeed = wheelLeg.getCurrentSpeed();
                            statusData2.angleX = imuData.angleX;
                            statusData2.gyroZ = imuData.gyroZ;
                            
                            // 分别更新两个特征值
                            bleController.updateStatusData1(statusData1);
                            bleController.updateStatusData2(statusData2);
                        }
                        lastBLEUpdate = currentTime;
                    }
                    
                    xSemaphoreGive(xMutexWheelLeg);
                }
            }
        }
    }
}

// 任务4: 电机FOC控制 - 高频运行
void taskMotorFOC(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2ms = 500Hz
    
    for(;;) {
        // 电机1 FOC控制
        if (motorController1.isReady()) {
            if (xSemaphoreTake(xMutexMotor1, pdMS_TO_TICKS(1)) == pdTRUE) {
                motorController1.loopFOC();
                motorController1.move();
                xSemaphoreGive(xMutexMotor1);
            }
        }
        
        // 电机2 FOC控制
        if (motorController2.isReady()) {
            if (xSemaphoreTake(xMutexMotor2, pdMS_TO_TICKS(1)) == pdTRUE) {
                motorController2.loopFOC();
                motorController2.move();
                xSemaphoreGive(xMutexMotor2);
            }
        }
        
        // 使用固定频率，给其他任务获取锁的机会
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==================== 初始化函数 ====================

bool initFreeRTOS() {
    // 创建互斥锁
    xMutexWheelLeg = xSemaphoreCreateMutex();
    xMutexMotor1 = xSemaphoreCreateMutex();
    xMutexMotor2 = xSemaphoreCreateMutex();
    
    if (!xMutexWheelLeg || !xMutexMotor1 || !xMutexMotor2) {
        Serial.printf("[错误] 互斥锁创建失败\n");
        return false;
    }
    
    // 创建任务
    xTaskCreatePinnedToCore(
        taskSerialCommand, 
        "Serial", 
        TASK_STACK_SERIAL, 
        NULL, 
        TASK_PRIORITY_SERIAL, 
        &taskSerialHandle, 
        TASK_CORE_SERIAL
    );
    
    xTaskCreatePinnedToCore(
        taskMotorControl, 
        "Motor", 
        TASK_STACK_MOTOR, 
        NULL, 
        TASK_PRIORITY_MOTOR, 
        &taskMotorControlHandle, 
        TASK_CORE_MOTOR
    );
    
    if (imuManager.isReady()) {
        xTaskCreatePinnedToCore(
            taskIMU, 
            "IMU", 
            TASK_STACK_IMU, 
            NULL, 
            TASK_PRIORITY_IMU, 
            &taskIMUHandle, 
            TASK_CORE_IMU
        );
    }
    
    // 创建电机FOC任务（高优先级，核心1）
    if (motorController1.isReady() || motorController2.isReady()) {
        xTaskCreatePinnedToCore(
            taskMotorFOC, 
            "MotorFOC", 
            3072, 
            NULL, 
            4,  // 高优先级
            &taskMotorFOCHandle, 
            1   // 核心1
        );
    }
    
    Serial.printf("[RTOS] 任务已创建 ✓\n");
    return true;
}

// ==================== 主程序 ====================

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);
    
    Serial.printf("\n========================================\n");
    Serial.printf("  %s %s\n", SYSTEM_NAME, SYSTEM_VERSION);
    Serial.printf("========================================\n");
    
    // 初始化LED指示器
    Serial.printf("\n[系统] 初始化LED指示器...\n");
    ledIndicator.begin();
    ledIndicator.setColor(LED_RED);  // 红色 - 开始初始化
    
    // ========== 步骤1: 初始化BLE ==========
    Serial.printf("\n[系统] 步骤1: 初始化BLE...\n");
    if (bleController.begin(BLE_DEVICE_NAME)) {
        bleController.setDataCallback(onBLEDataReceived);
    }
    ledIndicator.setColor(LED_ORANGE);  // 橙色 - BLE完成
    delay(300);
    
    // ========== 步骤2: 初始化舵机 ==========
    Serial.printf("\n[系统] 步骤2: 初始化舵机（轮腿系统）...\n");
    wheelLeg.begin();
    ledIndicator.setColor(LED_YELLOW);  // 黄色 - 舵机完成
    delay(300);
    
    // ========== 步骤3: 初始化其他组件 ==========
    Serial.printf("\n[系统] 步骤3: 初始化其他组件...\n");
    
    // 初始化命令处理器
    Serial.printf("\n[系统] 初始化命令处理器...\n");
    if (!cmdHandler.begin()) {
        Serial.printf("[错误] 命令处理器初始化失败！\n");
        ledIndicator.flash(LED_RED, 5000);  // 红色闪烁表示错误
        while(1) delay(1000);
    }
    
    // 初始化I2C（用于IMU和AS5600编码器）
    Serial.printf("\n[系统] 初始化I2C总线...\n");
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);
    Wire.setTimeOut(2000); // 设置2秒超时
    
    // 检查I2C总线状态
    if (digitalRead(I2C_SDA) != HIGH || digitalRead(I2C_SCL) != HIGH) {
        Serial.printf("[警告] I2C总线引脚状态异常!\n");
        Serial.printf("  SDA(%d)状态: %s, SCL(%d)状态: %s\n",
                      I2C_SDA, digitalRead(I2C_SDA) == HIGH ? "HIGH" : "LOW",
                      I2C_SCL, digitalRead(I2C_SCL) == HIGH ? "HIGH" : "LOW");
        Serial.printf("  请检查上拉电阻 (建议4.7kΩ)\n");
    }
    
    delay(500); // 增加延迟确保I2C初始化完成
    Serial.printf("[I2C] SDA=%d, SCL=%d, 频率=%dkHz, 超时=%dms ✓\n", 
                  I2C_SDA, I2C_SCL, I2C_CLOCK_SPEED/1000, 2000);
    
    // 初始化第二个I2C总线（用于第二个AS5600编码器）
    Serial.printf("\n[系统] 初始化I2C1总线...\n");
    Wire1.setPins(I2C1_SDA, I2C1_SCL);
    Wire1.begin();
    Wire1.setClock(I2C1_CLOCK_SPEED);
    Wire1.setTimeOut(2000); // 设置2秒超时
    
    // 检查I2C1总线状态
    if (digitalRead(I2C1_SDA) != HIGH || digitalRead(I2C1_SCL) != HIGH) {
        Serial.printf("[警告] I2C1总线引脚状态异常!\n");
        Serial.printf("  SDA(%d)状态: %s, SCL(%d)状态: %s\n",
                      I2C1_SDA, digitalRead(I2C1_SDA) == HIGH ? "HIGH" : "LOW",
                      I2C1_SCL, digitalRead(I2C1_SCL) == HIGH ? "HIGH" : "LOW");
        Serial.printf("  请检查上拉电阻 (建议4.7kΩ)\n");
    }
    
    delay(500); // 增加延迟确保I2C1初始化完成
    Serial.printf("[I2C1] SDA=%d, SCL=%d, 频率=%dkHz, 超时=%dms ✓\n", 
                  I2C1_SDA, I2C1_SCL, I2C1_CLOCK_SPEED/1000, 2000);
    
    ledIndicator.setColor(LED_CYAN);  // 青色 - I2C完成
    delay(300);
    
    // 初始化IMU
    Serial.printf("\n[系统] 初始化IMU...\n");
    imuManager.begin();  // IMU失败不影响整体运行
    ledIndicator.setColor(LED_BLUE);  // 蓝色 - IMU完成
    delay(300);
    
    // 初始化BLDC电机
    Serial.printf("\n[系统] 初始化双BLDC电机...\n");
    
    // 初始化电机1 (左轮)
    Serial.printf("--- 电机1 (左轮) ---\n");
    if (!motorController1.begin()) {
        Serial.printf("[警告] 电机1初始化失败\n");
    }
    
    // 初始化电机2 (右轮)
    Serial.printf("--- 电机2 (右轮) ---\n");
    if (!motorController2.begin()) {
        Serial.printf("[警告] 电机2初始化失败\n");
    }
    
    Serial.printf("[系统] 双电机初始化完成\n");
    ledIndicator.setColor(LED_PURPLE);  // 紫色 - 电机完成
    delay(300);
    
    // 显示平衡控制状态
    Serial.printf("\n[平衡控制] 系统状态: 启用 (默认)\n");
    Serial.printf("[平衡控制] SimpleFOC PID控制器已初始化\n");
    Serial.printf("[平衡控制] 速度环PID: P=%.3f, I=%.3f, D=%.3f\n", 
                 BALANCE_SPEED_PID_P, BALANCE_SPEED_PID_I, BALANCE_SPEED_PID_D);
    Serial.printf("[平衡控制] 角度环PID: P=%.3f, I=%.3f, D=%.3f\n", 
                 BALANCE_ANGLE_PID_P, BALANCE_ANGLE_PID_I, BALANCE_ANGLE_PID_D);
    Serial.printf("[平衡控制] 集成到IMU任务，运行频率: %d Hz\n", IMU_UPDATE_RATE_HZ);
    
    // 初始化FreeRTOS
    Serial.printf("\n[系统] 初始化FreeRTOS...\n");
    if (!initFreeRTOS()) {
        Serial.printf("[错误] FreeRTOS初始化失败！\n");
        ledIndicator.flash(LED_RED, 5000);  // 红色闪烁表示错误
        while(1) delay(1000);
    }
    ledIndicator.setColor(LED_WHITE);  // 白色 - FreeRTOS完成
    delay(300);
    
    // 全部初始化完成 - 绿色2秒后熄灭
    Serial.printf("\n[系统] 就绪 ✓\n");
    Serial.printf("命令: help, status, imu, reset\n");
    Serial.printf("========================================\n\n");
    
    ledIndicator.setColor(LED_GREEN);  // 绿色 - 系统就绪
    delay(2000);
    ledIndicator.turnOff();  // 熄灭
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
