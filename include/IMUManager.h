/**
 * IMU管理器
 * 封装MPU6050传感器的初始化、数据读取和状态管理
 * 使用SF_IMU库进行数据处理
 */

#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include "SF_IMU.h"
#include "Config.h"

// IMU数据结构
struct IMUData {
    float temperature;
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float angleX, angleY, angleZ;
    unsigned long lastUpdate;
    bool isReady;
    
    IMUData() : temperature(0), accX(0), accY(0), accZ(0),
                gyroX(0), gyroY(0), gyroZ(0), angleX(0), angleY(0), angleZ(0),
                lastUpdate(0), isReady(false) {}
};

class IMUManager {
private:
    SF_IMU imu;
    IMUData data;
    SemaphoreHandle_t xMutex;
    
    // 频率统计
    unsigned long lastUpdateTime;
    unsigned long updateCount;
    unsigned long rateStartTime;
    
public:
    IMUManager();
    
    // 初始化IMU
    bool begin();
    
    // 更新IMU数据
    void update();
    
    // 获取IMU数据（线程安全）
    bool getData(IMUData& outData, TickType_t timeout = portMAX_DELAY);
    
    // 打印IMU状态
    void printStatus();
    
    // 检查IMU是否就绪
    bool isReady() const { return data.isReady; }
    
    // 获取IMU更新频率统计
    float getUpdateRate();
    
    // 获取互斥锁
    SemaphoreHandle_t getMutex() { return xMutex; }
};

#endif // IMU_MANAGER_H

