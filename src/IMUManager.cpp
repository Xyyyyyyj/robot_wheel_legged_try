/**
 * IMU管理器实现
 */

#include "IMUManager.h"
#include "I2CScanner.h"

IMUManager::IMUManager() : imu(Wire) {
    xMutex = xSemaphoreCreateMutex();
    if (!xMutex) {
        Serial.printf("[IMU] 错误: 互斥锁创建失败\n");
    }
    
    // 初始化频率统计变量
    lastUpdateTime = 0;
    updateCount = 0;
    rateStartTime = millis();
}


bool IMUManager::begin() {
    Serial.printf("[IMU] 使用已初始化的I2C总线 (SDA:%d, SCL:%d)\n", I2C_SDA, I2C_SCL);
    // I2C总线已在main.cpp中初始化，这里不再重复初始化
    delay(100);
    
    // 使用全局I2C扫描功能
    scanI2CDevices();
    
    try {
        Serial.printf("[IMU] 初始化SF_IMU (MPU6050)...\n");
        Serial.printf("[IMU] 使用SF_IMU库，运行在高频模式\n");
        Serial.printf("[IMU] 校准中，请保持设备静止...\n");
        
        imu.init();  // SF_IMU的初始化方法，包含自动校准
        
        Serial.printf("[IMU] MPU6050 已连接 (0x%02X) ✓\n", IMU_ADDRESS);
        Serial.printf("[IMU] 校准完成 ✓\n");
        data.isReady = true;
        return true;
    } catch (...) {
        Serial.printf("[IMU] 初始化失败\n");
        Serial.printf("[IMU] 可能原因:\n");
        Serial.printf("  - MPU6050未连接或损坏\n");
        Serial.printf("  - I2C地址错误 (预期0x68或0x69)\n");
        Serial.printf("  - 硬件连接错误\n");
        data.isReady = false;
        return false;
    }
}

void IMUManager::update() {
    if (!data.isReady) return;
    
    try {
        imu.update();
        
        // 更新频率统计
        unsigned long currentTime = millis();
        updateCount++;
        
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            data.temperature = imu.temp;
            data.accX = imu.acc[0];
            data.accY = imu.acc[1];
            data.accZ = imu.acc[2];
            data.gyroX = imu.gyro[0];
            data.gyroY = imu.gyro[1];
            data.gyroZ = imu.gyro[2];
            data.angleX = imu.angle[0];
            data.angleY = imu.angle[1];
            data.angleZ = imu.angle[2];
            data.lastUpdate = currentTime;
            xSemaphoreGive(xMutex);
        }
        
        lastUpdateTime = currentTime;
    } catch (...) {
        Serial.printf("[IMU] 更新过程发生异常\n");
    }
}

bool IMUManager::getData(IMUData& outData, TickType_t timeout) {
    if (xSemaphoreTake(xMutex, timeout) == pdTRUE) {
        outData = data;
        xSemaphoreGive(xMutex);
        return true;
    }
    return false;
}

float IMUManager::getUpdateRate() {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - rateStartTime;
    
    if (elapsedTime >= 1000) {  // 每秒计算一次频率
        float rate = (float)updateCount * 1000.0f / (float)elapsedTime;
        
        // 重置计数器
        updateCount = 0;
        rateStartTime = currentTime;
        
        return rate;
    }
    
    return 0.0f;  // 还没有足够的数据计算频率
}

void IMUManager::printStatus() {
    IMUData currentData;
    if (getData(currentData)) {
        if (currentData.isReady) {
            Serial.printf("\n===== IMU状态 =====\n");
            Serial.printf("温度: %.2f°C\n", currentData.temperature);
            Serial.printf("加速度 X:%.2f Y:%.2f Z:%.2f (g)\n", 
                         currentData.accX, currentData.accY, currentData.accZ);
            Serial.printf("角速度 X:%.2f Y:%.2f Z:%.2f (°/s)\n", 
                         currentData.gyroX, currentData.gyroY, currentData.gyroZ);
            Serial.printf("角度   X:%.2f Y:%.2f Z:%.2f (°)\n", 
                         currentData.angleX, currentData.angleY, currentData.angleZ);
            Serial.printf("更新延迟: %lu ms\n", millis() - currentData.lastUpdate);
            Serial.printf("===================\n\n");
        } else {
            Serial.printf("IMU未就绪\n");
        }
    } else {
        Serial.printf("无法获取IMU数据\n");
    }
}

