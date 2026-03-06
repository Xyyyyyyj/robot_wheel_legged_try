/**
 * I2CScanner.cpp
 * I2C总线扫描工具 - 用于诊断AS5600等设备连接问题
 */

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

void scanI2CDevices() {
    Serial.printf("\n========== I2C总线扫描 ==========\n");
    Serial.printf("扫描I2C设备 (SDA:%d, SCL:%d, 频率:%dkHz)\n", 
                  I2C_SDA, I2C_SCL, I2C_CLOCK_SPEED/1000);
    Serial.printf("地址范围: 0x08-0x77\n");
    Serial.printf("-------------------------------------\n");
    
    int deviceCount = 0;
    bool foundAS5600 = false;
    bool foundIMU = false;
    
    for (byte address = 8; address < 120; address++) {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();
        
        if (error == 0) {
            deviceCount++;
            Serial.printf("✓ 发现设备: 0x%02X", address);
            
            // 识别已知设备
            switch (address) {
                case AS5600_ADDRESS:
                    Serial.printf(" (AS5600 磁编码器)");
                    foundAS5600 = true;
                    break;
                case IMU_ADDRESS:
                    Serial.printf(" (MPU6050 IMU)");
                    foundIMU = true;
                    break;
                case 0x0C:
                    Serial.printf(" (可能是磁力计)");
                    break;
                case 0x77:
                    Serial.printf(" (可能是气压计)");
                    break;
                default:
                    Serial.printf(" (未知设备)");
                    break;
            }
            Serial.printf("\n");
            
        } else if (error == 4) {
            Serial.printf("✗ 地址 0x%02X: 未知错误\n", address);
        }
        
        delay(10); // 避免I2C总线过载
    }
    
    Serial.printf("-------------------------------------\n");
    Serial.printf("扫描完成: 发现 %d 个设备\n", deviceCount);
    
    // 检查关键设备状态
    Serial.printf("\n========== 设备状态检查 ==========\n");
    Serial.printf("AS5600 (0x%02X): %s\n", AS5600_ADDRESS, 
                  foundAS5600 ? "✓ 已连接" : "✗ 未检测到");
    Serial.printf("MPU6050 (0x%02X): %s\n", IMU_ADDRESS, 
                  foundIMU ? "✓ 已连接" : "✗ 未检测到");
    
    if (!foundAS5600) {
        Serial.printf("\n========== AS5600故障排查 ==========\n");
        Serial.printf("AS5600未检测到，请检查:\n");
        Serial.printf("1. 硬件连接:\n");
        Serial.printf("   - VCC → 3.3V 或 5V\n");
        Serial.printf("   - GND → GND\n");
        Serial.printf("   - SDA → GPIO%d\n", I2C_SDA);
        Serial.printf("   - SCL → GPIO%d\n", I2C_SCL);
        Serial.printf("\n2. 电源检查:\n");
        Serial.printf("   - AS5600工作电压: 3.3V-5V\n");
        Serial.printf("   - 检查供电是否稳定\n");
        Serial.printf("\n3. I2C总线:\n");
        Serial.printf("   - 上拉电阻: 4.7kΩ (SDA和SCL到VCC)\n");
        Serial.printf("   - 总线长度: <30cm (短线缆)\n");
        Serial.printf("   - 避免与其他高频信号并行走线\n");
        Serial.printf("\n4. 磁铁位置:\n");
        Serial.printf("   - 磁铁需放置在AS5600芯片正上方\n");
        Serial.printf("   - 距离: 0.5-3mm\n");
        Serial.printf("   - 磁铁极性: N极朝向芯片\n");
        Serial.printf("\n5. 地址冲突:\n");
        Serial.printf("   - AS5600固定地址0x36\n");
        Serial.printf("   - 检查总线上是否有其他0x36设备\n");
    }
    
    Serial.printf("=====================================\n\n");
}

void testAS5600Communication() {
    Serial.printf("\n========== AS5600通信测试 ==========\n");
    
    // 检查设备是否响应
    Wire.beginTransmission(AS5600_ADDRESS);
    byte error = Wire.endTransmission();
    
    if (error != 0) {
        Serial.printf("✗ AS5600无响应 (错误代码: %d)\n", error);
        switch (error) {
            case 1:
                Serial.printf("  原因: 数据太长，超出传输缓冲区\n");
                break;
            case 2:
                Serial.printf("  原因: 传输地址时收到NACK\n");
                Serial.printf("  建议: 检查设备地址和连接\n");
                break;
            case 3:
                Serial.printf("  原因: 传输数据时收到NACK\n");
                break;
            case 4:
                Serial.printf("  原因: 其他错误\n");
                break;
            default:
                Serial.printf("  原因: 未知错误\n");
                break;
        }
        return;
    }
    
    Serial.printf("✓ AS5600响应正常\n");
    
    // 尝试读取状态寄存器
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(0x0B); // STATUS寄存器地址
    error = Wire.endTransmission(false);
    
    if (error == 0) {
        Wire.requestFrom(AS5600_ADDRESS, (uint8_t)1);
        if (Wire.available()) {
            uint8_t status = Wire.read();
            Serial.printf("✓ 状态寄存器: 0x%02X\n", status);
            
            // 解析状态位
            Serial.printf("  磁铁检测: %s\n", (status & 0x20) ? "✓ 检测到" : "✗ 未检测到");
            Serial.printf("  磁场强度: %s\n", (status & 0x10) ? "✗ 过强" : "✓ 正常");
            Serial.printf("  磁场强度: %s\n", (status & 0x08) ? "✗ 过弱" : "✓ 正常");
        } else {
            Serial.printf("✗ 无法读取状态寄存器\n");
        }
    } else {
        Serial.printf("✗ 无法访问状态寄存器 (错误: %d)\n", error);
    }
    
    // 尝试读取角度值
    Wire.beginTransmission(AS5600_ADDRESS);
    Wire.write(0x0E); // RAW ANGLE高字节
    error = Wire.endTransmission(false);
    
    if (error == 0) {
        Wire.requestFrom(AS5600_ADDRESS, (uint8_t)2);
        if (Wire.available() >= 2) {
            uint16_t rawAngle = (Wire.read() << 8) | Wire.read();
            float angleDeg = (rawAngle * 360.0f) / 4096.0f;
            Serial.printf("✓ 原始角度: %d (%.2f°)\n", rawAngle, angleDeg);
        } else {
            Serial.printf("✗ 无法读取角度数据\n");
        }
    } else {
        Serial.printf("✗ 无法访问角度寄存器 (错误: %d)\n", error);
    }
    
    Serial.printf("=====================================\n\n");
}

void checkI2CBusHealth() {
    Serial.printf("\n========== I2C总线健康检查 ==========\n");
    
    // 检查引脚状态
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    delay(10);
    
    bool sdaHigh = digitalRead(I2C_SDA);
    bool sclHigh = digitalRead(I2C_SCL);
    
    Serial.printf("引脚状态检查:\n");
    Serial.printf("  SDA (GPIO%d): %s\n", I2C_SDA, sdaHigh ? "HIGH ✓" : "LOW ✗");
    Serial.printf("  SCL (GPIO%d): %s\n", I2C_SCL, sclHigh ? "HIGH ✓" : "LOW ✗");
    
    if (!sdaHigh || !sclHigh) {
        Serial.printf("\n⚠️  I2C总线引脚异常!\n");
        Serial.printf("可能原因:\n");
        Serial.printf("- 缺少上拉电阻 (建议4.7kΩ)\n");
        Serial.printf("- 设备短路或损坏\n");
        Serial.printf("- 引脚配置错误\n");
        Serial.printf("- 供电问题\n");
    } else {
        Serial.printf("✓ I2C总线引脚状态正常\n");
    }
    
    // 检查I2C配置
    Serial.printf("\nI2C配置:\n");
    Serial.printf("  频率: %d Hz\n", I2C_CLOCK_SPEED);
    Serial.printf("  超时: 2000 ms\n");
    
    // 重新初始化I2C（恢复正常模式）
    Wire.setPins(I2C_SDA, I2C_SCL);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);
    Wire.setTimeOut(2000);
    
    Serial.printf("=====================================\n\n");
}
