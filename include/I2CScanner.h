/**
 * I2CScanner.h
 * I2C总线扫描工具 - 用于诊断AS5600等设备连接问题
 */

#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

/**
 * 扫描I2C总线上的所有设备
 * 显示发现的设备地址和识别信息
 */
void scanI2CDevices();

/**
 * 测试AS5600设备的通信
 * 读取状态寄存器和角度数据
 */
void testAS5600Communication();

/**
 * 检查I2C总线健康状态
 * 验证引脚状态和总线配置
 */
void checkI2CBusHealth();

#endif // I2C_SCANNER_H
