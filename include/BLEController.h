/**
 * BLE控制器
 * 封装BLE通信功能
 */

#ifndef BLE_CONTROLLER_H
#define BLE_CONTROLLER_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "Config.h"

// BLE数据回调函数类型
typedef void (*BLEDataCallback)(const uint8_t* data, size_t len);

// BLE状态数据1：电机和平衡数据（16字节）
struct BLEStatusData1 {
    float motor1RPM;           // 电机1转速 (RPM)
    float motor2RPM;           // 电机2转速 (RPM)
    float balanceAngleOffset;  // 平衡偏移角 (度)
    float leftOutput;          // 左轮输出
};

// BLE状态数据2：速度和姿态数据（16字节）
struct BLEStatusData2 {
    float rightOutput;         // 右轮输出
    float currentSpeed;        // 当前速度
    float angleX;              // X轴角度
    float gyroZ;               // Z轴角速度
};

class BLEController {
private:
    NimBLEServer* pServer;
    NimBLEService* pService;
    NimBLECharacteristic* pCharacteristic;
    NimBLECharacteristic* pStatusCharacteristic1;  // 状态数据特征值1（电机+平衡）
    NimBLECharacteristic* pStatusCharacteristic2;  // 状态数据特征值2（速度+姿态）
    BLEDataCallback dataCallback;
    
    // 服务器回调类（内部类）
    class ServerCallbacks : public NimBLEServerCallbacks {
    public:
        void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
        void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;
    };
    
    // 特征回调类（内部类）
    class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    private:
        BLEController* controller;
    public:
        CharacteristicCallbacks(BLEController* ctrl) : controller(ctrl) {}
        void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override;
    };
    
    ServerCallbacks serverCallbacks;
    CharacteristicCallbacks* chrCallbacks;
    
public:
    BLEController();
    ~BLEController();
    
    // 初始化BLE
    bool begin(const char* deviceName = BLE_DEVICE_NAME);
    
    // 设置数据接收回调函数
    void setDataCallback(BLEDataCallback callback);
    
    // 发送数据（写入特征值）
    bool sendData(const uint8_t* data, size_t len);
    
    // 更新状态数据1（电机+平衡）
    bool updateStatusData1(const BLEStatusData1& statusData);
    
    // 更新状态数据2（速度+姿态）
    bool updateStatusData2(const BLEStatusData2& statusData);
    
    // 检查是否有设备连接
    bool isConnected();
};

#endif // BLE_CONTROLLER_H

