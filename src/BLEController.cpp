/**
 * BLE控制器实现
 */

#include "BLEController.h"

// ==================== ServerCallbacks 实现 ====================
void BLEController::ServerCallbacks::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    Serial.printf("[BLE] 已连接: %s\n", connInfo.getAddress().toString().c_str());
}

void BLEController::ServerCallbacks::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    Serial.printf("[BLE] 断开连接 (原因:%d)，重新广播\n", reason);
    NimBLEDevice::startAdvertising();
}

// ==================== CharacteristicCallbacks 实现 ====================
void BLEController::CharacteristicCallbacks::onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) {
    std::string value = pCharacteristic->getValue();
    const uint8_t* data = (const uint8_t*)value.data();
    size_t len = value.length();
    
    Serial.printf("[BLE] 收到数据 [%d字节]\n", len);
    
    // 调用回调函数
    if (controller && controller->dataCallback) {
        controller->dataCallback(data, len);
    }
}

// ==================== BLEController 实现 ====================
BLEController::BLEController() 
    : pServer(nullptr), pService(nullptr), pCharacteristic(nullptr), 
      pStatusCharacteristic1(nullptr), pStatusCharacteristic2(nullptr),
      dataCallback(nullptr), chrCallbacks(nullptr) {
}

BLEController::~BLEController() {
    if (chrCallbacks) {
        delete chrCallbacks;
    }
}

bool BLEController::begin(const char* deviceName) {
    Serial.printf("[BLE] 初始化设备: %s\n", deviceName);
    
    NimBLEDevice::init(deviceName);
    
    // 创建服务器
    pServer = NimBLEDevice::createServer();
    if (!pServer) {
        Serial.printf("[BLE] 错误: 服务器创建失败\n");
        return false;
    }
    pServer->setCallbacks(&serverCallbacks);
    
    // 创建服务
    pService = pServer->createService(BLE_SERVICE_UUID);
    if (!pService) {
        Serial.printf("[BLE] 错误: 服务创建失败\n");
        return false;
    }
    
    // 创建控制特征（可读可写）
    pCharacteristic = pService->createCharacteristic(
        BLE_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
    );
    if (!pCharacteristic) {
        Serial.printf("[BLE] 错误: 控制特征创建失败\n");
        return false;
    }
    
    // 设置特征回调
    chrCallbacks = new CharacteristicCallbacks(this);
    pCharacteristic->setValue("0");
    pCharacteristic->setCallbacks(chrCallbacks);
    
    // 创建状态特征1（电机+平衡，只读，支持通知）
    pStatusCharacteristic1 = pService->createCharacteristic(
        BLE_STATUS_CHAR1_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    if (!pStatusCharacteristic1) {
        Serial.printf("[BLE] 错误: 状态特征1创建失败\n");
        return false;
    }
    
    // 初始化状态特征1值
    BLEStatusData1 initData1 = {0};
    pStatusCharacteristic1->setValue((uint8_t*)&initData1, sizeof(BLEStatusData1));
    
    // 创建状态特征2（速度+姿态，只读，支持通知）
    pStatusCharacteristic2 = pService->createCharacteristic(
        BLE_STATUS_CHAR2_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    if (!pStatusCharacteristic2) {
        Serial.printf("[BLE] 错误: 状态特征2创建失败\n");
        return false;
    }
    
    // 初始化状态特征2值
    BLEStatusData2 initData2 = {0};
    pStatusCharacteristic2->setValue((uint8_t*)&initData2, sizeof(BLEStatusData2));
    
    // 启动服务
    pService->start();
    
    // 启动广播
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setName(deviceName);
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->start();
    
    Serial.printf("[BLE] 已启动 ✓\n");
    return true;
}

void BLEController::setDataCallback(BLEDataCallback callback) {
    dataCallback = callback;
}

bool BLEController::sendData(const uint8_t* data, size_t len) {
    if (!pCharacteristic) {
        return false;
    }
    
    pCharacteristic->setValue(data, len);
    pCharacteristic->notify();
    return true;
}

bool BLEController::updateStatusData1(const BLEStatusData1& statusData) {
    if (!pStatusCharacteristic1) {
        return false;
    }
    
    // 更新状态特征值1
    pStatusCharacteristic1->setValue((uint8_t*)&statusData, sizeof(BLEStatusData1));
    
    // 如果有设备连接，发送通知
    if (isConnected()) {
        pStatusCharacteristic1->notify();
    }
    
    return true;
}

bool BLEController::updateStatusData2(const BLEStatusData2& statusData) {
    if (!pStatusCharacteristic2) {
        return false;
    }
    
    // 更新状态特征值2
    pStatusCharacteristic2->setValue((uint8_t*)&statusData, sizeof(BLEStatusData2));
    
    // 如果有设备连接，发送通知
    if (isConnected()) {
        pStatusCharacteristic2->notify();
    }
    
    return true;
}

bool BLEController::isConnected() {
    if (!pServer) {
        return false;
    }
    return pServer->getConnectedCount() > 0;
}

