# MiniWheel 轮腿机器人

<div align="center">
  <img src="https://img.shields.io/badge/version-v2.1-blue" alt="version">
  <img src="https://img.shields.io/badge/platform-ESP32--S3-green" alt="platform">
  <img src="https://img.shields.io/badge/license-MIT-orange" alt="license">
</div>

基于 ESP32-S3 的五连杆轮腿机器人控制系统，采用模块化设计，支持 BLE 无线控制和 IMU 姿态检测。

## 📋 目录

- [特性](#-特性)
- [硬件配置](#-硬件配置)
- [项目结构](#-项目结构)
- [快速开始](#-快速开始)
- [使用说明](#-使用说明)
- [开发文档](#-开发文档)
- [许可证](#-许可证)

## ✨ 特性

- 🦾 **五连杆轮腿系统** - 完整的正逆运动学求解
- 🚗 **差速驱动模型** - 支持线速度和角速度控制
- 📡 **BLE 无线控制** - 通过蓝牙低功耗协议远程控制
- 📐 **IMU 姿态检测** - MPU6050 六轴传感器实时监测
- ⚡ **FreeRTOS 多任务** - 高效的实时任务调度
- 🧩 **模块化设计** - 清晰的代码架构，易于扩展

## 🔧 硬件配置

### 主控
- **MCU**: ESP32-S3-DevKitM-1
- **核心**: 双核 Xtensa LX7 @ 240MHz
- **内存**: 512KB SRAM + 8MB PSRAM

### 传感器
- **IMU**: MPU6050 (I2C: 0x68)
  - SDA: GPIO 17
  - SCL: GPIO 18

### 执行器
- **舵机**: 2×270° + 2×180°
  - 舵机1 (270°): GPIO 12
  - 舵机2 (180°): GPIO 13
  - 舵机3 (270°): GPIO 15
  - 舵机4 (180°): GPIO 16

### 机械参数
- 轮距: 150mm
- 轮半径: 54mm
- 连杆长度: L1=60mm, L2=100mm, L3=100mm, L4=60mm, L5=47mm

## 📁 项目结构

```
miniwheel_legged_robot_esp/
├── include/                    # 头文件目录
│   ├── Config.h               # 系统配置（引脚、参数）
│   ├── BLEController.h        # BLE 通信模块
│   ├── IMUManager.h           # IMU 管理模块
│   ├── CommandHandler.h       # 命令处理模块
│   ├── DifferentialDrive.h    # 差速驱动模型
│   └── WheelLeg.h             # 五连杆轮腿控制
├── src/                       # 源文件目录
│   ├── main.cpp               # 主程序（重构版）
│   ├── BLEController.cpp      # BLE 实现
│   ├── IMUManager.cpp         # IMU 实现
│   ├── CommandHandler.cpp     # 命令处理实现
│   ├── DifferentialDrive.cpp  # 差速驱动实现
│   └── WheelLeg.cpp           # 轮腿控制实现
├── platformio.ini             # PlatformIO 配置
└── README.md                  # 项目文档
```

### 模块说明

| 模块 | 功能 | 文件 |
|------|------|------|
| **Config** | 统一配置管理 | `Config.h` |
| **BLEController** | BLE 通信管理 | `BLEController.h/cpp` |
| **IMUManager** | IMU 数据采集 | `IMUManager.h/cpp` |
| **CommandHandler** | 命令解析与队列 | `CommandHandler.h/cpp` |
| **DifferentialDrive** | 两轮差速驱动 | `DifferentialDrive.h/cpp` |
| **WheelLeg** | 五连杆运动学 | `WheelLeg.h/cpp` |

## 🚀 快速开始

### 1. 环境准备

安装 PlatformIO:
```bash
# 方法1: VS Code 插件
# 在 VS Code 中搜索并安装 "PlatformIO IDE"

# 方法2: 命令行安装
pip install platformio
```

### 2. 克隆项目

```bash
git clone <your-repo-url>
cd miniwheel_legged_robot_esp
```

### 3. 编译上传

```bash
# 编译
pio run

# 上传到开发板
pio run --target upload

# 打开串口监视器
pio device monitor
```

## 📖 使用说明

### 串口命令

连接串口（波特率 115200），输入以下命令：

| 命令 | 参数 | 说明 | 示例 |
|------|------|------|------|
| `help` | - | 显示帮助信息 | `help` |
| `status` | - | 显示系统状态 | `status` |
| `imu` | - | 显示 IMU 状态 | `imu` |
| `reset` | - | 重置所有舵机 | `reset` |
| `<1-4>` | `<angle>` | 设置舵机角度 | `1 90` |
| `left` | `<x> <y>` | 左腿位置控制 | `left 24 -100` |
| `right` | `<x> <y>` | 右腿位置控制 | `right 24 -100` |
| `height` | `<mm>` | 设置腿高 | `height 120` |

### BLE 控制

**服务 UUID**: `00001000-0000-1000-8000-00805f9b34fb`  
**特征 UUID**: `00001001-0000-1000-8000-00805f9b34fb`

#### 数据格式

| 长度 | 内容 | 数据类型 |
|------|------|----------|
| 4 字节 | 腿高 | `float` |
| 8 字节 | 线速度 + 角速度 | `float + float` |
| 12 字节 | 线速度 + 角速度 + 腿高 | `float + float + float` |

#### Python 示例

```python
import struct
from bluepy import btle

# 连接设备
dev = btle.Peripheral("MAC_ADDRESS")
char = dev.getCharacteristics(uuid="00001001-0000-1000-8000-00805f9b34fb")[0]

# 发送速度控制 (线速度 0.2 m/s, 角速度 0.5 rad/s)
data = struct.pack('<ff', 0.2, 0.5)
char.write(data)

# 发送腿高控制 (120mm)
data = struct.pack('<f', 120.0)
char.write(data)
```

## 🛠 开发文档

### 添加新模块

1. 在 `include/` 创建头文件
2. 在 `src/` 创建实现文件
3. 在 `Config.h` 添加相关配置
4. 在 `main.cpp` 集成模块

### FreeRTOS 任务

| 任务名 | 优先级 | 核心 | 功能 |
|--------|--------|------|------|
| `Serial` | 2 | 0 | 串口命令解析 |
| `Motor` | 3 | 1 | 电机控制执行 |
| `IMU` | 2 | 0 | IMU 数据采集 |

### 修改配置

所有配置集中在 `include/Config.h`：

```cpp
// 修改引脚定义
#define SERVO1_PIN 12

// 修改机械参数
#define WHEEL_BASE 0.15f

// 修改任务参数
#define TASK_PRIORITY_MOTOR 3
```

## 🔄 版本历史

### v2.1 (当前版本)
- ✅ 完整模块化重构
- ✅ 分离 BLE、IMU、命令处理模块
- ✅ 统一配置管理
- ✅ 优化代码结构

### v2.0
- 添加 FreeRTOS 多任务支持
- 集成 MPU6050 IMU
- BLE 无线控制

### v1.0
- 基础五连杆控制
- 串口命令系统

## 📝 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📧 联系方式

如有问题或建议，请通过以下方式联系：

- Issue: [GitHub Issues](<your-repo-url>/issues)
- Email: <your-email>

---

<div align="center">
  Made with ❤️ by MiniWheel Team
</div>

