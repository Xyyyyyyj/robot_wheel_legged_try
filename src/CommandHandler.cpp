/**
 * 命令处理器实现
 */

#include "CommandHandler.h"

// 前向声明（避免循环引用）
extern void printHelp();

CommandHandler::CommandHandler() : cmdQueue(nullptr) {
}

bool CommandHandler::begin() {
    cmdQueue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(Command));
    if (!cmdQueue) {
        Serial.printf("[CMD] 错误: 命令队列创建失败\n");
        return false;
    }
    Serial.printf("[CMD] 命令队列已创建 (大小:%d)\n", CMD_QUEUE_SIZE);
    return true;
}

bool CommandHandler::sendCommand(const Command& cmd, TickType_t timeout) {
    if (cmdQueue && xQueueSend(cmdQueue, &cmd, timeout) == pdTRUE) {
        return true;
    }
    Serial.printf("[CMD] 警告: 命令队列已满\n");
    return false;
}

bool CommandHandler::receiveCommand(Command& cmd, TickType_t timeout) {
    if (cmdQueue && xQueueReceive(cmdQueue, &cmd, timeout) == pdTRUE) {
        return true;
    }
    return false;
}

// 解析浮点数
static void parseFloat(const uint8_t* data, size_t offset, float& value) {
    memcpy(&value, &data[offset], sizeof(float));
}

void CommandHandler::parseBLEData(const uint8_t* data, size_t len) {
    Serial.printf("===== BLE数据 [%d字节] =====\n", len);
    
    Command cmd;
    
    if (len == 16) {
        // 解析16字节控制数据：期望高度、滚转角、线速度、角速度
        float height, rollAngle, linearVelocity, angularVelocity;
        parseFloat(data, 0, height);
        parseFloat(data, 4, rollAngle);
        parseFloat(data, 8, linearVelocity);
        parseFloat(data, 12, angularVelocity);
        
        Serial.printf("期望高度:%.2f, 滚转角:%.2f°, 线速度:%.2f, 角速度:%.2f\n", 
                     height, rollAngle, linearVelocity, angularVelocity);
        
        cmd.type = CMD_BLE_CONTROL_DATA;
        cmd.data.bleControlData.height = height;
        cmd.data.bleControlData.rollAngle = rollAngle;
        cmd.data.bleControlData.linearVelocity = linearVelocity;
        cmd.data.bleControlData.angularVelocity = angularVelocity;
        sendCommand(cmd);
    } else {
        Serial.printf("无效数据长度（需要16字节）\n");
    }
    
    Serial.printf("========================\n");
}

bool CommandHandler::parseSerialCommand(const String& input, Command& cmd) {
    String command = input;
    command.trim();
    
    if (command.length() == 0) return false;
    
    // 单词命令
    if (command.equalsIgnoreCase("reset")) {
        cmd.type = CMD_RESET;
        return true;
    }
    if (command.equalsIgnoreCase("status")) {
        cmd.type = CMD_SYSTEM_STATUS;
        return true;
    }
    if (command.equalsIgnoreCase("imu")) {
        cmd.type = CMD_IMU_STATUS;
        return true;
    }
    if (command.equalsIgnoreCase("motor")) {
        cmd.type = CMD_MOTOR_STATUS;
        Serial.printf("[CMD] 请求电机状态\n");
        return true;
    }
    if (command.equalsIgnoreCase("help")) {
        printHelp();
        return false;
    }
    if (command.equalsIgnoreCase("balance_status")) {
        cmd.type = CMD_BALANCE_STATUS;
        return true;
    }
    
    if (command.equalsIgnoreCase("imu_rate")) {
        cmd.type = CMD_IMU_RATE;
        return true;
    }
    
    // 带参数的命令
    int space = command.indexOf(' ');
    if (space <= 0) {
        Serial.printf("命令格式错误，输入'help'查看帮助\n");
        return false;
    }
    
    String cmd_name = command.substring(0, space);
    String params = command.substring(space + 1);
    params.trim();
    
    // 腿部位置命令: left/right x y
    if (cmd_name.equalsIgnoreCase("left") || cmd_name.equalsIgnoreCase("right")) {
        int space2 = params.indexOf(' ');
        if (space2 > 0) {
            float x = params.substring(0, space2).toFloat();
            float y = params.substring(space2 + 1).toFloat();
            cmd.type = cmd_name.equalsIgnoreCase("left") ? CMD_LEFT_LEG_POS : CMD_RIGHT_LEG_POS;
            cmd.data.position.x = x;
            cmd.data.position.y = y;
            return true;
        }
        Serial.printf("格式: %s <x> <y>\n", cmd_name.c_str());
        return false;
    }
    
    // 腿高命令: height value
    if (cmd_name.equalsIgnoreCase("height")) {
        float h = params.toFloat();
        if (h > 0) {
            cmd.type = CMD_SET_HEIGHT;
            cmd.data.legHeight.height = h;
            return true;
        }
        Serial.printf("腿高必须>0\n");
        return false;
    }
    
    
    // 目标速度命令: target_speed value
    if (cmd_name.equalsIgnoreCase("target_speed")) {
        float speed = params.toFloat();
        if (params.length() == 0 || (speed == 0.0f && !params.equals("0") && !params.equals("0.0"))) {
            Serial.printf("格式: target_speed <速度值>\n");
            Serial.printf("示例: target_speed 1.5\n");
            return false;
        }
        cmd.type = CMD_TARGET_SPEED;
        cmd.data.targetSpeed.speed = speed;
        Serial.printf("[CMD] 解析目标速度命令: %.2f\n", speed);
        return true;
    }
    
    // 平衡控制输出限制命令: balance_limit value
    if (cmd_name.equalsIgnoreCase("balance_limit")) {
        float limit = params.toFloat();
        if (params.length() == 0 || limit <= 0.0f) {
            Serial.printf("格式: balance_limit <限制值>\n");
            Serial.printf("示例: balance_limit 5.0 (设置输出限制为±5.0)\n");
            return false;
        }
        cmd.type = CMD_BALANCE_LIMIT;
        cmd.data.balanceLimit.limit = limit;
        Serial.printf("[CMD] 解析平衡控制输出限制命令: ±%.2f\n", limit);
        return true;
    }
    
    // 速度环PID命令: speed_pid P I D
    if (cmd_name.equalsIgnoreCase("speed_pid")) {
        int space1 = params.indexOf(' ');
        if (space1 > 0) {
            int space2 = params.indexOf(' ', space1 + 1);
            if (space2 > 0) {
                float P = params.substring(0, space1).toFloat();
                float I = params.substring(space1 + 1, space2).toFloat();
                float D = params.substring(space2 + 1).toFloat();
                cmd.type = CMD_SPEED_PID;
                cmd.data.pidParams.P = P;
                cmd.data.pidParams.I = I;
                cmd.data.pidParams.D = D;
                Serial.printf("[CMD] 解析速度环PID命令: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
                return true;
            }
        }
        Serial.printf("格式: speed_pid <P> <I> <D>\n");
        Serial.printf("示例: speed_pid 0.5 0.1 0.01\n");
        return false;
    }
    
    // 角度环PID命令: angle_pid P I D
    if (cmd_name.equalsIgnoreCase("angle_pid")) {
        int space1 = params.indexOf(' ');
        if (space1 > 0) {
            int space2 = params.indexOf(' ', space1 + 1);
            if (space2 > 0) {
                float P = params.substring(0, space1).toFloat();
                float I = params.substring(space1 + 1, space2).toFloat();
                float D = params.substring(space2 + 1).toFloat();
                cmd.type = CMD_ANGLE_PID;
                cmd.data.pidParams.P = P;
                cmd.data.pidParams.I = I;
                cmd.data.pidParams.D = D;
                Serial.printf("[CMD] 解析角度环PID命令: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
                return true;
            }
        }
        Serial.printf("格式: angle_pid <P> <I> <D>\n");
        Serial.printf("示例: angle_pid 2.0 0.0 0.1\n");
        return false;
    }
    
    // 转向环PID命令: steering_pid P I D
    if (cmd_name.equalsIgnoreCase("steering_pid")) {
        int space1 = params.indexOf(' ');
        if (space1 > 0) {
            int space2 = params.indexOf(' ', space1 + 1);
            if (space2 > 0) {
                float P = params.substring(0, space1).toFloat();
                float I = params.substring(space1 + 1, space2).toFloat();
                float D = params.substring(space2 + 1).toFloat();
                cmd.type = CMD_STEERING_PID;
                cmd.data.pidParams.P = P;
                cmd.data.pidParams.I = I;
                cmd.data.pidParams.D = D;
                Serial.printf("[CMD] 解析转向环PID命令: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
                return true;
            }
        }
        Serial.printf("格式: steering_pid <P> <I> <D>\n");
        Serial.printf("示例: steering_pid 0.8 0.0 0.02\n");
        return false;
    }
    
    // 目标转向速度命令: target_yaw_rate value
    if (cmd_name.equalsIgnoreCase("target_yaw_rate") || cmd_name.equalsIgnoreCase("yaw_rate")) {
        float yawRate = params.toFloat();
        if (params.length() == 0 || (yawRate == 0.0f && !params.equals("0") && !params.equals("0.0"))) {
            Serial.printf("格式: target_yaw_rate <转向速度值>\n");
            Serial.printf("示例: target_yaw_rate 30.0 (设置目标转向速度为30°/s)\n");
            return false;
        }
        cmd.type = CMD_TARGET_YAW_RATE;
        cmd.data.targetYawRate.yawRate = yawRate;
        Serial.printf("[CMD] 解析目标转向速度命令: %.2f°/s\n", yawRate);
        return true;
    }
    
    // 偏移修正PID命令: offset_pid P I D
    if (cmd_name.equalsIgnoreCase("offset_pid")) {
        int space1 = params.indexOf(' ');
        if (space1 > 0) {
            int space2 = params.indexOf(' ', space1 + 1);
            if (space2 > 0) {
                float P = params.substring(0, space1).toFloat();
                float I = params.substring(space1 + 1, space2).toFloat();
                float D = params.substring(space2 + 1).toFloat();
                cmd.type = CMD_OFFSET_PID;
                cmd.data.pidParams.P = P;
                cmd.data.pidParams.I = I;
                cmd.data.pidParams.D = D;
                Serial.printf("[CMD] 解析偏移修正PID命令: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
                return true;
            }
        }
        Serial.printf("格式: offset_pid <P> <I> <D>\n");
        Serial.printf("示例: offset_pid 0.01 0.001 0.0\n");
        return false;
    }
    
    // 滚转PID命令: roll_pid P I D
    if (cmd_name.equalsIgnoreCase("roll_pid")) {
        int space1 = params.indexOf(' ');
        if (space1 > 0) {
            int space2 = params.indexOf(' ', space1 + 1);
            if (space2 > 0) {
                float P = params.substring(0, space1).toFloat();
                float I = params.substring(space1 + 1, space2).toFloat();
                float D = params.substring(space2 + 1).toFloat();
                cmd.type = CMD_ROLL_PID;
                cmd.data.pidParams.P = P;
                cmd.data.pidParams.I = I;
                cmd.data.pidParams.D = D;
                Serial.printf("[CMD] 解析滚转PID命令: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
                return true;
            }
        }
        Serial.printf("格式: roll_pid <P> <I> <D>\n");
        Serial.printf("示例: roll_pid 0.01 0.0 0.001\n");
        return false;
    }
    
    // 自动偏移修正使能命令: auto_offset on/off
    if (cmd_name.equalsIgnoreCase("auto_offset")) {
        bool enabled;
        if (params.equalsIgnoreCase("on") || params.equalsIgnoreCase("true") || params.equals("1")) {
            enabled = true;
        } else if (params.equalsIgnoreCase("off") || params.equalsIgnoreCase("false") || params.equals("0")) {
            enabled = false;
        } else {
            Serial.printf("格式: auto_offset <on/off>\n");
            Serial.printf("示例: auto_offset on (启用自动偏移修正)\n");
            Serial.printf("示例: auto_offset off (禁用自动偏移修正)\n");
            return false;
        }
        cmd.type = CMD_AUTO_OFFSET;
        cmd.data.autoOffset.enabled = enabled;
        Serial.printf("[CMD] 解析自动偏移修正命令: %s\n", enabled ? "启用" : "禁用");
        return true;
    }
    
    // 速度死区命令: speed_dead_zone value
    if (cmd_name.equalsIgnoreCase("speed_dead_zone") || cmd_name.equalsIgnoreCase("dead_zone")) {
        float deadZone = params.toFloat();
        if (params.length() == 0 || deadZone < 0.0f) {
            Serial.printf("格式: speed_dead_zone <死区值>\n");
            Serial.printf("示例: speed_dead_zone 0.5 (设置速度死区为±0.5 RPM)\n");
            return false;
        }
        cmd.type = CMD_SPEED_DEAD_ZONE;
        cmd.data.speedDeadZone.deadZone = deadZone;
        Serial.printf("[CMD] 解析速度死区命令: ±%.2f RPM\n", deadZone);
        return true;
    }
    
    // 基础角度偏移量命令: base_angle_offset value
    if (cmd_name.equalsIgnoreCase("base_angle_offset") || cmd_name.equalsIgnoreCase("base_angle")) {
        float offset = params.toFloat();
        if (params.length() == 0 || (offset == 0.0f && !params.equals("0") && !params.equals("0.0"))) {
            Serial.printf("格式: base_angle_offset <角度偏移值>\n");
            Serial.printf("示例: base_angle_offset -170.0 (设置基础角度偏移为-170°)\n");
            return false;
        }
        cmd.type = CMD_BASE_ANGLE_OFFSET;
        cmd.data.baseAngleOffset.offset = offset;
        Serial.printf("[CMD] 解析基础角度偏移命令: %.2f°\n", offset);
        return true;
    }
    
    // 滚转角命令: roll_angle value
    if (cmd_name.equalsIgnoreCase("roll_angle") || cmd_name.equalsIgnoreCase("roll")) {
        float angle = params.toFloat();
        if (params.length() == 0 || (angle == 0.0f && !params.equals("0") && !params.equals("0.0"))) {
            Serial.printf("格式: roll_angle <滚转角度值>\n");
            Serial.printf("示例: roll_angle 5.0 (设置滚转角为5°)\n");
            Serial.printf("示例: roll_angle 0 (设置滚转角为0°，恢复水平)\n");
            return false;
        }
        cmd.type = CMD_ROLL_ANGLE;
        cmd.data.rollAngle.angle = angle;
        Serial.printf("[CMD] 解析滚转角命令: %.2f°\n", angle);
        return true;
    }
    
    // 变PID使能命令: adaptive_pid on/off
    if (cmd_name.equalsIgnoreCase("adaptive_pid") || cmd_name.equalsIgnoreCase("var_pid")) {
        bool enabled;
        if (params.equalsIgnoreCase("on") || params.equalsIgnoreCase("true") || params.equals("1")) {
            enabled = true;
        } else if (params.equalsIgnoreCase("off") || params.equalsIgnoreCase("false") || params.equals("0")) {
            enabled = false;
        } else {
            Serial.printf("格式: adaptive_pid <on/off>\n");
            Serial.printf("示例: adaptive_pid on (启用变PID)\n");
            Serial.printf("示例: adaptive_pid off (禁用变PID)\n");
            return false;
        }
        cmd.type = CMD_ADAPTIVE_PID;
        cmd.data.adaptivePID.enabled = enabled;
        Serial.printf("[CMD] 解析变PID命令: %s\n", enabled ? "启用" : "禁用");
        return true;
    }
    
    // 校准点命令: calib index height speedP speedI speedD angleP angleI angleD
    if (cmd_name.equalsIgnoreCase("calib") || cmd_name.equalsIgnoreCase("calibration")) {
        // 解析参数: index height speedP speedI speedD angleP angleI angleD
        int idx[8];
        idx[0] = params.indexOf(' ');
        for (int i = 1; i < 8; i++) {
            if (idx[i-1] > 0) {
                idx[i] = params.indexOf(' ', idx[i-1] + 1);
            } else {
                idx[i] = -1;
            }
        }
        
        if (idx[6] > 0) {
            int index = params.substring(0, idx[0]).toInt();
            float height = params.substring(idx[0] + 1, idx[1]).toFloat();
            float speedP = params.substring(idx[1] + 1, idx[2]).toFloat();
            float speedI = params.substring(idx[2] + 1, idx[3]).toFloat();
            float speedD = params.substring(idx[3] + 1, idx[4]).toFloat();
            float angleP = params.substring(idx[4] + 1, idx[5]).toFloat();
            float angleI = params.substring(idx[5] + 1, idx[6]).toFloat();
            float angleD = params.substring(idx[6] + 1).toFloat();
            
            cmd.type = CMD_CALIBRATION_POINT;
            cmd.data.calibrationPoint.index = index;
            cmd.data.calibrationPoint.height = height;
            cmd.data.calibrationPoint.speedP = speedP;
            cmd.data.calibrationPoint.speedI = speedI;
            cmd.data.calibrationPoint.speedD = speedD;
            cmd.data.calibrationPoint.angleP = angleP;
            cmd.data.calibrationPoint.angleI = angleI;
            cmd.data.calibrationPoint.angleD = angleD;
            
            Serial.printf("[CMD] 解析校准点命令: 索引=%d, 高度=%.1f, 速度PID[%.3f,%.3f,%.3f], 角度PID[%.3f,%.3f,%.3f]\n",
                         index, height, speedP, speedI, speedD, angleP, angleI, angleD);
            return true;
        }
        Serial.printf("格式: calib <索引> <高度> <速度P> <速度I> <速度D> <角度P> <角度I> <角度D>\n");
        Serial.printf("示例: calib 0 150 0.30 0.000 0.005 1.2 0.000 0.005\n");
        return false;
    }
    
    // 舵机命令: servo_num angle
    int servoNum = cmd_name.toInt();
    if (servoNum >= 1 && servoNum <= 4) {
        cmd.type = CMD_SERVO_ANGLE;
        cmd.data.servoAngle.servoNum = servoNum;
        cmd.data.servoAngle.angle = params.toFloat();
        return true;
    }
    
    Serial.printf("未知命令，输入'help'查看帮助\n");
    return false;
}

