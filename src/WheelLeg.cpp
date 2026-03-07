/**
 * 五连杆轮腿类实现
 */

#include "WheelLeg.h"
#include "Config.h"

// 构造函数
WheelLeg::WheelLeg(int pin1, int pin2, int pin3, int pin4) 
    : speedPID(BALANCE_SPEED_PID_P, BALANCE_SPEED_PID_I, BALANCE_SPEED_PID_D, 0, 1000.0f),
      anglePID(BALANCE_ANGLE_PID_P, BALANCE_ANGLE_PID_I, BALANCE_ANGLE_PID_D, 0, BALANCE_OUTPUT_LIMIT),
      steeringPID(STEERING_PID_P, STEERING_PID_I, STEERING_PID_D, 0, 10.0f),
      offsetPID(OFFSET_PID_P, OFFSET_PID_I, OFFSET_PID_D, 0, 2.0f),
      rollPID(ROLL_PID_P, ROLL_PID_I, ROLL_PID_D, 0, 5.0f),
      // LQR 风格多 PID 控制器初始化（参数来源于 3.Software，输出限制与角度环一致）
      lqrAnglePID(LQR_ANGLE_PID_P, LQR_ANGLE_PID_I, LQR_ANGLE_PID_D, 0, BALANCE_OUTPUT_LIMIT),
      lqrGyroPID(LQR_GYRO_PID_P, LQR_GYRO_PID_I, LQR_GYRO_PID_D, 0, BALANCE_OUTPUT_LIMIT),
      lqrDistancePID(LQR_DISTANCE_PID_P, LQR_DISTANCE_PID_I, LQR_DISTANCE_PID_D, 0, BALANCE_OUTPUT_LIMIT),
      lqrSpeedPID(LQR_SPEED_PID_P, LQR_SPEED_PID_I, LQR_SPEED_PID_D, 0, BALANCE_OUTPUT_LIMIT),
      lqrUPID(LQR_U_PID_P, LQR_U_PID_I, LQR_U_PID_D, 0, BALANCE_OUTPUT_LIMIT),
      lqrZeroPID(LQR_ZERO_PID_P, LQR_ZERO_PID_I, LQR_ZERO_PID_D, 0, 10.0f) {
    servo1Pin = pin1;
    servo2Pin = pin2;
    servo3Pin = pin3;
    servo4Pin = pin4;
    
    // 初始化角度为0度
    servoAngles[0] = 0.0f;
    servoAngles[1] = 0.0f;
    servoAngles[2] = 0.0f;
    servoAngles[3] = 0.0f;
    
    // 舵机零位角度
    zeroPositions[0] = 90;
    zeroPositions[1] = 90;
    zeroPositions[2] = 90;
    zeroPositions[3] = 90;
    
    // 舵机类型 (true: 270度, false: 180度)
    is270Servo[0] = false;  // 舵机1为180度
    is270Servo[1] = false;  // 舵机2为180度
    is270Servo[2] = false;  // 舵机3为180度
    is270Servo[3] = false;  // 舵机4为180度
    
    // 五连杆参数 (默认值，可通过配置修改)
    // 闭环五连杆机构：
    // L1: 舵机1到固定点1
    // L2: 固定点1到轮子
    // L3: 轮子到固定点2
    // L4: 固定点2到舵机2
    // L5: 舵机2到舵机1
    linkageParams.L1 = 60.0f;   // mm
    linkageParams.L2 = 100.0f;  // mm
    linkageParams.L3 = 100.0f;  // mm
    linkageParams.L4 = 60.0f;   // mm
    linkageParams.L5 = 47.0f;   // mm (两舵机之间的距离)
    
    // 初始高度设置
    initialHeight = INITIAL_LEG_HEIGHT;  // 轮子末端初始高度
    initialX = INITIAL_LEG_X;            // 轮子末端初始X坐标
    
    // 平衡控制参数初始化
    targetSpeed = 0.0f;      // 目标速度初始化为0
    targetSpeedFiltered = 0.0f; // 经过斜坡滤波后的目标速度
    speedRampRate = 10.0f;   // 默认速度斜坡变化率 20 RPM/s
    lastSpeedUpdateTime = millis();
    currentSpeed = 0.0f;     // 当前速度初始化为0
    balanceEnabled = true;   // 平衡控制默认启用
    balanceOutputLimit = BALANCE_OUTPUT_LIMIT;  // 平衡控制输出限制
    baseAngleOffset = BASE_ANGLE_OFFSET;  // 基础角度偏移量初始化
    
    // 转向控制参数初始化
    targetYawRate = 0.0f;    // 目标转向速度初始化为0
    currentYawRate = 0.0f;   // 当前转向速度初始化为0
    steeringEnabled = true;  // 转向控制默认启用
    
    // 滚转控制参数初始化
    targetRollAngle = 0.0f;   // 目标滚转角初始化为0
    rollAngleOffset = 0.0f;   // 滚转偏移角初始化为0
    rollControlEnabled = false; // 滚转控制默认启用
    
    // 自动偏移修正参数初始化
    baseAngleOffsetCorrection = 0.0f;  // 偏移修正值初始化为0
    autoOffsetEnabled = true;          // 自动偏移修正默认启用
    speedDeadZone = SPEED_DEAD_ZONE;   // 速度死区范围

    offsetFrozen = false;
    lastHeightForOffset = initialHeight;
    lastBaseAngleOffsetCorrection = baseAngleOffsetCorrection;
    lastOffsetChangeTime = millis();
    
    // 变PID控制参数初始化
    adaptivePIDEnabled = true;  // 默认启用变PID
    
    // PID平滑过渡参数初始化
    pidTransitioning = false;
    pidTransitionStartTime = 0;
    pidTransitionDuration = 500.0f;  // 默认500ms过渡时间
    targetSpeedP = targetSpeedI = targetSpeedD = 0.0f;
    targetAngleP = targetAngleI = targetAngleD = 0.0f;
    startSpeedP = startSpeedI = startSpeedD = 0.0f;
    startAngleP = startAngleI = startAngleD = 0.0f;
    
    // 初始化校准点（腿高 -> PID参数）
    // 腿高150mm
    calibrationPoints[0].height = 150.0f;
    calibrationPoints[0].speedP = 0.31f;
    calibrationPoints[0].speedI = 0.000f;
    calibrationPoints[0].speedD = 0.005f;
    calibrationPoints[0].angleP = 0.9f;
    calibrationPoints[0].angleI = 0.000f;
    calibrationPoints[0].angleD = 0.005f;
    
    // 腿高100mm
    calibrationPoints[1].height = 100.0f;
    calibrationPoints[1].speedP = 0.250f;
    calibrationPoints[1].speedI = 0.000f;
    calibrationPoints[1].speedD = 0.005f;
    calibrationPoints[1].angleP = 1.0f;
    calibrationPoints[1].angleI = 0.000f;
    calibrationPoints[1].angleD = 0.005f;
    
    // 腿高80mm
    calibrationPoints[2].height = 80.0f;
    calibrationPoints[2].speedP = 0.20f;
    calibrationPoints[2].speedI = 0.000f;
    calibrationPoints[2].speedD = 0.005f;
    calibrationPoints[2].angleP = 0.7f;
    calibrationPoints[2].angleI = 0.000f;
    calibrationPoints[2].angleD = 0.005f;
    
    // SimpleFOC PID控制器已在初始化列表中完成初始化
    // speedPID: 速度环PID（外环）- 输出目标角度，限制±10度
    // anglePID: 角度环PID（内环）- 输出电机扭矩，限制±BALANCE_OUTPUT_LIMIT
    // steeringPID: 转向环PID - 输出转向力矩，限制±10.0
    // offsetPID: 偏移修正PID（增量式）- 输出角度偏移增量，限制±2.0度
    // 额外的 LQR 风格 PID 控制器用于在姿态环内部实现类似 3.Software 的 LQR+PID 组合

    // LQR-PID 控制状态初始化
    lqrAngleZeroPoint = 0.0f;       // 初始认为直立基准即零点，后续通过自适应慢慢修正
    lqrDistanceZeroPoint = 0.0f;    // 位移零点（使用积分位移衡量）
    lqrRobotSpeed = 0.0f;
    lqrRobotSpeedLast = 0.0f;
    lqrMoveStopFlag = false;
}

// 配置五连杆参数
void WheelLeg::setLinkageParams(float L1, float L2, float L3, float L4, float L5) {
    linkageParams.L1 = L1;
    linkageParams.L2 = L2;
    linkageParams.L3 = L3;
    linkageParams.L4 = L4;
    linkageParams.L5 = L5;
    Serial.printf("五连杆参数已更新: L1=%.2f, L2=%.2f, L3=%.2f, L4=%.2f, L5=%.2f\n",
                 L1, L2, L3, L4, L5);
}

// 设置初始高度和X坐标
void WheelLeg::setInitialPosition(float x, float height) {
    // 验证高度范围
    if (height < MIN_HEIGHT || height > MAX_HEIGHT) {
        Serial.printf("警告: 目标高度 %.2f mm 超出范围 (%.0f-%.0f mm)，已限制到范围内\n", 
                     height, MIN_HEIGHT, MAX_HEIGHT);
        height = constrain(height, MIN_HEIGHT, MAX_HEIGHT);
    }
    
    initialX = x;
    initialHeight = height;

    if (abs(initialHeight - lastHeightForOffset) > HEIGHT_CHANGE_THRESHOLD) {
        offsetFrozen = false;
        offsetPID.reset();
        lastHeightForOffset = initialHeight;
        lastBaseAngleOffsetCorrection = baseAngleOffsetCorrection;
        lastOffsetChangeTime = millis();
    }
    
    // 根据新的腿高更新PID参数（如果变PID已启用）
    updatePIDForHeight(height);
    
    Serial.printf("初始位置已更新: (%.2f, -%.2f) mm\n", initialX, initialHeight);
}

// 初始化所有舵机
void WheelLeg::begin() {
    // 分配ESP32 PWM定时器
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    // 设置舵机频率
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo3.setPeriodHertz(50);
    servo4.setPeriodHertz(50);
    
    // 连接舵机到引脚
    servo1.attach(servo1Pin, 500, 2500);
    servo2.attach(servo2Pin, 500, 2500);
    servo3.attach(servo3Pin, 500, 2500);
    servo4.attach(servo4Pin, 500, 2500);
    
    Serial.printf("[轮腿] 已初始化 (引脚: %d,%d,%d,%d)\n", 
                  servo1Pin, servo2Pin, servo3Pin, servo4Pin);
    
    // 设置初始位置：轮子末端在(initialX, -initialHeight)
    Serial.printf("[轮腿] 设置初始位置: (%.2f, -%.2f) mm\n", initialX, initialHeight);
    
    // 左腿设置到初始位置
    if (setLeftLegPosition(initialX, -initialHeight)) {
        Serial.printf("[轮腿] 左腿已移动到初始位置\n");
    } else {
        Serial.printf("[轮腿] 警告: 左腿无法到达初始位置\n");
    }
    
    // 右腿设置到初始位置
    if (setRightLegPosition(initialX, -initialHeight)) {
        Serial.printf("[轮腿] 右腿已移动到初始位置\n");
    } else {
        Serial.printf("[轮腿] 警告: 右腿无法到达初始位置\n");
    }
}

// 设置单个舵机角度
bool WheelLeg::setServoAngle(int servoNum, float relativeAngle) {
    // 验证舵机号
    if (servoNum < 1 || servoNum > 4) {
        Serial.printf("错误: 舵机号必须在1-4之间\n");
        return false;
    }
    
    // 根据零位和舵机特性计算实际角度
    int actualAngle = 0;
    switch (servoNum) {
        case 1:  // 舵机1: 零位78度，正向偏移
            actualAngle = (int)(zeroPositions[0] + relativeAngle);
            break;
        case 2:  // 舵机2: 零位138度，反向偏移
            actualAngle = (int)(zeroPositions[1] - relativeAngle);
            break;
        case 3:  // 舵机3: 零位180度，反向偏移
            actualAngle = (int)(zeroPositions[2] - relativeAngle);
            break;
        case 4:  // 舵机4: 零位35度，正向偏移
            actualAngle = (int)(zeroPositions[3] + relativeAngle);
            break;
    }
    
    // 验证实际角度是否在舵机的有效范围内
    int maxAngle = is270Servo[servoNum - 1] ? 270 : 180;
    if (actualAngle < 0 || actualAngle > maxAngle) {
        Serial.printf("错误: 舵机%d 计算后的实际角度 %d 超出范围 (0-%d)\n", 
                     servoNum, actualAngle, maxAngle);
        Serial.printf("      输入角度: %.2f, 零位: %d\n", 
                     relativeAngle, zeroPositions[servoNum - 1]);
        return false;
    }
    
    // 保存相对角度
    servoAngles[servoNum - 1] = relativeAngle;
    
    // 控制对应的舵机（统一使用writeMicroseconds提高响应一致性）
    int pulseWidth;
    switch (servoNum) {
        case 1:  // 180度舵机，0-180度映射到500-2500μs
            pulseWidth = 500 + (actualAngle * 2000 / 180);
            servo1.writeMicroseconds(pulseWidth);
            break;
        case 2:  // 180度舵机，0-180度映射到500-2500μs
            pulseWidth = 500 + (actualAngle * 2000 / 180);
            servo2.writeMicroseconds(pulseWidth);
            break;
        case 3:  // 180度舵机，0-180度映射到500-2500μs
            pulseWidth = 500 + (actualAngle * 2000 / 180);
            servo3.writeMicroseconds(pulseWidth);
            break;
        case 4:  // 180度舵机，0-180度映射到500-2500μs
            pulseWidth = 500 + (actualAngle * 2000 / 180);
            servo4.writeMicroseconds(pulseWidth);
            break;
    }
    
    return true;
}

// 获取单个舵机当前角度
float WheelLeg::getServoAngle(int servoNum) {
    if (servoNum >= 1 && servoNum <= 4) {
        return servoAngles[servoNum - 1];
    }
    return -1.0f;
}

// 设置左腿末端位置
bool WheelLeg::setLeftLegPosition(float x, float y) {
    // 验证高度范围（Y坐标为负值，表示向下）
    float height = -y;  // 转换为正值高度
    if (height < MIN_HEIGHT || height > MAX_HEIGHT) {
        Serial.printf("错误: 左腿高度 %.2f mm 超出范围 (%.0f-%.0f mm)\n", 
                     height, MIN_HEIGHT, MAX_HEIGHT);
        return false;
    }
    
    float theta1, theta2;
    
    // 使用逆运动学计算舵机角度
    if (!inverseKinematics(x, y, theta1, theta2)) {
        Serial.printf("左腿: 无法到达目标位置 (%.2f, %.2f)\n", x, y);
        return false;
    }
    
    // 计算舵机实际角度并验证范围
    int actualAngle1 = (int)(zeroPositions[0] + theta1);
    int actualAngle2 = (int)(zeroPositions[1] - theta2);
    
    // 验证角度范围
    if (actualAngle1 < 0 || actualAngle1 > 180) {
        Serial.printf("错误: 舵机1 计算后的实际角度 %d 超出范围 (0-180)\n", actualAngle1);
        Serial.printf("      输入角度: %.2f, 零位: %d\n", theta1, zeroPositions[0]);
        return false;
    }
    
    if (actualAngle2 < 0 || actualAngle2 > 180) {
        Serial.printf("错误: 舵机2 计算后的实际角度 %d 超出范围 (0-180)\n", actualAngle2);
        Serial.printf("      输入角度: %.2f, 零位: %d\n", theta2, zeroPositions[1]);
        return false;
    }
    
    // 保存相对角度
    servoAngles[0] = theta1;
    servoAngles[1] = theta2;
    
    // 同时写入两个舵机（减少延迟，统一使用writeMicroseconds提高响应一致性）
    // 舵机1: 180度舵机，0-180度映射到500-2500μs
    int pulseWidth1 = 500 + (actualAngle1 * 2000 / 180);
    servo1.writeMicroseconds(pulseWidth1);
    
    // 舵机2: 180度舵机，0-180度映射到500-2500μs
    int pulseWidth2 = 500 + (actualAngle2 * 2000 / 180);
    servo2.writeMicroseconds(pulseWidth2);
    
    // 调试输出已简化
    
    return true;
}

// 设置右腿末端位置
bool WheelLeg::setRightLegPosition(float x, float y) {
    // 验证高度范围（Y坐标为负值，表示向下）
    float height = -y;  // 转换为正值高度
    if (height < MIN_HEIGHT || height > MAX_HEIGHT) {
        Serial.printf("错误: 右腿高度 %.2f mm 超出范围 (%.0f-%.0f mm)\n", 
                     height, MIN_HEIGHT, MAX_HEIGHT);
        return false;
    }
    
    float theta3, theta4;
    
    // 使用右腿逆运动学计算舵机角度
    if (!inverseKinematicsRight(x, y, theta3, theta4)) {
        Serial.printf("右腿: 无法到达目标位置 (%.2f, %.2f)\n", x, y);
        return false;
    }
    
    // 计算舵机实际角度并验证范围
    int actualAngle3 = (int)(zeroPositions[2] - theta3);
    int actualAngle4 = (int)(zeroPositions[3] + theta4);
    
    // 验证角度范围
    if (actualAngle3 < 0 || actualAngle3 > 180) {
        Serial.printf("错误: 舵机3 计算后的实际角度 %d 超出范围 (0-180)\n", actualAngle3);
        Serial.printf("      输入角度: %.2f, 零位: %d\n", theta3, zeroPositions[2]);
        return false;
    }
    
    if (actualAngle4 < 0 || actualAngle4 > 180) {
        Serial.printf("错误: 舵机4 计算后的实际角度 %d 超出范围 (0-180)\n", actualAngle4);
        Serial.printf("      输入角度: %.2f, 零位: %d\n", theta4, zeroPositions[3]);
        return false;
    }
    
    // 保存相对角度
    servoAngles[2] = theta3;
    servoAngles[3] = theta4;
    
    // 同时写入两个舵机（减少延迟，统一使用writeMicroseconds提高响应一致性）
    // 舵机3: 180度舵机，0-180度映射到500-2500μs
    int pulseWidth3 = 500 + (actualAngle3 * 2000 / 180);
    servo3.writeMicroseconds(pulseWidth3);
    
    // 舵机4: 180度舵机，0-180度映射到500-2500μs
    int pulseWidth4 = 500 + (actualAngle4 * 2000 / 180);
    servo4.writeMicroseconds(pulseWidth4);
    
    // 调试输出已简化
    
    return true;
}

// 分阶段设置两条腿位置（先设置舵机1、3，延迟后设置舵机2、4）
bool WheelLeg::setBothLegsPositionStaged(float x, float y, int delayMs) {
    // 验证高度范围（Y坐标为负值，表示向下）
    float height = -y;  // 转换为正值高度
    if (height < MIN_HEIGHT || height > MAX_HEIGHT) {
        Serial.printf("错误: 目标高度 %.2f mm 超出范围 (%.0f-%.0f mm)\n", 
                     height, MIN_HEIGHT, MAX_HEIGHT);
        return false;
    }
    
    float theta1, theta2, theta3, theta4;
    
    // 计算左腿逆运动学
    if (!inverseKinematics(x, y, theta1, theta2)) {
        Serial.printf("左腿: 无法到达目标位置 (%.2f, %.2f)\n", x, y);
        return false;
    }
    
    // 计算右腿逆运动学
    if (!inverseKinematicsRight(x, y, theta3, theta4)) {
        Serial.printf("右腿: 无法到达目标位置 (%.2f, %.2f)\n", x, y);
        return false;
    }
    
    // 计算所有舵机实际角度
    int actualAngle1 = (int)(zeroPositions[0] + theta1);
    int actualAngle2 = (int)(zeroPositions[1] - theta2);
    int actualAngle3 = (int)(zeroPositions[2] - theta3);
    int actualAngle4 = (int)(zeroPositions[3] + theta4);
    
    // 验证所有舵机角度范围
    if (actualAngle1 < 0 || actualAngle1 > 180) {
        Serial.printf("错误: 舵机1 计算后的实际角度 %d 超出范围 (0-180)\n", actualAngle1);
        return false;
    }
    if (actualAngle2 < 0 || actualAngle2 > 180) {
        Serial.printf("错误: 舵机2 计算后的实际角度 %d 超出范围 (0-180)\n", actualAngle2);
        return false;
    }
    if (actualAngle3 < 0 || actualAngle3 > 180) {
        Serial.printf("错误: 舵机3 计算后的实际角度 %d 超出范围 (0-180)\n", actualAngle3);
        return false;
    }
    if (actualAngle4 < 0 || actualAngle4 > 180) {
        Serial.printf("错误: 舵机4 计算后的实际角度 %d 超出范围 (0-180)\n", actualAngle4);
        return false;
    }
    
    // 保存相对角度
    servoAngles[0] = theta1;
    servoAngles[1] = theta2;
    servoAngles[2] = theta3;
    servoAngles[3] = theta4;
    
    // 阶段1：先设置舵机1和舵机3（两条腿的主舵机）
    Serial.printf("[阶段1] 设置舵机1、3...\n");
    int pulseWidth1 = 500 + (actualAngle1 * 2000 / 180);
    int pulseWidth3 = 500 + (actualAngle3 * 2000 / 180);
    servo1.writeMicroseconds(pulseWidth1);
    servo3.writeMicroseconds(pulseWidth3);
    
    Serial.printf("  舵机1: 输入=%.2f°, 实际=%d°\n", theta1, actualAngle1);
    Serial.printf("  舵机3: 输入=%.2f°, 实际=%d°\n", theta3, actualAngle3);
    
    // 延迟指定时间
    delay(delayMs);
    
    // 阶段2：再设置舵机2和舵机4（两条腿的副舵机）
    Serial.printf("[阶段2] 延迟%dms后设置舵机2、4...\n", delayMs);
    int pulseWidth2 = 500 + (actualAngle2 * 2000 / 180);
    int pulseWidth4 = 500 + (actualAngle4 * 2000 / 180);
    servo2.writeMicroseconds(pulseWidth2);
    servo4.writeMicroseconds(pulseWidth4);
    
    Serial.printf("  舵机2: 输入=%.2f°, 实际=%d°\n", theta2, actualAngle2);
    Serial.printf("  舵机4: 输入=%.2f°, 实际=%d°\n", theta4, actualAngle4);
    
    Serial.printf("两条腿已分阶段移动到: (%.2f, %.2f) mm\n", x, y);
    
    return true;
}

// 获取左腿末端位置
EndEffectorPos WheelLeg::getLeftLegPosition() {
    return forwardKinematics(servoAngles[0], servoAngles[1]);
}

// 获取右腿末端位置
EndEffectorPos WheelLeg::getRightLegPosition() {
    return forwardKinematicsRight(servoAngles[2], servoAngles[3]);
}

// 获取舵机零位
int WheelLeg::getZeroPosition(int servoNum) {
    if (servoNum >= 1 && servoNum <= 4) {
        return zeroPositions[servoNum - 1];
    }
    return -1;
}

// 获取舵机最大角度
int WheelLeg::getMaxAngle(int servoNum) {
    if (servoNum >= 1 && servoNum <= 4) {
        return is270Servo[servoNum - 1] ? 270 : 180;
    }
    return -1;
}

// 重置所有舵机到0度
void WheelLeg::reset() {
    for (int i = 1; i <= 4; i++) {
        setServoAngle(i, 0.0f);
    }
    Serial.printf("所有舵机已重置到0度\n");
}

// 打印当前状态
void WheelLeg::printStatus() {
    Serial.printf("\n========== 五连杆轮腿状态 ==========\n");
    
    // 左腿状态
    Serial.printf("左腿 (舵机1 + 舵机2):\n");
    Serial.printf("  舵机1: %.2f° (实际: %d°)\n", 
                 servoAngles[0], (int)(zeroPositions[0] + servoAngles[0]));
    Serial.printf("  舵机2: %.2f° (实际: %d°)\n", 
                 servoAngles[1], (int)(zeroPositions[1] - servoAngles[1]));
    EndEffectorPos leftPos = getLeftLegPosition();
    Serial.printf("  末端位置: (%.2f, %.2f) mm\n", leftPos.x, leftPos.y);
    
    Serial.printf("\n");
    
    // 右腿状态
    Serial.printf("右腿 (舵机3 + 舵机4):\n");
    Serial.printf("  舵机3: %.2f° (实际: %d°)\n", 
                 servoAngles[2], (int)(zeroPositions[2] - servoAngles[2]));
    Serial.printf("  舵机4: %.2f° (实际: %d°)\n", 
                 servoAngles[3], (int)(zeroPositions[3] + servoAngles[3]));
    EndEffectorPos rightPos = getRightLegPosition();
    Serial.printf("  末端位置: (%.2f, %.2f) mm\n", rightPos.x, rightPos.y);
    
    Serial.printf("\n");
    
    // 平衡控制状态
    Serial.printf("平衡控制:\n");
    Serial.printf("  状态: %s\n", balanceEnabled ? "启用" : "禁用");
    Serial.printf("  速度环PID: P=%.3f, I=%.3f, D=%.3f\n", speedPID.P, speedPID.I, speedPID.D);
    Serial.printf("  角度环PID: P=%.3f, I=%.3f, D=%.3f\n", anglePID.P, anglePID.I, anglePID.D);
    Serial.printf("  输出限制: ±%.2f\n", balanceOutputLimit);
    Serial.printf("  目标速度: %.2f\n", targetSpeed);
    Serial.printf("  当前速度: %.2f\n", currentSpeed);
    
    Serial.printf("===================================\n\n");
}

// 打印使用帮助
void WheelLeg::printHelp() {
    Serial.printf("\n========== 五连杆轮腿控制系统 ==========\n");
    
    Serial.printf("\n闭环五连杆配置:\n");
    Serial.printf("  结构: 舵机1--L1--固定点1--L2--轮子--L3--固定点2--L4--舵机2\n");
    Serial.printf("              \\_______________L5_______________/\n");
    Serial.printf("  左腿: 舵机1 (GPIO %d) + 舵机2 (GPIO %d)\n", servo1Pin, servo2Pin);
    Serial.printf("  右腿: 舵机3 (GPIO %d) + 舵机4 (GPIO %d)\n", servo3Pin, servo4Pin);
    Serial.printf("  连杆长度:\n");
    Serial.printf("    L1 (舵机1→固定点1): %.2f mm\n", linkageParams.L1);
    Serial.printf("    L2 (固定点1→轮子): %.2f mm\n", linkageParams.L2);
    Serial.printf("    L3 (轮子→固定点2): %.2f mm\n", linkageParams.L3);
    Serial.printf("    L4 (固定点2→舵机2): %.2f mm\n", linkageParams.L4);
    Serial.printf("    L5 (舵机2→舵机1): %.2f mm\n", linkageParams.L5);
    
    Serial.printf("\n串口命令格式:\n");
    Serial.printf("  舵机控制: <舵机号(1-4)> <角度>\n");
    Serial.printf("  左腿控制: left <x> <y>\n");
    Serial.printf("  右腿控制: right <x> <y>\n");
    Serial.printf("  腿高控制: height <高度(mm)>\n");
    
    Serial.printf("\n舵机信息:\n");
    Serial.printf("  舵机1 (180°): 零位 %d°, 控制方式: %d+输入角度\n", zeroPositions[0], zeroPositions[0]);
    Serial.printf("  舵机2 (180°): 零位 %d°, 控制方式: %d-输入角度\n", zeroPositions[1], zeroPositions[1]);
    Serial.printf("  舵机3 (180°): 零位 %d°, 控制方式: %d-输入角度\n", zeroPositions[2], zeroPositions[2]);
    Serial.printf("  舵机4 (180°): 零位 %d°, 控制方式: %d+输入角度\n", zeroPositions[3], zeroPositions[3]);
    
    Serial.printf("\n命令示例:\n");
    Serial.printf("  1 0      - 舵机1设置为0° (水平向左)\n");
    Serial.printf("  2 90     - 舵机2设置为90° (竖直向下)\n");
    Serial.printf("  left 23.5 -100 - 左腿末端移动到(23.5,-100)mm\n");
    Serial.printf("  right 50 -80   - 右腿末端移动到(50,-80)mm\n");
    Serial.printf("  height 120     - 设置腿高为120mm\n");
    Serial.printf("\n电机控制命令:\n");
    Serial.printf("  motor          - 显示详细双电机状态\n");
    Serial.printf("\n平衡控制命令 (默认启用):\n");
    Serial.printf("  speed_pid <P> <I> <D>  - 设置速度环PID参数\n");
    Serial.printf("  angle_pid <P> <I> <D>  - 设置角度环PID参数\n");
    Serial.printf("  balance_limit <值>     - 设置平衡控制输出限制 (±值)\n");
    Serial.printf("  target_speed <速度>    - 设置目标速度\n");
    Serial.printf("  balance_status         - 显示平衡控制状态\n");
    Serial.printf("\n系统命令:\n");
    Serial.printf("  reset    - 所有舵机重置到0度\n");
    Serial.printf("  status   - 显示当前状态\n");
    Serial.printf("  imu      - 显示IMU状态\n");
    Serial.printf("  imu_rate - 显示IMU更新频率统计\n");
    Serial.printf("  help     - 显示此帮助信息\n");
    
    Serial.printf("\n工作空间:\n");
    float maxReach1 = linkageParams.L1 + linkageParams.L2;  // 从舵机1
    float minReach1 = abs(linkageParams.L1 - linkageParams.L2);
    float maxReach2 = linkageParams.L4 + linkageParams.L3;  // 从舵机2
    float minReach2 = abs(linkageParams.L4 - linkageParams.L3);
    Serial.printf("  舵机间距: %.2f mm\n", linkageParams.L5);
    Serial.printf("  从舵机1可达范围: [%.2f, %.2f] mm\n", minReach1, maxReach1);
    Serial.printf("  从舵机2可达范围: [%.2f, %.2f] mm\n", minReach2, maxReach2);
    Serial.printf("  初始位置: (%.2f, -%.2f) mm\n", initialX, initialHeight);
    Serial.printf("  高度限制: [%.0f, %.0f] mm\n", MIN_HEIGHT, MAX_HEIGHT);
    
    Serial.printf("=========================================\n\n");
}

// ===== 私有辅助函数实现 =====

// 五连杆正运动学（左腿）
EndEffectorPos WheelLeg::forwardKinematics(float theta1_deg, float theta2_deg) {
    // 将相对角度转换为弧度
    float theta1 = theta1_deg * PI / 180.0f;
    float theta2 = theta2_deg * PI / 180.0f;
    
    // 全局坐标系：舵机1在原点(0, 0)，舵机2在(L5, 0)
    //           X轴向右为正，Y轴向上为正
    //           角度定义：水平向右为0°，逆时针为正
    float servo1_x = 0.0f;
    float servo1_y = 0.0f;
    float servo2_x = linkageParams.L5;
    float servo2_y = 0.0f;
    
    // 固定点1的位置（由舵机1和L1确定）
    // 舵机1相对角度: 0度=水平向左，逆时针为正
    // 全局角度 = 180° + theta1
    float global_angle1 = PI + theta1;
    float joint1_x = servo1_x + linkageParams.L1 * cos(global_angle1);
    float joint1_y = servo1_y + linkageParams.L1 * sin(global_angle1);
    
    // 固定点2的位置（由舵机2和L4确定）
    // 舵机2相对角度: 0度=水平向右，顺时针为正
    // 全局角度 = -theta2
    float global_angle2 = -theta2;
    float joint2_x = servo2_x + linkageParams.L4 * cos(global_angle2);
    float joint2_y = servo2_y + linkageParams.L4 * sin(global_angle2);
    
    // 轮子位置：需要满足两个约束
    // 1. 距离固定点1为L2
    // 2. 距离固定点2为L3
    // 使用两圆相交法求解
    
    float dx = joint2_x - joint1_x;
    float dy = joint2_y - joint1_y;
    float d = sqrt(dx * dx + dy * dy);  // 两个固定点之间的距离
    
    // 检查是否能形成三角形
    if (d > linkageParams.L2 + linkageParams.L3 || 
        d < abs(linkageParams.L2 - linkageParams.L3) ||
        d < 0.001f) {
        // 无法到达，返回一个默认位置
        EndEffectorPos pos;
        pos.x = 0.0f;
        pos.y = 0.0f;
        return pos;
    }
    
    // 使用余弦定理计算角度
    float a = linkageParams.L2;
    float b = linkageParams.L3;
    float cos_alpha = (a * a + d * d - b * b) / (2 * a * d);
    float alpha = acos(constrain(cos_alpha, -1.0f, 1.0f));
    
    // 计算轮子位置（选择下方的点：base_angle - alpha）
    float base_angle = atan2(dy, dx);
    float wheel_angle = base_angle - alpha;
    
    float wheel_x = joint1_x + linkageParams.L2 * cos(wheel_angle);
    float wheel_y = joint1_y + linkageParams.L2 * sin(wheel_angle);
    
    EndEffectorPos pos;
    pos.x = wheel_x;
    pos.y = wheel_y;
    
    return pos;
}

// 五连杆逆运动学（左腿）
bool WheelLeg::inverseKinematics(float x, float y, float &theta1_deg, float &theta2_deg) {
    // 坐标系：舵机1在原点(0, 0)，舵机2在(L5, 0)
    float servo1_x = 0.0f;
    float servo1_y = 0.0f;
    float servo2_x = linkageParams.L5;
    float servo2_y = 0.0f;
    
    float wheel_x = x;
    float wheel_y = y;
    
    // 计算从舵机到轮子的距离
    float d1_wheel = sqrt(wheel_x * wheel_x + wheel_y * wheel_y);
    float d2_wheel = sqrt((wheel_x - servo2_x) * (wheel_x - servo2_x) + wheel_y * wheel_y);
    
    // 检查工作空间约束
    // 轮子到舵机1的距离应该在 |L1-L2| 到 L1+L2 之间
    if (d1_wheel > linkageParams.L1 + linkageParams.L2 || 
        d1_wheel < abs(linkageParams.L1 - linkageParams.L2)) {
        Serial.printf("目标位置超出左侧工作空间\n");
        return false;
    }
    
    // 轮子到舵机2的距离应该在 |L4-L3| 到 L4+L3 之间
    if (d2_wheel > linkageParams.L4 + linkageParams.L3 || 
        d2_wheel < abs(linkageParams.L4 - linkageParams.L3)) {
        Serial.printf("目标位置超出右侧工作空间\n");
        return false;
    }
    
    // ===== 求解舵机1的角度 =====
    // 使用余弦定理求解舵机1-固定点1-轮子的角度
    float cos_angle1 = (linkageParams.L1 * linkageParams.L1 + d1_wheel * d1_wheel - 
                       linkageParams.L2 * linkageParams.L2) / 
                      (2 * linkageParams.L1 * d1_wheel);
    
    if (cos_angle1 < -1.0f || cos_angle1 > 1.0f) {
        Serial.printf("左侧逆解失败\n");
        return false;
    }
    
    float angle1 = acos(constrain(cos_angle1, -1.0f, 1.0f));  // L1和轮子方向的夹角
    float wheel_angle_global = atan2(wheel_y, wheel_x);  // 轮子相对全局坐标的角度
    
    // 舵机1指向固定点1的全局角度（选择下方点配置）
    float global_angle1 = wheel_angle_global - angle1;
    
    // 转换为舵机1的相对角度：全局180°对应舵机1的0°，逆时针为正
    // theta1 = global_angle1 - 180°
    float theta1 = global_angle1 - PI;
    
    // 角度归一化到[-PI, PI]范围
    while (theta1 > PI) theta1 -= 2.0f * PI;
    while (theta1 < -PI) theta1 += 2.0f * PI;
    
    // ===== 求解舵机2的角度 =====
    // 使用余弦定理求解舵机2-固定点2-轮子的角度
    float cos_angle2 = (linkageParams.L4 * linkageParams.L4 + d2_wheel * d2_wheel - 
                       linkageParams.L3 * linkageParams.L3) / 
                      (2 * linkageParams.L4 * d2_wheel);
    
    if (cos_angle2 < -1.0f || cos_angle2 > 1.0f) {
        Serial.printf("右侧逆解失败\n");
        return false;
    }
    
    float angle2 = acos(constrain(cos_angle2, -1.0f, 1.0f));  // L4和轮子方向的夹角
    float wheel_angle2_global = atan2(wheel_y, wheel_x - servo2_x);  // 从舵机2看轮子的全局角度
    
    // 舵机2指向固定点2的全局角度（选择下方点配置）
    float global_angle2 = wheel_angle2_global + angle2;
    
    // 转换为舵机2的相对角度：全局0°对应舵机2的0°，顺时针为正
    // theta2 = -global_angle2
    float theta2 = -global_angle2;
    
    // 角度归一化到[-PI, PI]范围
    while (theta2 > PI) theta2 -= 2.0f * PI;
    while (theta2 < -PI) theta2 += 2.0f * PI;
    
    // 转换为角度
    theta1_deg = theta1 * 180.0f / PI;
    theta2_deg = theta2 * 180.0f / PI;
    
    // 验证计算出的角度是否在舵机物理范围内
    int actualAngle1 = (int)(zeroPositions[0] + theta1_deg);
    int actualAngle2 = (int)(zeroPositions[1] - theta2_deg);
    
    if (actualAngle1 < 0 || actualAngle1 > 180) {
        Serial.printf("左腿逆解失败: 舵机1角度 %d° 超出范围 (0-180)\n", actualAngle1);
        Serial.printf("  目标位置: (%.2f, %.2f), 计算角度: theta1=%.2f°\n", x, y, theta1_deg);
        return false;
    }
    
    if (actualAngle2 < 0 || actualAngle2 > 180) {
        Serial.printf("左腿逆解失败: 舵机2角度 %d° 超出范围 (0-180)\n", actualAngle2);
        Serial.printf("  目标位置: (%.2f, %.2f), 计算角度: theta2=%.2f°\n", x, y, theta2_deg);
        return false;
    }
    
    return true;
}

// 五连杆正运动学（右腿）
EndEffectorPos WheelLeg::forwardKinematicsRight(float theta3_deg, float theta4_deg) {
    // 将相对角度转换为弧度
    float theta3 = theta3_deg * PI / 180.0f;
    float theta4 = theta4_deg * PI / 180.0f;
    
    // 全局坐标系：舵机3在原点(0, 0)，舵机4在(L5, 0)（向后）
    //           X轴向后为正，Y轴向上为正
    //           角度定义：向后为0°，逆时针为正（从上往下看）
    float servo3_x = 0.0f;
    float servo3_y = 0.0f;
    float servo4_x = linkageParams.L5;  // 舵机4在舵机3后方
    float servo4_y = 0.0f;
    
    // 固定点1的位置（由舵机3和L1确定）
    // 舵机3相对角度: 0度=水平向右（向机器人右侧），顺时针为正（从上往下看）
    // 舵机3对应舵机1，使用相同的角度转换公式
    // 全局角度 = 180° + theta3
    float global_angle3 = PI + theta3;
    float joint1_x = servo3_x + linkageParams.L1 * cos(global_angle3);
    float joint1_y = servo3_y + linkageParams.L1 * sin(global_angle3);
    
    // 固定点2的位置（由舵机4和L4确定）
    // 舵机4相对角度: 0度=水平向左（向机器人左侧），逆时针为正（从上往下看）
    // 舵机4对应舵机2，使用相同的角度转换公式
    // 全局角度 = -theta4
    float global_angle4 = -theta4;
    float joint2_x = servo4_x + linkageParams.L4 * cos(global_angle4);
    float joint2_y = servo4_y + linkageParams.L4 * sin(global_angle4);
    
    // 轮子位置：需要满足两个约束
    // 1. 距离固定点1为L2
    // 2. 距离固定点2为L3
    // 使用两圆相交法求解
    
    float dx = joint2_x - joint1_x;
    float dy = joint2_y - joint1_y;
    float d = sqrt(dx * dx + dy * dy);  // 两个固定点之间的距离
    
    // 检查是否能形成三角形
    if (d > linkageParams.L2 + linkageParams.L3 || 
        d < abs(linkageParams.L2 - linkageParams.L3) ||
        d < 0.001f) {
        // 无法到达，返回一个默认位置
        EndEffectorPos pos;
        pos.x = 0.0f;
        pos.y = 0.0f;
        return pos;
    }
    
    // 使用余弦定理计算角度
    float a = linkageParams.L2;
    float b = linkageParams.L3;
    float cos_alpha = (a * a + d * d - b * b) / (2 * a * d);
    float alpha = acos(constrain(cos_alpha, -1.0f, 1.0f));
    
    // 计算轮子位置（选择下方的点：base_angle - alpha）
    float base_angle = atan2(dy, dx);
    float wheel_angle = base_angle - alpha;
    
    float wheel_x = joint1_x + linkageParams.L2 * cos(wheel_angle);
    float wheel_y = joint1_y + linkageParams.L2 * sin(wheel_angle);
    
    EndEffectorPos pos;
    pos.x = wheel_x;
    pos.y = wheel_y;
    
    return pos;
}

// 五连杆逆运动学（右腿）
bool WheelLeg::inverseKinematicsRight(float x, float y, float &theta3_deg, float &theta4_deg) {
    // 右腿坐标系：舵机3在原点(0, 0)，舵机4在(L5, 0)（向后）
    float servo3_x = 0.0f;
    float servo3_y = 0.0f;
    float servo4_x = linkageParams.L5;  // 舵机4在舵机3后方
    float servo4_y = 0.0f;
    
    float wheel_x = x;
    float wheel_y = y;
    
    // 计算从舵机到轮子的距离
    float d3_wheel = sqrt(wheel_x * wheel_x + wheel_y * wheel_y);
    float d4_wheel = sqrt((wheel_x - servo4_x) * (wheel_x - servo4_x) + wheel_y * wheel_y);
    
    // 检查工作空间约束
    // 轮子到舵机3的距离应该在 |L1-L2| 到 L1+L2 之间
    if (d3_wheel > linkageParams.L1 + linkageParams.L2 || 
        d3_wheel < abs(linkageParams.L1 - linkageParams.L2)) {
        Serial.printf("目标位置超出右腿工作空间（舵机3侧）\n");
        return false;
    }
    
    // 轮子到舵机4的距离应该在 |L4-L3| 到 L4+L3 之间
    if (d4_wheel > linkageParams.L4 + linkageParams.L3 || 
        d4_wheel < abs(linkageParams.L4 - linkageParams.L3)) {
        Serial.printf("目标位置超出右腿工作空间（舵机4侧）\n");
        return false;
    }
    
    // ===== 求解舵机3的角度 =====
    // 使用余弦定理求解舵机3-固定点1-轮子的角度
    float cos_angle3 = (linkageParams.L1 * linkageParams.L1 + d3_wheel * d3_wheel - 
                       linkageParams.L2 * linkageParams.L2) / 
                      (2 * linkageParams.L1 * d3_wheel);
    
    if (cos_angle3 < -1.0f || cos_angle3 > 1.0f) {
        Serial.printf("右腿逆解失败（舵机3侧）\n");
        return false;
    }
    
    float angle3 = acos(constrain(cos_angle3, -1.0f, 1.0f));  // L1和轮子方向的夹角
    float wheel_angle_global = atan2(wheel_y, wheel_x);  // 轮子相对全局坐标的角度
    
    // 舵机3指向固定点1的全局角度（选择下方点配置）
    float global_angle3 = wheel_angle_global - angle3;
    
    // 转换为舵机3的相对角度
    // 舵机3对应舵机1，全局角度 = 180° + theta3
    // 因此 theta3 = global_angle3 - 180°
    float theta3 = global_angle3 - PI;
    
    // 角度归一化到[-PI, PI]范围
    while (theta3 > PI) theta3 -= 2.0f * PI;
    while (theta3 < -PI) theta3 += 2.0f * PI;
    
    // ===== 求解舵机4的角度 =====
    // 使用余弦定理求解舵机4-固定点2-轮子的角度
    float cos_angle4 = (linkageParams.L4 * linkageParams.L4 + d4_wheel * d4_wheel - 
                       linkageParams.L3 * linkageParams.L3) / 
                      (2 * linkageParams.L4 * d4_wheel);
    
    if (cos_angle4 < -1.0f || cos_angle4 > 1.0f) {
        Serial.printf("右腿逆解失败（舵机4侧）\n");
        return false;
    }
    
    float angle4 = acos(constrain(cos_angle4, -1.0f, 1.0f));  // L4和轮子方向的夹角
    float wheel_angle4_global = atan2(wheel_y, wheel_x - servo4_x);  // 从舵机4看轮子的全局角度
    
    // 舵机4指向固定点2的全局角度（选择下方点配置）
    float global_angle4 = wheel_angle4_global + angle4;
    
    // 转换为舵机4的相对角度
    // 舵机4对应舵机2，全局角度 = -theta4
    // 因此 theta4 = -global_angle4
    float theta4 = -global_angle4;
    
    // 角度归一化到[-PI, PI]范围
    while (theta4 > PI) theta4 -= 2.0f * PI;
    while (theta4 < -PI) theta4 += 2.0f * PI;
    
    // 转换为角度
    theta3_deg = theta3 * 180.0f / PI;
    theta4_deg = theta4 * 180.0f / PI;
    
    // 验证计算出的角度是否在舵机物理范围内
    int actualAngle3 = (int)(zeroPositions[2] - theta3_deg);
    int actualAngle4 = (int)(zeroPositions[3] + theta4_deg);
    
    if (actualAngle3 < 0 || actualAngle3 > 180) {
        Serial.printf("右腿逆解失败: 舵机3角度 %d° 超出范围 (0-180)\n", actualAngle3);
        Serial.printf("  目标位置: (%.2f, %.2f), 计算角度: theta3=%.2f°\n", x, y, theta3_deg);
        return false;
    }
    
    if (actualAngle4 < 0 || actualAngle4 > 180) {
        Serial.printf("右腿逆解失败: 舵机4角度 %d° 超出范围 (0-180)\n", actualAngle4);
        Serial.printf("  目标位置: (%.2f, %.2f), 计算角度: theta4=%.2f°\n", x, y, theta4_deg);
        return false;
    }
    
    return true;
}

// ===== 平衡控制方法实现 =====

// 设置平衡控制输出限制
void WheelLeg::setBalanceOutputLimit(float limit) {
    if (limit > 0.0f) {
        balanceOutputLimit = limit;
        anglePID.limit = limit;  // 同时更新角度环PID的输出限制
        Serial.printf("[平衡控制] 输出限制已更新: ±%.2f\n", limit);
    } else {
        Serial.printf("[平衡控制] 错误: 输出限制必须大于0\n");
    }
}


// 设置目标速度
void WheelLeg::setTargetSpeed(float speed) {
    targetSpeed = speed;
    lastSpeedUpdateTime = millis();  // 重置时间戳
    
    // 当接收到非零目标速度时，冻结偏移修正并将总偏移角保存到基础偏移
    if (abs(speed) > 0.1f && !offsetFrozen && autoOffsetEnabled) {
        // 将当前的总偏移角（基础偏移+自动修正）保存到基础偏移
        baseAngleOffset = baseAngleOffset + baseAngleOffsetCorrection;
        baseAngleOffsetCorrection = 0.0f;  // 清零自动修正值
        offsetFrozen = true;  // 冻结自动修正
        Serial.printf("[平衡控制] 目标速度设置为: %.2f，冻结偏移修正，总偏移角=%.3f°\n", speed, baseAngleOffset);
    } else {
        Serial.printf("[平衡控制] 目标速度设置为: %.2f\n", speed);
    }
}

// 设置速度斜坡变化率
void WheelLeg::setSpeedRampRate(float rampRate) {
    speedRampRate = rampRate;
    Serial.printf("[平衡控制] 速度斜坡变化率设置为: %.2f RPM/s\n", rampRate);
}

// 设置基础角度偏移量
void WheelLeg::setBaseAngleOffset(float offset) {
    baseAngleOffset = offset;
    Serial.printf("[平衡控制] 基础角度偏移量设置为: %.2f°\n", offset);
}

// 设置速度环PID参数
void WheelLeg::setSpeedPID(float P, float I, float D) {
    speedPID.P = P;
    speedPID.I = I;
    speedPID.D = D;
    speedPID.reset();  // 重置积分项
    Serial.printf("[平衡控制] 速度环PID已更新: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
}

// 设置角度环PID参数
void WheelLeg::setAnglePID(float P, float I, float D) {
    anglePID.P = P;
    anglePID.I = I;
    anglePID.D = D;
    anglePID.reset();  // 重置积分项
    Serial.printf("[平衡控制] 角度环PID已更新: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
}

// 设置转向环PID参数
void WheelLeg::setSteeringPID(float P, float I, float D) {
    steeringPID.P = P;
    steeringPID.I = I;
    steeringPID.D = D;
    steeringPID.reset();  // 重置积分项
    Serial.printf("[转向控制] 转向环PID已更新: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
}

// 设置目标转向速度
void WheelLeg::setTargetYawRate(float yawRate) {
    targetYawRate = yawRate;
    Serial.printf("[转向控制] 目标转向速度设置为: %.2f°/s\n", yawRate);
}

// 设置目标滚转角
void WheelLeg::setTargetRollAngle(float rollAngle) {
    targetRollAngle = rollAngle;
    Serial.printf("[滚转控制] 目标滚转角设置为: %.2f°\n", rollAngle);
}

// 设置滚转控制使能
void WheelLeg::setRollControlEnabled(bool enabled) {
    rollControlEnabled = enabled;
    if (!enabled) {
        rollAngleOffset = 0.0f;  // 禁用时清零偏移角
        rollPID.reset();  // 重置PID状态
    }
    Serial.printf("[滚转控制] 滚转控制%s\n", enabled ? "启用" : "禁用");
}

// 设置滚转PID参数
void WheelLeg::setRollPID(float P, float I, float D) {
    rollPID.P = P;
    rollPID.I = I;
    rollPID.D = D;
    rollPID.reset();  // 重置积分项
    Serial.printf("[滚转控制] 滚转PID已更新: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
}

// 设置偏移修正PID参数
void WheelLeg::setOffsetPID(float P, float I, float D) {
    offsetPID.P = P;
    offsetPID.I = I;
    offsetPID.D = D;
    offsetPID.reset();  // 重置积分项
    Serial.printf("[偏移修正] 偏移修正PID已更新: P=%.3f, I=%.3f, D=%.3f\n", P, I, D);
}

// 设置自动偏移修正使能
void WheelLeg::setAutoOffsetEnabled(bool enabled) {
    autoOffsetEnabled = enabled;
    if (!enabled) {
        baseAngleOffsetCorrection = 0.0f;  // 禁用时清零修正值
        offsetPID.reset();  // 重置PID状态
        offsetFrozen = false;
    } else {
        offsetFrozen = false;
        lastHeightForOffset = initialHeight;
        lastBaseAngleOffsetCorrection = baseAngleOffsetCorrection;
        lastOffsetChangeTime = millis();
    }
    Serial.printf("[偏移修正] 自动偏移修正%s\n", enabled ? "启用" : "禁用");
}

// 设置速度死区
void WheelLeg::setSpeedDeadZone(float deadZone) {
    speedDeadZone = deadZone;
    Serial.printf("[偏移修正] 速度死区设置为: ±%.2f RPM\n", deadZone);
}

// 解锁平衡偏移角冻结状态
void WheelLeg::unlockOffsetFreeze() {
    if (offsetFrozen) {
        offsetFrozen = false;
        offsetPID.reset();
        lastHeightForOffset = initialHeight;
        lastBaseAngleOffsetCorrection = baseAngleOffsetCorrection;
        lastOffsetChangeTime = millis();
        Serial.printf("[偏移修正] 解除冻结状态\n");
    }
}

// 设置变PID使能
void WheelLeg::setAdaptivePIDEnabled(bool enabled) {
    adaptivePIDEnabled = enabled;
    if (enabled) {
        // 启用时立即更新PID参数
        updatePIDForHeight(initialHeight);
    }
    Serial.printf("[变PID] 变PID控制%s\n", enabled ? "启用" : "禁用");
}

// 设置校准点
void WheelLeg::setCalibrationPoint(int index, float height, float speedP, float speedI, float speedD, 
                                   float angleP, float angleI, float angleD) {
    if (index < 0 || index >= NUM_CALIBRATION_POINTS) {
        Serial.printf("[变PID] 错误: 校准点索引 %d 超出范围 (0-%d)\n", index, NUM_CALIBRATION_POINTS - 1);
        return;
    }
    
    calibrationPoints[index].height = height;
    calibrationPoints[index].speedP = speedP;
    calibrationPoints[index].speedI = speedI;
    calibrationPoints[index].speedD = speedD;
    calibrationPoints[index].angleP = angleP;
    calibrationPoints[index].angleI = angleI;
    calibrationPoints[index].angleD = angleD;
    
    Serial.printf("[变PID] 校准点%d已更新: 高度=%.1fmm, 速度PID[%.3f,%.3f,%.3f], 角度PID[%.3f,%.3f,%.3f]\n",
                 index, height, speedP, speedI, speedD, angleP, angleI, angleD);
    
    // 如果变PID已启用，立即更新当前PID参数
    if (adaptivePIDEnabled) {
        updatePIDForHeight(initialHeight);
    }
}

// 设置PID平滑过渡时间
void WheelLeg::setPIDTransitionDuration(float durationMs) {
    if (durationMs < 0.0f) {
        Serial.printf("[变PID] 错误: 过渡时间必须为正数\n");
        return;
    }
    pidTransitionDuration = durationMs;
    Serial.printf("[变PID] PID平滑过渡时间设置为: %.0f ms\n", durationMs);
}

// 根据腿高插值计算并更新PID参数
void WheelLeg::updatePIDForHeight(float height) {
    if (!adaptivePIDEnabled) {
        return;  // 变PID未启用，不更新
    }
    
    // 限制高度在有效范围内
    height = constrain(height, MIN_HEIGHT, MAX_HEIGHT);
    
    float speedP, speedI, speedD, angleP, angleI, angleD;
    
    // 查找合适的插值区间
    if (height <= calibrationPoints[2].height) {
        // 低于最低校准点，使用最低点参数
        speedP = calibrationPoints[2].speedP;
        speedI = calibrationPoints[2].speedI;
        speedD = calibrationPoints[2].speedD;
        angleP = calibrationPoints[2].angleP;
        angleI = calibrationPoints[2].angleI;
        angleD = calibrationPoints[2].angleD;
    } else if (height >= calibrationPoints[0].height) {
        // 高于最高校准点，使用最高点参数
        speedP = calibrationPoints[0].speedP;
        speedI = calibrationPoints[0].speedI;
        speedD = calibrationPoints[0].speedD;
        angleP = calibrationPoints[0].angleP;
        angleI = calibrationPoints[0].angleI;
        angleD = calibrationPoints[0].angleD;
    } else {
        // 在校准点之间，进行线性插值
        int lowerIdx = -1, upperIdx = -1;
        
        // 找到包围当前高度的两个校准点
        for (int i = 0; i < NUM_CALIBRATION_POINTS - 1; i++) {
            if (height >= calibrationPoints[i + 1].height && height <= calibrationPoints[i].height) {
                lowerIdx = i + 1;
                upperIdx = i;
                break;
            }
        }
        
        if (lowerIdx >= 0 && upperIdx >= 0) {
            // 线性插值
            float t = (height - calibrationPoints[lowerIdx].height) / 
                     (calibrationPoints[upperIdx].height - calibrationPoints[lowerIdx].height);
            
            speedP = calibrationPoints[lowerIdx].speedP + 
                    t * (calibrationPoints[upperIdx].speedP - calibrationPoints[lowerIdx].speedP);
            speedI = calibrationPoints[lowerIdx].speedI + 
                    t * (calibrationPoints[upperIdx].speedI - calibrationPoints[lowerIdx].speedI);
            speedD = calibrationPoints[lowerIdx].speedD + 
                    t * (calibrationPoints[upperIdx].speedD - calibrationPoints[lowerIdx].speedD);
            angleP = calibrationPoints[lowerIdx].angleP + 
                    t * (calibrationPoints[upperIdx].angleP - calibrationPoints[lowerIdx].angleP);
            angleI = calibrationPoints[lowerIdx].angleI + 
                    t * (calibrationPoints[upperIdx].angleI - calibrationPoints[lowerIdx].angleI);
            angleD = calibrationPoints[lowerIdx].angleD + 
                    t * (calibrationPoints[upperIdx].angleD - calibrationPoints[lowerIdx].angleD);
        } else {
            // 如果找不到合适的区间（不应该发生），使用中间点参数
            speedP = calibrationPoints[1].speedP;
            speedI = calibrationPoints[1].speedI;
            speedD = calibrationPoints[1].speedD;
            angleP = calibrationPoints[1].angleP;
            angleI = calibrationPoints[1].angleI;
            angleD = calibrationPoints[1].angleD;
        }
    }
    
    // 检查PID参数是否有显著变化（避免不必要的过渡）
    float pidChangeTolerance = 0.01f;
    bool needTransition = (abs(speedP - speedPID.P) > pidChangeTolerance) ||
                         (abs(angleP - anglePID.P) > pidChangeTolerance);
    
    if (needTransition) {
        // 保存当前PID参数作为起始值
        startSpeedP = speedPID.P;
        startSpeedI = speedPID.I;
        startSpeedD = speedPID.D;
        startAngleP = anglePID.P;
        startAngleI = anglePID.I;
        startAngleD = anglePID.D;
        
        // 设置目标PID参数
        targetSpeedP = speedP;
        targetSpeedI = speedI;
        targetSpeedD = speedD;
        targetAngleP = angleP;
        targetAngleI = angleI;
        targetAngleD = angleD;
        
        // 启动平滑过渡
        pidTransitioning = true;
        pidTransitionStartTime = millis();
        
        // 立即重置PID积分项，避免旧积分值影响新参数
        speedPID.reset();
        anglePID.reset();
        
        Serial.printf("[变PID] 腿高%.1fmm -> 启动PID平滑过渡(%.0fms)\n", height, pidTransitionDuration);
        Serial.printf("        速度PID: [%.3f,%.3f,%.3f] -> [%.3f,%.3f,%.3f]\n",
                     startSpeedP, startSpeedI, startSpeedD, targetSpeedP, targetSpeedI, targetSpeedD);
        Serial.printf("        角度PID: [%.3f,%.3f,%.3f] -> [%.3f,%.3f,%.3f]\n",
                     startAngleP, startAngleI, startAngleD, targetAngleP, targetAngleI, targetAngleD);
    } else {
        Serial.printf("[变PID] 腿高%.1fmm -> PID参数变化小，无需过渡\n", height);
    }
}

// 更新PID平滑过渡
void WheelLeg::updatePIDTransition() {
    if (!pidTransitioning) {
        return;  // 没有正在进行的过渡
    }
    
    unsigned long now = millis();
    float elapsed = (float)(now - pidTransitionStartTime);
    
    if (elapsed >= pidTransitionDuration) {
        // 过渡完成，设置为目标值
        speedPID.P = targetSpeedP;
        speedPID.I = targetSpeedI;
        speedPID.D = targetSpeedD;
        anglePID.P = targetAngleP;
        anglePID.I = targetAngleI;
        anglePID.D = targetAngleD;
        
        pidTransitioning = false;
        Serial.printf("[变PID] 平滑过渡完成 -> 速度PID[P=%.3f,I=%.3f,D=%.3f] 角度PID[P=%.3f,I=%.3f,D=%.3f]\n",
                     speedPID.P, speedPID.I, speedPID.D, anglePID.P, anglePID.I, anglePID.D);
    } else {
        // 计算过渡进度 (0.0 到 1.0)
        float progress = elapsed / pidTransitionDuration;
        
        // 使用平滑的S曲线插值（ease-in-out）
        // 公式: t = t * t * (3 - 2 * t)
        float smoothProgress = progress * progress * (3.0f - 2.0f * progress);
        
        // 线性插值PID参数
        speedPID.P = startSpeedP + (targetSpeedP - startSpeedP) * smoothProgress;
        speedPID.I = startSpeedI + (targetSpeedI - startSpeedI) * smoothProgress;
        speedPID.D = startSpeedD + (targetSpeedD - startSpeedD) * smoothProgress;
        anglePID.P = startAngleP + (targetAngleP - startAngleP) * smoothProgress;
        anglePID.I = startAngleI + (targetAngleI - startAngleI) * smoothProgress;
        anglePID.D = startAngleD + (targetAngleD - startAngleD) * smoothProgress;
    }
}


// 计算平衡控制输出 - 包含转向控制和滚转控制
// 在原有双环 PID 基础上，引入 LQR 增强项，以提高姿态与位移响应精度
void WheelLeg::calculateBalanceOutput(float currentAngleX, float currentAngleY, float currentYawRate, float currentPitchRate, float motor1RPM, float motor2RPM, float& leftOutput, float& rightOutput) {
    if (!balanceEnabled) {
        leftOutput = 0.0f;
        rightOutput = 0.0f;
        return;  // 平衡控制未启用，返回0输出
    }
    
    // 更新PID平滑过渡（如果正在进行）
    updatePIDTransition();
    
    unsigned long now = millis();
    
    // 应用速度斜坡滤波：平滑过渡目标速度
    float dt = (now - lastSpeedUpdateTime) / 1000.0f;  // 转换为秒
    lastSpeedUpdateTime = now;
    
    if (dt > 0.0f && dt < 1.0f) {  // 防止异常dt值
        float maxSpeedChange = speedRampRate * dt;  // 最大允许的速度变化
        float speedDiff = targetSpeed - targetSpeedFiltered;
        
        if (abs(speedDiff) <= maxSpeedChange) {
            targetSpeedFiltered = targetSpeed;  // 已达到目标
        } else {
            // 按斜坡率逐步接近目标
            targetSpeedFiltered += (speedDiff > 0 ? maxSpeedChange : -maxSpeedChange);
        }
    }

    float currentHeight = initialHeight;
    if (abs(currentHeight - lastHeightForOffset) > HEIGHT_CHANGE_THRESHOLD) {
        offsetFrozen = false;  // 腿高变化时解除冻结
        offsetPID.reset();
        lastHeightForOffset = currentHeight;
        lastBaseAngleOffsetCorrection = baseAngleOffsetCorrection;
        lastOffsetChangeTime = now;
        Serial.printf("[偏移修正] 腿高变化，解除冻结\n");
    }

    // 计算当前机器人速度（电机RPM平均值，负号表示前进方向）
    float avgMotorSpeed = -(motor1RPM + motor2RPM) / 2.0f;
    
    // 更新当前速度（用于外环控制）
    currentSpeed = -avgMotorSpeed;
    
    // 更新当前转向速度
    float yawRateDeg = -currentYawRate;
    this->currentYawRate = yawRateDeg;
    
    // 自动偏移修正：当目标速度为0且未冻结时，根据当前速度自动调整基础角度偏移
    if (autoOffsetEnabled && !offsetFrozen && abs(targetSpeed) < 0.1f) {  // 目标速度接近0时启用
        if (abs(currentSpeed) > speedDeadZone) {  // 当前速度超出死区范围
            // 使用增量式PID控制器计算偏移修正量
            // 如果速度为正（前进），需要减小baseAngleOffset（负修正）
            // 如果速度为负（后退），需要增大baseAngleOffset（正修正）
            float offsetError = -currentSpeed;  // 期望速度为0，误差为-currentSpeed
            float offsetIncrement = offsetPID(offsetError);
            baseAngleOffsetCorrection += offsetIncrement;
            
            // 限制修正值范围，避免过度修正
            baseAngleOffsetCorrection = constrain(baseAngleOffsetCorrection, -15.0f, 15.0f);
        }
    }
    // 注意：不再在目标速度不为0时清零修正值，而是在setTargetSpeed中冻结并合并到baseAngleOffset
    
    // 使用SimpleFOC PID控制器的三环控制算法：
    // 1. 速度环（外环）：使用PID控制器计算目标角度
    // 2. 角度环（内环）：使用PID控制器计算平衡电机输出
    // 3. 转向环：使用PID控制器计算转向力矩
    // 4. 偏移修正：自动调整基础角度偏移以消除静态偏差
    
    // 速度环：使用PID控制器计算目标角度（包含偏移修正）
    // 使用经过斜坡滤波后的目标速度，避免突变
    float speedError = targetSpeedFiltered - currentSpeed;
    float targetAngle = speedPID(speedError) + baseAngleOffset + baseAngleOffsetCorrection;
    
    // 角度环：作为反馈闭环，LQR 作为前馈力矩
    float angleError = targetAngle - currentAngleX;
    float pidU = anglePID(angleError);   // PID 闭环修正力矩
    
    // 转向环：使用PID控制器计算转向力矩
    float steeringTorque = 0.0f;
    if (steeringEnabled) {
        float yawError = targetYawRate - currentYawRate;
        steeringTorque = steeringPID(yawError);
    }
    
    // 滚转控制：根据IMU Y角度调整左右腿高度
    // 期望滚转角 = 基础期望角度 - 输入滚转角
    // 当IMU Y角度小于期望滚转角时，滚转偏移角逐步增大（反之亦然）
    if (rollControlEnabled) {
        // 归一化IMU Y角度：如果为负值，加上360度
        float normalizedAngleY = currentAngleY;
        if (normalizedAngleY < 0) {
            normalizedAngleY += 360.0f;
        }
        
        float expectedRollAngle = ROLL_BASE_EXPECTED_ANGLE - targetRollAngle;
        float rollError = expectedRollAngle - normalizedAngleY;  // 反转误差符号
        
        // 使用增量式PID计算滚转偏移角的增量
        // 当currentAngleY < expectedRollAngle时，rollError > 0，偏移角增大
        // 当currentAngleY > expectedRollAngle时，rollError < 0，偏移角减小
        float rollIncrement = rollPID(rollError);
        rollAngleOffset += rollIncrement;
        
        // 限制滚转偏移角范围（使用配置的限制值）
        rollAngleOffset = constrain(rollAngleOffset, -ROLL_ANGLE_OFFSET_LIMIT, ROLL_ANGLE_OFFSET_LIMIT);
        
        // 根据滚转偏移角调整左右腿高度
        // 轮距L5 = 47.0mm
        float wheelDistance = 47.0f;
        float rollOffsetRad = rollAngleOffset * PI / 180.0f;
        float heightAdjustment = sin(rollOffsetRad) * wheelDistance / 2.0f;
        
        // 左腿高度 = 基础高度 + 调整量
        // 右腿高度 = 基础高度 - 调整量
        float leftHeight = initialHeight + heightAdjustment;
        float rightHeight = initialHeight - heightAdjustment;
        
        // 检查腿高是否超出范围，如果超出则整体调整
        float heightOffset = 0.0f;
        if (leftHeight < MIN_HEIGHT) {
            // 左腿太低，整体上移
            heightOffset = MIN_HEIGHT - leftHeight;
        } else if (leftHeight > MAX_HEIGHT) {
            // 左腿太高，整体下移
            heightOffset = MAX_HEIGHT - leftHeight;
        }
        
        if (rightHeight < MIN_HEIGHT) {
            // 右腿太低，整体上移（取较大的偏移量）
            float rightOffset = MIN_HEIGHT - rightHeight;
            if (rightOffset > heightOffset) {
                heightOffset = rightOffset;
            }
        } else if (rightHeight > MAX_HEIGHT) {
            // 右腿太高，整体下移（取较小的偏移量）
            float rightOffset = MAX_HEIGHT - rightHeight;
            if (abs(rightOffset) > abs(heightOffset)) {
                heightOffset = rightOffset;
            }
        }
        
        // 应用整体偏移
        if (abs(heightOffset) > 0.1f) {
            leftHeight += heightOffset;
            rightHeight += heightOffset;
            
            // 更新基础高度
            float newBaseHeight = initialHeight + heightOffset;
            if (newBaseHeight >= MIN_HEIGHT && newBaseHeight <= MAX_HEIGHT) {
                initialHeight = newBaseHeight;
                Serial.printf("[滚转控制] 腿高超限，自动调整基础高度: %.2f mm (偏移: %.2f mm)\n", 
                             initialHeight, heightOffset);
            }
        }
        
        // 应用腿高调整
        setLeftLegPosition(initialX, -leftHeight);
        setRightLegPosition(initialX, -rightHeight);
    }
    
    // 差速控制：平衡输出 + 转向力矩
    // 在此基础上引入 LQR 风格的多 PID 组合（参考 3.Software 中的 lqr_balance_loop 实现）
    // ===== LQR-PID 自平衡控制 =====
    // 1) 状态获取：俯仰角 currentAngleX、俯仰角速度 currentPitchRate、电机线速度 vel、积分位移 lqrPosition
    static float lqrPosition = 0.0f;                 // 机器人前后位移积分 (m)
    static unsigned long lastPosUpdateMicros = 0;    // 上次位移积分时间

    // 电机角速度 (rad/s) 与线速度 (m/s)
    const float rpmToRadPerSec = 2.0f * PI / 60.0f;
    float motor1Rad = motor1RPM * rpmToRadPerSec;
    float motor2Rad = motor2RPM * rpmToRadPerSec;
    float vel = -((motor1Rad * WHEEL_RADIUS_M) + (motor2Rad * WHEEL_RADIUS_M)) / 2.0f;  // 前进为正

    // 位移积分，使用 micros() 提高时间精度
    unsigned long nowMicros = micros();
    if (lastPosUpdateMicros == 0) {
        lastPosUpdateMicros = nowMicros;
    }
    float dtPos = (nowMicros - lastPosUpdateMicros) * 1e-6f;
    lastPosUpdateMicros = nowMicros;
    if (dtPos > 0.0f && dtPos < 1.0f) { // 简单异常 dt 过滤
        lqrPosition += vel * dtPos;
        lqrPosition = constrain(lqrPosition, -LQR_POSITION_LIMIT, LQR_POSITION_LIMIT);
    }

    // LQR 与 PID 使用同一“直立基准”：IMU角度 + baseAngleOffset + baseAngleOffsetCorrection
    // 这里将 IMU 原始角度转换为相对直立基准的偏差角（单位：度）
    float uprightDeg = baseAngleOffset + baseAngleOffsetCorrection;
    float pitchForLQRDeg = currentAngleX - uprightDeg;  // 俯仰角相对直立基准偏差

    // 状态量（尽量与 3.Software 中的命名对应）
    float lqrAngle = pitchForLQRDeg;     // 俯仰角（deg）
    float lqrGyro = currentPitchRate;    // 俯仰角速度（deg/s）
    float lqrSpeed = vel;                // 线速度（m/s）
    float lqrDistance = lqrPosition;     // 前后位移（m）

    // 2) 计算自平衡输出的各个 PID 分量
    float angleControl = lqrAnglePID(lqrAngle - lqrAngleZeroPoint);
    float gyroControl  = lqrGyroPID(lqrGyro);

    // 运动细节优化：当有明显速度指令时，仅依靠姿态+角速度控制，不锁定位移
    if (fabs(targetSpeedFiltered) > 1.0f) { // 目标速度显著非零时，认为正在运动
        lqrDistanceZeroPoint = lqrDistance; // 重置位移零点，避免长时间积分漂移
        lqrUPID.reset();                    // 小扭矩补偿积分清零
    }

    // 原地停车：当目标速度回到 0 且实际速度接近 0 时，重置位移零点
    if (fabs(targetSpeedFiltered) < 0.1f && fabs(lqrSpeed) < 0.05f) {
        lqrDistanceZeroPoint = lqrDistance;
    }

    // 被外力快速推动时的原地停车处理：检测速度突变或速度过大
    lqrRobotSpeedLast = lqrRobotSpeed;
    lqrRobotSpeed = lqrSpeed;
    bool wheelOffGround = (fabs(lqrRobotSpeed - lqrRobotSpeedLast) > 0.5f) || (fabs(lqrRobotSpeed) > 1.5f);

    // 位移与速度控制分量（这里将目标速度设为 0，由外层速度环负责前后运动指令）
    float distanceControl = lqrDistancePID(lqrDistance - lqrDistanceZeroPoint);
    float speedControl    = lqrSpeedPID(lqrSpeed);

    float lqrU = 0.0f;
    if (wheelOffGround) {
        // 轮部疑似离地，仅使用姿态+角速度分量，避免位移/速度控制带来剧烈输出
        lqrDistanceZeroPoint = lqrDistance;
        lqrU = angleControl + gyroControl;
        lqrUPID.reset();
    } else {
        // 正常情况下，完整输出 LQR 风格控制量
        lqrU = angleControl + gyroControl + distanceControl + speedControl;
    }

    // 3) 小扭矩非线性补偿 + 重心自适应（参考 3.Software 中 LQR_u 的后处理）
    if (fabs(lqrU) < 5.0f && fabs(targetSpeedFiltered) < 0.1f && fabs(distanceControl) < 0.05f) {
        // 小扭矩区域、无明显前后运动指令且位移误差较小 → 启用 PI 补偿与重心自适应
        lqrU = lqrUPID(lqrU);                               // 补偿电机与机构的小扭矩非线性
        lqrAngleZeroPoint -= lqrZeroPID(distanceControl);   // 缓慢调整俯仰角零点，做重心自适应
    } else {
        lqrUPID.reset();
    }

    // 4) 将 LQR-PID 控制量与原角度环 PID 输出混合
    float combinedBalance = pidU + LQR_MIX_K * lqrU;
    combinedBalance = constrain(combinedBalance, -balanceOutputLimit, balanceOutputLimit);

    // 左轮：平衡输出 - 转向力矩（负转向力矩使左轮减速，机器人左转）
    // 右轮：平衡输出 + 转向力矩（正转向力矩使右轮加速，机器人左转）
    leftOutput = combinedBalance - steeringTorque;
    rightOutput = combinedBalance + steeringTorque;
    
    // 调试输出（每2秒一次，包含响应时间监控）
    static unsigned long lastDebugTime = 0;
    static unsigned long lastCallTime = 0;
    unsigned long currentTime = millis();
    unsigned long deltaTime = currentTime - lastCallTime;
    lastCallTime = currentTime;
    
    if (currentTime - lastDebugTime > 2000) {
        Serial.printf("[控制系统] 调用间隔: %lu ms (目标: %d ms)\n", 
                     deltaTime, 1000/100);  // 100Hz = 10ms间隔
        Serial.printf("[平衡控制] 电机RPM: M1=%.1f, M2=%.1f, 平均=%.1f\n", 
                     motor1RPM, motor2RPM, avgMotorSpeed);
        Serial.printf("[平衡控制] 速度环: 目标=%.2f, 当前=%.2f, 误差=%.2f → 目标角度=%.2f°\n", 
                     targetSpeed, currentSpeed, speedError, targetAngle);
        Serial.printf("[平衡控制] 角度环: 目标=%.2f°, 当前=%.2f°, 误差=%.2f° → 角度PID输出=%.2f\n", 
                     targetAngle, currentAngleX, angleError, pidU);
        Serial.printf("[LQR-PID] pitchRel=%.2f°, gyro=%.2f°/s, pos=%.3fm(零点=%.3f), vel=%.3fm/s, "
                      "angleCtrl=%.3f, gyroCtrl=%.3f, distCtrl=%.3f, speedCtrl=%.3f, u=%.3f, mix=%.2f → 合成平衡=%.2f\n",
                     pitchForLQRDeg, lqrGyro, lqrDistance, lqrDistanceZeroPoint, lqrSpeed,
                     angleControl, gyroControl, distanceControl, speedControl, lqrU, LQR_MIX_K, combinedBalance);
        Serial.printf("[转向控制] 转向环: 目标=%.2f°/s, 当前=%.2f°/s, 误差=%.2f°/s → 转向力矩=%.2f\n", 
                     targetYawRate, currentYawRate, targetYawRate - currentYawRate, steeringTorque);
        
        // 滚转控制调试信息
        if (rollControlEnabled) {
            // 归一化IMU Y角度用于调试显示
            float normalizedAngleY = currentAngleY;
            if (normalizedAngleY < 0) {
                normalizedAngleY += 360.0f;
            }
            
            float expectedRollAngle = ROLL_BASE_EXPECTED_ANGLE - targetRollAngle;
            float rollError = expectedRollAngle - normalizedAngleY;
            Serial.printf("[滚转控制] 目标=%.2f°, 期望IMU_Y=%.2f°, 当前IMU_Y=%.2f° (原始=%.2f°), 误差=%.2f° → 偏移角=%.3f°\n", 
                         targetRollAngle, expectedRollAngle, normalizedAngleY, currentAngleY, rollError, rollAngleOffset);
        }
        
        Serial.printf("[偏移修正] 基础偏移=%.2f°, 修正值=%.3f°, 总偏移=%.2f° (自动修正:%s)\n", 
                     baseAngleOffset, baseAngleOffsetCorrection, baseAngleOffset + baseAngleOffsetCorrection, 
                     autoOffsetEnabled ? "启用" : "禁用");
        Serial.printf("[输出] 左轮=%.2f, 右轮=%.2f (角度PID=%.2f, LQR-PID=%.2f, 转向=%.2f)\n", 
                     leftOutput, rightOutput, pidU, lqrU, steeringTorque);
        Serial.printf("[PID状态] 速度[P=%.3f,I=%.3f,D=%.3f] 角度[P=%.3f,I=%.3f,D=%.3f] 转向[P=%.3f,I=%.3f,D=%.3f] 偏移[P=%.3f,I=%.3f,D=%.3f] 滚转[P=%.3f,I=%.3f,D=%.3f]\n", 
                     speedPID.P, speedPID.I, speedPID.D, anglePID.P, anglePID.I, anglePID.D, steeringPID.P, steeringPID.I, steeringPID.D, offsetPID.P, offsetPID.I, offsetPID.D, rollPID.P, rollPID.I, rollPID.D);
        lastDebugTime = currentTime;
    }
}


