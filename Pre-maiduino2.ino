/*
 * Pre-maiduino2_FINAL_IK.ino
 * 最終修正版 + IK 步行控制 + Hotkey + 安全STOP + GYRO平衡 (可開關)
 * 
 * 功能:
 * - 所有 ASCII 指令 (S MV, S HV, S MULTI, FREE, ?)
 * - MPU6050 陀螺儀 (G 指令)
 * - Batch Mode
 * - 呼吸燈效果
 * - 0x18 多軸同步二進制指令 (有 XOR)
 * - 三個 SHAKE 版本
 * - IK 步行控制 (WALK 指令 + Hotkey)
 * - 更安全版 STOP (先落腳再返 home)
 * - GYRO 平衡檢測 (可開關，預設關閉)
 * - 單腳測試功能 (TEST_LEGS)
 * 
 * 修正:
 * - ASCII 指令用 binaryID 嚟搵伺服 (MV1=21, MV2=22, ...)
 * - 修正 getLeftFootTarget() 正負號 BUG (唔再一字馬)
 * - 交換 MV1 同 MV3 嘅 servoID (硬件交換)
 * - IK 轉換函數跟據實際伺服方向設定:
 *   HV7: 6500向前, 8500向後
 *   HV8: 8500向前, 6500向後
 *   HV9: 6500彎曲, 8500伸直
 *   HV10: 8500彎曲, 6500伸直
 *   HV11: 大數腳尖向下, 細數腳尖向上
 *   HV12: 細數腳尖向下, 大數腳尖向上
 * 
 * 更新日期：2026-03-08
 */

#include <Arduino.h>
#include <Wire.h>

// ===== 手動宣告 Serial2 同 Serial3 =====
HardwareSerial Serial2(PA3, PA2);
HardwareSerial Serial3(PB11, PB10);

// ===== 引入官方 ICS 函式庫 =====
#include <IcsHardSerialClass.h>

// ===== 定義 ICS_FALSE 常數 =====
#ifndef ICS_FALSE
#define ICS_FALSE -1
#endif

// ===== 定義 EN 腳位 =====
#define EN_HV_PIN   PA4
#define EN_MV_PIN   PB2

// ===== 建立兩個通訊物件 =====
IcsHardSerialClass icsHV(&Serial2, EN_HV_PIN, 1250000, 50);
IcsHardSerialClass icsMV(&Serial3, EN_MV_PIN, 1250000, 50);

// ===== 速度定義 =====
#define MIN_SPEED 1
#define MAX_SPEED 127
#define DEFAULT_SPEED 64

// ===== LED 相關定義 =====
#define LED_RED_PIN   PA7
#define LED_GREEN_PIN PB0
#define LED_BLUE_PIN  PB1

// ===== 呼吸燈速度定義 (你改咗做 4) =====
#define BREATH_SPEED 4

// ===== Batch Mode 設定 (你改咗) =====
#define BATCH_BUFFER_SIZE 1024
#define BATCH_TIMEOUT 300

// ===== MPU6050 相關定義 =====
#define MPU6050_ADDR 0x68

struct MPU6050Data {
  float ax, ay, az;
  float gx, gy, gz;
  float temperature;
  int16_t gyroXOffset, gyroYOffset, gyroZOffset;
  bool calibrated;
};

MPU6050Data mpuData;

// ===== Gyro 保護開關 (預設關閉) =====
bool gyroProtectionEnabled = false;

// ===== 25軸伺服資訊 (使用 binaryID 系統, HV:1-14, MV:21-31) =====
// 注意：已經交換 MV1 (binaryID 21) 同 MV3 (binaryID 23) 嘅 servoID
struct ServoInfo {
  uint8_t binaryID;        // 二進制用嘅 ID (HV:1-14, MV:21-31)
  uint8_t servoID;         // 實際 ICS 伺服 ID (1-14 for HV, 1-11 for MV)
  uint16_t homePosition;
  uint16_t currentTunePos;
  uint8_t currentSpeed;
  IcsHardSerialClass* icsPort;
  const char* name;
  uint16_t minAngle;
  uint16_t maxAngle;
  bool isHV;
};

ServoInfo servoList[] = {
  // ===== HV群 (下半身) - binaryID 1-14 =====
  {1,  1,  10200, 10200, DEFAULT_SPEED, &icsHV, "肩ピッチR",   4300, 11500, true},
  {2,  2,  4700, 4700, DEFAULT_SPEED, &icsHV, "肩ピッチL",    3500, 10700, true},
  {3,  3,  7780, 7780, DEFAULT_SPEED, &icsHV, "ヒップヨーR",  6530,  9030, true},
  {4,  4,  7500, 7500, DEFAULT_SPEED, &icsHV, "ヒップヨーL",  6250,  8750, true},
  {5,  5,  7400, 7400, DEFAULT_SPEED, &icsHV, "ヒップロールR",6700,  8300, true},
  {6,  6,  7600, 7600, DEFAULT_SPEED, &icsHV, "ヒップロールL",6700,  8300, true},
  {7,  7,  7500, 7500, DEFAULT_SPEED, &icsHV, "腿ピッチR",    4700, 10200, true},
  {8,  8,  7500, 7500, DEFAULT_SPEED, &icsHV, "腿ピッチL",    4700, 10200, true},
  {9,  9,  7500, 7500, DEFAULT_SPEED, &icsHV, "膝ピッチR",    3950,  7600, true},
  {10, 10, 7500, 7500, DEFAULT_SPEED, &icsHV, "膝ピッチL",    7400, 11050, true},
  {11, 11, 7500, 7500, DEFAULT_SPEED, &icsHV, "足首ピッチR",  5700,  8300, true},
  {12, 12, 7550, 7550, DEFAULT_SPEED, &icsHV, "足首ピッチL",  6750,  9350, true},
  {13, 13, 7825, 7825, DEFAULT_SPEED, &icsHV, "足首ロールR",  6800,  9150, true},
  {14, 14, 7450, 7450, DEFAULT_SPEED, &icsHV, "足首ロールL",  6200,  8450, true},

  // ===== MV群 (上半身) - binaryID 21-31 =====
  // 注意：以下兩個 entry 已經交換咗 servoID
  {21, 3,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭萌",         7200,  8400, false},  // 原本 MV3 嘅 data，而家用 ID 3
  {22, 2,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭ヨー",       5000, 10000, false},
  {23, 1,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭ピッチ",     6900,  8100, false},  // 原本 MV1 嘅 data，而家用 ID 1
  {24, 4,  9800, 9800, DEFAULT_SPEED, &icsMV, "肩ロールR",    7450, 10350, false},
  {25, 5,  5200, 5200, DEFAULT_SPEED, &icsMV, "肩ロールL",    4550,  7550, false},
  {26, 6,  7500, 7500, DEFAULT_SPEED, &icsMV, "上腕ヨーR",    4000, 11000, false},
  {27, 7,  7500, 7500, DEFAULT_SPEED, &icsMV, "上腕ヨーL",    4000, 11000, false},
  {28, 8,  7500, 7500, DEFAULT_SPEED, &icsMV, "肘ピッチR",    7100, 11000, false},
  {29, 9,  7500, 7500, DEFAULT_SPEED, &icsMV, "肘ピッチL",    4000,  7900, false},
  {30, 10, 5000, 5000, DEFAULT_SPEED, &icsMV, "手首ヨーR",    3500, 11500, false},
  {31, 11, 10000, 10000, DEFAULT_SPEED, &icsMV, "手首ヨーL",  3500, 11500, false}
};

#define TOTAL_SERVO_NUM (sizeof(servoList) / sizeof(servoList[0]))

// ===== 系統狀態 =====
String inputBuffer = "";

// ===== Batch Mode 變數 =====
char batchBuffer[BATCH_BUFFER_SIZE];
int batchIndex = 0;
bool batchMode = false;
unsigned long lastCharTime = 0;

// ===== IK 步行控制類別前置宣告 =====
class IKSolver;
class WalkGenerator;

// ===== 函式原型 =====
void initLED();
void setLEDRed();
void setLEDGreen();
void setLEDBlue();
void setLEDPurple();
void setLEDOff();
void breathLED(int pin, int speed);
void initMPU6050();
void calibrateGyro(int samples = 500);
bool readMPU6050();
void initServos();
void moveAllServosToHome();
void safeStop();
void processCommand(String cmd);
void showHelp();
bool processASCIICommand(String cmd);
bool processMultiCommand(String cmd);
bool processFreeCommand(String cmd);
void executeBatchCommands();
void process0x18();
ServoInfo* findServoByBinaryID(uint8_t binaryID);
void actionShakeBox_ASCII();
void actionShakeBox_CPP();
void actionShakeBox_ICS();
void testSingleLegs();
// ===== IK Solver 類別 (重寫版 - 修正縮進和邏輯) =====
class IKSolver {
private:
  const float thighLength = 65.0;    // 大腿長度 (mm)
  const float shinLength = 65.0;     // 小腿長度 (mm)
  const float BASE_Z = 60.0;         // 腳踝到腳底距離 (mm)
  const float hipWidth = 40.0;       // 髖寬 (mm)
  
  // Home 位置 (對應 0 度)
  const uint16_t HOME_HV3 = 7780;    // 右髖yaw
  const uint16_t HOME_HV4 = 7500;    // 左髖yaw
  const uint16_t HOME_HV5 = 7400;    // 右髖roll
  const uint16_t HOME_HV6 = 7600;    // 左髖roll
  const uint16_t HOME_HV7 = 7500;    // 右髖pitch
  const uint16_t HOME_HV8 = 7500;    // 左髖pitch
  const uint16_t HOME_HV9 = 7500;    // 右膝
  const uint16_t HOME_HV10 = 7500;   // 左膝
  const uint16_t HOME_HV11 = 7500;   // 右踝pitch
  const uint16_t HOME_HV12 = 7550;   // 左踝pitch
  const uint16_t HOME_HV13 = 7825;   // 右踝roll
  const uint16_t HOME_HV14 = 7450;   // 左踝roll
  
  // Servo 轉換常數 (3500-11500 = 8000單位對應270度)
  const float PULSE_PER_DEG = 8000.0 / 270.0;  // ≈ 29.63
  
  // 角度限制 (根據硬件實際範圍)
  const float MAX_HIP_PITCH = radians(85);    // 向前最大 85度
  const float MIN_HIP_PITCH = radians(-85);   // 向後最大 85度
  const float MAX_KNEE = radians(115);        // 膝蓋彎曲最大 115度
  const float MIN_KNEE = radians(0);          // 膝蓋伸直 0度
  const float MAX_ANKLE_RIGHT = radians(60);  // 右腳向上 60度
  const float MIN_ANKLE_RIGHT = radians(-27); // 右腳向下 27度
  const float MAX_ANKLE_LEFT = radians(60);   // 左腳向上 60度
  const float MIN_ANKLE_LEFT = radians(-27);  // 左腳向下 27度
  
public:
  struct RightLegAngles {
    uint16_t hipYaw;      // HV3
    uint16_t hipRoll;     // HV5
    uint16_t hipPitch;    // HV7
    uint16_t knee;        // HV9
    uint16_t anklePitch;  // HV11
    uint16_t ankleRoll;   // HV13
  };
  
  struct LeftLegAngles {
    uint16_t hipYaw;      // HV4
    uint16_t hipRoll;     // HV6
    uint16_t hipPitch;    // HV8
    uint16_t knee;        // HV10
    uint16_t anklePitch;  // HV12
    uint16_t ankleRoll;   // HV14
  };
  
  bool solveRightLeg(float targetX, float targetY, float targetZ,
                     float bodyYaw, float bodyRoll,
                     RightLegAngles &angles) {
    // 設置髖部yaw和roll
    angles.hipYaw = hipYawToServoRight(bodyYaw);
    angles.hipRoll = hipRollToServoRight(bodyRoll);
    
    // 計算腳踝位置
    float x = targetX;
    float y = targetY;
    float z = targetZ + BASE_Z;
    
    // IK計算
    float distance = sqrt(x*x + z*z);
    if (distance > thighLength + shinLength + 1.0 || distance < abs(thighLength - shinLength) - 1.0) {
      return false;
    }
    
    // 膝蓋角度
    float cosKnee = (thighLength*thighLength + shinLength*shinLength - distance*distance) 
                   / (2 * thighLength * shinLength);
float kneeAngle = PI - acos(constrain(cosKnee, -1.0, 1.0));  // 轉換為相對角度
    kneeAngle = constrain(kneeAngle, MIN_KNEE, MAX_KNEE);
    
    // 髖部pitch角度
float alpha = atan2(x, -z);  // 修正坐標系統 (z是負數)
    float beta = acos((thighLength*thighLength + distance*distance - shinLength*shinLength) 
                     / (2 * thighLength * distance));
    float hipAngle = alpha - beta;
    hipAngle = constrain(hipAngle, MIN_HIP_PITCH, MAX_HIP_PITCH);
    
    // 踝部pitch角度
    float ankleAngle = -(hipAngle + kneeAngle);
    ankleAngle = constrain(ankleAngle, MIN_ANKLE_RIGHT, MAX_ANKLE_RIGHT);
    
    // 轉換為servo值
    angles.hipPitch = hipPitchToServoRight(hipAngle);
    angles.knee = kneeToServoRight(kneeAngle);
    angles.anklePitch = anklePitchToServoRight(ankleAngle);
    angles.ankleRoll = ankleRollToServoRight(bodyRoll);
    
    return true;
  }
  
  bool solveLeftLeg(float targetX, float targetY, float targetZ,
                    float bodyYaw, float bodyRoll,
                    LeftLegAngles &angles) {
    // 設置髖部yaw和roll
    angles.hipYaw = hipYawToServoLeft(bodyYaw);
    angles.hipRoll = hipRollToServoLeft(bodyRoll);
    
    // 計算腳踝位置
    float x = targetX;
    float y = targetY;
    float z = targetZ + BASE_Z;
    
    // IK計算
    float distance = sqrt(x*x + z*z);
    if (distance > thighLength + shinLength + 1.0 || distance < abs(thighLength - shinLength) - 1.0) {
      return false;
    }
    
    // 膝蓋角度
    float cosKnee = (thighLength*thighLength + shinLength*shinLength - distance*distance) 
                   / (2 * thighLength * shinLength);
float kneeAngle = PI - acos(constrain(cosKnee, -1.0, 1.0));  // 轉換為相對角度
    kneeAngle = constrain(kneeAngle, MIN_KNEE, MAX_KNEE);
    
    // 髖部pitch角度
float alpha = atan2(x, -z);  // 修正坐標系統 (z是負數)
    float beta = acos((thighLength*thighLength + distance*distance - shinLength*shinLength) 
                     / (2 * thighLength * distance));
    float hipAngle = alpha - beta;
    hipAngle = constrain(hipAngle, MIN_HIP_PITCH, MAX_HIP_PITCH);
    
    // 踝部pitch角度
    float ankleAngle = -(hipAngle + kneeAngle);
    ankleAngle = constrain(ankleAngle, MIN_ANKLE_LEFT, MAX_ANKLE_LEFT);
    
    // 轉換為servo值
    angles.hipPitch = hipPitchToServoLeft(hipAngle);
    angles.knee = kneeToServoLeft(kneeAngle);
    angles.anklePitch = anklePitchToServoLeft(ankleAngle);
    angles.ankleRoll = ankleRollToServoLeft(bodyRoll);
    
    return true;
  }
  
private:
  // ===== 右腿轉換函數 =====
  uint16_t hipYawToServoRight(float rad) {
    float deg = degrees(rad);
    int16_t diff = round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV3 + diff, 6530, 9030);
  }
  
  uint16_t hipRollToServoRight(float rad) {
    float deg = degrees(rad);
    int16_t diff = round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV5 + diff, 6700, 8300);
  }
  
  uint16_t hipPitchToServoRight(float rad) {
    float deg = degrees(rad);
    int16_t diff = -round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV7 + diff, 4700, 10200);
  }
  
  uint16_t kneeToServoRight(float rad) {
    float deg = degrees(rad);
    int16_t diff = -round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV9 + diff, 3950, 7600);
  }
  
  uint16_t anklePitchToServoRight(float rad) {
    float deg = degrees(rad);
    int16_t diff = -round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV11 + diff, 5700, 8300);
  }
  
  uint16_t ankleRollToServoRight(float rad) {
    float deg = degrees(rad);
    int16_t diff = round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV13 + diff, 6800, 9150);
  }
  
  // ===== 左腿轉換函數 =====
  uint16_t hipYawToServoLeft(float rad) {
    float deg = degrees(rad);
    int16_t diff = -round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV4 + diff, 6250, 8750);
  }
  
  uint16_t hipRollToServoLeft(float rad) {
    float deg = degrees(rad);
    int16_t diff = -round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV6 + diff, 6700, 8300);
  }
  
  uint16_t hipPitchToServoLeft(float rad) {
    float deg = degrees(rad);
    int16_t diff = round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV8 + diff, 4700, 10200);
  }
  
  uint16_t kneeToServoLeft(float rad) {
    float deg = degrees(rad);
    int16_t diff = round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV10 + diff, 7400, 11050);
  }
  
  uint16_t anklePitchToServoLeft(float rad) {
    float deg = degrees(rad);
    int16_t diff = round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV12 + diff, 6750, 9350);
  }
  
  uint16_t ankleRollToServoLeft(float rad) {
    float deg = degrees(rad);
    int16_t diff = -round(deg * PULSE_PER_DEG);
    return constrain(HOME_HV14 + diff, 6200, 8450);
  }
};

// ===== 步行軌跡生成器 (重寫版) =====
class WalkGenerator {
private:
  IKSolver ik;
  
  float stepLength = 40.0;      // 步長 - 增大到40mm
  float stepHeight = 30.0;      // 抬腳高度 - 增大到30mm
  float hipWidth = 40.0;        // 髖寬
  float cycleTime = 2.0;        // 步態週期 (秒)
  const float BASE_Z = 60.0;    // 腳踝到腳底距離
  
  float phase = 0.0;
  unsigned long lastUpdate = 0;
  
  float targetVelX = 0.0;
  float targetVelY = 0.0;
  float targetTurnRate = 0.0;
  
  uint8_t walkSpeed = 40;
  bool walking = false;
  int stepsRemaining = 0;
  
  float currentRX = 0, currentRY = 20, currentRZ = 0;
  float currentLX = 0, currentLY = -20, currentLZ = 0;
  
  unsigned long lastServoSend = 0;
  IKSolver::RightLegAngles lastRightAngles;
  IKSolver::LeftLegAngles lastLeftAngles;
  bool lastAnglesValid = false;
  
public:
  WalkGenerator() {
    lastUpdate = millis();
    lastServoSend = millis();
  }
  
  void setWalkParams(float length, float height, float time) {
    stepLength = length;
    stepHeight = height;
    cycleTime = time;
  }
  
  void setSpeed(uint8_t speed) {
    walkSpeed = speed;
  }
  
  void setVelocity(float vx, float vy, float turn) {
    targetVelX = vx;
    targetVelY = vy;
    targetTurnRate = turn;
  }
  
  void stop() {
    targetVelX = 0;
    targetVelY = 0;
    targetTurnRate = 0;
    walking = false;
  }
  
  void updatePhase() {
    unsigned long now = millis();
    float deltaTime = (now - lastUpdate) / 1000.0;
    if (deltaTime > 0.05) deltaTime = 0.02;
    
    if (walking) {
      float phaseSpeed = 2.0 / cycleTime;
      phase += phaseSpeed * deltaTime;
      
      while (phase >= 2.0) {
        phase -= 2.0;
        stepsRemaining--;
      }
      
      if (stepsRemaining <= 0) {
        walking = false;
        phase = 0;
      }
    }
    
    lastUpdate = now;
  }
void getRightFootTarget(float &x, float &y, float &z, float &roll) {
    float t = phase;
    
    if (t < 1.0) {
      // 支撐相: x從 +stepLength/2 到 -stepLength/2
      x = stepLength * (0.5 - t);
      y = hipWidth / 2;
      z = -130.0 - BASE_Z;  // 腳底位置
    } else {
      // 擺動相: x從 -stepLength/2 到 +stepLength/2
      float swingT = t - 1.0;
      x = stepLength * (swingT - 0.5);
      y = hipWidth / 2;
      z = -130.0 - BASE_Z + stepHeight * sin(swingT * PI);
    }
roll = 0;
    currentRX = x;
    currentRY = y;
    currentRZ = z;
  }
  
  void getLeftFootTarget(float &x, float &y, float &z, float &roll) {
    float t = phase;
    
    if (t < 1.0) {
      // 擺動相: x從 -stepLength/2 到 +stepLength/2
      float swingT = t;
      x = stepLength * (swingT - 0.5);
      y = -hipWidth / 2;
      z = -130.0 - BASE_Z + stepHeight * sin(swingT * PI);
    } else {
      // 支撐相: x從 +stepLength/2 到 -stepLength/2
      float supportT = t - 1.0;
      x = stepLength * (0.5 - supportT);
      y = -hipWidth / 2;
      z = -130.0 - BASE_Z;
    }
    roll = 0;
    currentLX = x;
    currentLY = y;
    currentLZ = z;
  }
  
bool computeIK() {
    Serial1.println(F("DEBUG: computeIK called"));
    
    // 调试：打印步态参数
    Serial1.print(F("DEBUG: stepLength="));
    Serial1.print(stepLength);
    Serial1.print(F(" stepHeight="));
    Serial1.println(stepHeight);
    
    float rx, ry, rz, rroll;
    float lx, ly, lz, lroll;
    getRightFootTarget(rx, ry, rz, rroll);
    getLeftFootTarget(lx, ly, lz, lroll);
    Serial1.print(rx);
    Serial1.print(F(" y="));
    Serial1.print(ry);
    Serial1.print(F(" z="));
    Serial1.println(rz);
    
    Serial1.print(F("DEBUG: 左腿坐标 x="));
    Serial1.print(lx);
    Serial1.print(F(" y="));
    Serial1.print(ly);
    Serial1.print(F(" z="));
    Serial1.println(lz);
    
    float turnYaw = 0, bodyRoll = 0;
    bool rightOK = ik.solveRightLeg(rx, ry, rz, turnYaw, bodyRoll, lastRightAngles);
    bool leftOK = ik.solveLeftLeg(lx, ly, lz, turnYaw, bodyRoll, lastLeftAngles);
    
    Serial1.print(F("DEBUG: rightOK="));
    Serial1.print(rightOK);
    Serial1.print(F(" leftOK="));
    Serial1.println(leftOK);
    
    lastAnglesValid = rightOK && leftOK;
    return lastAnglesValid;
  }
  
  void sendAngles() {
    Serial1.print(F("DEBUG: 右腿servo: HV3="));
    Serial1.print(lastRightAngles.hipYaw);
    Serial1.print(F(" HV5="));
    Serial1.print(lastRightAngles.hipRoll);
    Serial1.print(F(" HV7="));
    Serial1.print(lastRightAngles.hipPitch);
    Serial1.print(F(" HV9="));
    Serial1.print(lastRightAngles.knee);
    Serial1.print(F(" HV11="));
    Serial1.print(lastRightAngles.anklePitch);
    Serial1.print(F(" HV13="));
    Serial1.println(lastRightAngles.ankleRoll);
    
    Serial1.print(F("DEBUG: 左腿servo: HV4="));
    Serial1.print(lastLeftAngles.hipYaw);
    Serial1.print(F(" HV6="));
    Serial1.print(lastLeftAngles.hipRoll);
    Serial1.print(F(" HV8="));
    Serial1.print(lastLeftAngles.hipPitch);
    Serial1.print(F(" HV10="));
    Serial1.print(lastLeftAngles.knee);
    Serial1.print(F(" HV12="));
    Serial1.print(lastLeftAngles.anklePitch);
    Serial1.print(F(" HV14="));
    Serial1.println(lastLeftAngles.ankleRoll);
    
    if (!lastAnglesValid) return;
    
    // 設置速度
    static bool speedSet = false;
    if (!speedSet) {
      icsHV.setSpd(3, walkSpeed);
      icsHV.setSpd(5, walkSpeed);
      icsHV.setSpd(7, walkSpeed);
      icsHV.setSpd(9, walkSpeed);
      icsHV.setSpd(11, walkSpeed);
      icsHV.setSpd(13, walkSpeed);
      icsHV.setSpd(4, walkSpeed);
      icsHV.setSpd(6, walkSpeed);
      icsHV.setSpd(8, walkSpeed);
      icsHV.setSpd(10, walkSpeed);
      icsHV.setSpd(12, walkSpeed);
      icsHV.setSpd(14, walkSpeed);
      speedSet = true;
      delay(2);
    }
    
    // 發送位置指令
    icsHV.setPos(3, lastRightAngles.hipYaw);
    icsHV.setPos(5, lastRightAngles.hipRoll);
    icsHV.setPos(7, lastRightAngles.hipPitch);
    icsHV.setPos(9, lastRightAngles.knee);
    icsHV.setPos(11, lastRightAngles.anklePitch);
    icsHV.setPos(13, lastRightAngles.ankleRoll);
    icsHV.setPos(4, lastLeftAngles.hipYaw);
    icsHV.setPos(6, lastLeftAngles.hipRoll);
    icsHV.setPos(8, lastLeftAngles.hipPitch);
    icsHV.setPos(10, lastLeftAngles.knee);
    icsHV.setPos(12, lastLeftAngles.anklePitch);
    icsHV.setPos(14, lastLeftAngles.ankleRoll);
  }
  
  void updateWalk() {
    if (!walking) return;
    updatePhase();
    computeIK();
    unsigned long now = millis();
    if (now - lastServoSend >= 30) {
      sendAngles();
      lastServoSend = now;
      Serial1.print(F("DEBUG: phase="));
      Serial1.print(phase);
      Serial1.print(F(" steps="));
      Serial1.println(stepsRemaining);
    }
  }
  
  void walkSteps(int steps) {
    walking = true;
    stepsRemaining = steps;
    phase = 0;
    lastUpdate = millis();
    lastServoSend = millis();
    
    Serial1.print(F("開始行 "));
    Serial1.print(steps);
    Serial1.println(F(" 步"));
    
    // 執行步行循環
    while (stepsRemaining > 0) {
      updateWalk();
      delay(20);  // 50Hz 更新頻率
    }
    
    // 完成後回到 home 位置
    moveAllServosToHome();
    Serial1.println(F("✅ 步行完成"));
  }
  
  bool isWalking() {
    return walking;
  }
  
  void safeStop() {
    Serial1.println(F("🛑 安全停止中..."));
    walking = false;
    moveAllServosToHome();
    Serial1.println(F("✅ 安全停止完成"));
  }
};
WalkGenerator walkGen;

// ===== LED 控制 =====
void initLED() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  setLEDOff();
}

void setLEDRed() {
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
}

void setLEDGreen() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_BLUE_PIN, LOW);
}

void setLEDBlue() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, HIGH);
}

void setLEDPurple() {
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, HIGH);
}

void setLEDOff() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
}

void breathLED(int pin, int speed) {
  for (int brightness = 0; brightness <= 255; brightness++) {
    analogWrite(pin, brightness);
    delay(speed);
  }
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(pin, brightness);
    delay(speed);
  }
}

// ===== MPU6050 函式 =====
void writeMPU6050Reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readMPU6050Reg(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

int16_t readMPU6050Word(uint8_t regH) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(regH);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

void initMPU6050() {
  Wire.begin();
  Wire.setClock(400000);
  
  uint8_t whoami = readMPU6050Reg(0x75);
  if (whoami != 0x68) {
    Serial1.println(F("MPU6050 連接失敗！"));
    mpuData.calibrated = false;
    return;
  }
  
  writeMPU6050Reg(0x6B, 0x00);
  delay(100);
  
  writeMPU6050Reg(0x1B, 0x00);
  writeMPU6050Reg(0x1C, 0x00);
  writeMPU6050Reg(0x1A, 0x03);
  writeMPU6050Reg(0x19, 0x07);
  
  Serial1.println(F("MPU6050 初始化成功"));
  mpuData.calibrated = false;
}

void calibrateGyro(int samples) {
  Serial1.println(F("校準陀螺儀，請保持靜止..."));
  
  int32_t sumGx = 0, sumGy = 0, sumGz = 0;
  
  for (int i = 0; i < samples; i++) {
    sumGx += readMPU6050Word(0x43);
    sumGy += readMPU6050Word(0x45);
    sumGz += readMPU6050Word(0x47);
    delay(5);
  }
  
  mpuData.gyroXOffset = sumGx / samples;
  mpuData.gyroYOffset = sumGy / samples;
  mpuData.gyroZOffset = sumGz / samples;
  mpuData.calibrated = true;
  
  Serial1.println(F("校準完成"));
}

bool readMPU6050() {
  if (!mpuData.calibrated) return false;
  
  uint8_t buffer[14];
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050_ADDR, (uint8_t)14);
  if (Wire.available() < 14) return false;
  
  for (int i = 0; i < 14; i++) {
    buffer[i] = Wire.read();
  }
  
  int16_t ax_raw = (buffer[0] << 8) | buffer[1];
  int16_t ay_raw = (buffer[2] << 8) | buffer[3];
  int16_t az_raw = (buffer[4] << 8) | buffer[5];
  int16_t temp_raw = (buffer[6] << 8) | buffer[7];
  int16_t gx_raw = (buffer[8] << 8) | buffer[9];
  int16_t gy_raw = (buffer[10] << 8) | buffer[11];
  int16_t gz_raw = (buffer[12] << 8) | buffer[13];
  
  mpuData.ax = ax_raw / 16384.0f;
  mpuData.ay = ay_raw / 16384.0f;
  mpuData.az = az_raw / 16384.0f;
  mpuData.temperature = (temp_raw / 340.0f) + 36.53f;
  
  mpuData.gx = (gx_raw - mpuData.gyroXOffset) / 131.0f;
  mpuData.gy = (gy_raw - mpuData.gyroYOffset) / 131.0f;
  mpuData.gz = (gz_raw - mpuData.gyroZOffset) / 131.0f;
  
  return true;
}

// ===== 初始化伺服 =====
void initServos() {
  pinMode(EN_HV_PIN, OUTPUT);
  pinMode(EN_MV_PIN, OUTPUT);
  
  Serial1.print(F("初始化 HV 伺服..."));
  digitalWrite(EN_HV_PIN, HIGH);
  delay(10);
  digitalWrite(EN_HV_PIN, LOW);
  
  if (icsHV.begin()) {
    Serial1.println(F("完成"));
  } else {
    Serial1.println(F("失敗！"));
  }
  
  Serial1.print(F("初始化 MV 伺服..."));
  digitalWrite(EN_MV_PIN, HIGH);
  delay(10);
  digitalWrite(EN_MV_PIN, LOW);
  
  if (icsMV.begin()) {
    Serial1.println(F("完成"));
  } else {
    Serial1.println(F("失敗！"));
  }
  
  delay(10);
}

// ===== 根據 binaryID 搵伺服 =====
ServoInfo* findServoByBinaryID(uint8_t binaryID) {
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    if (servoList[i].binaryID == binaryID) {
      return &servoList[i];
    }
  }
  return NULL;
}

// ===== 處理 0x18 多軸同步二進制指令 (有 XOR) =====
void process0x18() {
  while (Serial1.available() >= 2) {
    uint8_t firstByte = Serial1.peek();
    
    if (firstByte < 5 || firstByte > 64) break;
    if (Serial1.available() < firstByte) return;
    
    uint8_t packet[64];
    for (int i = 0; i < firstByte; i++) {
      packet[i] = Serial1.read();
    }
    
    if (packet[1] != 0x18) continue;
    
    uint8_t xorVal = 0;
    for (int i = 0; i < firstByte - 1; i++) {
      xorVal ^= packet[i];
    }
    
    if (xorVal != packet[firstByte - 1]) {
      Serial1.println(F("[0x18] XOR 錯誤"));
      setLEDRed();
      delay(50);
      setLEDBlue();
      continue;
    }
    
    setLEDPurple();
    
    uint8_t speed = packet[2];
    int servoCount = (firstByte - 4) / 3;
    
    for (int i = 0; i < servoCount; i++) {
      uint8_t binaryID = packet[3 + i*3];
      uint8_t high = packet[4 + i*3];
      uint8_t low = packet[5 + i*3];
      uint16_t angle = (high << 7) | low;
      
      ServoInfo* servo = findServoByBinaryID(binaryID);
      
      if (servo != NULL) {
        servo->icsPort->setSpd(servo->servoID, speed);
        delay(1);
        servo->icsPort->setPos(servo->servoID, angle);
        servo->currentTunePos = angle;
        servo->currentSpeed = speed;
      }
    }
    
    setLEDBlue();
  }
}

// ===== 移動到 Home Point =====
void moveAllServosToHome() {
  Serial1.println(F("\n🚀 所有伺服回家，速度64"));
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    s->icsPort->setSpd(s->servoID, 64);
    s->currentSpeed = 64;
  }
  
  delay(5);
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    s->icsPort->setPos(s->servoID, s->homePosition);
    s->currentTunePos = s->homePosition;
  }
  
  Serial1.println(F("✅ 回家指令已發送"));
}

// ===== 脫力功能 =====
bool processFreeCommand(String cmd) {
  cmd.trim();
  
  if (!cmd.startsWith("FREE ")) return false;
  
  String params = cmd.substring(5);
  params.trim();
  
  if (params == "ALL") {
    Serial1.println(F("💤 脫力所有伺服"));
    for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
      ServoInfo *s = &servoList[i];
      s->icsPort->setFree(s->servoID);
      delay(2);
    }
    Serial1.println(F("✅ 所有伺服已脫力"));
    return true;
  }
  
  int spacePos = params.indexOf(' ');
  if (spacePos <= 0) return false;
  
  String group = params.substring(0, spacePos);
  group.toUpperCase();
  int id = params.substring(spacePos + 1).toInt();
  
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    bool groupMatch = (group == "HV" && servoList[i].isHV) || 
                      (group == "MV" && !servoList[i].isHV);
    
    if (groupMatch && servoList[i].binaryID == id) {
      servoList[i].icsPort->setFree(servoList[i].servoID);
      Serial1.print(F("💤 脫力 "));
      Serial1.print(group);
      Serial1.print(F(" ID "));
      Serial1.println(id);
      return true;
    }
  }
  
  Serial1.println(F("❌ 找不到對應的伺服"));
  return true;
}

// ===== 處理多軸同步指令 (ASCII 版) =====
bool processMultiCommand(String cmd) {
  cmd.trim();
  
  if (!cmd.startsWith("S MULTI ")) return false;
  
  String params = cmd.substring(8);
  params.trim();
  
  int firstSpace = params.indexOf(' ');
  if (firstSpace <= 0) return false;
  
  int speed = params.substring(0, firstSpace).toInt();
  if (speed < 0) speed = 0;
  if (speed > 127) speed = 127;
  
  String rest = params.substring(firstSpace + 1);
  rest.trim();
  
  int secondSpace = rest.indexOf(' ');
  if (secondSpace <= 0) return false;
  
  int count = rest.substring(0, secondSpace).toInt();
  if (count < 1 || count > 25) {
    Serial1.println(F("❌ 數量錯誤 (1-25)"));
    return true;
  }
  
  String data = rest.substring(secondSpace + 1);
  data.trim();
  
  int idCount = 0;
  int index = 0;
  
  while (idCount < count && index < data.length()) {
    int spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
    String idStr = data.substring(index, spacePos);
    idStr.trim();
    index = spacePos + 1;
    
    if (idStr.length() < 3) break;
    
    String group = idStr.substring(0, 2);
    group.toUpperCase();
    int binaryId = idStr.substring(2).toInt();
    
    spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
    int angle = data.substring(index, spacePos).toInt();
    index = spacePos + 1;
    
    ServoInfo* servo = findServoByBinaryID(binaryId);
    
    if (servo != NULL) {
      servo->icsPort->setSpd(servo->servoID, speed);
      servo->currentSpeed = speed;
    }
    
    idCount++;
  }
  
  delay(5);
  
  idCount = 0;
  index = 0;
  
  while (idCount < count && index < data.length()) {
    int spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
    String idStr = data.substring(index, spacePos);
    idStr.trim();
    index = spacePos + 1;
    
    if (idStr.length() < 3) break;
    
    String group = idStr.substring(0, 2);
    group.toUpperCase();
    int binaryId = idStr.substring(2).toInt();
    
    spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
    int angle = data.substring(index, spacePos).toInt();
    index = spacePos + 1;
    
    ServoInfo* servo = findServoByBinaryID(binaryId);
    
    if (servo != NULL) {
      if (angle >= servo->minAngle && angle <= servo->maxAngle) {
        servo->icsPort->setPos(servo->servoID, angle);
        servo->currentTunePos = angle;
      }
    }
    
    idCount++;
  }
  
  setLEDGreen();
  delay(20);
  setLEDBlue();
  
  return true;
}

// ===== 處理ASCII指令 =====
bool processASCIICommand(String cmd) {
  cmd.trim();
  
  if (cmd.startsWith("FREE ")) return processFreeCommand(cmd);
  if (cmd.startsWith("S MULTI ")) return processMultiCommand(cmd);
  
  if (cmd.startsWith("S ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
    int fourthSpace = cmd.indexOf(' ', thirdSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0 && thirdSpace > 0) {
      String group = cmd.substring(firstSpace + 1, secondSpace);
      int binaryId = cmd.substring(secondSpace + 1, thirdSpace).toInt();
      int angle = cmd.substring(thirdSpace + 1, (fourthSpace > 0 ? fourthSpace : cmd.length())).toInt();
      
      group.toUpperCase();
      
      ServoInfo* servo = findServoByBinaryID(binaryId);
      
      if (servo != NULL) {
        bool groupMatch = (group == "MV" && !servo->isHV) || (group == "HV" && servo->isHV);
        
        if (groupMatch) {
          if (angle >= servo->minAngle && angle <= servo->maxAngle) {
            
            if (fourthSpace > 0) {
              int speed = cmd.substring(fourthSpace + 1).toInt();
              if (speed < 0) speed = 0;
              if (speed > 127) speed = 127;
              
              servo->icsPort->setSpd(servo->servoID, speed);
              servo->currentSpeed = speed;
            }
            
            servo->icsPort->setPos(servo->servoID, angle);
            servo->currentTunePos = angle;
            
            setLEDGreen();
            delay(20);
            setLEDBlue();
          } else {
            Serial1.print(F("角度超出範圍: "));
            Serial1.print(servo->minAngle);
            Serial1.print(F("-"));
            Serial1.println(servo->maxAngle);
          }
          return true;
        }
      }
      Serial1.println(F("找不到對應的伺服"));
    }
    return true;
  }
  
  if (cmd.startsWith("? ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      String group = cmd.substring(firstSpace + 1, secondSpace);
      int binaryId = cmd.substring(secondSpace + 1).toInt();
      
      group.toUpperCase();
      
      ServoInfo* servo = findServoByBinaryID(binaryId);
      
      if (servo != NULL) {
        bool groupMatch = (group == "MV" && !servo->isHV) || (group == "HV" && servo->isHV);
        
        if (groupMatch) {
          int pos = servo->icsPort->setPos(servo->servoID, servo->currentTunePos);
          if (pos != ICS_FALSE) {
            Serial1.print(group);
            Serial1.print(F(" ID "));
            Serial1.print(binaryId);
            Serial1.print(F(" 角度: "));
            Serial1.println(pos);
            
            servo->currentTunePos = pos;
          }
          return true;
        }
      }
    }
    return true;
  }
  
  return false;
}

// ===== 執行 Batch 指令 =====
void executeBatchCommands() {
  if (batchIndex == 0) return;
  
  batchBuffer[batchIndex] = '\0';
  
  char* line = strtok(batchBuffer, "\n");
  while (line != NULL) {
    char* end = line + strlen(line) - 1;
    while (end >= line && (*end == '\r' || *end == '\n')) {
      *end-- = '\0';
    }
    
    if (strlen(line) > 0) {
      String cmd = String(line);
      if (!processASCIICommand(cmd)) {
        processCommand(cmd);
      }
    }
    
    line = strtok(NULL, "\n");
  }
  
  batchIndex = 0;
  setLEDGreen();
  delay(50);
  setLEDBlue();
}

// ===== 三個 SHAKE 版本 =====
void actionShakeBox_ASCII() {
  Serial1.println(F("\n📦 [ASCII] Shake a Box"));
  
  processMultiCommand("S MULTI 37 6 HV2 6538 MV5 5247 MV9 6763 HV1 8362 MV4 9753 MV8 8237");
  delay(700);
  
  for (int i = 0; i < 8; i++) {
    if (i % 2 == 0) {
      processMultiCommand("S MULTI 46 2 HV2 5848 HV1 9052");
    } else {
      processMultiCommand("S MULTI 46 2 HV2 6538 HV1 8362");
    }
    delay(300);
  }
  
  Serial1.println(F("✅ [ASCII] 完成"));
}

void actionShakeBox_CPP() {
  Serial1.println(F("\n📦 [C++] Shake a Box"));
  
  icsHV.setSpd(2, 37); icsMV.setSpd(5, 37); icsMV.setSpd(9, 37);
  icsHV.setSpd(1, 37); icsMV.setSpd(4, 37); icsMV.setSpd(8, 37);
  delay(2);
  
  icsHV.setPos(2, 6538);
  icsMV.setPos(5, 5247);
  icsMV.setPos(9, 6763);
  icsHV.setPos(1, 8362);
  icsMV.setPos(4, 9753);
  icsMV.setPos(8, 8237);
  
  delay(700);
  
  for (int i = 0; i < 8; i++) {
    if (i % 2 == 0) {
      icsHV.setSpd(2, 46); icsHV.setSpd(1, 46);
      delay(2);
      icsHV.setPos(2, 5848);
      icsHV.setPos(1, 9052);
    } else {
      icsHV.setSpd(2, 46); icsHV.setSpd(1, 46);
      delay(2);
      icsHV.setPos(2, 6538);
      icsHV.setPos(1, 8362);
    }
    delay(300);
  }
  
  Serial1.println(F("✅ [C++] 完成"));
}

// ===== SHAKE_ICS 修正版：直接解析 frame，唔經 Serial1 =====
void actionShakeBox_ICS() {
    Serial1.println(F("\n📦 [ICS] Shake a Box (直接解析)"));

    // frame0: 初始化姿勢
    uint8_t frame0[] = {
        0x16, 0x18, 0x25, 
        0x02, 0x33, 0x0A,  // HV2 6538
        0x19, 0x28, 0x7F,  // MV5 5247
        0x1D, 0x34, 0x6B,  // MV9 6763
        0x01, 0x41, 0x2A,  // HV1 8362
        0x18, 0x4C, 0x19,  // MV4 9753
        0x1C, 0x40, 0x2D,  // MV8 8237
        0x4A               // XOR
    };

    // frame1: HV2=5848, HV1=9052
    uint8_t frame1[] = {
        0x0A, 0x18, 0x2E,
        0x02, 0x2D, 0x58,
        0x01, 0x46, 0x5C,
        0x50
    };

    // frame2: HV2=6538, HV1=8362
    uint8_t frame2[] = {
        0x0A, 0x18, 0x2E,
        0x02, 0x33, 0x0A,
        0x01, 0x41, 0x2A,
        0x6D
    };

    // 解析 frame0 並執行
    uint8_t speed = frame0[2];
    int servoCount = (frame0[0] - 4) / 3;
    
    for (int i = 0; i < servoCount; i++) {
        uint8_t binaryID = frame0[3 + i*3];
        uint8_t high = frame0[4 + i*3];
        uint8_t low = frame0[5 + i*3];
        uint16_t angle = (high << 7) | low;
        
        ServoInfo* servo = findServoByBinaryID(binaryID);
        if (servo != NULL) {
            servo->icsPort->setSpd(servo->servoID, speed);
            delay(1);
            servo->icsPort->setPos(servo->servoID, angle);
        }
    }
    
    delay(700);

    // 來回 8 次
    for (int i = 0; i < 8; i++) {
        uint8_t* frame = (i % 2 == 0) ? frame1 : frame2;
        
        speed = frame[2];
        servoCount = (frame[0] - 4) / 3;
        
        for (int j = 0; j < servoCount; j++) {
            uint8_t binaryID = frame[3 + j*3];
            uint8_t high = frame[4 + j*3];
            uint8_t low = frame[5 + j*3];
            uint16_t angle = (high << 7) | low;
            
            ServoInfo* servo = findServoByBinaryID(binaryID);
            if (servo != NULL) {
                servo->icsPort->setSpd(servo->servoID, speed);
                delay(1);
                servo->icsPort->setPos(servo->servoID, angle);
            }
        }
        delay(300);
    }

    Serial1.println(F("✅ [ICS] 完成"));
}

// ===== 單腳測試函數 =====
void testSingleLegs() {
  Serial1.println(F("\n🦿 測試單腳動作 (速度63)"));
  Serial1.println(F("步驟1: 測試右腳..."));
  
  uint8_t testSpeed = 63;
  
  moveAllServosToHome();
  delay(1000);
  
  Serial1.println(F("  右腳向前伸..."));
  icsHV.setSpd(7, testSpeed); icsHV.setSpd(9, testSpeed); icsHV.setSpd(11, testSpeed);
  delay(2);
  icsHV.setPos(7, 6500);
  icsHV.setPos(9, 6500);
  icsHV.setPos(11, 8300);
  delay(1000);
  
  Serial1.println(F("  右腳提高..."));
  icsHV.setPos(9, 6000);
  icsHV.setPos(11, 8500);
  delay(1000);
  
  Serial1.println(F("  右腳放低..."));
  icsHV.setPos(9, 7500);
  icsHV.setPos(11, 7500);
  delay(1000);
  
  Serial1.println(F("  右腳回中..."));
  icsHV.setPos(7, 7500);
  delay(1000);
  
  Serial1.println(F("✅ 右腳測試完成"));
  Serial1.println(F("\n步驟2: 測試左腳..."));
  delay(1000);
  
  moveAllServosToHome();
  delay(1000);
  
  Serial1.println(F("  左腳向前伸..."));
  icsHV.setSpd(8, testSpeed); icsHV.setSpd(10, testSpeed); icsHV.setSpd(12, testSpeed);
  delay(2);
  icsHV.setPos(8, 8500);
  icsHV.setPos(10, 8500);
  icsHV.setPos(12, 6800);
  delay(1000);
  
  Serial1.println(F("  左腳提高..."));
  icsHV.setPos(10, 9000);
  icsHV.setPos(12, 6500);
  delay(1000);
  
  Serial1.println(F("  左腳放低..."));
  icsHV.setPos(10, 7500);
  icsHV.setPos(12, 7550);
  delay(1000);
  
  Serial1.println(F("  左腳回中..."));
  icsHV.setPos(8, 7500);
  delay(1000);
  
  Serial1.println(F("✅ 左腳測試完成"));
  Serial1.println(F("\n🦿 所有單腳測試完成"));
  
  moveAllServosToHome();
}

// ===== setup() =====
void setup() {
  initLED();
  setLEDRed();
  
  for (int i = 0; i < 2; i++) {
    breathLED(LED_RED_PIN, BREATH_SPEED);
  }
  setLEDRed();
  
  Serial1.begin(115200);
  delay(100);
  
  Serial1.println(F("\n========================================"));
  Serial1.println(F("プリメイドAI - 最終修正版 + IK 步行控制 + Hotkey + 安全STOP + GYRO平衡 (可開關) + 單腳測試"));
  Serial1.println(F("========================================"));
  Serial1.println(F("支援:"));
  Serial1.println(F("  - ASCII 指令 (S MV, S HV, S MULTI, FREE, ?)"));
  Serial1.println(F("  - MPU6050 陀螺儀 (G 指令)"));
  Serial1.println(F("  - Batch Mode"));
  Serial1.println(F("  - 0x18 二進制 (有 XOR)"));
  Serial1.println(F("  - 三個 SHAKE 版本"));
  Serial1.println(F("  - IK 步行控制 (WALK 指令 + Hotkey)"));
  Serial1.println(F("  - 更安全版 STOP (先落腳再返 home)"));
  Serial1.println(F("  - GYRO 平衡檢測 (可開關，預設關閉)"));
  Serial1.println(F("  - 單腳測試功能 (TEST_LEGS)"));
  Serial1.println(F("\n🔢 binaryID: HV=1-14, MV=21-31"));
  Serial1.println(F("📝 ASCII 指令都用同一套 ID"));
  Serial1.println(F("📝 已交換 MV1 同 MV3 嘅伺服 ID (硬件交換)"));
  Serial1.println(F("📝 使用者修改：BREATH_SPEED=4, BATCH_BUFFER_SIZE=2048, BATCH_TIMEOUT=500"));
  Serial1.println(F("📝 SHAKE_ICS 改為直接解析 frame"));
  
  Serial1.print(F("\n初始化伺服..."));
  initServos();
  Serial1.println(F("完成"));
  
  Serial1.println(F("\n🏠 回家..."));
  moveAllServosToHome();
  
  Serial1.println(F("\n初始化 MPU6050..."));
  initMPU6050();
  calibrateGyro();
  
  for (int i = 0; i < 2; i++) {
    breathLED(LED_GREEN_PIN, BREATH_SPEED);
  }
  setLEDGreen();
  
  for (int i = 0; i < 3; i++) {
    breathLED(LED_BLUE_PIN, BREATH_SPEED);
  }
  setLEDBlue();
  
  Serial1.println(F("\n✅ 系統就緒"));
  Serial1.println(F("========================================\n"));
  
  Wire.begin();
}

// ===== loop() =====
void loop() {
  // 不斷更新步行（如果正在行路）
  walkGen.updateWalk();
  
  process0x18();
  
  while (Serial1.available()) {
    char c = Serial1.read();
    lastCharTime = millis();
    
    if (!batchMode) {
      batchMode = true;
      batchIndex = 0;
      if (batchIndex < BATCH_BUFFER_SIZE - 1) {
        batchBuffer[batchIndex++] = c;
      }
    } else {
      if (batchIndex < BATCH_BUFFER_SIZE - 1) {
        batchBuffer[batchIndex++] = c;
      }
    }
    
    inputBuffer += c;
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    }
  }
  
  if (batchMode && (millis() - lastCharTime > BATCH_TIMEOUT)) {
    executeBatchCommands();
    batchMode = false;
  }
  
  static unsigned long lastGyroCheck = 0;
  if (gyroProtectionEnabled && (millis() - lastGyroCheck > 100)) {
    if (readMPU6050()) {
      if (abs(mpuData.gx) > 8 || abs(mpuData.gy) > 8) {
        Serial1.println(F("⚠️ 傾斜過大，安全停止"));
        walkGen.safeStop();
      }
    }
    lastGyroCheck = millis();
  }
  
  static unsigned long lastBreath = 0;
  if (millis() - lastBreath > 10000) {
    breathLED(LED_BLUE_PIN, BREATH_SPEED);
    setLEDBlue();
    lastBreath = millis();
  }
  
  delay(1);
}

// ===== 命令處理 =====
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "H" || cmd == "HELP" || cmd == "?") {
    showHelp();
  }
  else if (cmd == "G") {
    if (mpuData.calibrated) {
      if (readMPU6050()) {
        Serial1.print(F("加速度: X=")); Serial1.print(mpuData.ax);
        Serial1.print(F(" Y=")); Serial1.print(mpuData.ay);
        Serial1.print(F(" Z=")); Serial1.println(mpuData.az);
        
        Serial1.print(F("陀螺儀: X=")); Serial1.print(mpuData.gx);
        Serial1.print(F(" Y=")); Serial1.print(mpuData.gy);
        Serial1.print(F(" Z=")); Serial1.println(mpuData.gz);
        
        Serial1.print(F("溫度: ")); Serial1.println(mpuData.temperature);
      } else {
        Serial1.println(F("讀取失敗"));
      }
    } else {
      Serial1.println(F("請先校準陀螺儀 (CALIBRATE)"));
    }
  }
  else if (cmd == "CALIBRATE") {
    calibrateGyro();
  }
  else if (cmd == "HOME") {
    moveAllServosToHome();
  }
  else if (cmd == "FREE ALL") {
    processFreeCommand("FREE ALL");
  }
  else if (cmd == "SHAKE_ASCII") {
    actionShakeBox_ASCII();
  }
  else if (cmd == "SHAKE_CPP") {
    actionShakeBox_CPP();
  }
  else if (cmd == "SHAKE_ICS") {
    actionShakeBox_ICS();
  }
  else if (cmd == "GYRO_ON") {
    gyroProtectionEnabled = true;
    Serial1.println(F("🟢 Gyro 保護已開啟"));
  }
  else if (cmd == "GYRO_OFF") {
    gyroProtectionEnabled = false;
    Serial1.println(F("🔴 Gyro 保護已關閉"));
  }
  else if (cmd == "TEST_LEGS") {
    testSingleLegs();
  }
  else if (cmd == "W" || cmd == "WALK_F") {
    walkGen.setWalkParams(20, 10, 2.0);
    walkGen.setSpeed(40);
    walkGen.setVelocity(20, 0, 0);
    walkGen.walkSteps(5);
    Serial1.println(F("🚶 向前"));
  }
  else if (cmd == "X" || cmd == "WALK_B") {
    walkGen.setWalkParams(20, 10, 2.0);
    walkGen.setSpeed(40);
    walkGen.setVelocity(-20, 0, 0);
    walkGen.walkSteps(5);
    Serial1.println(F("🚶 向後"));
  }
  else if (cmd == "A" || cmd == "WALK_L") {
    walkGen.setWalkParams(20, 10, 2.0);
    walkGen.setSpeed(40);
    walkGen.setVelocity(0, 20, 0);
    walkGen.walkSteps(5);
    Serial1.println(F("🚶 向左"));
  }
  else if (cmd == "D" || cmd == "WALK_R") {
    walkGen.setWalkParams(20, 10, 2.0);
    walkGen.setSpeed(40);
    walkGen.setVelocity(0, -20, 0);
    walkGen.walkSteps(5);
    Serial1.println(F("🚶 向右"));
  }
  else if (cmd == "Q" || cmd == "TURN_L") {
    walkGen.setWalkParams(20, 10, 2.0);
    walkGen.setSpeed(40);
    walkGen.setVelocity(0, 0, radians(10));
    walkGen.walkSteps(5);
    Serial1.println(F("🚶 轉左"));
  }
  else if (cmd == "E" || cmd == "TURN_R") {
    walkGen.setWalkParams(20, 10, 2.0);
    walkGen.setSpeed(40);
    walkGen.setVelocity(0, 0, radians(-10));
    walkGen.walkSteps(5);
    Serial1.println(F("🚶 轉右"));
  }
  else if (cmd == "S" || cmd == "STOP") {
    walkGen.safeStop();
  }
  else if (cmd.startsWith("WALK ")) {
    String params = cmd.substring(5);
    params.trim();
    
    char dir = params.charAt(0);
    int steps = params.substring(2).toInt();
    
    if (steps < 1) steps = 1;
    if (steps > 100) steps = 100;
    
    walkGen.setWalkParams(20, 10, 2.0);
    walkGen.setSpeed(40);
    
    if (dir == 'F') {
      walkGen.setVelocity(20, 0, 0);
      Serial1.print(F("向前行 "));
    } else if (dir == 'B') {
      walkGen.setVelocity(-20, 0, 0);
      Serial1.print(F("向後行 "));
    } else if (dir == 'L') {
      walkGen.setVelocity(0, 20, 0);
      Serial1.print(F("向左行 "));
    } else if (dir == 'R') {
      walkGen.setVelocity(0, -20, 0);
      Serial1.print(F("向右行 "));
    } else {
      Serial1.println(F("❌ 方向錯誤 (F/B/L/R)"));
      return;
    }
    
    Serial1.print(steps);
    Serial1.println(F(" 步"));
    
    walkGen.walkSteps(steps);
    
    Serial1.println(F("✅ 步行指令已接收"));
  }
  else {
    Serial1.println(F("未知命令，輸入 H 查看說明"));
  }
}

void showHelp() {
  Serial1.println(F("\n=== 命令列表 ==="));
  Serial1.println(F("H, HELP, ? : 顯示說明"));
  Serial1.println(F("G          : 顯示陀螺儀數據"));
  Serial1.println(F("CALIBRATE  : 校準陀螺儀"));
  Serial1.println(F("HOME       : 全部回家"));
  Serial1.println(F("FREE ALL   : 全部脫力"));
  Serial1.println(F("\n=== Gyro 保護 ==="));
  Serial1.println(F("GYRO_ON    : 開啟傾斜保護"));
  Serial1.println(F("GYRO_OFF   : 關閉傾斜保護 (預設)"));
  Serial1.println(F("\n=== 單腳測試 ==="));
  Serial1.println(F("TEST_LEGS  : 測試右腳 → 左腳 (速度63)"));
  Serial1.println(F("\n=== 即時控制 (Hotkey) ==="));
  Serial1.println(F("W          : 向前"));
  Serial1.println(F("X          : 向後"));
  Serial1.println(F("A          : 向左"));
  Serial1.println(F("D          : 向右"));
  Serial1.println(F("Q          : 轉左"));
  Serial1.println(F("E          : 轉右"));
  Serial1.println(F("S          : 安全停止"));
  Serial1.println(F("\n=== 步行指令 (行幾步) ==="));
  Serial1.println(F("WALK F 10  : 向前行10步"));
  Serial1.println(F("WALK B 5   : 向後行5步"));
  Serial1.println(F("WALK L 3   : 向左行3步"));
  Serial1.println(F("WALK R 3   : 向右行3步"));
  Serial1.println(F("\n=== 三個 SHAKE 版本 ==="));
  Serial1.println(F("SHAKE_ASCII - ASCII 版"));
  Serial1.println(F("SHAKE_CPP   - C++ 版"));
  Serial1.println(F("SHAKE_ICS   - ICS 二進制版 (直接解析)"));
  Serial1.println(F("\n=== 伺服控制 ==="));
  Serial1.println(F("S MV 21 7500 64  (MV1 - 頭萌)"));
  Serial1.println(F("S MV 22 7500 64  (MV2 - 頭ヨー)"));
  Serial1.println(F("S MV 23 7500 64  (MV3 - 頭ピッチ)"));
  Serial1.println(F("S HV 1 10200 64  (HV1)"));
  Serial1.println(F("S MULTI 30 2 HV1 10200 HV2 4700"));
  Serial1.println(F("FREE MV 21"));
  Serial1.println(F("? HV 1"));
  Serial1.println(F("==================="));
}

