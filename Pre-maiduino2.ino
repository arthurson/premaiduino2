/*
 * Pre-maiduino2_FINAL_IK.ino
 * 最終修正版 + IK 步行控制 + 安全STOP + 電壓檢查
 * 
 * 功能:
 * - 所有 ASCII 指令 (S MV, S HV, S MULTI, FREE, ?)
 * - MPU6050 陀螺儀 (G 指令)
 * - Batch Mode
 * - 呼吸燈效果
 * - 0x18 多軸同步二進制指令 (有 XOR)
 * - 三個 SHAKE 版本
 * - IK 步行控制 (WALK 指令)
 * - 更安全版 STOP (先落腳再返 home)
 * - 電壓檢查
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

// ===== 電壓檢查相關定義 =====
#define VOLTAGE_PIN          PA0  // ADC 腳位
#define VOLTAGE_WARNING      9.0  // 低電壓警告閾值 (9V)
#define VOLTAGE_CHECK_INTERVAL 5000 // 檢查間隔 (5秒)
#define VOLTAGE_SAMPLES      10   // 取樣次數
#define VOLTAGE_DIVIDER_RATIO 22.9 // 分壓電阻比例（校準後：12.03V實際顯示1.05V -> 12.03/1.05×2.0）

// ===== 建立兩個通訊物件 =====
IcsHardSerialClass icsHV(&Serial2, EN_HV_PIN, 1250000, 50);
IcsHardSerialClass icsMV(&Serial3, EN_MV_PIN, 1250000, 50);

// ===== 速度定義 =====
#define MIN_SPEED 1
#define MAX_SPEED 127
#define DEFAULT_SPEED 63

// ===== LED 相關定義 =====
#define LED_RED_PIN   PA7
#define LED_GREEN_PIN PB0
#define LED_BLUE_PIN  PB1

// ===== 呼吸燈速度定義 =====
#define BREATH_SPEED 4

// ===== Batch Mode 設定 =====
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

// ===== 電壓數據結構 =====
struct VoltageData {
  float currentVoltage;
  float minVoltage;
  float maxVoltage;
  unsigned long lastCheckTime;
  bool warningActive;
  bool shutdownInitiated;
} voltageData;

// ===== 25軸伺服資訊 (使用 binaryID 系統, HV:1-14, MV:21-31) =====
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
  {21, 1,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭萌",         7200,  8400, false},
  {22, 2,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭ヨー",       5000, 10000, false},
  {23, 3,  7500, 7500, DEFAULT_SPEED, &icsMV, "頭ピッチ",     6900,  8100, false}, 
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

// ============================================================
// 從原 IK Solver 複製過來的 Home 位置常數
// ============================================================
// Home 位置 (對應 0 度)
#define HOME_HV3   7780  // 右髖yaw
#define HOME_HV4   7500  // 左髖yaw
#define HOME_HV5   7400  // 右髖roll
#define HOME_HV6   7600  // 左髖roll
#define HOME_HV7   7500  // 右髖pitch
#define HOME_HV8   7500  // 左髖pitch
#define HOME_HV9   7500  // 右膝
#define HOME_HV10  7500  // 左膝
#define HOME_HV11  7500  // 右踝pitch
#define HOME_HV12  7550  // 左踝pitch
#define HOME_HV13  7825  // 右踝roll
#define HOME_HV14  7450  // 左踝roll

// ============================================================
// 全新 IK Solver - 2D 簡化版
// ============================================================

// 腿的幾何尺寸
#define THIGH_LENGTH    65.0  // 大腿長度 (mm)
#define SHIN_LENGTH     65.0  // 小腿長度 (mm)
#define ANKLE_HEIGHT    60.0  // 腳踝到腳底高度 (mm)
#define LEG_LENGTH      (THIGH_LENGTH + SHIN_LENGTH + ANKLE_HEIGHT)  // 190mm

// 伺服脈衝轉換
#define SERVO_MIN       3500
#define SERVO_MAX       11500
#define SERVO_RANGE     (SERVO_MAX - SERVO_MIN)  // 8000
#define ANGLE_RANGE     270.0                     // 對應角度
#define PULSE_PER_DEG   (SERVO_RANGE / ANGLE_RANGE)  // 29.63

// 角度限制 (度)
#define HIP_PITCH_MAX   85
#define HIP_PITCH_MIN   -85
#define KNEE_MAX        115
#define KNEE_MIN        0

// 步行參數
#define STEP_LENGTH     30.0  // 步長 (mm)
#define STEP_HEIGHT     25.0  // 抬腳高度 (mm)
#define CYCLE_TIME      3.0   // 步態週期 (秒)

// ===== 2D IK 解算器 =====
class IKSolver2D {
private:
  float L1, L2;  // 大腿和小腿長度
  
public:
  IKSolver2D(float thigh, float shin) {
    L1 = thigh;
    L2 = shin;
  }
  
  // 輸入：目標位置 (x, z)，輸出：髖和膝角度 (度)
  bool solve(float x, float z, float &hipAngle, float &kneeAngle) {
    // 計算目標距離
    float distance = sqrt(x*x + z*z);
    
// 檢查是否可到達（加1mm容差）
    if (distance > L1 + L2 + 1.0 || distance < fabs(L1 - L2)) {
      return false;
    }
    
    // 用餘弦定律計算膝蓋角度
// 用餘弦定律計算膝蓋角度
    float cosKnee = (L1*L1 + L2*L2 - distance*distance) / (2 * L1 * L2);
    // 限制在有效範圍內（避免浮點誤差）
    cosKnee = constrain(cosKnee, -1.0, 1.0);
    kneeAngle = acos(cosKnee);
    
    // 計算髖部角度
    float alpha = atan2(x, -z);  // z是負值（向下）
    float beta = acos((L1*L1 + distance*distance - L2*L2) / (2 * L1 * distance));
    hipAngle = alpha - beta;
    // 轉換為度
    hipAngle = hipAngle * 180.0 / PI;
    kneeAngle = kneeAngle * 180.0 / PI;
    
    return true;
  }
};

// ===== 步行軌跡生成器 =====
class WalkGenerator {
private:
  IKSolver2D ikSolver;
  
  // 步行參數
  float stepLength;
  float stepHeight;
  float cycleTime;
  
  // 狀態
  float phase;           // 0-2.0 的相位
  bool walking;
  int stepsRemaining;
  unsigned long lastUpdate;
  unsigned long lastServoSend;
  
  // 目標位置
  float rightTargetX, rightTargetZ;
  float leftTargetX, leftTargetZ;
  
  // 計算出的角度
  float rightHipAngle, rightKneeAngle;
  float leftHipAngle, leftKneeAngle;
  
  // 伺服值
  uint16_t rightHipPulse, rightKneePulse, rightAnklePulse;
  uint16_t leftHipPulse, leftKneePulse, leftAnklePulse;
  
public:
  WalkGenerator() : ikSolver(THIGH_LENGTH, SHIN_LENGTH) {
    stepLength = STEP_LENGTH;
    stepHeight = STEP_HEIGHT;
    cycleTime = CYCLE_TIME;
    phase = 0;
    walking = false;
    stepsRemaining = 0;
    lastUpdate = millis();
    lastServoSend = millis();
  }
  
  void setWalkParams(float length, float height, float time) {
    stepLength = length;
    stepHeight = height;
    cycleTime = time;
  }
  
  void stop() {
    walking = false;
    stepsRemaining = 0;
    phase = 0;
  }
  
  void safeStop() {
    Serial1.println(F("🛑 安全停止中..."));
    walking = false;
    stepsRemaining = 0;
    phase = 0;
    moveAllServosToHome();
    Serial1.println(F("✅ 安全停止完成"));
  }
  
  bool isWalking() {
    return walking;
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
  }
  
  void updatePhase() {
    unsigned long now = millis();
    float deltaTime = (now - lastUpdate) / 1000.0;
    lastUpdate = now;
    
    if (!walking) return;
    
    // 更新相位 (0-2.0)
    phase += (2.0 / cycleTime) * deltaTime;
    
    // 处理相位循环
    while (phase >= 2.0) {
      phase -= 2.0;
      if (stepsRemaining > 0) {
        stepsRemaining--;
        if (stepsRemaining == 0) {
          // 完成所有步数，立即停止
          walking = false;
          phase = 0;
          moveAllServosToHome();
          Serial1.println(F("✅ 步行完成"));
          return;
        }
      }
    }
  }
  
  void computeRightTarget() {
    float t = phase;
    
    if (t < 1.0) {
      // 支撐相：腳在地面，身體向前移動，腳相對身體向後
      rightTargetX = stepLength * (0.5 - t);
      float maxZ = sqrt(130.0 * 130.0 - rightTargetX * rightTargetX);
      rightTargetZ = -maxZ - ANKLE_HEIGHT;
    } else {
      // 擺動相：腳抬起，從後向前擺動
      float swingT = t - 1.0;
      float smoothT = sin(swingT * PI / 2.0);
      smoothT = smoothT * smoothT;
      rightTargetX = stepLength * (smoothT - 0.5);
      float lift = stepHeight * sin(swingT * PI) * sin(swingT * PI);
      float maxZ = sqrt(130.0 * 130.0 - rightTargetX * rightTargetX);
      rightTargetZ = -(maxZ + ANKLE_HEIGHT - lift);
    }
  }
void computeLeftTarget() {
    float t = phase + 1.0;
    if (t >= 2.0) t -= 2.0;
    
    if (t < 1.0) {
      leftTargetX = stepLength * (0.5 - t);
      float maxZ = sqrt(130.0 * 130.0 - leftTargetX * leftTargetX);
      leftTargetZ = -maxZ - ANKLE_HEIGHT;
    } else {
      float swingT = t - 1.0;
      float smoothT = sin(swingT * PI / 2.0);
      smoothT = smoothT * smoothT;
      leftTargetX = stepLength * (smoothT - 0.5);
      float lift = stepHeight * sin(swingT * PI) * sin(swingT * PI);
      float maxZ = sqrt(130.0 * 130.0 - leftTargetX * leftTargetX);
      leftTargetZ = -(maxZ + ANKLE_HEIGHT - lift);
    }
  }
  
  bool computeIK() {
    float rightZ = rightTargetZ + ANKLE_HEIGHT;
    float leftZ = leftTargetZ + ANKLE_HEIGHT;
    
    // 除錯輸出 - 目標位置
    Serial1.print(F("DEBUG: 右腿目標 x="));
    Serial1.print(rightTargetX);
    Serial1.print(F(" z="));
    Serial1.print(rightTargetZ);
    Serial1.print(F(" (IK z="));
    Serial1.print(rightZ);
    Serial1.print(F(") 左腿目標 x="));
    Serial1.print(leftTargetX);
    Serial1.print(F(" z="));
    Serial1.print(leftTargetZ);
    Serial1.print(F(" (IK z="));
    Serial1.print(leftZ);
    Serial1.println(F(")"));
    
    // 解算右腿
    bool rightOK = ikSolver.solve(rightTargetX, rightZ, 
                                   rightHipAngle, rightKneeAngle);
    
    // 解算左腿
    bool leftOK = ikSolver.solve(leftTargetX, leftZ,
                                  leftHipAngle, leftKneeAngle);

    if (rightOK && leftOK) {
      // 角度限制
      rightHipAngle = constrain(rightHipAngle, HIP_PITCH_MIN, HIP_PITCH_MAX);
      leftHipAngle = constrain(leftHipAngle, HIP_PITCH_MIN, HIP_PITCH_MAX);
      
      // 膝蓋角度轉換
      float rightServoKnee = 180.0 - rightKneeAngle;
      float leftServoKnee = 180.0 - leftKneeAngle;
      rightServoKnee = constrain(rightServoKnee, KNEE_MIN, KNEE_MAX);
      leftServoKnee = constrain(leftServoKnee, KNEE_MIN, KNEE_MAX);
      
      // 除錯輸出 - 角度
      Serial1.print(F("DEBUG: 右腿角度 hip="));
      Serial1.print(rightHipAngle);
      Serial1.print(F(" knee="));
      Serial1.print(rightServoKnee);
      Serial1.print(F(" 左腿角度 hip="));
      Serial1.print(leftHipAngle);
      Serial1.print(F(" knee="));
      Serial1.println(leftServoKnee);
      
      // 轉換為伺服脈衝
      int16_t rightHipDiff = -round(rightHipAngle * PULSE_PER_DEG);
      rightHipPulse = constrain(HOME_HV7 + rightHipDiff, 4700, 10200);
      
      int16_t rightKneeDiff = round(rightServoKnee * PULSE_PER_DEG);
      rightKneePulse = constrain(HOME_HV9 - rightKneeDiff, 3950, 7600);
      
      int16_t leftHipDiff = round(leftHipAngle * PULSE_PER_DEG);
      leftHipPulse = constrain(HOME_HV8 + leftHipDiff, 4700, 10200);
      
int16_t leftKneeDiff = round(leftServoKnee * PULSE_PER_DEG);
      leftKneePulse = constrain(HOME_HV10 + leftKneeDiff, 7400, 11050);
      
      // 腳掌角度補償
      float rightAnkleAngle = rightServoKnee - rightHipAngle;
      float leftAnkleAngle = leftServoKnee - leftHipAngle;
      
      int16_t rightAnkleDiff = -round(rightAnkleAngle * PULSE_PER_DEG);
      rightAnklePulse = constrain(HOME_HV11 + rightAnkleDiff, 4700, 10200);
      
      int16_t leftAnkleDiff = round(leftAnkleAngle * PULSE_PER_DEG);
      leftAnklePulse = constrain(HOME_HV12 + leftAnkleDiff, 4700, 10200);
      
      // 除錯輸出 - 伺服脈衝
      Serial1.print(F("DEBUG: 右腿伺服 HV7="));
      Serial1.print(rightHipPulse);
      Serial1.print(F(" HV9="));
      Serial1.print(rightKneePulse);
      Serial1.print(F(" 左腿伺服 HV8="));
      Serial1.print(leftHipPulse);
      Serial1.print(F(" HV10="));
      Serial1.println(leftKneePulse);
    }
    
    return rightOK && leftOK;
  }
  
  void sendAngles() {
    // 設定速度（較慢速度讓動作更平滑）
    static bool speedSet = false;
    if (!speedSet) {
      icsHV.setSpd(7, 40);
      icsHV.setSpd(9, 40);
      icsHV.setSpd(8, 40);
      icsHV.setSpd(10, 40);
      icsHV.setSpd(11, 40);
      icsHV.setSpd(12, 40);
      speedSet = true;
      delay(2);
    }
    
    // 發送位置
    icsHV.setPos(7, rightHipPulse);
    icsHV.setPos(9, rightKneePulse);
    icsHV.setPos(11, rightAnklePulse);
    icsHV.setPos(8, leftHipPulse);
    icsHV.setPos(10, leftKneePulse);
    icsHV.setPos(12, leftAnklePulse);
    
    // 除錯輸出 - 狀態
    Serial1.print(F("DEBUG: phase="));
    Serial1.print(phase);
    Serial1.print(F(" steps="));
    Serial1.println(stepsRemaining);
  }
  
  void updateWalk() {
    if (!walking) return;
    updatePhase();
    computeRightTarget();
    computeLeftTarget();
    computeIK();
unsigned long now = millis();
    if (now - lastServoSend >= 30) {
      sendAngles();
      lastServoSend = now;
    }
  }
};

WalkGenerator walkGen;

// ===== 電壓檢查函式 =====
void initVoltageCheck() {
  pinMode(VOLTAGE_PIN, INPUT_ANALOG);
  voltageData.currentVoltage = 0.0;
  voltageData.minVoltage = 99.0;
  voltageData.maxVoltage = 0.0;
  voltageData.lastCheckTime = 0;
  voltageData.warningActive = false;
  voltageData.shutdownInitiated = false;
}

float readBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
    sum += analogRead(VOLTAGE_PIN);
    delay(1);
  }
  float average = (float)sum / VOLTAGE_SAMPLES;
  
  float voltageAtPin = (average / 4095.0) * 3.3;
  float actualVoltage = voltageAtPin * VOLTAGE_DIVIDER_RATIO;
  
  return actualVoltage;
}

void checkVoltage() {
  unsigned long now = millis();
  
  if (now - voltageData.lastCheckTime < VOLTAGE_CHECK_INTERVAL) {
    return;
  }
  
  voltageData.lastCheckTime = now;
  voltageData.currentVoltage = readBatteryVoltage();
  
  if (voltageData.currentVoltage > voltageData.maxVoltage) {
    voltageData.maxVoltage = voltageData.currentVoltage;
  }
  if (voltageData.currentVoltage < voltageData.minVoltage) {
    voltageData.minVoltage = voltageData.currentVoltage;
  }
  
  Serial1.print(F("🔋 當前電壓: "));
  Serial1.print(voltageData.currentVoltage);
  Serial1.println(F("V"));
  
  if (voltageData.currentVoltage < VOLTAGE_WARNING) {
    if (!voltageData.warningActive) {
      Serial1.print(F("\n⚠️ 低電壓警告: "));
      Serial1.print(voltageData.currentVoltage);
      Serial1.println(F("V"));
      voltageData.warningActive = true;
    }
    setLEDRed();
    
    // 檢查是否需要自動關機（低於8.5V）
    if (voltageData.currentVoltage < 8.5 && !voltageData.shutdownInitiated) {
      Serial1.println(F("\n🚨 電壓過低！自動關機..."));
      voltageData.shutdownInitiated = true;
      
      // 停止步行
      walkGen.stop();
      
      // 所有伺服回到HOME位置
      moveAllServosToHome();
      
      // 關閉LED
      setLEDOff();
      
      // 進入無限循環（安全模式）
while(1) {
        delay(1000);
      }
    }
  } else {
    if (voltageData.warningActive) {
      Serial1.println(F("✅ 電壓恢復正常"));
      voltageData.warningActive = false;
    }
    if (!voltageData.warningActive) {
      setLEDBlue();
    }
  }
}

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

// ===== 處理 0x18 多軸同步二進制指令 =====
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

void actionShakeBox_ICS() {
    Serial1.println(F("\n📦 [ICS] Shake a Box (直接解析)"));

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

    uint8_t frame1[] = {
        0x0A, 0x18, 0x2E,
        0x02, 0x2D, 0x58,
        0x01, 0x46, 0x5C,
        0x50
    };

    uint8_t frame2[] = {
        0x0A, 0x18, 0x2E,
        0x02, 0x33, 0x0A,
        0x01, 0x41, 0x2A,
        0x6D
    };

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
  
  initVoltageCheck();
  
  Serial1.println(F("\n========================================"));
  Serial1.println(F("プリメイドAI - 最終修正版 + IK 步行控制 + 安全STOP + 電壓檢查"));
  Serial1.println(F("========================================"));
  Serial1.println(F("支援:"));
  Serial1.println(F("  - ASCII 指令 (S MV, S HV, S MULTI, FREE, ?)"));
  Serial1.println(F("  - MPU6050 陀螺儀 (G 指令)"));
  Serial1.println(F("  - Batch Mode"));
  Serial1.println(F("  - 0x18 二進制 (有 XOR)"));
  Serial1.println(F("  - 三個 SHAKE 版本"));
  Serial1.println(F("  - IK 步行控制 (WALK 指令)"));
  Serial1.println(F("  - 更安全版 STOP (先落腳再返 home)"));
  Serial1.println(F("  - 電壓檢查 (低於9V紅燈)"));
  Serial1.println(F("\n🔢 binaryID: HV=1-14, MV=21-31"));
  
  Serial1.print(F("\n初始化伺服..."));
  initServos();
  Serial1.println(F("完成"));
  
  Serial1.println(F("\n🏠 回家..."));
  moveAllServosToHome();
  
  Serial1.println(F("\n初始化 MPU6050..."));
  initMPU6050();
  calibrateGyro();
  
  voltageData.currentVoltage = readBatteryVoltage();
  Serial1.print(F("\n🔋 當前電壓: "));
  Serial1.print(voltageData.currentVoltage);
  Serial1.println(F("V"));
  
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
  checkVoltage();
  
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
  
  static unsigned long lastBreath = 0;
  if (millis() - lastBreath > 10000) {
    if (!voltageData.warningActive) {
      breathLED(LED_BLUE_PIN, BREATH_SPEED);
      setLEDBlue();
    }
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
  else if (cmd == "VOLTAGE") {
    float voltage = readBatteryVoltage();
    Serial1.print(F("🔋 當前電壓: "));
    Serial1.print(voltage);
    Serial1.println(F("V"));
    Serial1.print(F("📊 記錄範圍: "));
    Serial1.print(voltageData.minVoltage);
    Serial1.print(F("V - "));
    Serial1.print(voltageData.maxVoltage);
    Serial1.println(F("V"));
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
  else if (cmd == "STOP") {
    walkGen.safeStop();
  }
  else if (cmd.startsWith("WALK ")) {
    String params = cmd.substring(5);
    params.trim();
    
    char dir = params.charAt(0);
    int steps = params.substring(2).toInt();
    
    if (steps < 1) steps = 1;
    if (steps > 100) steps = 100;
    
    walkGen.setWalkParams(STEP_LENGTH, STEP_HEIGHT, CYCLE_TIME);
    
    if (dir == 'F') {
      Serial1.print(F("向前行 "));
    } else if (dir == 'B') {
      Serial1.print(F("向後行 "));
    } else if (dir == 'L') {
      Serial1.print(F("向左行 "));
    } else if (dir == 'R') {
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
  Serial1.println(F("VOLTAGE    : 顯示當前電壓"));
  Serial1.println(F("CALIBRATE  : 校準陀螺儀"));
  Serial1.println(F("HOME       : 全部回家"));
  Serial1.println(F("FREE ALL   : 全部脫力"));
  Serial1.println(F("\n=== 步行指令 (行幾步) ==="));
  Serial1.println(F("WALK F 10  : 向前行10步"));
  Serial1.println(F("WALK B 5   : 向後行5步"));
  Serial1.println(F("WALK L 3   : 向左行3步"));
  Serial1.println(F("WALK R 3   : 向右行3步"));
  Serial1.println(F("STOP       : 安全停止"));
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
