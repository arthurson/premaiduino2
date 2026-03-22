/*
 * Pre-maiduino2_FINAL_IK.ino
 * 最終修正版 + IK 步行控制 + 安全STOP + 電壓檢查
 * 
 * 功能:
 * - 所有 ASCII 指令 (S MV, S HV, S MULTI, FREE, ?)
 * - MPU6050 陀螺儀 (G 指令)
 * - Batch Mode
 * - 呼吸燈效果
 * - 官方 ICS Binary 協議支援 (所有功能)
 * - 三個 SHAKE 版本
 * - IK 步行控制 (WALK 指令) - 只控制 HV3-14 (12軸下肢)
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
#define VOLTAGE_DIVIDER_RATIO 22.9 // 分壓電阻比例

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

// ===== 25軸伺服資訊 =====
struct ServoInfo {
  uint8_t binaryID;        // binaryID (HV:1-14, MV:21-31)
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
  {1,  1,  10200, 10200, DEFAULT_SPEED, &icsHV, "右肩前後",  4300, 11500, true},
  {2,  2,  4700, 4700,   DEFAULT_SPEED, &icsHV, "左肩前後",  3500, 10700, true},
  {3,  3,  7780, 7780,   DEFAULT_SPEED, &icsHV, "右髖轉向",  6530,  9030, true},
  {4,  4,  7500, 7500,   DEFAULT_SPEED, &icsHV, "左髖轉向",  6250,  8750, true},
  {5,  5,  7400, 7400,   DEFAULT_SPEED, &icsHV, "右髖側擺",  6700,  8300, true},
  {6,  6,  7600, 7600,   DEFAULT_SPEED, &icsHV, "左髖側擺",  6700,  8300, true},
  {7,  7,  7500, 7500,   DEFAULT_SPEED, &icsHV, "右髖前後",  4700, 10200, true},
  {8,  8,  7500, 7500,   DEFAULT_SPEED, &icsHV, "左髖前後",  4700, 10200, true},
  {9,  9,  7500, 7500,   DEFAULT_SPEED, &icsHV, "右膝屈伸",  3950,  7600, true},
  {10, 10, 7500, 7500,   DEFAULT_SPEED, &icsHV, "左膝屈伸",  7400, 11050, true},
  {11, 11, 7500, 7500,   DEFAULT_SPEED, &icsHV, "右踝前後",  5700,  8300, true},
  {12, 12, 7550, 7550,   DEFAULT_SPEED, &icsHV, "左踝前後",  6750,  9350, true},
  {13, 13, 7825, 7825,   DEFAULT_SPEED, &icsHV, "右踝側擺",  6800,  9150, true},
  {14, 14, 7450, 7450,   DEFAULT_SPEED, &icsHV, "左踝側擺",  6200,  8450, true},

  // ===== MV群 (上半身) - binaryID 21-31 =====
  {21, 1,  7500, 7500,   DEFAULT_SPEED, &icsMV, "頭部前後",  7200,  8400, false},
  {22, 2,  7500, 7500,   DEFAULT_SPEED, &icsMV, "頭部轉向",  5000, 10000, false},
  {23, 3,  7500, 7500,   DEFAULT_SPEED, &icsMV, "頭部側傾",  6900,  8100, false}, 
  {24, 4,  9800, 9800,   DEFAULT_SPEED, &icsMV, "右肩側擺",  7450, 10350, false},
  {25, 5,  5200, 5200,   DEFAULT_SPEED, &icsMV, "左肩側擺",  4550,  7550, false},
  {26, 6,  7500, 7500,   DEFAULT_SPEED, &icsMV, "右臂轉向",  4000, 11000, false},
  {27, 7,  7500, 7500,   DEFAULT_SPEED, &icsMV, "左臂轉向",  4000, 11000, false},
  {28, 8,  7500, 7500,   DEFAULT_SPEED, &icsMV, "右肘屈伸",  7100, 11000, false},
  {29, 9,  7500, 7500,   DEFAULT_SPEED, &icsMV, "左肘屈伸",  4000,  7900, false},
  {30, 10, 5000, 5000,   DEFAULT_SPEED, &icsMV, "右手轉向",  3500, 11500, false},
  {31, 11, 10000, 10000, DEFAULT_SPEED, &icsMV, "左手轉向",  3500, 11500, false}
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
void processOfficialBinary();
ServoInfo* findServoByBinaryID(uint8_t binaryID);
void actionShakeBox_ASCII();
void actionShakeBox_CPP();
void actionShakeBox_ICS();

// ===== IK Solver 常數 =====
#define HOME_HV3   7780
#define HOME_HV4   7500
#define HOME_HV5   7400
#define HOME_HV6   7600
#define HOME_HV7   7500
#define HOME_HV8   7500
#define HOME_HV9   7500
#define HOME_HV10  7500
#define HOME_HV11  7500
#define HOME_HV12  7550
#define HOME_HV13  7825
#define HOME_HV14  7450

#define THIGH_LENGTH    65.0
#define SHIN_LENGTH     65.0
#define ANKLE_HEIGHT    60.0
#define HIP_WIDTH       40.0
#define LEG_LENGTH      (THIGH_LENGTH + SHIN_LENGTH + ANKLE_HEIGHT)

#define SERVO_MIN       3500
#define SERVO_MAX       11500
#define SERVO_RANGE     (SERVO_MAX - SERVO_MIN)
#define ANGLE_RANGE     270.0
#define PULSE_PER_DEG   (SERVO_RANGE / ANGLE_RANGE)

#define HIP_PITCH_MAX   85
#define HIP_PITCH_MIN   -85
#define KNEE_MAX        115
#define KNEE_MIN        0
#define HIP_YAW_MAX     45
#define HIP_YAW_MIN     -45
#define HIP_ROLL_MAX    30
#define HIP_ROLL_MIN    -30
#define ANKLE_ROLL_MAX  25
#define ANKLE_ROLL_MIN  -25
#define ANKLE_PITCH_MAX 45
#define ANKLE_PITCH_MIN -45

#define STEP_LENGTH     30.0
#define STEP_WIDTH      20.0
#define STEP_HEIGHT     45.0
#define CYCLE_TIME      4.5

class IKSolver3D {
private:
  float L1, L2;
  float hipWidth;
  
public:
  IKSolver3D(float thigh, float shin, float width) {
    L1 = thigh;
    L2 = shin;
    hipWidth = width;
  }
  
  bool solve(float x, float y, float z, 
             float &hipYaw, float &hipRoll, float &hipPitch, 
             float &kneePitch, float &anklePitch, float &ankleRoll,
             bool isRightLeg) {
    
    if (abs(z) > 0.1) {
      hipYaw = atan2(x, -z) * 180.0 / PI;
    } else {
      hipYaw = 0;
    }
    hipYaw = constrain(hipYaw, HIP_YAW_MIN, HIP_YAW_MAX);
    
    float lateralFactor = (isRightLeg ? 1.0 : -1.0);
    if (hipWidth > 0.1) {
      hipRoll = atan2(y * lateralFactor, hipWidth) * 180.0 / PI;
    } else {
      hipRoll = 0;
    }
    hipRoll = constrain(hipRoll, HIP_ROLL_MIN, HIP_ROLL_MAX);
    
    float xzDistance = sqrt(x*x + z*z);
    
    float cosKnee = (L1*L1 + L2*L2 - xzDistance*xzDistance) / (2 * L1 * L2);
    cosKnee = constrain(cosKnee, -1.0, 1.0);
    kneePitch = acos(cosKnee) * 180.0 / PI;
    kneePitch = constrain(kneePitch, KNEE_MIN, KNEE_MAX);
    
    float alpha = atan2(x, -z);
    float beta = acos((L1*L1 + xzDistance*xzDistance - L2*L2) / (2 * L1 * xzDistance));
    hipPitch = (alpha + beta) * 180.0 / PI;
    hipPitch = constrain(hipPitch, HIP_PITCH_MIN, HIP_PITCH_MAX);
    
    float targetFootAngle = 0;
    anklePitch = targetFootAngle - (hipPitch + kneePitch);
    anklePitch = constrain(anklePitch, ANKLE_PITCH_MIN, ANKLE_PITCH_MAX);
    
    ankleRoll = -hipRoll;
    ankleRoll = constrain(ankleRoll, ANKLE_ROLL_MAX, ANKLE_ROLL_MIN);
    
    return true;
  }
};

class WalkGenerator {
private:
  IKSolver3D ikSolver;
  
  float stepLength;
  float stepWidth;
  float stepHeight;
  float cycleTime;
  
  bool walkForward;
  bool walkBackward;
  bool walkLeft;
  bool walkRight;
  
  float phase;
  bool walking;
  int stepsRemaining;
  unsigned long lastUpdate;
  unsigned long lastServoSend;
  
  float rightTargetX, rightTargetY, rightTargetZ;
  float leftTargetX, leftTargetY, leftTargetZ;
  
  float rightYaw, rightRoll, rightPitch, rightKnee, rightAnklePitch, rightAnkleRoll;
  float leftYaw, leftRoll, leftPitch, leftKnee, leftAnklePitch, leftAnkleRoll;
  
  uint16_t rightYawPulse;
  uint16_t leftYawPulse;
  uint16_t rightRollPulse;
  uint16_t leftRollPulse;
  uint16_t rightPitchPulse;
  uint16_t leftPitchPulse;
  uint16_t rightKneePulse;
  uint16_t leftKneePulse;
  uint16_t rightAnklePitchPulse;
  uint16_t leftAnklePitchPulse;
  uint16_t rightAnkleRollPulse;
  uint16_t leftAnkleRollPulse;
  
  void resetDirection() {
    walkForward = false;
    walkBackward = false;
    walkLeft = false;
    walkRight = false;
  }
  
  void computeRightTarget() {
    float t = phase;
    float dirX = 0, dirY = 0;
    
    if (walkForward) dirX = 1.0;
    else if (walkBackward) dirX = -1.0;
    
    if (walkLeft) dirY = 1.0;
    else if (walkRight) dirY = -1.0;
    
    if (t < 1.0) {
      float supportFactor = 0.3 + 0.2 * sin(t * PI);
      rightTargetX = dirX * stepLength * supportFactor;
      rightTargetY = dirY * stepWidth * supportFactor;
      rightTargetZ = -LEG_LENGTH + 30 * (1.0 - supportFactor);
    } else {
      float swingT = t - 1.0;
      float smoothT = swingT * swingT * (3 - 2 * swingT);
      rightTargetX = dirX * stepLength * (smoothT - 0.5);
      rightTargetY = dirY * stepWidth * sin(swingT * PI);
      float lift = stepHeight * sin(swingT * PI) * sin(swingT * PI);
      rightTargetZ = -(LEG_LENGTH - lift);
    }
  }
  
  void computeLeftTarget() {
    float t = phase + 1.0;
    if (t >= 2.0) t -= 2.0;
    
    float dirX = 0, dirY = 0;
    
    if (walkForward) dirX = 1.0;
    else if (walkBackward) dirX = -1.0;
    
    if (walkLeft) dirY = 1.0;
    else if (walkRight) dirY = -1.0;
    
    if (t < 1.0) {
      float supportFactor = 0.3 + 0.2 * sin(t * PI);
      leftTargetX = dirX * stepLength * supportFactor;
      leftTargetY = dirY * stepWidth * supportFactor;
      leftTargetZ = -LEG_LENGTH + 30 * (1.0 - supportFactor);
    } else {
      float swingT = t - 1.0;
      float smoothT = swingT * swingT * (3 - 2 * swingT);
      leftTargetX = dirX * stepLength * (smoothT - 0.5);
      leftTargetY = dirY * stepWidth * sin(swingT * PI);
      float lift = stepHeight * sin(swingT * PI) * sin(swingT * PI);
      leftTargetZ = -(LEG_LENGTH - lift);
    }
  }
  
  bool computeIK() {
    bool rightOK = ikSolver.solve(rightTargetX, rightTargetY, rightTargetZ + ANKLE_HEIGHT,
                                   rightYaw, rightRoll, rightPitch, rightKnee, 
                                   rightAnklePitch, rightAnkleRoll, true);
    
    bool leftOK = ikSolver.solve(leftTargetX, leftTargetY, leftTargetZ + ANKLE_HEIGHT,
                                  leftYaw, leftRoll, leftPitch, leftKnee,
                                  leftAnklePitch, leftAnkleRoll, false);
    
    if (rightOK && leftOK) {
      int16_t rightYawDiff = round(rightYaw * PULSE_PER_DEG);
      rightYawPulse = constrain(HOME_HV3 + rightYawDiff, 6530, 9030);
      
      int16_t leftYawDiff = -round(leftYaw * PULSE_PER_DEG);
      leftYawPulse = constrain(HOME_HV4 + leftYawDiff, 6250, 8750);
      
      int16_t rightRollDiff = round(rightRoll * PULSE_PER_DEG);
      rightRollPulse = constrain(HOME_HV5 + rightRollDiff, 6700, 8300);
      
      int16_t leftRollDiff = -round(leftRoll * PULSE_PER_DEG);
      leftRollPulse = constrain(HOME_HV6 + leftRollDiff, 6700, 8300);
      
      int16_t rightPitchDiff = -round(rightPitch * PULSE_PER_DEG);
      rightPitchPulse = constrain(HOME_HV7 + rightPitchDiff, 4700, 10200);
      
      int16_t leftPitchDiff = round(leftPitch * PULSE_PER_DEG);
      leftPitchPulse = constrain(HOME_HV8 + leftPitchDiff, 4700, 10200);
      
      int16_t rightKneeDiff = -round(rightKnee * PULSE_PER_DEG);
      rightKneePulse = constrain(HOME_HV9 + rightKneeDiff, 3950, 7600);
      
      int16_t leftKneeDiff = round(leftKnee * PULSE_PER_DEG);
      leftKneePulse = constrain(HOME_HV10 + leftKneeDiff, 7400, 11050);
      
      int16_t rightAnklePitchDiff = round(rightAnklePitch * PULSE_PER_DEG);
      rightAnklePitchPulse = constrain(HOME_HV11 + rightAnklePitchDiff, 5700, 8300);
      
      int16_t leftAnklePitchDiff = -round(leftAnklePitch * PULSE_PER_DEG);
      leftAnklePitchPulse = constrain(HOME_HV12 + leftAnklePitchDiff, 6750, 9350);
      
      int16_t rightAnkleRollDiff = round(rightAnkleRoll * PULSE_PER_DEG);
      rightAnkleRollPulse = constrain(HOME_HV13 + rightAnkleRollDiff, 6800, 9150);
      
      int16_t leftAnkleRollDiff = -round(leftAnkleRoll * PULSE_PER_DEG);
      leftAnkleRollPulse = constrain(HOME_HV14 + leftAnkleRollDiff, 6200, 8450);
    }
    
    return rightOK && leftOK;
  }
  
  void sendAngles() {
    static bool speedSet = false;
    if (!speedSet) {
      int legServos[] = {3,4,5,6,7,8,9,10,11,12,13,14};
      for (int i = 0; i < 12; i++) {
        icsHV.setSpd(legServos[i], 20);
      }
      speedSet = true;
      delay(2);
    }
    
    icsHV.setPos(3, rightYawPulse);
    icsHV.setPos(5, rightRollPulse);
    icsHV.setPos(7, rightPitchPulse);
    icsHV.setPos(9, rightKneePulse);
    icsHV.setPos(11, rightAnklePitchPulse);
    icsHV.setPos(13, rightAnkleRollPulse);
    
    icsHV.setPos(4, leftYawPulse);
    icsHV.setPos(6, leftRollPulse);
    icsHV.setPos(8, leftPitchPulse);
    icsHV.setPos(10, leftKneePulse);
    icsHV.setPos(12, leftAnklePitchPulse);
    icsHV.setPos(14, leftAnkleRollPulse);
    
    // DEBUG 輸出
    Serial1.print(F("DEBUG: Phase="));
    Serial1.print(phase);
    Serial1.print(F(" Steps="));
    Serial1.println(stepsRemaining);
    
    Serial1.print(F("  右腿 轉向(HV3)="));
    Serial1.print(rightYawPulse);
    Serial1.print(F(" 側擺(HV5)="));
    Serial1.print(rightRollPulse);
    Serial1.print(F(" 前後(HV7)="));
    Serial1.print(rightPitchPulse);
    Serial1.print(F(" 屈伸(HV9)="));
    Serial1.print(rightKneePulse);
    Serial1.print(F(" 踝前後(HV11)="));
    Serial1.print(rightAnklePitchPulse);
    Serial1.print(F(" 踝側擺(HV13)="));
    Serial1.println(rightAnkleRollPulse);
    
    Serial1.print(F("  左腿 轉向(HV4)="));
    Serial1.print(leftYawPulse);
    Serial1.print(F(" 側擺(HV6)="));
    Serial1.print(leftRollPulse);
    Serial1.print(F(" 前後(HV8)="));
    Serial1.print(leftPitchPulse);
    Serial1.print(F(" 屈伸(HV10)="));
    Serial1.print(leftKneePulse);
    Serial1.print(F(" 踝前後(HV12)="));
    Serial1.print(leftAnklePitchPulse);
    Serial1.print(F(" 踝側擺(HV14)="));
    Serial1.println(leftAnkleRollPulse);
    
    Serial1.print(F("   ankle calc: R_hip="));
    Serial1.print(rightPitch);
    Serial1.print(F(" R_knee="));
    Serial1.print(rightKnee);
    Serial1.print(F(" R_ankle="));
    Serial1.print(rightAnklePitch);
    Serial1.print(F(" pulse="));
    Serial1.println(rightAnklePitchPulse);
    
    Serial1.print(F("   ankle calc: L_hip="));
    Serial1.print(leftPitch);
    Serial1.print(F(" L_knee="));
    Serial1.print(leftKnee);
    Serial1.print(F(" L_ankle="));
    Serial1.print(leftAnklePitch);
    Serial1.print(F(" pulse="));
    Serial1.println(leftAnklePitchPulse);
  }
  
public:
  WalkGenerator() : ikSolver(THIGH_LENGTH, SHIN_LENGTH, HIP_WIDTH) {
    stepLength = STEP_LENGTH;
    stepWidth = STEP_WIDTH;
    stepHeight = STEP_HEIGHT;
    cycleTime = CYCLE_TIME;
    phase = 0;
    walking = false;
    stepsRemaining = 0;
    lastUpdate = millis();
    lastServoSend = millis();
    resetDirection();
  }
  
  void setDirection(char dir) {
    resetDirection();
    switch(dir) {
      case 'F': walkForward = true; break;
      case 'B': walkBackward = true; break;
      case 'L': walkLeft = true; break;
      case 'R': walkRight = true; break;
    }
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
    
    phase += (2.0 / cycleTime) * deltaTime;
    
    while (phase >= 2.0) {
      phase -= 2.0;
      if (stepsRemaining > 0) {
        stepsRemaining--;
        if (stepsRemaining == 0) {
          walking = false;
          phase = 0;
          moveAllServosToHome();
          Serial1.println(F("✅ 步行完成"));
          return;
        }
      }
    }
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
    
    if (voltageData.currentVoltage < 8.5 && !voltageData.shutdownInitiated) {
      Serial1.println(F("\n🚨 電壓過低！自動關機..."));
      voltageData.shutdownInitiated = true;
      walkGen.stop();
      moveAllServosToHome();
      setLEDOff();
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

// ===== 處理官方 ICS Binary 協議 =====
void processOfficialBinary() {
  // 檢查是否有足夠數據
  while (Serial1.available() >= 3) {
    uint8_t firstByte = Serial1.peek();
    
    // 有效 ICS 指令的判斷條件：
    // 1. 位置指令: 0x80-0x9F (高2位為 10)
    // 2. 參數寫入: 0xC0-0xDF (高2位為 11)
    // 3. 參數讀取: 0xA0-0xBF (高2位為 10? 實際上是 101)
    bool isValidICS = false;
    uint8_t cmdType = (firstByte >> 6) & 0x03;
    
    // 檢查是否為有效的 ICS 指令
    if (cmdType == 0b10 || cmdType == 0b11) {
      // 位置指令 (0b10) 或 參數寫入 (0b11)
      isValidICS = true;
    } else if ((firstByte & 0xE0) == 0xA0) {
      // 參數讀取指令 (0b101)
      isValidICS = true;
    } else if (firstByte == 0xFF) {
      // ID 讀取指令
      isValidICS = true;
    }
    
    // 如果不是有效的 ICS 指令，退出循環讓 ASCII 處理
    if (!isValidICS) {
      break;
    }
    
    // 讀取 3 個字節
    uint8_t cmd = Serial1.read();
    if (Serial1.available() < 2) break;
    uint8_t byte2 = Serial1.read();
    uint8_t byte3 = Serial1.read();
    
    cmdType = (cmd >> 6) & 0x03;
    uint8_t id = cmd & 0x1F;
    
    ServoInfo* servo = findServoByBinaryID(id);
    if (!servo) continue;
    
    switch(cmdType) {
      case 0b10:  // 位置指令 (0b10XXXXXX)
        {
          uint16_t pos = (byte2 << 7) | byte3;
          if (pos >= servo->minAngle && pos <= servo->maxAngle) {
            servo->icsPort->setPos(servo->servoID, pos);
            servo->currentTunePos = pos;
          }
        }
        break;
        
      case 0b11:  // 參數寫入 (0b11XXXXXX)
        {
          uint8_t paramType = byte2;
          uint8_t value = byte3;
          
          switch(paramType) {
            case 0x01:  // 扭力
              if (value >= 1 && value <= 127) {
                servo->icsPort->setStrc(servo->servoID, value);
              }
              break;
            case 0x02:  // 速度
              if (value >= 1 && value <= 127) {
                servo->icsPort->setSpd(servo->servoID, value);
                servo->currentSpeed = value;
              }
              break;
            case 0x03:  // 電流限制
              if (value >= 1 && value <= 63) {
                servo->icsPort->setCur(servo->servoID, value);
              }
              break;
            case 0x04:  // 溫度限制
              if (value >= 1 && value <= 127) {
                servo->icsPort->setTmp(servo->servoID, value);
              }
              break;
            default:
              break;
          }
        }
        break;
        
      case 0b01:  // 讀取指令 (0b101XXXXX)
        {
          uint8_t readType = byte2;
          int result = ICS_FALSE;
          
          switch(readType) {
            case 0x01:  // 讀取扭力
              result = servo->icsPort->getStrc(servo->servoID);
              break;
            case 0x02:  // 讀取速度
              result = servo->icsPort->getSpd(servo->servoID);
              break;
            case 0x03:  // 讀取電流
              result = servo->icsPort->getCur(servo->servoID);
              break;
            case 0x04:  // 讀取溫度
              result = servo->icsPort->getTmp(servo->servoID);
              break;
            case 0x05:  // 讀取位置 (ICS3.6+)
              result = servo->icsPort->getPos(servo->servoID);
              break;
          }
          
          if (result != ICS_FALSE) {
            // 回傳數據 (格式: [CMD] [高7位] [低7位])
            uint8_t reply[3];
            reply[0] = 0x80 | id;
            reply[1] = (result >> 7) & 0x7F;
            reply[2] = result & 0x7F;
            Serial1.write(reply, 3);
          }
        }
        break;
        
      default:
        break;
    }
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
  Serial1.println(F("\n📦 [ICS] Shake a Box (官方 Binary 協議)"));
  
  // 使用 C++ 函數實現（正確做法）
  // 設定速度
  icsHV.setSpd(2, 37);   // HV2 速度37
  icsMV.setSpd(5, 37);   // MV5 速度37
  icsMV.setSpd(9, 37);   // MV9 速度37
  icsHV.setSpd(1, 37);   // HV1 速度37
  icsMV.setSpd(4, 37);   // MV4 速度37
  icsMV.setSpd(8, 37);   // MV8 速度37
  delay(5);
  
  // 設定位置
  icsHV.setPos(2, 6538);  // HV2
  icsMV.setPos(5, 5247);  // MV5
  icsMV.setPos(9, 6763);  // MV9
  icsHV.setPos(1, 8362);  // HV1
  icsMV.setPos(4, 9753);  // MV4
  icsMV.setPos(8, 8237);  // MV8
  
  delay(700);
  
  for (int i = 0; i < 8; i++) {
    if (i % 2 == 0) {
      // 搖向左
      icsHV.setSpd(2, 46);
      icsHV.setSpd(1, 46);
      delay(2);
      icsHV.setPos(2, 5848);
      icsHV.setPos(1, 9052);
    } else {
      // 搖向右
      icsHV.setSpd(2, 46);
      icsHV.setSpd(1, 46);
      delay(2);
      icsHV.setPos(2, 6538);
      icsHV.setPos(1, 8362);
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
  Serial1.println(F("  - 官方 ICS Binary 協議 (所有功能)"));
  Serial1.println(F("  - MPU6050 陀螺儀 (G 指令)"));
  Serial1.println(F("  - Batch Mode"));
  Serial1.println(F("  - 三個 SHAKE 版本"));
  Serial1.println(F("  - IK 步行控制 (WALK 指令) - 只控制 HV3-14 (12軸下肢)"));
  Serial1.println(F("  - 更安全版 STOP (先落腳再返 home)"));
  Serial1.println(F("  - 電壓檢查 (低於9V紅燈)"));
  
  Serial1.println(F("\n🔢 binaryID: HV=1-14, MV=21-31"));
  Serial1.println(F("  注意: HV1-2 係肩 (步行時唔會郁)"));
  
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
  
  // 處理官方 ICS Binary 協議
  processOfficialBinary();
  
  // 處理 ASCII 指令
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
    
    walkGen.setDirection(dir);
    
    switch(dir) {
      case 'F': Serial1.print(F("向前行 ")); break;
      case 'B': Serial1.print(F("向後行 ")); break;
      case 'L': Serial1.print(F("向左行 ")); break;
      case 'R': Serial1.print(F("向右行 ")); break;
      default: 
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
  Serial1.println(F("WALK L 3   : 向左行3步 (側移)"));
  Serial1.println(F("WALK R 3   : 向右行3步 (側移)"));
  Serial1.println(F("STOP       : 安全停止"));
  Serial1.println(F("\n=== 三個 SHAKE 版本 ==="));
  Serial1.println(F("SHAKE_ASCII - ASCII 版"));
  Serial1.println(F("SHAKE_CPP   - C++ 版"));
  Serial1.println(F("SHAKE_ICS   - ICS 二進制版 (官方協議)"));
  Serial1.println(F("\n=== 伺服控制 ==="));
  Serial1.println(F("S MV 21 7500 64  (MV1 - 頭部前後)"));
  Serial1.println(F("S MV 22 7500 64  (MV2 - 頭部轉向)"));
  Serial1.println(F("S MV 23 7500 64  (MV3 - 頭部側傾)"));
  Serial1.println(F("S HV 1 10200 64  (HV1 - 右肩前後)"));
  Serial1.println(F("S HV 2 4700 64   (HV2 - 左肩前後)"));
  Serial1.println(F("S MULTI 30 2 HV1 10200 HV2 4700"));
  Serial1.println(F("FREE MV 21"));
  Serial1.println(F("? HV 1"));
  Serial1.println(F("\n=== 官方 Binary 協議 ==="));
  Serial1.println(F("直接發送 3-byte 指令即可"));
  Serial1.println(F("範例: 0x83 0x3A 0x4C = HV3 到 7500"));
  Serial1.println(F("==================="));
}
