/*
 * Pre-maiduino2_REWRITTEN_IK_FIXED.ino
 * 修正了：腳踝反向問題、行路唔夠力問題、以及「一格格」窒步問題
 * 新增：即時 IK 調試功能 + 改良版 IK Solver + 進階測試功能
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
#define VOLTAGE_PIN          PA0
#define VOLTAGE_WARNING      9.0
#define VOLTAGE_CHECK_INTERVAL 5000
#define VOLTAGE_SAMPLES      10
#define VOLTAGE_DIVIDER_RATIO 22.9

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
  uint8_t binaryID;
  uint8_t servoID;
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
#define THIGH_LENGTH    65.0
#define SHIN_LENGTH     65.0
#define ANKLE_HEIGHT    60.0
#define HIP_WIDTH       40.0
#define LEG_LENGTH      (THIGH_LENGTH + SHIN_LENGTH + ANKLE_HEIGHT)

#define PULSE_PER_DEG   29.6296  // (11500-3500)/270

// =========================================================
// ↓↓↓ 重新編寫的核心 IK Solver (改良版) ↓↓↓
// =========================================================

class IKSolver3D {
private:
    float L1, L2; // 移除了未被使用的 hipWidth 變數

public:
    // 建構子：只需傳入大腿和小腿長度
    IKSolver3D(float thigh, float shin) : L1(thigh), L2(shin) {}

    bool solve(float x, float y, float z, float targetYaw,
               float &hYaw, float &hRoll, float &hPitch, 
               float &kPitch, float &aPitch, float &aRoll, bool isRight) {
        
        hYaw = targetYaw;

        // 計算側擺 (Roll)
        float lateralSide = isRight ? 1.0 : -1.0;
        // 優化：使用 Arduino 內建的 RAD_TO_DEG 替換 180.0 / PI
        hRoll = atan2(y * lateralSide, -z) * RAD_TO_DEG;

        // 計算腿部實際伸展長度的「平方和」(節省重複計算)
        float legLenSq = x*x + y*y + z*z;
        float legLen = sqrt(legLenSq);

        // 限制最大伸展長度，避免超出機械極限 (防止微軟體崩潰)
        float maxLen = L1 + L2 - 0.5;
        if (legLen > maxLen) {
            legLen = maxLen;
            legLenSq = legLen * legLen; // 同步更新平方值
        }

        // 膝蓋屈伸 (利用餘弦定理)
        // 優化：直接使用 legLenSq，減少開根號又平方的運算耗損
        float cosK = (legLenSq - L1*L1 - L2*L2) / (2 * L1 * L2);
        kPitch = acos(constrain(cosK, -1.0, 1.0)) * RAD_TO_DEG;

        // 髖關節前後 Pitch (幾何投影修正)
        // 修正：當腿向外展(y不為0)時，從側面看的有效Z深度會變大
        float effectiveZ = sqrt(y*y + z*z); 
        float alpha = atan2(x, effectiveZ); 
        
        float cosBeta = (L1*L1 + legLenSq - L2*L2) / (2 * L1 * legLen);
        float beta = acos(constrain(cosBeta, -1.0, 1.0));
        
        hPitch = (alpha + beta) * RAD_TO_DEG;

        // 腳踝前後 Pitch：抵銷膝蓋與髖部角度，保持腳掌與地面平行
        aPitch = kPitch - hPitch;

        // 腳踝側擺 Roll：抵銷髖關節側擺，保持腳掌貼地
        aRoll = -hRoll;
        
        return true;
    }
    
    // =========================================================
    // ↓↓↓ 改良版的即時 IK 測試函數 ↓↓↓
    // =========================================================
    void testSolver(float x, float y, float z, float yaw, bool isRight, bool plotMode = false) {
        float hYaw, hRoll, hPitch, kPitch, aPitch, aRoll;
        
        // 呼叫我們剛剛改良過的 solve 函數
        bool isValid = solve(x, y, z, yaw, hYaw, hRoll, hPitch, kPitch, aPitch, aRoll, isRight);
        
        // 1. 安全檢查：確保 IK 有成功算出結果，且沒有超出極限
        if (!isValid) {
            Serial1.println("警告：超出 IK 運算範圍！指令已忽略。");
            return; // 立即中止，保護硬體
        }

        // 2. 輸出模式選擇
        if (plotMode) {
            // 繪圖模式：專門給 Arduino Serial Plotter 使用
            // 格式：數值1,數值2,數值3... (不加多餘文字)
            Serial1.print(hYaw);   Serial1.print(",");
            Serial1.print(hRoll);  Serial1.print(",");
            Serial1.print(hPitch); Serial1.print(",");
            Serial1.print(kPitch); Serial1.print(",");
            Serial1.print(aPitch); Serial1.print(",");
            Serial1.println(aRoll); 
        } else {
            // 文字除錯模式：整理得更乾淨易讀
            Serial1.print(F("[IK即時測試] "));
            Serial1.print(isRight ? F("右腳") : F("左腳"));
            Serial1.print(F(" | 座標("));
            Serial1.print(x); Serial1.print(F(","));
            Serial1.print(y); Serial1.print(F(","));
            Serial1.print(z); 
            Serial1.print(F(") -> 角度: 髖Y=")); Serial1.print(hYaw);
            Serial1.print(F(", 髖R=")); Serial1.print(hRoll);
            Serial1.print(F(", 髖P=")); Serial1.print(hPitch);
            Serial1.print(F(", 膝=")); Serial1.print(kPitch);
            Serial1.print(F(", 踝P=")); Serial1.print(aPitch);
            Serial1.print(F(", 踝R=")); Serial1.println(aRoll);
        }

        // 3. 預留實體聯動 (需配合你專案中的 ICS 函數使用)
        // 如果想即時看到硬體動起來，可以在這裡加入馬達驅動指令
        /*
        if (isRight) {
             // 假設你要把角度轉回 ICS 數值並傳送
             // setServoAngle(RIGHT_HIP_PITCH_ID, hPitch);
             // setServoAngle(RIGHT_KNEE_ID, kPitch);
             // ...
        }
        */
    }
};

// =========================================================
// WalkGenerator 類別 (使用改良版 IK Solver)
// =========================================================

class WalkGenerator {
private:
    IKSolver3D ikSolver;
    float phase;
    bool walking;
    int stepsRemaining;
    unsigned long lastUpdate, lastServoSend;

    bool walkF, walkB, turnL, turnR;
    
    float rX, rY, rZ, rYaw, lX, lY, lZ, lYaw;
    float ry, rr, rp, rk, rap, rar, ly, lr, lp, lk, lap, lar;
    float p3, p5, p7, p9, p11, p13, p4, p6, p8, p10, p12, p14;
    
    // 可即時調整的 IK 參數
    float swayAmount = 14.0;
    float strideAmount = 20.0;
    float liftAmount = 22.0;
    float turnAmount = 12.0;
    float walkSpeed = 1.5;
    float squatDepth = -120.0;
    float ankleGain = 0.9;
    float hipPitchGain = 1.0;
    float hipRollGain = 1.0;
    float kneeGain = 1.0;
    float anklePitchGain = 1.0;
    float ankleRollGain = 1.0;
    
    // 調試模式開關
    bool ikDebugMode = false;
    bool walkDebugMode = false;
    int ikVersion = 1;

public:
    // 修改建構子：移除了 HIP_WIDTH 參數
    WalkGenerator() : ikSolver(THIGH_LENGTH, SHIN_LENGTH), phase(0), walking(false), stepsRemaining(0) {
        walkF = walkB = turnL = turnR = false;
        lastUpdate = lastServoSend = 0;
        memset(&rX, 0, sizeof(rX));
    }

    void setDirection(char dir) {
        walkF = (dir == 'F');
        walkB = (dir == 'B');
        turnL = (dir == 'L'); 
        turnR = (dir == 'R');
    }

    void walkSteps(int s) { 
        stepsRemaining = s;
        walking = true; 
        phase = 0; 
        lastUpdate = millis();
        lastServoSend = millis();
        Serial1.print(F("開始行 ")); 
        Serial1.print(s); 
        Serial1.println(F(" 步"));
    }
    
    void safeStop() { 
        Serial1.println(F("🛑 安全停止中..."));
        walking = false; 
        stepsRemaining = 0; 
        phase = 0;
        moveAllServosToHome();
        Serial1.println(F("✅ 安全停止完成"));
    }
    
    bool isWalking() { return walking; }
    
    // 即時參數調整方法
    void setParam(String name, float value) {
        if (name == "SWAY") {
            swayAmount = constrain(value, 5, 35);
            Serial1.print("✅ 重心擺幅 = "); Serial1.println(swayAmount);
        }
        else if (name == "STRIDE") {
            strideAmount = constrain(value, 10, 45);
            Serial1.print("✅ 步幅 = "); Serial1.println(strideAmount);
        }
        else if (name == "LIFT") {
            liftAmount = constrain(value, 15, 45);
            Serial1.print("✅ 抬腿高度 = "); Serial1.println(liftAmount);
        }
        else if (name == "TURN") {
            turnAmount = constrain(value, 5, 30);
            Serial1.print("✅ 轉彎幅度 = "); Serial1.println(turnAmount);
        }
        else if (name == "SPEED") {
            walkSpeed = constrain(value, 0.8, 2.5);
            Serial1.print("✅ 步行速度 = "); Serial1.println(walkSpeed);
        }
        else if (name == "SQUAT") {
            squatDepth = constrain(value, -140, -100);
            Serial1.print("✅ 蹲踞深度 = "); Serial1.println(squatDepth);
        }
        else if (name == "ANKLEGAIN") {
            ankleGain = constrain(value, 0.5, 1.5);
            Serial1.print("✅ 腳踝增益 = "); Serial1.println(ankleGain);
        }
        else if (name == "HIPPITCH") {
            hipPitchGain = constrain(value, 0.7, 1.3);
            Serial1.print("✅ 髖 Pitch 增益 = "); Serial1.println(hipPitchGain);
        }
        else if (name == "HIPROLL") {
            hipRollGain = constrain(value, 0.7, 1.3);
            Serial1.print("✅ 髖 Roll 增益 = "); Serial1.println(hipRollGain);
        }
        else if (name == "KNEE") {
            kneeGain = constrain(value, 0.7, 1.3);
            Serial1.print("✅ 膝蓋增益 = "); Serial1.println(kneeGain);
        }
        else if (name == "ANKLEPITCH") {
            anklePitchGain = constrain(value, 0.5, 1.5);
            Serial1.print("✅ 腳踝 Pitch 增益 = "); Serial1.println(anklePitchGain);
        }
        else if (name == "ANKLEROLL") {
            ankleRollGain = constrain(value, 0.5, 1.5);
            Serial1.print("✅ 腳踝 Roll 增益 = "); Serial1.println(ankleRollGain);
        }
        else {
            Serial1.print("❌ 未知參數: "); Serial1.println(name);
            Serial1.println("可用參數: SWAY, STRIDE, LIFT, TURN, SPEED, SQUAT, ANKLEGAIN, HIPPITCH, HIPROLL, KNEE, ANKLEPITCH, ANKLEROLL");
        }
    }
    
    void setIKVersion(int ver) {
        ikVersion = constrain(ver, 1, 3);
        Serial1.print("✅ 切換到 IK 版本 "); Serial1.println(ikVersion);
        if (ikVersion == 1) Serial1.println("  版本1: 標準 IK");
        else if (ikVersion == 2) Serial1.println("  版本2: 改良版 (+5% X, -2% Z)");
        else if (ikVersion == 3) Serial1.println("  版本3: 實驗版 (+2% Z, 腳踝+20%)");
    }
    
    void showParams() {
        Serial1.println("\n╔════════════════════════════════════════╗");
        Serial1.println("║       當前 IK 步行參數                  ║");
        Serial1.println("╠════════════════════════════════════════╣");
        Serial1.print  ("║ SWAY:     "); Serial1.print(swayAmount, 1); Serial1.println("                    ║");
        Serial1.print  ("║ STRIDE:   "); Serial1.print(strideAmount, 1); Serial1.println("                    ║");
        Serial1.print  ("║ LIFT:     "); Serial1.print(liftAmount, 1); Serial1.println("                    ║");
        Serial1.print  ("║ TURN:     "); Serial1.print(turnAmount, 1); Serial1.println("                    ║");
        Serial1.print  ("║ SPEED:    "); Serial1.print(walkSpeed, 2); Serial1.println("                    ║");
        Serial1.print  ("║ SQUAT:    "); Serial1.print(squatDepth, 1); Serial1.println("                    ║");
        Serial1.print  ("║ ANKLEGAIN:"); Serial1.print(ankleGain, 2); Serial1.println("                    ║");
        Serial1.print  ("║ HIPPITCH: "); Serial1.print(hipPitchGain, 2); Serial1.println("                    ║");
        Serial1.print  ("║ HIPROLL:  "); Serial1.print(hipRollGain, 2); Serial1.println("                    ║");
        Serial1.print  ("║ KNEE:     "); Serial1.print(kneeGain, 2); Serial1.println("                    ║");
        Serial1.print  ("║ ANKLEPITCH:"); Serial1.print(anklePitchGain, 2); Serial1.println("                    ║");
        Serial1.print  ("║ ANKLEROLL: "); Serial1.print(ankleRollGain, 2); Serial1.println("                    ║");
        Serial1.print  ("║ IK版本:   "); Serial1.print(ikVersion); 
        if (ikVersion == 1) Serial1.println(" (標準)             ║");
        else if (ikVersion == 2) Serial1.println(" (改良)             ║");
        else Serial1.println(" (實驗)             ║");
        Serial1.print  ("║ IK調試:   "); Serial1.print(ikDebugMode ? "開啟" : "關閉"); 
        Serial1.println("                    ║");
        Serial1.println("╚════════════════════════════════════════╝\n");
    }
    
    void setDebugMode(String mode, bool enable) {
        if (mode == "IK") {
            ikDebugMode = enable;
            Serial1.print("✅ IK 調試模式 "); Serial1.println(enable ? "開啟" : "關閉");
        } else if (mode == "WALK") {
            walkDebugMode = enable;
            Serial1.print("✅ 步行調試模式 "); Serial1.println(enable ? "開啟" : "關閉");
        }
    }
    
    void testIK(float x, float y, float z, float yaw, bool isRight, bool plotMode = false) {
        ikSolver.testSolver(x, y, z, yaw, isRight, plotMode);
    }
    
    void updateWalk() {
        if (!walking) return;

        unsigned long now = millis();
        float dt = (now - lastUpdate) / 1000.0;
        if (dt <= 0) return;

        lastUpdate = now;

        phase += (2.0 / walkSpeed) * dt;
        
        if (phase >= 2.0) {
            phase -= 2.0;
            if (stepsRemaining > 0) {
                stepsRemaining--;
                if (stepsRemaining <= 0) { 
                    walking = false;
                    moveAllServosToHome(); 
                    Serial1.println(F("✅ 步行完成"));
                    return; 
                }
            }
        }

        float sway = swayAmount * sin(phase * PI);
        float liftH = liftAmount;
        float stride = strideAmount;
        float turn = turnAmount;
        
        float tR = phase;
        float tL = fmod(phase + 1.0, 2.0);

        auto calcLeg = [&](float t, float &tx, float &ty, float &tz, float &tyaw, bool isR) {
            float moveDir = (walkF ? 1.0 : (walkB ? -1.0 : 0.0));
            float turnDir = (turnL ? 1.0 : (turnR ? -1.0 : 0.0));
            
            ty = -sway;
            
            if (t < 1.0) {
                float smooth = cos(t * PI);
                tx = moveDir * (stride / 2.0) * smooth;
                tyaw = turnDir * (turn / 2.0) * smooth;
                tz = squatDepth;
            } else {
                float swingT = t - 1.0;
                float smooth = -cos(swingT * PI);
                tx = moveDir * (stride / 2.0) * smooth;
                tyaw = turnDir * (turn / 2.0) * smooth;
                tz = squatDepth + liftH * sin(swingT * PI);
            }
        };

        calcLeg(tR, rX, rY, rZ, rYaw, true);
        calcLeg(tL, lX, lY, lZ, lYaw, false);

        // IK 求解（支援版本切換）
        switch(ikVersion) {
            case 1:  // 標準版本
                ikSolver.solve(rX, rY, rZ, rYaw, ry, rr, rp, rk, rap, rar, true);
                ikSolver.solve(lX, lY, lZ, lYaw, ly, lr, lp, lk, lap, lar, false);
                break;
            case 2:  // 改良版：增加 X 方向行程，減少 Z 方向
                ikSolver.solve(rX * 1.05, rY, rZ * 0.98, rYaw, ry, rr, rp, rk, rap, rar, true);
                ikSolver.solve(lX * 1.05, lY, lZ * 0.98, lYaw, ly, lr, lp, lk, lap, lar, false);
                break;
            case 3:  // 實驗版：增加 Z 方向行程，增加腳踝角度
                ikSolver.solve(rX, rY, rZ * 1.02, rYaw * 0.95, ry, rr, rp, rk, rap, rar, true);
                ikSolver.solve(lX, lY, lZ * 1.02, lYaw * 0.95, ly, lr, lp, lk, lap, lar, false);
                rap = rap * 1.2;
                lap = lap * 1.2;
                break;
        }

        // 應用增益
        rp = rp * hipPitchGain;
        rr = rr * hipRollGain;
        rk = rk * kneeGain;
        rap = rap * ankleGain * anklePitchGain;
        rar = rar * ankleRollGain;
        
        lp = lp * hipPitchGain;
        lr = lr * hipRollGain;
        lk = lk * kneeGain;
        lap = lap * ankleGain * anklePitchGain;
        lar = lar * ankleRollGain;

        // 抬腿時，腳尖微向上勾 5 度
        if (rZ > squatDepth + 1.0) rap += 5.0;
        if (lZ > squatDepth + 1.0) lap += 5.0;

        // IK 調試輸出
        if (ikDebugMode) {
            static unsigned long lastIKDebug = 0;
            if (millis() - lastIKDebug >= 200) {
                lastIKDebug = millis();
                Serial1.print("IK-DEBUG: Phase="); Serial1.print(phase, 2);
                Serial1.print(" R: P="); Serial1.print(rp, 1);
                Serial1.print(" K="); Serial1.print(rk, 1);
                Serial1.print(" A="); Serial1.print(rap, 1);
                Serial1.print(" L: P="); Serial1.print(lp, 1);
                Serial1.print(" K="); Serial1.print(lk, 1);
                Serial1.print(" A="); Serial1.println(lap, 1);
            }
        }

        // 角度映射
        p3  = 7780 + ry * PULSE_PER_DEG;    
        p5  = 7400 + rr * PULSE_PER_DEG;    
        p7  = 7500 - rp * PULSE_PER_DEG;    
        p9  = 7500 - rk * PULSE_PER_DEG;    
        p11 = 7500 - rap * PULSE_PER_DEG;  
        p13 = 7825 + rar * PULSE_PER_DEG;  
        
        p4  = 7500 + ly * PULSE_PER_DEG;    
        p6  = 7600 - lr * PULSE_PER_DEG;    
        p8  = 7500 + lp * PULSE_PER_DEG;    
        p10 = 7500 + lk * PULSE_PER_DEG;    
        p12 = 7550 + lap * PULSE_PER_DEG;  
        p14 = 7450 - lar * PULSE_PER_DEG;   

        if (now - lastServoSend >= 20) {
            sendAngles();
            lastServoSend = now;
        }
    }

    void sendAngles() {
        static bool speedSet = false;
        static unsigned long lastDebugTime = 0;
        
        if (!speedSet) {
            int legServos[] = {3,4,5,6,7,8,9,10,11,12,13,14};
            for (int i = 0; i < 12; i++) {
                icsHV.setSpd(legServos[i], 50);
            }
            speedSet = true;
            delay(2);
        }
        
        uint16_t s3  = constrain((uint16_t)round(p3),  6530, 9030);
        uint16_t s5  = constrain((uint16_t)round(p5),  6700, 8300);
        uint16_t s7  = constrain((uint16_t)round(p7),  4700, 10200);
        uint16_t s9  = constrain((uint16_t)round(p9),  3950, 7600);
        uint16_t s11 = constrain((uint16_t)round(p11), 5700, 8300);
        uint16_t s13 = constrain((uint16_t)round(p13), 6800, 9150);
        
        uint16_t s4  = constrain((uint16_t)round(p4),  6250, 8750);
        uint16_t s6  = constrain((uint16_t)round(p6),  6700, 8300);
        uint16_t s8  = constrain((uint16_t)round(p8),  4700, 10200);
        uint16_t s10 = constrain((uint16_t)round(p10), 7400, 11050);
        uint16_t s12 = constrain((uint16_t)round(p12), 6750, 9350);
        uint16_t s14 = constrain((uint16_t)round(p14), 6200, 8450);
        
        if (walkDebugMode && (millis() - lastDebugTime >= 100)) {
            lastDebugTime = millis();
            Serial1.print("WALK-DEBUG: Phase="); Serial1.print(phase, 2);
            Serial1.print(" R7="); Serial1.print(s7);
            Serial1.print(" R9="); Serial1.print(s9);
            Serial1.print(" R11="); Serial1.print(s11);
            Serial1.print(" L8="); Serial1.print(s8);
            Serial1.print(" L10="); Serial1.print(s10);
            Serial1.print(" L12="); Serial1.println(s12);
        }
        
        icsHV.setPos(3, s3);
        icsHV.setPos(5, s5);   
        icsHV.setPos(7, s7);
        icsHV.setPos(9, s9);   
        icsHV.setPos(11, s11); 
        icsHV.setPos(13, s13);
        icsHV.setPos(4, s4);   
        icsHV.setPos(6, s6);   
        icsHV.setPos(8, s8);
        icsHV.setPos(10, s10); 
        icsHV.setPos(12, s12);
        icsHV.setPos(14, s14);
    }
};

WalkGenerator walkGen;

// =========================================================
// ↑↑↑ IK 核心邏輯結束，以下全部原封不動 ↑↑↑
// =========================================================

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
  if (now - voltageData.lastCheckTime < VOLTAGE_CHECK_INTERVAL) return;
  
  voltageData.lastCheckTime = now;
  voltageData.currentVoltage = readBatteryVoltage();
  
  if (voltageData.currentVoltage > voltageData.maxVoltage) voltageData.maxVoltage = voltageData.currentVoltage;
  if (voltageData.currentVoltage < voltageData.minVoltage) voltageData.minVoltage = voltageData.currentVoltage;
  
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
      walkGen.safeStop();
      setLEDOff();
      while(1) delay(1000);
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
  if (Wire.available() >= 2) return (Wire.read() << 8) | Wire.read();
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
  while (Serial1.available() >= 3) {
    uint8_t firstByte = Serial1.peek();
    
    bool isValidICS = false;
    uint8_t cmdType = (firstByte >> 6) & 0x03;
    
    if (cmdType == 0b10 || cmdType == 0b11) isValidICS = true;
    else if ((firstByte & 0xE0) == 0xA0) isValidICS = true;
    else if (firstByte == 0xFF) isValidICS = true;
    
    if (!isValidICS) break;
    
    uint8_t cmd = Serial1.read();
    if (Serial1.available() < 2) break;
    uint8_t byte2 = Serial1.read();
    uint8_t byte3 = Serial1.read();
    
    cmdType = (cmd >> 6) & 0x03;
    uint8_t id = cmd & 0x1F;
    
    ServoInfo* servo = findServoByBinaryID(id);
    if (!servo) continue;
    
    switch(cmdType) {
      case 0b10:
        {
          uint16_t pos = (byte2 << 7) | byte3;
          if (pos >= servo->minAngle && pos <= servo->maxAngle) {
            servo->icsPort->setPos(servo->servoID, pos);
            servo->currentTunePos = pos;
          }
        }
        break;
      
      case 0b11:
        {
          uint8_t paramType = byte2;
          uint8_t value = byte3;
          switch(paramType) {
            case 0x01:
              if (value >= 1 && value <= 127) servo->icsPort->setStrc(servo->servoID, value);
              break;
            case 0x02:
              if (value >= 1 && value <= 127) {
                servo->icsPort->setSpd(servo->servoID, value);
                servo->currentSpeed = value;
              }
              break;
            case 0x03:
              if (value >= 1 && value <= 63) servo->icsPort->setCur(servo->servoID, value);
              break;
            case 0x04:
              if (value >= 1 && value <= 127) servo->icsPort->setTmp(servo->servoID, value);
              break;
          }
        }
        break;
      
      case 0b01:
        {
          uint8_t readType = byte2;
          int result = ICS_FALSE;
          switch(readType) {
            case 0x01: result = servo->icsPort->getStrc(servo->servoID); break;
            case 0x02: result = servo->icsPort->getSpd(servo->servoID); break;
            case 0x03: result = servo->icsPort->getCur(servo->servoID); break;
            case 0x04: result = servo->icsPort->getTmp(servo->servoID); break;
            case 0x05: result = servo->icsPort->getPos(servo->servoID); break;
          }
          if (result != ICS_FALSE) {
            uint8_t reply[3];
            reply[0] = 0x80 | id;
            reply[1] = (result >> 7) & 0x7F;
            reply[2] = result & 0x7F;
            Serial1.write(reply, 3);
          }
        }
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
    
    int binaryId = idStr.substring(2).toInt();
    
    spacePos = data.indexOf(' ', index);
    if (spacePos < 0) spacePos = data.length();
    
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
        bool groupMatch = (group == "MV" && !servo->isHV) ||
                          (group == "HV" && servo->isHV);
        
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
        bool groupMatch = (group == "MV" && !servo->isHV) ||
                          (group == "HV" && servo->isHV);
        
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
  icsHV.setSpd(1, 37); icsMV.setSpd(4, 37);
  icsMV.setSpd(8, 37);
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
      icsHV.setSpd(2, 46);
      icsHV.setSpd(1, 46);
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
  Serial1.println(F("\n📦 [ICS] Shake a Box"));
  icsHV.setSpd(2, 37);   icsMV.setSpd(5, 37);
  icsMV.setSpd(9, 37);   icsHV.setSpd(1, 37);
  icsMV.setSpd(4, 37);
  icsMV.setSpd(8, 37);
  delay(5);
  
  icsHV.setPos(2, 6538);
  icsMV.setPos(5, 5247);
  icsMV.setPos(9, 6763);
  icsHV.setPos(1, 8362);
  icsMV.setPos(4, 9753);
  icsMV.setPos(8, 8237);
  
  delay(700);
  
  for (int i = 0; i < 8; i++) {
    if (i % 2 == 0) {
      icsHV.setSpd(2, 46);
      icsHV.setSpd(1, 46);
      delay(2);
      icsHV.setPos(2, 5848);
      icsHV.setPos(1, 9052);
    } else {
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
  Serial1.println(F("プリメイドAI - 重寫 IK 步行控制 + 安全約束"));
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
  Serial1.println(F("  - 即時 IK 調試功能 (IK 指令)"));
  Serial1.println(F("  - 改良版 IK Solver (幾何投影修正)"));
  
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
  processOfficialBinary();
  
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
          Serial1.print(F("加速度: X="));
          Serial1.print(mpuData.ax);
          Serial1.print(F(" Y=")); Serial1.print(mpuData.ay);
          Serial1.print(F(" Z=")); Serial1.println(mpuData.az);
          
          Serial1.print(F("陀螺儀: X=")); Serial1.print(mpuData.gx);
          Serial1.print(F(" Y=")); Serial1.print(mpuData.gy);
          Serial1.print(F(" Z=")); Serial1.println(mpuData.gz);
          
          Serial1.print(F("溫度: ")); Serial1.println(mpuData.temperature);
        }
      } else {
        Serial1.println(F("❌ 陀螺儀未校準，請輸入 CALIBRATE"));
      }
  }
  else if (cmd == "VOLTAGE") {
      Serial1.print(F("🔋 當前電壓: "));
      Serial1.print(voltageData.currentVoltage);
      Serial1.println(F("V"));
  }
  else if (cmd == "CALIBRATE") {
      calibrateGyro();
  }
  else if (cmd == "HOME") {
      walkGen.safeStop();
      moveAllServosToHome();
  }
  else if (cmd == "FREE ALL") {
      walkGen.safeStop(); 
      processFreeCommand("FREE ALL");
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
          case 'L': Serial1.print(F("向左轉 ")); break;
          case 'R': Serial1.print(F("向右轉 ")); break;
          default: 
              Serial1.println(F("❌ 方向錯誤 (F/B/L/R)"));
              return;
      }
      
      Serial1.print(steps);
      Serial1.println(F(" 步"));
      
      walkGen.walkSteps(steps);
      
      Serial1.println(F("✅ 步行指令已接收"));
  }
  else if (cmd == "STOP") {
      walkGen.safeStop();
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
  // ===== 新增：IK 調試命令 =====
  else if (cmd.startsWith("IK ")) {
      String params = cmd.substring(3);
      params.trim();
      
      if (params == "PARAM" || params == "SHOW") {
          walkGen.showParams();
      }
      else if (params.startsWith("SET ")) {
          String setCmd = params.substring(4);
          int spacePos = setCmd.indexOf(' ');
          if (spacePos > 0) {
              String paramName = setCmd.substring(0, spacePos);
              float value = setCmd.substring(spacePos + 1).toFloat();
              walkGen.setParam(paramName, value);
          } else {
              Serial1.println(F("用法: IK SET <參數> <數值>"));
          }
      }
      else if (params.startsWith("VERSION ")) {
          int ver = params.substring(8).toInt();
          walkGen.setIKVersion(ver);
      }
      else if (params.startsWith("DEBUG ")) {
          String debugCmd = params.substring(6);
          int spacePos = debugCmd.indexOf(' ');
          if (spacePos > 0) {
              String mode = debugCmd.substring(0, spacePos);
              bool enable = (debugCmd.substring(spacePos + 1) == "ON");
              walkGen.setDebugMode(mode, enable);
          } else {
              bool enable = (debugCmd == "ON");
              walkGen.setDebugMode("IK", enable);
              walkGen.setDebugMode("WALK", enable);
          }
      }
      else if (params.startsWith("TEST ")) {
          // 支援兩種模式：
          // IK TEST <x> <y> <z> <yaw> <R/L>           (文字模式)
          // IK TEST PLOT <x> <y> <z> <yaw> <R/L>      (繪圖模式)
          String testCmd = params.substring(5);
          testCmd.trim();
          
          bool plotMode = false;
          if (testCmd.startsWith("PLOT ")) {
              plotMode = true;
              testCmd = testCmd.substring(5);
              testCmd.trim();
          }
          
          float x, y, z, yaw;
          char side;
          int parsed = sscanf(testCmd.c_str(), "%f %f %f %f %c", &x, &y, &z, &yaw, &side);
          if (parsed == 5) {
              walkGen.testIK(x, y, z, yaw, (side == 'R' || side == 'r'), plotMode);
          } else {
              Serial1.println(F("用法: IK TEST <x> <y> <z> <yaw> <R/L>"));
              Serial1.println(F("      IK TEST PLOT <x> <y> <z> <yaw> <R/L> (繪圖模式)"));
              Serial1.println(F("範例: IK TEST 20 -15 -120 0 R"));
              Serial1.println(F("      IK TEST PLOT 20 -15 -120 0 R"));
          }
      }
      else {
          Serial1.println(F("IK 子命令: PARAM, SET, VERSION, DEBUG, TEST"));
      }
  }
  else {
      // 嘗試作為標準 ASCII 指令處理
      if (!processASCIICommand(cmd)) {
          Serial1.println(F("❓ 未知命令，輸入 H 查看說明"));
      }
  }
}

// ===== 顯示幫助選單 =====
void showHelp() {
  Serial1.println(F("\n=== 命令列表 ==="));
  Serial1.println(F("H, HELP, ? : 顯示此說明"));
  Serial1.println(F("G          : 顯示陀螺儀與加速度數據"));
  Serial1.println(F("VOLTAGE    : 顯示當前電池電壓"));
  Serial1.println(F("CALIBRATE  : 重新校準陀螺儀"));
  Serial1.println(F("HOME       : 所有伺服機回到初始位置"));
  Serial1.println(F("FREE ALL   : 所有伺服機脫力 (關閉扭力)"));
  Serial1.println(F("\n=== 步行與動作指令 ==="));
  Serial1.println(F("WALK F [步數] : 向前行指定步數 (例如: WALK F 10)"));
  Serial1.println(F("WALK B [步數] : 向後行指定步數"));
  Serial1.println(F("WALK L [步數] : 向左轉彎指定步數"));
  Serial1.println(F("WALK R [步數] : 向右轉彎指定步數"));
  Serial1.println(F("STOP          : 立即安全停止當前動作並回到 Home Point"));
  Serial1.println(F("SHAKE_ASCII   : 執行 Shake Box (ASCII 指令版)"));
  Serial1.println(F("SHAKE_CPP     : 執行 Shake Box (C++ 函式版)"));
  Serial1.println(F("SHAKE_ICS     : 執行 Shake Box (ICS 二進制版)"));
  Serial1.println(F("\n=== 單軸/多軸微調 (ASCII) ==="));
  Serial1.println(F("S [群組] [ID] [角度] [速度] : 設定單一伺服 (例: S HV 1 8000 64)"));
  Serial1.println(F("S MULTI [速度] [數量] [群組ID 角度 ...] : 多軸同步 (例: S MULTI 64 2 HV1 8000 HV2 7000)"));
  Serial1.println(F("? [群組] [ID] : 查詢伺服當前位置 (例: ? HV 1)"));
  Serial1.println(F("\n=== IK 即時調試指令 ==="));
  Serial1.println(F("IK PARAM        : 顯示當前所有 IK 參數"));
  Serial1.println(F("IK SET <參數> <值> : 調整 IK 參數"));
  Serial1.println(F("  可用參數: SWAY, STRIDE, LIFT, TURN, SPEED, SQUAT,"));
  Serial1.println(F("            ANKLEGAIN, HIPPITCH, HIPROLL, KNEE,"));
  Serial1.println(F("            ANKLEPITCH, ANKLEROLL"));
  Serial1.println(F("IK VERSION <1-3> : 切換 IK 版本 (1:標準 2:改良 3:實驗)"));
  Serial1.println(F("IK DEBUG ON/OFF  : 開關 IK 調試輸出"));
  Serial1.println(F("IK DEBUG <IK/WALK> ON/OFF : 開關特定調試模式"));
  Serial1.println(F("IK TEST <x> <y> <z> <yaw> <R/L> : 測試 IK 求解器"));
  Serial1.println(F("IK TEST PLOT <x> <y> <z> <yaw> <R/L> : 測試 IK (繪圖模式)"));
  Serial1.println(F("  範例: IK TEST 20 -15 -120 0 R"));
  Serial1.println(F("        IK TEST PLOT 20 -15 -120 0 R"));
}
