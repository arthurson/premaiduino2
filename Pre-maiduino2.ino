

#include <Arduino.h>
#include <math.h>  
#include <string.h>  // strlen()/strcmp()/strncmp()/strchr()/memmove()，指令 parsing 用
#include <stdlib.h>  // atoi()，指令參數轉整數用（浮點另用 parseSimpleFloat()，避免拉入 strtod）
#include <ctype.h>   // toupper()，指令轉大寫用
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_flash_ex.h>

// MotionIdEntry 定義擺喺呢度（檔案最頭）而唔係喺後面實際使用嗰個位置，
// 係因為 Arduino IDE 嘅 sketch preprocessor 會自動幫所有 function
// 產生 forward-declaration 塞去檔案最頭。如果 struct 定義擺喺後面，
// 自動生成嘅 "static MotionIdEntry *motionIdFind(uint16_t);" 呢類
// prototype 會出現喺 struct 定義之前，導致 "MotionIdEntry does not
// name a type" 編譯錯誤。搬嚟呢度確保個 struct 一定喺任何自動生成
// prototype 之前已經定義好。
struct MotionIdEntry {
  bool used;
  uint16_t motionId;
  uint8_t startPage;
  uint16_t totalLen;
};

// 同上，setupServoParam() 用嘅 function pointer 型別要喺呢度定義，
// 否則 Arduino IDE 自動生成嘅 forward declaration（會擺喺呢個位置）
// 會用到仲未定義嘅 ServoSetFn，導致 "has not been declared" 編譯錯誤。
struct ServoInfo;
typedef bool (*ServoSetFn)(ServoInfo *, uint8_t);

HardwareSerial Serial2(PA2);
HardwareSerial Serial3(PB10);

// ===== 引入官方 ICS 函式庫 =====
#include <IcsHardSerialClass.h>

// ===== 定義 ICS_FALSE 常數 =====
#ifndef ICS_FALSE
#define ICS_FALSE -1
#endif

// ===== 定義 EN 腳位 =====
#define EN_HV_PIN PA4
#define EN_MV_PIN PB2

#include "LEDControl.h"
#include "VoltageMonitor.h"
#include "MPU6050IMU.h"
#include "BalanceGyro.h"

// ===== 建立兩個通訊物件 =====
IcsHardSerialClass icsHV(&Serial2, EN_HV_PIN, 1250000, 10);
IcsHardSerialClass icsMV(&Serial3, EN_MV_PIN, 1250000, 10);

// ===== 25軸伺服資訊 =====
#include "ServoConfig.h"

// ===== 系統狀態 =====
// ASCII 指令輸入 buffer：固定大細 char array 代替 String，避免 heap
// allocation/fragmentation。63 字元夠晒最長嘅指令（"WALKSET BALANCE
// KNEE_PITCH_P 0.123" 之類），加 1 個 null terminator。
#define INPUT_BUFFER_SIZE 64
char inputBuffer[INPUT_BUFFER_SIZE];
uint8_t inputBufferLen = 0;

// 將一個字元 append 落 inputBuffer，滿咗就唔再收（防止 buffer overflow；
// 呢種情況代表指令異常長，多半係雜訊，直接丟棄多出嘅字元）。
inline void inputBufferAppend(char c) {
  if (inputBufferLen < INPUT_BUFFER_SIZE - 1) {
    inputBuffer[inputBufferLen++] = c;
    inputBuffer[inputBufferLen] = '\0';
  }
}
inline void inputBufferClear() {
  inputBufferLen = 0;
  inputBuffer[0] = '\0';
}

// 最近一次有 servo 動作嘅時間戳（safeSetPos() 更新）。
// 俾 loop() 用嚟判斷而家係咪「有動作進行緊」，決定 LED 顯示忙碌
// 提示定係待機呼吸燈。table_walk/.pma player 都經 safeSetPos()，
// 唔使逐個功能加 LED code。
unsigned long lastActivityMs = 0;

// FREE ALL 之後進入「脫力模式」，LED 顯示紫色，直到下次有 servo 動作
// （safeSetPos() 被 call）先自動解除，交返俾 updateActivityLED() 判斷。
bool isFreeMode = false;

// ===== IMU (MPU-6050) 狀態 =====
#define IMU_UPDATE_INTERVAL_MS 10  // 100Hz，試下加密 tick 對 balance 反應速度嘅影響
unsigned long lastImuUpdateMs = 0;

bool imuOk = false;               // imuInit() 結果，setup() 設定
bool imuGyroCalibrated = false;   // 開機自動校正係咪已經做咗（做過一次就唔再做）
#define IMU_AUTO_CAL_DELAY_MS 4000  // 開機後等 servo 行到 home 先校正，非阻塞
// ===== 函式原型 =====
void initServos();
void moveAllServosToHome();
void processCommand(char *cmd);
void showHelp();

bool processASCIICommand(char *cmd);
bool processFreeCommand(char *cmd);
ServoInfo *findServoByBinaryID(uint8_t binaryID);
bool safeSetPos(ServoInfo *servo, uint16_t pos);
bool safeSetSpd(ServoInfo *servo, uint8_t spd);
bool safeSetStrc(ServoInfo *servo, uint8_t strc);
bool safeSetCur(ServoInfo *servo, uint8_t curlim);
bool safeSetTmp(ServoInfo *servo, uint8_t tmplim);
uint16_t applyHomeOffset(ServoInfo *servo, int rawAngle, int baselineCenter);

// 查詢用讀取（? 指令用），set 一律經 safeSetXxx() 唔經呢度
enum ServoParamField { PARAM_STRC, PARAM_SPD, PARAM_CUR, PARAM_TMP };
int getServoParam(byte id, ServoParamField field);

// ===== .pma / ICS binary 封包接收（實際定義喺 PmaProtocol.h） =====
void pmaReceiveUpdate();

// =========================================================
// ===== 伺服基本操作（初始化 / 查找 / 安全讀寫） =====

// =========================================================
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
ServoInfo *findServoByBinaryID(uint8_t binaryID) {
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    if (servoList[i].binaryID == binaryID) {
      return &servoList[i];
    }
  }
  return NULL;
}

// 失敗時 return false；setPos/setSpd 會 retry 一次（半雙工單線偶發 race condition）
// 共用 helper：safeSetSpd/Strc/Cur/Tmp 四個 function 嘅範圍檢查
// 錯誤訊息全部係同一個 pattern（"setXxx 範圍錯誤 (lo-hi): 名 val="），
// 抽出嚟慳返 4 次重複嘅 print 骨架。fnName/rangeStr 用 F() 包住
// 嘅 flash string，唔會拉入額外 RAM。
inline void printServoRangeError(const __FlashStringHelper *fnName,
                                  const __FlashStringHelper *rangeStr,
                                  const char *servoName, int val) {
  Serial1.print(fnName);
  Serial1.print(F(" 範圍錯誤 ("));
  Serial1.print(rangeStr);
  Serial1.print(F("): "));
  Serial1.print(servoName);
  Serial1.print(F(" val="));
  Serial1.println(val);
}

bool safeSetPos(ServoInfo *servo, uint16_t pos) {
  if (!servo) return false;
  if (!servo->enabled) return true;
  lastActivityMs = millis();
  isFreeMode = false;
  int result = servo->icsPort->setPos(servo->servoID, pos);
  if (result == ICS_FALSE) {
    result = servo->icsPort->setPos(servo->servoID, pos);
    if (result == ICS_FALSE) {
      return false;
    }
  }
  return true;
}

bool safeSetSpd(ServoInfo *servo, uint8_t spd) {
  if (!servo) return false;
  if (!servo->enabled) return true;
  if (spd < 1 || spd > 127) {
    printServoRangeError(F("setSpd"), F("1-127"), servo->name, spd);
    return false;
  }
  int result = servo->icsPort->setSpd(servo->servoID, spd);
  if (result == ICS_FALSE) {
    result = servo->icsPort->setSpd(servo->servoID, spd);  // retry 一次
    if (result == ICS_FALSE) {
      return false;
    }
  }
  return true;
}

bool safeSetStrc(ServoInfo *servo, uint8_t strc) {
  if (!servo) return false;
  if (strc < 1 || strc > 127) {
    printServoRangeError(F("setStrc"), F("1-127"), servo->name, strc);
    return false;
  }
  int result = servo->icsPort->setStrc(servo->servoID, strc);
  return (result != ICS_FALSE);
}

bool safeSetCur(ServoInfo *servo, uint8_t curlim) {
  if (!servo) return false;
  if (curlim < 1 || curlim > 63) {
    printServoRangeError(F("setCur"), F("1-63"), servo->name, curlim);
    return false;
  }
  int result = servo->icsPort->setCur(servo->servoID, curlim);
  return (result != ICS_FALSE);
}

bool safeSetTmp(ServoInfo *servo, uint8_t tmplim) {
  if (!servo) return false;
  if (tmplim < 1 || tmplim > 127) {
    printServoRangeError(F("setTmp"), F("1-127"), servo->name, tmplim);
    return false;
  }
  int result = servo->icsPort->setTmp(servo->servoID, tmplim);
  return (result != ICS_FALSE);
}

int getServoParam(byte id, ServoParamField field) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  switch (field) {
    case PARAM_STRC: return servo->icsPort->getStrc(servo->servoID);
    case PARAM_SPD:  return servo->icsPort->getSpd(servo->servoID);
    case PARAM_CUR:  return servo->icsPort->getCur(servo->servoID);
    case PARAM_TMP:  return servo->icsPort->getTmp(servo->servoID);
  }
  return ICS_FALSE;
}

// =========================================================
// ===== 共用角度轉換（.pma 同 table_walk 都用） =====

// =========================================================
// ===== 共用轉換：將「相對7500嘅offset」疊加去伺服真正嘅homePosition =====
// table_walk（TW_CENTER=7500）同 .pma 檔案格式（已實測10個真實動作檔案，
// 證實全部伺服統一以7500做基準）都係用呢個假設砌出嚟嘅相對角度。
// 但 homePosition 係你手動實測校準、令機械人真正企得直嘅權威數值
// （唔等於7500，例如兩邊肩膟裝配唔對稱），所以呢兩個來源嘅角度都
// 一定要經呢條轉換式，先可以送去伺服，否則會用錯基準，企得直、
// 一播動作就變形。
uint16_t applyHomeOffset(ServoInfo *servo, int rawAngle, int baselineCenter) {
  int offset = rawAngle - baselineCenter;
  int targetAngle = (int)servo->homePosition + offset;
  return (uint16_t)constrain(targetAngle, (int)servo->minAngle, (int)servo->maxAngle);
}

// =========================================================
// ===== table_walk 步行系統 =====

// =========================================================

#include "table_walk.h"

tw_walker_t tableWalker;
tw_params_t tableWalkParams;

// 上一次真正送咗去伺服嘅 table_walk 輸出值，用嚟偵測「呢次 tick 嘅
// 輸出同上次係咪完全一樣」——如果一樣，代表機械人其實冇郁（例如
// TW_STOPPED 底下 x2/y2 都係 0），唔應該當「有動作」，咁樣先可以
// 令 lastActivityMs 準確反映真正嘅動作狀態，LED 先可以真正顯示
// 「靜止待機」。
int lastSentServoVal[TW_NUM_SERVOS];
bool lastSentServoValInit = false;

// tick 間隔（毫秒）。kazz 原版 15ms×9=135ms（10分割表），
// 現用 20 分割表，同一段真實步行時間需行多一倍 frame 先完成一個週期。
#define TABLE_WALK_TICK_MS 60
unsigned long lastWalkTickMs = 0;

// ===== .pma-style servo ID → ServoInfo* 對照 =====
// table_walk 輸出嘅係「理論pulse值」，冇考慮你 servoList[] 入面
// 每隻伺服實際嘅安全角度範圍。每次都要經呢個 helper 攞返
// ServoInfo，再用佢嘅 minAngle/maxAngle 做 clamp 先可以送去伺服。
ServoInfo *findServoByPmaId(uint8_t pmaId) {
  // HV bus：pma_id 為偶數，0x02→binaryID1, 0x04→binaryID2, ... 0x1C→binaryID14
  // MV bus（上半身）：pma_id 為奇數 0x03~0x17，對應官方 MV servoID 1~11，
  // binaryID = 20 + servoID（0x03→21頭ピッチ, 0x05→22頭ヨー, 0x07→23頭ロール,
  // 0x09→24右肩側擺, 0x0B→25左肩側擺, ... 0x17→31左手轉向）
  uint8_t binaryID;
  if (pmaId % 2 == 0) {
    binaryID = pmaId / 2;                // HV: 0x02->1, 0x04->2 ... 0x1C->14
  } else if (pmaId >= 0x03 && pmaId <= 0x17) {
    binaryID = 20 + ((pmaId - 1) / 2);   // MV: 0x03->21 ... 0x17->31
  } else {
    return NULL;                        // 未定義嘅 pma ID
  }
  return findServoByBinaryID(binaryID);
}
// ===== 將 table_walk 嘅輸出，經 clamp 之後送去 ICS bus =====
// 只有喺呢隻 servo 嘅目標值同上一次 tick 唔同嗰陣先真正送指令，
// 避免 TW_STOPPED（企定不動）都不斷刷新相同角度，一來慳返 ICS bus
// 流量，二來令 safeSetPos() 準確反映「真正有郁」，LED 活動指示
// （updateActivityLED()）先可以喺靜止時正確顯示待機藍燈。
void sendTableWalkAngles() {
  for (int i = 0; i < TW_NUM_SERVOS; i++) {
    uint8_t pmaId = table_walk_servo_pma_id(i);
    int rawAngle = tableWalker.servo_val[i];

    ServoInfo *servo = findServoByPmaId(pmaId);
    if (!servo) continue;

    if (lastSentServoValInit && rawAngle == lastSentServoVal[i]) {
      continue;  // 同上次一樣，冇實質變化，唔使送
    }
    lastSentServoVal[i] = rawAngle;

    uint16_t safeAngle = applyHomeOffset(servo, rawAngle, TW_CENTER);
    safeSetPos(servo, safeAngle);
    servo->currentTunePos = safeAngle;   // 保持更新，方便之後其他系統（例如將來重做嘅
                                          // balance）讀取呢個做基準位置
  }
  lastSentServoValInit = true;
}

// ===== 簡易 wrapper，模擬原本 WalkGenerator 嘅 isWalking()/safeStop() 介面 =====
// 方便下面 processCommand() 同 loop() 嘅呼叫方式盡量少改
bool tableWalkIsWalking() {
  return !table_walk_is_stopped(&tableWalker);
}

void tableWalkSafeStop() {
  tw_walker_init(&tableWalker);   // 強制歸零，即時停（唔等收步動畫）
  moveAllServosToHome();
}

// =========================================================
// ===== .pma / ICS binary 封包播放器 + Flash motion storage =====
// =========================================================
#include "PmaProtocol.h"

// ===== 移動到 Home Point =====
void moveAllServosToHome() {
  Serial1.println(F("\n所有伺服回家，速度32"));

  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    safeSetSpd(s, 32);
    s->currentSpeed = 32;
  }

  delay(5);

  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    ServoInfo *s = &servoList[i];
    safeSetPos(s, s->homePosition);
    s->currentTunePos = s->homePosition;
  }

  Serial1.println(F("回家指令已發送"));
}

// ===== 脫力功能 =====
bool processFreeCommand(char *cmd) {
  // cmd 入嚟之前 processCommand() 已經 trim+toUpperCase 咗，呢度假設
  // 冇前後空白、全大寫。

  if (strncmp(cmd, "FREE ", 5) != 0) return false;

  char *params = cmd + 5;
  while (*params == ' ') params++;  // 額外 trim（保留同原本 trim() 一致嘅行為）

  if (strcmp(params, "ALL") == 0) {
    Serial1.println(F("脫力所有伺服"));

    for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
      ServoInfo *s = &servoList[i];
      s->icsPort->setFree(s->servoID);
      delay(2);
    }
    isFreeMode = true;
    Serial1.println(F("所有伺服已脫力"));
    return true;
  }

  char *spacePos = strchr(params, ' ');
  if (!spacePos || spacePos == params) return false;

  uint8_t groupLen = (uint8_t)(spacePos - params);
  bool isHVGroup = (groupLen == 2 && strncmp(params, "HV", 2) == 0);
  bool isMVGroup = (groupLen == 2 && strncmp(params, "MV", 2) == 0);
  int id = atoi(spacePos + 1);

  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    bool groupMatch = (isHVGroup && servoList[i].isHV) || (isMVGroup && !servoList[i].isHV);

    if (groupMatch && servoList[i].binaryID == id) {
      servoList[i].icsPort->setFree(servoList[i].servoID);
      Serial1.print(F("脫力 "));
      Serial1.print(isHVGroup ? "HV" : "MV");
      Serial1.print(F(" ID "));
      Serial1.println(id);
      return true;
    }
  }

  Serial1.println(F("找不到對應的伺服"));
  return true;
}

// ===== 處理ASCII指令 =====
bool processASCIICommand(char *cmd) {
  // cmd 入嚟之前 processCommand() 已經 trim+toUpperCase 咗。

  if (strncmp(cmd, "FREE ", 5) == 0) return processFreeCommand(cmd);

  if (cmd[0] == '?' && cmd[1] != '\0' && isDigit(cmd[1])) {
    // 格式: ?<binaryID> [STRC|SPD|CUR|TMP]，例如 ?13 或 ?13 STRC
    // binaryID 本身已經唯一分辨 HV(1-14)/MV(21-31)，唔需要再指定群組。
    char *spacePos = strchr(cmd, ' ');
    int binaryId = atoi(cmd + 1);
    // 注意：field 冇 space 時 fallback 落一個真正嘅 static char buffer
    // 而唔係 string literal——雖然而家淨係讀（strcmp/whitespace skip），
    // 但用 literal 一旦將來被改成寫入（例如加咗 trimInPlace(field)）
    // 就會 UB（寫落 flash 嘅 .rodata），養成用真正 buffer 嘅習慣更安全。
    static char emptyFieldBuf[1] = { '\0' };
    char *field = spacePos ? spacePos + 1 : emptyFieldBuf;
    while (*field == ' ') field++;

    ServoInfo *servo = findServoByBinaryID(binaryId);

    if (servo != NULL) {
      const char *groupLabel = servo->isHV ? "HV" : "MV";

      if (!servo->enabled) {
        Serial1.print(servo->name);
        Serial1.println(F(" 未駁線 (enabled=false)，略過"));
        return true;
      }

      bool isStrc = strcmp(field, "STRC") == 0;
      bool isSpd  = strcmp(field, "SPD") == 0;
      bool isCur  = strcmp(field, "CUR") == 0;
      bool isTmp  = strcmp(field, "TMP") == 0;

      if (isStrc || isSpd || isCur || isTmp) {
        int result = ICS_FALSE;
        const char *label = "";
        if (isStrc) { result = getServoParam(binaryId, PARAM_STRC); label = "Stretch"; }
        else if (isSpd)  { result = getServoParam(binaryId, PARAM_SPD);  label = "Speed"; }
        else if (isCur)  { result = getServoParam(binaryId, PARAM_CUR);  label = "電流上限"; }
        else if (isTmp)  { result = getServoParam(binaryId, PARAM_TMP);  label = "溫度上限"; }

        if (result != ICS_FALSE) {
          Serial1.print(groupLabel);
          Serial1.print(F(" ID "));
          Serial1.print(binaryId);
          Serial1.print(F(" "));
          Serial1.print(label);
          Serial1.print(F(": "));
          Serial1.println(result);
        } else {
          Serial1.println(F("讀取失敗（通訊錯誤）"));
        }
        return true;
      }

      int pos = servo->icsPort->setPos(servo->servoID, servo->currentTunePos);
      if (pos != ICS_FALSE) {
        servo->currentTunePos = pos;
      }
      int strc = getServoParam(binaryId, PARAM_STRC);
      int spd  = getServoParam(binaryId, PARAM_SPD);
      int cur  = getServoParam(binaryId, PARAM_CUR);
      int tmp  = getServoParam(binaryId, PARAM_TMP);

      Serial1.print(groupLabel);
      Serial1.print(F(" ID "));
      Serial1.print(binaryId);
      Serial1.print(F(" ("));
      Serial1.print(servo->name);
      Serial1.print(F(") 角度="));
      if (pos != ICS_FALSE) Serial1.print(pos); else Serial1.print(F("讀取失敗"));
      Serial1.print(F(" Stretch="));
      if (strc != ICS_FALSE) Serial1.print(strc); else Serial1.print(F("讀取失敗"));
      Serial1.print(F(" Speed="));
      if (spd != ICS_FALSE) Serial1.print(spd); else Serial1.print(F("讀取失敗"));
      Serial1.print(F(" 電流上限="));
      if (cur != ICS_FALSE) Serial1.print(cur); else Serial1.print(F("讀取失敗"));
      Serial1.print(F(" 溫度上限="));
      if (tmp != ICS_FALSE) Serial1.println(tmp); else Serial1.println(F("讀取失敗"));
      return true;
    }
    return true;
  }

  return false;
}

static void setupServoParam(const char *label, uint16_t hvVal, uint16_t mvVal,
                             ServoSetFn setFn, bool showFailures) {
  Serial1.print(F("設定伺服"));
  Serial1.print(label);
  Serial1.print(F(" (HV="));
  Serial1.print(hvVal);
  Serial1.print(F(", MV="));
  Serial1.print(mvVal);
  Serial1.print(F(")..."));
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    uint16_t val = servoList[i].isHV ? hvVal : mvVal;
    bool ok = setFn(&servoList[i], (uint8_t)val);
    if (showFailures && !ok) {
      Serial1.print(F(" [失敗:")); Serial1.print(servoList[i].name); Serial1.print(F("]"));
    }
  }
  if (showFailures) {
    Serial1.println(F(" 完成"));
  } else {
    Serial1.println(F("完成"));
  }
}
// ===== WALKSET STRC/SPD/CUR/TMP 共用邏輯 =====
static void walkSetServoParam(const char *paramName, const char *group, int rawVal) {
  bool wantHV = (strcmp(group, "HV") == 0);
  int value;
  const char *label;
  ServoSetFn setFn;

  if (strcmp(paramName, "STRC") == 0) {
    value = constrain(rawVal, 1, 127); label = "Stretch"; setFn = safeSetStrc;
  } else if (strcmp(paramName, "SPD") == 0) {
    value = constrain(rawVal, 1, 127); label = "Speed"; setFn = safeSetSpd;
  } else if (strcmp(paramName, "CUR") == 0) {
    value = constrain(rawVal, 1, 63);  label = "電流上限"; setFn = safeSetCur;
  } else {  // "TMP"
    value = constrain(rawVal, 1, 127); label = "溫度上限"; setFn = safeSetTmp;
  }

  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    if (servoList[i].isHV == wantHV) {
      setFn(&servoList[i], (uint8_t)value);
    }
  }
  Serial1.print(group);
  Serial1.print(F(" 伺服 ")); Serial1.print(label);
  Serial1.print(F(" = ")); Serial1.println(value);
}

// 手動 in-place trim（頭尾空白），用於 char buffer，等同 String::trim()。
static void trimInPlace(char *s) {
  char *start = s;
  while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') start++;
  size_t len = strlen(start);
  while (len > 0 && (start[len - 1] == ' ' || start[len - 1] == '\t' ||
                     start[len - 1] == '\r' || start[len - 1] == '\n')) {
    len--;
  }
  memmove(s, start, len);
  s[len] = '\0';
}

static void toUpperInPlace(char *s) {
  for (; *s; s++) *s = toupper((unsigned char)*s);
}

// 手動 parse 一個簡單十進制浮點數（例如 "0.05"、"-1.2"、"3"），代替
// atof()/strtod()。呢個 sketch 嘅浮點輸入淨係 PID gain 呢類簡單數值，
// 唔需要支援科學記數法(1e-5)或者特殊值(inf/nan)，用手動 parse 換嚟
// 唔使 link 標準庫嘅完整 strtod 浮點解析（同 printf 家族一樣，一旦
// call 咗一次，成份浮點格式化/解析 code 就會被 linker 全部拉入）。
static float parseSimpleFloat(const char *s) {
  while (*s == ' ') s++;
  bool neg = false;
  if (*s == '-') { neg = true; s++; }
  else if (*s == '+') { s++; }

  float result = 0.0f;
  while (isDigit(*s)) {
    result = result * 10.0f + (*s - '0');
    s++;
  }
  if (*s == '.') {
    s++;
    float frac = 0.1f;
    while (isDigit(*s)) {
      result += (*s - '0') * frac;
      frac *= 0.1f;
      s++;
    }
  }
  return neg ? -result : result;
}

// 命令最長要容納 "WALKSET BALANCE " (17) + 短碼原文（最長約 20 字）
// 加埋前綴嘅工作 buffer，48 bytes 留有充裕 margin。
#define CMD_WORK_BUF_SIZE 48

// processCommand()/prependToCmd() 假設咗 cmd 呢個 buffer 本身最少有
// CMD_WORK_BUF_SIZE 咁大（先可以安全咁喺原地覆寫展開後嘅短碼字串）。
// 目前 processCommand() 嘅唯一呼叫者傳嘅係 inputBuffer[INPUT_BUFFER_SIZE]，
// 呢個 static_assert 確保如果將來有人改細 INPUT_BUFFER_SIZE、或者加多
//一個傳細 buffer 嘅呼叫者，編譯期就會報錯，而唔係靜默 buffer overflow。
static_assert(INPUT_BUFFER_SIZE >= CMD_WORK_BUF_SIZE,
              "processCommand()/prependToCmd() write back into cmd assuming "
              "it has at least CMD_WORK_BUF_SIZE bytes; INPUT_BUFFER_SIZE must not shrink below it");

// 將 prefix（例如 "WALKSET "）加落 cmd 前面，喺原地覆寫（透過一個
// stack 暫存 buffer）。刻意用 strcpy/strcat 而唔用 snprintf(...,"%s",...)——
// STM32duino 嘅 newlib 冇因為淨係用到 %s 就精簡 snprintf 嘅實現，一旦
// 呢個 sketch 有任何一處 call 過 snprintf/printf 家族，linker 就要
// 成份 vfprintf 格式化引擎（包括浮點支援）成份拉埋入嚟，flash 開銷
// 遠大過 String class 本身，得不償失。strcpy/strcat 淨係 string
// operation，唔會扯埋呢份重嘢。
static void prependToCmd(char *cmd, const char *prefix) {
  char tmp[CMD_WORK_BUF_SIZE];
  strncpy(tmp, prefix, CMD_WORK_BUF_SIZE - 1);
  tmp[CMD_WORK_BUF_SIZE - 1] = '\0';
  size_t prefixLen = strlen(tmp);
  strncat(tmp, cmd, CMD_WORK_BUF_SIZE - 1 - prefixLen);
  strncpy(cmd, tmp, CMD_WORK_BUF_SIZE - 1);
  cmd[CMD_WORK_BUF_SIZE - 1] = '\0';
}

void processCommand(char *cmd) {
  trimInPlace(cmd);
  toUpperInPlace(cmd);

  // 短寫 alias：轉返做完整指令，之後行返原本邏輯，唔使逐個分支改。
  {
    struct AliasEntry { const char *shortCode; const char *fullCmd; };
    static const AliasEntry aliases[] = {
      { "V",     "VOLTAGE"      },
      { "G",     "IMU"          },
      { "GC",    "IMU CAL"      },
      { "HM",    "HOME"         },
      { "F",     "FREE ALL"     },
      { "ST",    "STOP"         },
      { "PS",    "PMASTAT"      },
      { "A",     "ANGLES"       },
      { "B ON",  "BALANCE ON"   },
      { "B OFF", "BALANCE OFF"  },
      { "B",     "BALANCE"      },
    };
    for (uint8_t i = 0; i < sizeof(aliases) / sizeof(aliases[0]); i++) {
      if (strcmp(cmd, aliases[i].shortCode) == 0) {
        strncpy(cmd, aliases[i].fullCmd, CMD_WORK_BUF_SIZE - 1);
        cmd[CMD_WORK_BUF_SIZE - 1] = '\0';
        break;
      }
    }
  }

  // WALKSET 短碼直達：短碼本身可以直接當指令打，唔使成日打
  // 「WALKSET BALANCE」/「WALKSET」呢個字首。轉譯之後行返原有
  // WALKSET parsing 邏輯，唔使重寫成套 parsing。
  //
  // BALANCE 子指令短碼（PP/PI/PD/RP/RI/RD/KPP/KPI/KPD）：
  // 呢批短碼本身喺 WALKSET BALANCE 入面已經識別，呢度淨係補返
  // 「唔使打 WALKSET BALANCE 呢個字首」嘅捷徑。
  size_t cmdLen = strlen(cmd);
  {
    const char *balanceAxes[] = {"PP", "PI", "PD", "RP", "RI", "RD",
                                  "KPP", "KPI", "KPD"};
    for (uint8_t i = 0; i < sizeof(balanceAxes) / sizeof(balanceAxes[0]); i++) {
      const char *axis = balanceAxes[i];
      uint8_t axisLen = strlen(axis);
      if (strcmp(cmd, axis) == 0 ||
          (strncmp(cmd, axis, axisLen) == 0 && cmdLen > axisLen && cmd[axisLen] == ' ')) {
        prependToCmd(cmd, "WALKSET BALANCE ");
        cmdLen = strlen(cmd);
        break;
      }
    }
  }

  // STRC/SPD/CUR/TMP + HV/MV 短碼直達：例如 "TMP MV 40" 直接打，
  // 唔使打 "WALKSET TMP MV 40"。
  {
    const char *servoParams[] = {"STRC", "SPD", "CUR", "TMP"};
    for (uint8_t i = 0; i < sizeof(servoParams) / sizeof(servoParams[0]); i++) {
      const char *p = servoParams[i];
      uint8_t pLen = strlen(p);
      bool matchHV = strncmp(cmd, p, pLen) == 0 && cmdLen > (size_t)(pLen + 2)
                     && cmd[pLen] == ' ' && cmd[pLen + 1] == 'H' && cmd[pLen + 2] == 'V';
      bool matchMV = strncmp(cmd, p, pLen) == 0 && cmdLen > (size_t)(pLen + 2)
                     && cmd[pLen] == ' ' && cmd[pLen + 1] == 'M' && cmd[pLen + 2] == 'V';
      if (matchHV || matchMV) {
        prependToCmd(cmd, "WALKSET ");
        cmdLen = strlen(cmd);
        break;
      }
    }
  }

  // LEN/ROLL/PITCH 短碼直達：例如 "LEN 150" 直接打，唔使打
  // "WALKSET LEN 150"。注意 "ROLL" 呢個短碼同 BALANCE 嘅 "RP/RI/RD"
  // 唔會撞名（呢度係完整字 ROLL，唔係 R 開頭嘅短碼）。
  {
    const char *walkParams[] = {"LEN", "ROLL", "PITCH"};
    for (uint8_t i = 0; i < sizeof(walkParams) / sizeof(walkParams[0]); i++) {
      const char *p = walkParams[i];
      uint8_t pLen = strlen(p);
      if (strncmp(cmd, p, pLen) == 0 && cmdLen > pLen && cmd[pLen] == ' ') {
        prependToCmd(cmd, "WALKSET ");
        cmdLen = strlen(cmd);
        break;
      }
    }
  }

  if (strcmp(cmd, "H") == 0 || strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
    showHelp();
  } else if (strcmp(cmd, "VOLTAGE") == 0) {
    Serial1.print(F("當前電壓: "));
    Serial1.print(voltageData.currentVoltage);
    Serial1.println(F("V"));
  } else if (strcmp(cmd, "PMASTAT") == 0) {
    Serial1.print(F(".pma 封包統計: 成功="));
    Serial1.print(pmaPacketOkCount);
    Serial1.print(F(" 失敗="));
    Serial1.println(pmaPacketFailCount);
  } else if (strcmp(cmd, "ANGLES") == 0) {
    Serial1.println(F("---- 全部伺服角度 ----"));
    for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
      ServoInfo *s = &servoList[i];
      Serial1.print(s->isHV ? F("HV") : F("MV"));
      Serial1.print(F(" ID "));
      Serial1.print(s->binaryID);
      Serial1.print(F(" ("));
      Serial1.print(s->name);
      Serial1.print(F("): "));
      if (!s->enabled) {
        Serial1.println(F("未駁線"));
        continue;
      }
      int pos = s->icsPort->setPos(s->servoID, s->currentTunePos);
      if (pos != ICS_FALSE) {
        s->currentTunePos = pos;
        Serial1.println(pos);
      } else {
        Serial1.println(F("讀取失敗"));
      }
    }
  } else if (strcmp(cmd, "IMU") == 0) {
    if (!imuData.present) {
      Serial1.println(F("IMU 未偵測到"));
    } else {
      Serial1.println(F("---- IMU (MPU-6050) ----"));
      Serial1.print(F("accel(g)  X=")); Serial1.print(imuData.accelX, 3);
      Serial1.print(F(" Y=")); Serial1.print(imuData.accelY, 3);
      Serial1.print(F(" Z=")); Serial1.println(imuData.accelZ, 3);
      Serial1.print(F("gyro(dps) X=")); Serial1.print(imuData.gyroX, 2);
      Serial1.print(F(" Y=")); Serial1.print(imuData.gyroY, 2);
      Serial1.print(F(" Z=")); Serial1.println(imuData.gyroZ, 2);
      Serial1.print(F("pitch=")); Serial1.print(imuData.pitch, 2);
      Serial1.print(F("°  roll=")); Serial1.print(imuData.roll, 2);
      Serial1.println(F("°"));
    }
  } else if (strcmp(cmd, "IMU CAL") == 0) {
    if (!imuData.present) {
      Serial1.println(F("IMU 未偵測到，無法校正"));
    } else {
      Serial1.print(F("校正陀螺零偏（機身請保持靜止）..."));
      imuCalibrateGyro();
      Serial1.println(F("完成"));
    }
  } else if (strcmp(cmd, "HOME") == 0) {
    tableWalkSafeStop();
    moveAllServosToHome();
  } else if (strcmp(cmd, "FREE ALL") == 0) {
    tableWalkSafeStop();
    processFreeCommand((char *)"FREE ALL");
  }
  else if (strncmp(cmd, "WALK ", 5) == 0) {
    char *params = cmd + 5;
    while (*params == ' ') params++;

    char dir = params[0];

    // table_walk 用持續性方向輸入（x1/y1），冇「行N步」概念，
    // 淨係設定方向同開始行，直到收到 STOP 為止。
    switch (dir) {
      case 'F': tableWalker.y1 = 1.0f;  tableWalker.x1 = 0.0f; Serial1.print(F("向前行")); break;
      case 'B': tableWalker.y1 = -1.0f; tableWalker.x1 = 0.0f; Serial1.print(F("向後行")); break;
      case 'L': tableWalker.y1 = 0.0f;  tableWalker.x1 = -1.0f; Serial1.print(F("向左轉")); break;
      case 'R': tableWalker.y1 = 0.0f;  tableWalker.x1 = 1.0f; Serial1.print(F("向右轉")); break;
      default:
        Serial1.println(F("方向錯誤 (F/B/L/R)"));
        return;
    }
    Serial1.println();

    tableWalker.lt_held = 1;

    Serial1.println(F("步行指令已接收（輸入 STOP 停止）"));
  } else if (strcmp(cmd, "STOP") == 0) {
    tableWalkSafeStop();
  } else if (strcmp(cmd, "BALANCE ON") == 0) {
    if (!imuData.present) {
      Serial1.println(F("IMU 未偵測到，無法啟用平衡"));
    } else {
      // 每次 B ON 之前先重新校正陀螺零偏：開機嗰次校正可能已經
      // 事隔好耐（行走/郁動咗一輪），零偏會隨溫度/時間漂移，
      // 呢度用最新鮮嘅零偏先啟用 balance，追得更準。
      // 校正嗰 400ms（200 sample × delay(2)）會阻塞 loop()，
      // 期間 servo 通訊/LED/電壓監察會停，機身企定緊嘅話冇影響。
      Serial1.println(F("重新校正陀螺零偏中（機身請保持靜止）..."));
      imuCalibrateGyro();
      Serial1.print(F("[IMU] 校正完成 offsetX=")); Serial1.print(gyroOffsetX, 2);
      Serial1.print(F(" Y=")); Serial1.print(gyroOffsetY, 2);
      Serial1.print(F(" Z=")); Serial1.println(gyroOffsetZ, 2);

      balanceEnabled = true;
      Serial1.println(F("Gyro 平衡已啟用（只喺企定時生效；如要睇log用 BALANCE LOG ON）"));
    }
  } else if (strcmp(cmd, "BALANCE OFF") == 0) {
    balanceEnabled = false;
    Serial1.println(F("Gyro 平衡已關閉"));
  } else if (strcmp(cmd, "BALANCE LOG ON") == 0) {
    balanceLogEnabled = true;
    Serial1.println(F("Balance log 已啟用（每次 IMU tick 印一行）"));
  } else if (strcmp(cmd, "BALANCE LOG OFF") == 0) {
    balanceLogEnabled = false;
    Serial1.println(F("Balance log 已關閉"));
  } else if (strcmp(cmd, "BALANCE PLOT ON") == 0) {
    balancePlotEnabled = true;
    Serial1.println(F("Balance plot 已啟用（Arduino IDE Serial Plotter 格式）"));
  } else if (strcmp(cmd, "BALANCE PLOT OFF") == 0) {
    balancePlotEnabled = false;
    Serial1.println(F("Balance plot 已關閉"));
  } else if (strcmp(cmd, "BALANCE") == 0) {
    Serial1.print(F("BALANCE: "));
    Serial1.println(balanceEnabled ? F("ON") : F("OFF"));
    if (!imuData.present) {
      Serial1.println(F("IMU 未偵測到"));
    } else {
      BalanceOffsets bo;
      computeBalanceOffsets(imuData.pitch, imuData.roll,
                             imuData.gyroZ, imuData.gyroX,
                             IMU_UPDATE_INTERVAL_MS / 1000.0f, bo);
      Serial1.print(F("pitch=")); Serial1.print(imuData.pitch, 2);
      Serial1.print(F("  roll=")); Serial1.println(imuData.roll, 2);
      Serial1.print(F("PID pitch: Kp=")); Serial1.print(balanceGains.pitchKp, 3);
      Serial1.print(F(" Ki=")); Serial1.print(balanceGains.pitchKi, 3);
      Serial1.print(F(" Kd=")); Serial1.print(balanceGains.pitchKd, 3);
      Serial1.print(F(" I_accum=")); Serial1.println(pitchIntegral, 3);
      Serial1.print(F("PID roll:  Kp=")); Serial1.print(balanceGains.rollKp, 3);
      Serial1.print(F(" Ki=")); Serial1.print(balanceGains.rollKi, 3);
      Serial1.print(F(" Kd=")); Serial1.print(balanceGains.rollKd, 3);
      Serial1.print(F(" I_accum=")); Serial1.println(rollIntegral, 3);
      Serial1.print(F("hipLRR=")); Serial1.print(bo.hipLRR);
      Serial1.print(F(" hipLRL=")); Serial1.print(bo.hipLRL);
      Serial1.print(F(" ankleLRR=")); Serial1.print(bo.ankleLRR);
      Serial1.print(F(" ankleLRL=")); Serial1.println(bo.ankleLRL);
      Serial1.print(F("hipFBR=")); Serial1.print(bo.hipFBR);
      Serial1.print(F(" hipFBL=")); Serial1.print(bo.hipFBL);
      Serial1.print(F(" kneeFBR=")); Serial1.print(bo.kneeFBR);
      Serial1.print(F(" kneeFBL=")); Serial1.println(bo.kneeFBL);
      Serial1.print(F("ankleFBR=")); Serial1.print(bo.ankleFBR);
      Serial1.print(F(" ankleFBL=")); Serial1.println(bo.ankleFBL);
      Serial1.print(F("行走中=")); Serial1.println(tableWalkIsWalking() ? F("是(不會套用)") : F("否(會套用)"));
    }
  }
  // ===== table_walk 參數調整命令（取代原本嘅 IK SET）=====
  else if (strcmp(cmd, "WALKSET") == 0 || strncmp(cmd, "WALKSET ", 8) == 0) {
    // 注意：cmd=="WALKSET"（冇參數）時 cmdLen==7，冇 char 喺 index 8 之後，
    // 唔可以用 string literal "" 做 fallback——trimInPlace() 會寫入呢個
    // buffer（memmove+設 null terminator），寫落 string literal 係 undefined
    // behavior，喺呢隻 MCU 上 literal 通常擺喺 flash（.rodata），寫落去會
    // hard fault。用一個真正得 1 byte 嘅 static char buffer 代替。
    static char emptyParamsBuf[1] = { '\0' };
    char *params = cmdLen > 7 ? cmd + 8 : emptyParamsBuf;
    trimInPlace(params);
    char *spacePos = strchr(params, ' ');
    if (spacePos) {
      *spacePos = '\0';
      char *paramName = params;
      char *rest = spacePos + 1;
      int value = atoi(rest);
      if (strcmp(paramName, "LEN") == 0) {
        tableWalkParams.dura_len = constrain(value, 20, 400);
        Serial1.print(F("DuraLen = ")); Serial1.println(tableWalkParams.dura_len);
      } else if (strcmp(paramName, "ROLL") == 0) {
        tableWalkParams.dura_roll = constrain(value, 10, 200);
        Serial1.print(F("DuraRoll = ")); Serial1.println(tableWalkParams.dura_roll);
      } else if (strcmp(paramName, "PITCH") == 0) {
        tableWalkParams.dura_pitch = constrain(value, 100, 1200);
        Serial1.print(F("DuraPitch = ")); Serial1.println(tableWalkParams.dura_pitch);
      } else if (strcmp(paramName, "TICKMS") == 0) {
        Serial1.println(F("TICKMS 需要重新編譯（#define TABLE_WALK_TICK_MS），暫不支援即時調整"));
      } else if (strcmp(paramName, "BALANCE") == 0) {
        // 格式: WALKSET BALANCE <PP|PI|PD|RP|RI|RD|KPP|KPI|KPD> <值>
        // 短寫對照：P=Pitch R=Roll K=Knee，尾字 P/I/D=PID三項。
        //   PP =PITCH_P  PI =PITCH_I  PD =PITCH_D
        //   RP =ROLL_P   RI =ROLL_I   RD =ROLL_D
        //   KPP=KNEE_PITCH_P  KPI=KNEE_PITCH_I  KPD=KNEE_PITCH_D
        // 注：冇 KRP/KRI/KRD（膝屈曲-roll方向）—— roll 平面
        // （HV5-HV13）結構上冇膝可屈，呢組指令已移除（見 LegIK.h
        // solveRollIK() 説明）。
        // 舊長寫法（PITCH_P/PITCH/ROLL_P/ROLL/...）仍然支援，保持向下相容。
        trimInPlace(rest);
        char *argSpacePos = strchr(rest, ' ');
        if (argSpacePos) {
          *argSpacePos = '\0';
          char *axis = rest;
          toUpperInPlace(axis);
          float gainVal = parseSimpleFloat(argSpacePos + 1);

          struct GainEntry { const char *shortCode; const char *longCode; float *field; const char *label; };
          static const GainEntry gainTable[] = {
            { "PP",  "PITCH_P",      &balanceGains.pitchKp,     "PP(PITCH_P)"      },
            { "PI",  "PITCH_I",      &balanceGains.pitchKi,     "PI(PITCH_I)"      },
            { "PD",  "PITCH_D",      &balanceGains.pitchKd,     "PD(PITCH_D)"      },
            { "RP",  "ROLL_P",       &balanceGains.rollKp,      "RP(ROLL_P)"       },
            { "RI",  "ROLL_I",       &balanceGains.rollKi,      "RI(ROLL_I)"       },
            { "RD",  "ROLL_D",       &balanceGains.rollKd,      "RD(ROLL_D)"       },
            { "KPP", "KNEE_PITCH_P", &balanceGains.kneePitchKp, "KPP(KNEE_PITCH_P)"},
            { "KPI", "KNEE_PITCH_I", &balanceGains.kneePitchKi, "KPI(KNEE_PITCH_I)"},
            { "KPD", "KNEE_PITCH_D", &balanceGains.kneePitchKd, "KPD(KNEE_PITCH_D)"},
          };
          // "ROLL" 舊長寫法對應 RP，"PITCH" 舊長寫法對應 PP（保持向下相容，
          // 呢兩個歷史別名唔跟返 shortCode==longCode 前綴嘅一般規律，
          // 要獨立判斷）。
          bool matched = false;
          bool isOldRoll = (strcmp(axis, "ROLL") == 0);
          bool isOldPitch = (strcmp(axis, "PITCH") == 0);
          for (uint8_t i = 0; i < sizeof(gainTable) / sizeof(gainTable[0]); i++) {
            const GainEntry &e = gainTable[i];
            bool isRP = (strcmp(e.shortCode, "RP") == 0);
            bool isPP = (strcmp(e.shortCode, "PP") == 0);
            if (strcmp(axis, e.shortCode) == 0 || strcmp(axis, e.longCode) == 0 ||
                (isOldRoll && isRP) || (isOldPitch && isPP)) {
              *e.field = gainVal;
              Serial1.print(F("BalanceGain ")); Serial1.print(e.label);
              Serial1.print(F(" = ")); Serial1.println(gainVal, 3);
              matched = true;
              break;
            }
          }
          if (!matched) {
            Serial1.println(F("格式錯誤，用法: WALKSET BALANCE <PP|PI|PD|RP|RI|RD|KPP|KPI|KPD> <值>"));
          }
        } else {
          Serial1.print(F("PITCH(PP/PI/PD): Kp=")); Serial1.print(balanceGains.pitchKp, 3);
          Serial1.print(F(" Ki=")); Serial1.print(balanceGains.pitchKi, 3);
          Serial1.print(F(" Kd=")); Serial1.println(balanceGains.pitchKd, 3);
          Serial1.print(F("ROLL(RP/RI/RD):  Kp=")); Serial1.print(balanceGains.rollKp, 3);
          Serial1.print(F(" Ki=")); Serial1.print(balanceGains.rollKi, 3);
          Serial1.print(F(" Kd=")); Serial1.println(balanceGains.rollKd, 3);
          Serial1.print(F("KNEE_PITCH(KPP/KPI/KPD): Kp=")); Serial1.print(balanceGains.kneePitchKp, 3);
          Serial1.print(F(" Ki=")); Serial1.print(balanceGains.kneePitchKi, 3);
          Serial1.print(F(" Kd=")); Serial1.println(balanceGains.kneePitchKd, 3);
        }
      } else if (strcmp(paramName, "STRC") == 0 || strcmp(paramName, "SPD") == 0 ||
                 strcmp(paramName, "CUR") == 0 || strcmp(paramName, "TMP") == 0) {
        // 格式: WALKSET <STRC|SPD|CUR|TMP> <HV|MV> <值>
        // 4個參數 parsing+套用邏輯完全一致，只係 label/上限/set function 唔同，
        // 統一交俾 walkSetServoParam() 處理。
        trimInPlace(rest);
        char *argSpacePos = strchr(rest, ' ');
        if (argSpacePos) {
          *argSpacePos = '\0';
          char *group = rest;
          toUpperInPlace(group);
          if (strcmp(group, "HV") == 0 || strcmp(group, "MV") == 0) {
            int rawVal = atoi(argSpacePos + 1);
            walkSetServoParam(paramName, group, rawVal);
          } else {
            Serial1.print(F("格式錯誤，用法: WALKSET ")); Serial1.print(paramName);
            Serial1.print(F(" HV <值> 或 WALKSET ")); Serial1.print(paramName);
            Serial1.println(F(" MV <值>"));
          }
        } else {
          Serial1.print(F("格式錯誤，用法: WALKSET ")); Serial1.print(paramName);
          Serial1.print(F(" HV <值> 或 WALKSET ")); Serial1.print(paramName);
          Serial1.println(F(" MV <值>"));
        }
      } else {
        Serial1.println(F("未知參數，可用: LEN, ROLL, PITCH, BALANCE, STRC, SPD, CUR, TMP"));
      }
    } else {
      Serial1.println(F("用法: WALKSET <LEN/ROLL/PITCH/STRC/SPD> <數值>"));
      Serial1.print(F("目前: LEN=")); Serial1.print(tableWalkParams.dura_len);
      Serial1.print(F(" ROLL=")); Serial1.print(tableWalkParams.dura_roll);
      Serial1.print(F(" PITCH=")); Serial1.println(tableWalkParams.dura_pitch);
    }
  } else {
    // 嘗試作為標準 ASCII 指令處理
    if (!processASCIICommand(cmd)) {
      Serial1.println(F("未知命令，輸入 H 查看說明"));
    }
  }
}

void showHelp() {
  Serial1.print(F(
    "\n=== 命令列表 (括號內為短寫) ===\n"
    "H, HELP, ?      : 顯示此說明\n"
    "VOLTAGE (V)     : 顯示當前電池電壓\n"
    "PMASTAT (PS)    : 顯示 .pma 封包接收統計 (成功/失敗數)\n"
    "ANGLES (A)      : 一次過顯示全部 25 隻伺服嘅目前角度\n"
    "?binaryID [STRC|SPD|CUR|TMP] : 讀取伺服參數，例如 ?13 或 ?13 STRC\n"
    "IMU (G)         : 顯示 IMU (MPU-6050) 當前姿態 (accel/gyro/pitch/roll)\n"
    "IMU CAL (GC)    : 重新校正陀螺零偏 (機身須靜止)\n"
    "HOME (HM)       : 所有伺服機回到初始位置\n"
    "FREE ALL (F)    : 所有伺服機脫力 (關閉扭力)\n"
    "\n=== 步行與動作指令 ===\n"
    "WALK F/B/L/R  : 開始向前/後/左轉/右轉行走，直到收到 STOP\n"
    "STOP (ST)     : 立即安全停止當前動作並回到 Home Point\n"
    "\n=== Gyro 自動平衡（企定時生效，PID+IK）===\n"
    "BALANCE ON (B ON)   : 啟用陀螺自動平衡（需 IMU 已偵測到）\n"
    "BALANCE OFF (B OFF) : 關閉陀螺自動平衡\n"
    "BALANCE (B)         : 顯示即時 pitch/roll 同各 servo offset 數值\n"
    "BALANCE LOG ON/OFF  : 每次套用時印出每隻 servo 實際送去嘅角度（預設關）\n"
    "BALANCE PLOT ON/OFF : Serial Plotter 格式輸出 pitch/roll/HV5/HV7（畫圖用，預設關）\n"
    "WALKSET BALANCE          : 顯示目前 PID 全部數值\n"
    "WALKSET BALANCE PP <值>  : 前後傾 P (預設0.8)\n"
    "WALKSET BALANCE PI <值>  : 前後傾 I (預設0，由0.01試起)\n"
    "WALKSET BALANCE PD <值>  : 前後傾 D (預設0，由0.02~0.05試起)\n"
    "WALKSET BALANCE RP <值>  : 左右傾 P (預設0.8)\n"
    "WALKSET BALANCE RI <值>  : 左右傾 I (預設0)\n"
    "WALKSET BALANCE RD <值>  : 左右傾 D (預設0)\n"
    "WALKSET BALANCE KPP <值> : 前後傾→膝屈曲 P (預設0，由0.1~0.3試起)\n"
    "WALKSET BALANCE KPI <值> : 前後傾→膝屈曲 I (預設0)\n"
    "WALKSET BALANCE KPD <值> : 前後傾→膝屈曲 D (預設0)\n"
    "\n=== table_walk 步態參數調整 ===\n"
    "WALKSET             : 顯示目前 LEN/ROLL/PITCH 數值\n"
    "WALKSET LEN <值>    : 調整抬腳/伸展幅度 (預設140)\n"
    "WALKSET ROLL <值>   : 調整重心左右搖擺幅度 (預設80)\n"
    "WALKSET PITCH <值>  : 調整步幅前後幅度 (預設600)\n"
    "WALKSET STRC HV <值>: HV 伺服 Stretch 硬度 (1-127, 原廠60)\n"
    "WALKSET STRC MV <值>: MV 伺服 Stretch 硬度 (1-127, 原廠100)\n"
    "WALKSET SPD HV <值> : HV 伺服 Speed 速度 (1-127, 原廠127)\n"
    "WALKSET SPD MV <值> : MV 伺服 Speed 速度 (1-127, 原廠100)\n"
    "WALKSET CUR HV <值> : HV 伺服 電流上限 (1-63, 原廠20)\n"
    "WALKSET CUR MV <值> : MV 伺服 電流上限 (1-63, 原廠40)\n"
    "WALKSET TMP HV <值> : HV 伺服 溫度上限 (1-127, 原廠80)\n"
    "WALKSET TMP MV <值> : MV 伺服 溫度上限 (1-127, 原廠40)\n"
  ));
}
// =========================================================
// ===== setup() / loop() =====

// =========================================================
// ===== setup() =====
void setup() {
  initLED();
  setLEDRed();

  Serial1.begin(115200);
  delay(100);

  initVoltageCheck();

  // 立即初始化 servo 並發送 home 指令；servo 收到指令後自行按設定速度移動，
  // 唔需要 MCU 等待，之後嘅 LED 動畫等可與 servo 移動同步進行
  Serial1.print(F(
    "\n========================================\n"
    "プリメイドAI - table_walk 步行\n"
    "========================================\n"
  ));

  // ===== table_walk 步行系統初始化 =====
  tw_walker_init(&tableWalker);
  tw_params_init(&tableWalkParams);
  lastWalkTickMs = millis();

  Serial1.print(F("\n初始化伺服..."));
  initServos();
  Serial1.println(F("完成"));

  setupServoParam("速度", DEFAULT_SPEED_HV, DEFAULT_SPEED_MV, safeSetSpd, false);
  setupServoParam("Stretch", DEFAULT_STRETCH_HV, DEFAULT_STRETCH_MV, safeSetStrc, false);
  setupServoParam("電流上限", DEFAULT_CUR_HV, DEFAULT_CUR_MV, safeSetCur, true);
  setupServoParam("溫度上限", DEFAULT_TMP_HV, DEFAULT_TMP_MV, safeSetTmp, true);

  Serial1.println(F("\n發送 home 指令（servo 移動中）..."));
  moveAllServosToHome();
  Serial1.println(F("Servo home 指令已發送，正在移動中..."));

  voltageData.currentVoltage = readBatteryVoltage();
  Serial1.print(F("\n當前電壓: "));
  Serial1.print(voltageData.currentVoltage);
  Serial1.println(F("V"));

  // ===== IMU (MPU-6050) 初始化 =====
  Serial1.print(F("\n初始化 IMU (MPU-6050)..."));
  imuOk = imuInit();
  if (imuOk) {
    Serial1.println(F("完成（陀螺零偏將於開機 4 秒後、servo 企定時自動校正）"));
  } else {
    Serial1.println(F("未偵測到（WHO_AM_I 驗證失敗），跳過 IMU 功能"));
  }

  // LED 交返俾 updateActivityLED() 根據 lastActivityMs 自動判斷：
  // servo 郁緊顯示綠色（忙碌），冇再郁就喺 300ms 內跳返青色（待機）。
  updateActivityLED();

  Serial1.println(F("\n系統就緒"));
  Serial1.println(F("========================================\n"));
}
// ===== loop() =====
void loop() {
  // Serial1 嘅 ASCII/binary 接收全部交俾 pmaReceiveUpdate()
  // (PmaProtocol.h) 一份邏輯處理，唔喺呢度另外讀 Serial1，
  // 否則會同 pmaReceiveUpdate() 爭緊同一個 buffer。

  checkVoltage();
  updateActivityLED();

  // table_walk 用固定 tick 驅動（唔似原本 WalkGenerator 咁連續時間積分）
  unsigned long nowTick = millis();
  if (nowTick - lastWalkTickMs >= TABLE_WALK_TICK_MS) {
    lastWalkTickMs = nowTick;
    table_walk_update(&tableWalker, &tableWalkParams);
    sendTableWalkAngles();
  }

  // .pma / ICS binary 封包接收（非阻塞，逐 byte 儲，收齊即行）
  pmaReceiveUpdate();

  // Flash-based motion playback（跟原裝協議：host 上傳完動作去
  // Flash 之後淨係送一次 StartMotion，之後 MCU 自己喺呢度逐個
  // packet 由 Flash 讀出嚟播，唔再靠 host 逐個送）
  motionPlaybackUpdate();

  // IMU 開機自動校正（非阻塞）：等開機滿 IMU_AUTO_CAL_DELAY_MS（servo
  // 應已行到 home 企定）先做一次，唔用 delay() 卡住 loop()，LED/電壓/
  // 伺服通信全部正常運作，唔會再出現「LED 卡住幾秒」嘅問題。
  // 校正嗰刻閃白燈 + 清晰 log，方便唔開 Serial Monitor 都睇到發生咗。
  if (imuOk && !imuGyroCalibrated && millis() >= IMU_AUTO_CAL_DELAY_MS) {
    imuGyroCalibrated = true;  // 即刻標記，防止呢段重複觸發
    ledManualOverride = true;
    setLEDWhite();
    Serial1.print(F("\n[IMU] 開機自動校正陀螺零偏（機身請保持靜止）... t="));
    Serial1.print(millis());
    Serial1.println(F("ms"));
    imuCalibrateGyro();
    Serial1.print(F("[IMU] 校正完成 offsetX=")); Serial1.print(gyroOffsetX, 2);
    Serial1.print(F(" Y=")); Serial1.print(gyroOffsetY, 2);
    Serial1.print(F(" Z=")); Serial1.println(gyroOffsetZ, 2);
    setLEDOff();
    ledManualOverride = false;  // 交返俾 updateActivityLED() 自動判斷
  }

  // IMU 固定周期更新（純感測，唔影響 table_walk/伺服控制）
  unsigned long nowImuTick = millis();
  if (imuData.present && nowImuTick - lastImuUpdateMs >= IMU_UPDATE_INTERVAL_MS) {
    lastImuUpdateMs = nowImuTick;
    imuUpdate();

    // ===== Gyro 自動平衡 =====
    // 疊加基準用 servo->currentTunePos（table_walk 每個 walk tick
    // 更新一次嘅「純步態目標」），等 balance offset 疊喺步態之上；
    // 企定時 currentTunePos 同 homePosition 相等。
    //
    // ⚠ balance 送出（基準+offset）之後刻意唔寫返 currentTunePos：
    // computeBalanceOffsets() 嘅 PID 輸出係「相對呢個乾淨基準嘅絕對
    // 修正量」，唔係增量。如果寫返落 currentTunePos，下一個 IMU
    // tick 嘅 offset 會疊落上次殘留之上，不斷累積、唔會收斂。
    // currentTunePos 淨係俾 table_walk 更新，balance 每次都由呢個
    // 乾淨基準重新算。
    if (balanceEnabled) {
      BalanceOffsets bo;
      float imuDt = IMU_UPDATE_INTERVAL_MS / 1000.0f;  // 50Hz tick，同 imuUpdate() 週期一致
      computeBalanceOffsets(imuData.pitch, imuData.roll,
                             imuData.gyroZ, imuData.gyroX,
                             imuDt, bo);

      // 逐隻 servo：基準取自 currentTunePos，疊加 offset、clamp、
      // 送出，但唔寫返 currentTunePos。
      struct BalHVEntry {
        ServoInfo *servo;
        int offset;
        int target;      // apply 之後填入，俾下面 log/plot 用
        const char *label;
      };
      BalHVEntry hvList[10] = {
        { findServoByBinaryID(5),  bo.hipLRR,   0, "HV5="  },
        { findServoByBinaryID(6),  bo.hipLRL,   0, "HV6="  },
        { findServoByBinaryID(7),  bo.hipFBR,   0, "HV7="  },
        { findServoByBinaryID(8),  bo.hipFBL,   0, "HV8="  },
        { findServoByBinaryID(9),  bo.kneeFBR,  0, "HV9="  },
        { findServoByBinaryID(10), bo.kneeFBL,  0, "HV10=" },
        { findServoByBinaryID(11), bo.ankleFBR, 0, "HV11=" },
        { findServoByBinaryID(12), bo.ankleFBL, 0, "HV12=" },
        { findServoByBinaryID(13), bo.ankleLRR, 0, "HV13=" },
        { findServoByBinaryID(14), bo.ankleLRL, 0, "HV14=" },
      };
      for (uint8_t i = 0; i < 10; i++) {
        BalHVEntry &e = hvList[i];
        if (!e.servo) continue;
        int base = e.servo->currentTunePos;
        int t = constrain(base + e.offset, (int)e.servo->minAngle, (int)e.servo->maxAngle);
        safeSetPos(e.servo, (uint16_t)t);
        e.target = t;
      }
      if (balanceLogEnabled) {
        Serial1.print(F("[BAL] p=")); Serial1.print(imuData.pitch, 1);
        Serial1.print(F(" r=")); Serial1.print(imuData.roll, 1);
        for (uint8_t i = 0; i < 10; i++) {
          if (!hvList[i].servo) continue;
          Serial1.print(' ');
          Serial1.print(hvList[i].label);
          Serial1.print(hvList[i].target);
        }
        Serial1.println();
      }

      // Serial Plotter 格式輸出（"BALANCE PLOT ON"）：Arduino IDE
      // Serial Plotter 認 "label:數值" 逗號分隔格式，同 balanceLogEnabled
      // 嗰種人手可讀格式獨立。印 pitch/roll + HV5/HV7 代表 roll/pitch
      // 兩個方向嘅實際郁動幅度，方便睇收斂定震盪。
      if (balancePlotEnabled) {
        Serial1.print(F("pitch:")); Serial1.print(imuData.pitch, 2);
        Serial1.print(F(",roll:")); Serial1.print(imuData.roll, 2);
        if (hvList[0].servo) { Serial1.print(F(",HV5:")); Serial1.print(hvList[0].target); }
        if (hvList[2].servo) { Serial1.print(F(",HV7:")); Serial1.print(hvList[2].target); }
        Serial1.println();
      }
    } else {
      // 行走緊，或者 balance 冇 enable：清 I term，防止呢段時間內
      // 累積落嘅（同企定修正唔相關嘅）誤差喺下次企定套用時爆疊落去。
      resetBalanceIntegral();
    }
  }
}