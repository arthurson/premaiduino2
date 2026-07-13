

#include <Arduino.h>
#include <math.h>  
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

// ===== 建立兩個通訊物件 =====
IcsHardSerialClass icsHV(&Serial2, EN_HV_PIN, 1250000, 10);
IcsHardSerialClass icsMV(&Serial3, EN_MV_PIN, 1250000, 10);

// ===== 25軸伺服資訊 =====
// struct 定義、servoList[] 資料表、同伺服參數預設值（DEFAULT_SPEED/
// STRETCH/CUR/TMP）都搬咗去 ServoConfig.h（純資料，冇邏輯）。
// 依賴 icsHV/icsMV，所以 include 要擺喺呢兩者之後。
#include "ServoConfig.h"

// ===== 系統狀態 =====
String inputBuffer = "";

// 最近一次有 servo 動作嘅時間戳（safeSetPos() 更新）。
// 俾 loop() 用嚟判斷而家係咪「有動作進行緊」，決定 LED 顯示忙碌
// 提示定係待機呼吸燈。table_walk/.pma player 都經 safeSetPos()，
// 唔使逐個功能加 LED code。
unsigned long lastActivityMs = 0;

// FREE ALL 之後進入「脫力模式」，LED 顯示紫色，直到下次有 servo 動作
// （safeSetPos() 被 call）先自動解除，交返俾 updateActivityLED() 判斷。
bool isFreeMode = false;

// ===== IMU (MPU-6050) 狀態 =====
// 純感測模組，唔接入任何控制邏輯（table_walk 完全唔會讀呢啲值）。
// 加呢個純粹為咗俾 IMU 指令查姿態，方便實機驗證。
#define IMU_UPDATE_INTERVAL_MS 20  // 50Hz，夠 IMU 指令即時睇數用
unsigned long lastImuUpdateMs = 0;

bool imuOk = false;               // imuInit() 結果，setup() 設定
bool imuGyroCalibrated = false;   // 開機自動校正係咪已經做咗（做過一次就唔再做）
#define IMU_AUTO_CAL_DELAY_MS 4000  // 開機後等 servo 行到 home 先校正，非阻塞
// ===== 函式原型 =====
void initServos();
void moveAllServosToHome();
void processCommand(String cmd);
void showHelp();

bool processASCIICommand(String cmd);
bool processFreeCommand(String cmd);
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
    Serial1.print(F("setSpd 範圍錯誤 (1-127): "));
    Serial1.print(servo->name);
    Serial1.print(F(" spd="));
    Serial1.println(spd);
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
    Serial1.print(F("setStrc 範圍錯誤 (1-127): "));
    Serial1.print(servo->name);
    Serial1.print(F(" strc="));
    Serial1.println(strc);
    return false;
  }
  int result = servo->icsPort->setStrc(servo->servoID, strc);
  return (result != ICS_FALSE);
}

bool safeSetCur(ServoInfo *servo, uint8_t curlim) {
  if (!servo) return false;
  if (curlim < 1 || curlim > 63) {
    Serial1.print(F("setCur 範圍錯誤 (1-63): "));
    Serial1.print(servo->name);
    Serial1.print(F(" curlim="));
    Serial1.println(curlim);
    return false;
  }
  int result = servo->icsPort->setCur(servo->servoID, curlim);
  return (result != ICS_FALSE);
}

bool safeSetTmp(ServoInfo *servo, uint8_t tmplim) {
  if (!servo) return false;
  if (tmplim < 1 || tmplim > 127) {
    Serial1.print(F("setTmp 範圍錯誤 (1-127): "));
    Serial1.print(servo->name);
    Serial1.print(F(" tmplim="));
    Serial1.println(tmplim);
    return false;
  }
  int result = servo->icsPort->setTmp(servo->servoID, tmplim);
  return (result != ICS_FALSE);
}

// 合併原本 getStrc/getSpd/getCur/getTmp 4份幾乎一樣嘅樣板；
// 原本嘅 setStrc/setSpd/setCur/setTmp 4個 wrapper 完全冇被 call 過
// （實際 set 一律經 safeSetXxx()），已一併移除。
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
// 完整實作（即時 streaming 解碼 + STM32 Flash motion storage +
// pmaReceiveUpdate() 接收狀態機）搬咗去 PmaProtocol.h。
// 依賴 findServoByPmaId/applyHomeOffset（above）、safeSetXxx、
// isFreeMode、tableWalker/tableWalkIsWalking，所以 include 要擺喺
// 呢啲之後；moveAllServosToHome() 用 forward declaration 就夠。
#include "PmaProtocol.h"



// =========================================================
// ===== Home / 脫力 =====

// =========================================================
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
bool processFreeCommand(String cmd) {
  cmd.trim();

  if (!cmd.startsWith("FREE ")) return false;

  String params = cmd.substring(5);
  params.trim();

  if (params == "ALL") {
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

  int spacePos = params.indexOf(' ');

  if (spacePos <= 0) return false;

  String group = params.substring(0, spacePos);
  group.toUpperCase();
  int id = params.substring(spacePos + 1).toInt();

  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    bool groupMatch = (group == "HV" && servoList[i].isHV) || (group == "MV" && !servoList[i].isHV);

    if (groupMatch && servoList[i].binaryID == id) {
      servoList[i].icsPort->setFree(servoList[i].servoID);
      Serial1.print(F("脫力 "));
      Serial1.print(group);
      Serial1.print(F(" ID "));
      Serial1.println(id);
      return true;
    }
  }

  Serial1.println(F("找不到對應的伺服"));
  return true;
}

// =========================================================
// ===== 指令處理（ASCII command dispatch） =====

// =========================================================
// ===== 處理ASCII指令 =====
bool processASCIICommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("FREE ")) return processFreeCommand(cmd);

  if (cmd.startsWith("?") && cmd.length() > 1 && isDigit(cmd.charAt(1))) {
    // 格式: ?<binaryID> [STRC|SPD|CUR|TMP]，例如 ?13 或 ?13 STRC
    // binaryID 本身已經唯一分辨 HV(1-14)/MV(21-31)，唔需要再指定群組。
    int spacePos = cmd.indexOf(' ');
    int binaryId = cmd.substring(1, (spacePos > 0 ? spacePos : cmd.length())).toInt();
    String field = (spacePos > 0) ? cmd.substring(spacePos + 1) : "";
    field.trim();

    ServoInfo *servo = findServoByBinaryID(binaryId);

    if (servo != NULL) {
      const char *groupLabel = servo->isHV ? "HV" : "MV";

      if (!servo->enabled) {
        Serial1.print(servo->name);
        Serial1.println(F(" 未駁線 (enabled=false)，略過"));
        return true;
      }

      if (field == "STRC" || field == "SPD" || field == "CUR" || field == "TMP") {
        int result = ICS_FALSE;
        const char *label = "";
        if (field == "STRC") { result = getServoParam(binaryId, PARAM_STRC); label = "Stretch"; }
        else if (field == "SPD")  { result = getServoParam(binaryId, PARAM_SPD);  label = "Speed"; }
        else if (field == "CUR")  { result = getServoParam(binaryId, PARAM_CUR);  label = "電流上限"; }
        else if (field == "TMP")  { result = getServoParam(binaryId, PARAM_TMP);  label = "溫度上限"; }

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
      Serial1.print(pos != ICS_FALSE ? String(pos) : String("讀取失敗"));
      Serial1.print(F(" Stretch="));
      Serial1.print(strc != ICS_FALSE ? String(strc) : String("讀取失敗"));
      Serial1.print(F(" Speed="));
      Serial1.print(spd != ICS_FALSE ? String(spd) : String("讀取失敗"));
      Serial1.print(F(" 電流上限="));
      Serial1.print(cur != ICS_FALSE ? String(cur) : String("讀取失敗"));
      Serial1.print(F(" 溫度上限="));
      Serial1.println(tmp != ICS_FALSE ? String(tmp) : String("讀取失敗"));
      return true;
    }
    return true;
  }

  return false;
}
// ===== setup() 用：批次設定全部伺服某一項參數，並印出統一格式嘅 log =====
// 取代原本 SPD/STRC/CUR/TMP 4段幾乎一樣嘅 print+for迴圈；showFailures=true
// 時（CUR/TMP 原本行為）會逐個列印失敗嘅伺服名，SPD/STRC 原本冇呢個提示。
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
// 4個參數嘅 range/label/對應 set function 唔同，其餘 parsing 同套用
// 邏輯完全一致，統一喺呢度處理，processCommand() 淨係負責讀指令、
// 格式檢查交呢個 function。
static void walkSetServoParam(const String &paramName, const String &group, int rawVal) {
  bool wantHV = (group == "HV");
  int value;
  const char *label;
  ServoSetFn setFn;

  if (paramName == "STRC") {
    value = constrain(rawVal, 1, 127); label = "Stretch"; setFn = safeSetStrc;
  } else if (paramName == "SPD") {
    value = constrain(rawVal, 1, 127); label = "Speed"; setFn = safeSetSpd;
  } else if (paramName == "CUR") {
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

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  // 短寫 alias：轉返做完整指令，之後行返原本邏輯，唔使逐個分支改
  if (cmd == "V") cmd = "VOLTAGE";
  else if (cmd == "G") cmd = "IMU";
  else if (cmd == "GC") cmd = "IMU CAL";
  else if (cmd == "HM") cmd = "HOME";
  else if (cmd == "F") cmd = "FREE ALL";
  else if (cmd == "ST") cmd = "STOP";
  else if (cmd == "PS") cmd = "PMASTAT";
  else if (cmd == "A") cmd = "ANGLES";

  if (cmd == "H" || cmd == "HELP" || cmd == "?") {
    showHelp();
  } else if (cmd == "VOLTAGE") {
    Serial1.print(F("當前電壓: "));
    Serial1.print(voltageData.currentVoltage);
    Serial1.println(F("V"));
  } else if (cmd == "PMASTAT") {
    Serial1.print(F(".pma 封包統計: 成功="));
    Serial1.print(pmaPacketOkCount);
    Serial1.print(F(" 失敗="));
    Serial1.println(pmaPacketFailCount);
  } else if (cmd == "ANGLES") {
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
  } else if (cmd == "IMU") {
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
  } else if (cmd == "IMU CAL") {
    if (!imuData.present) {
      Serial1.println(F("IMU 未偵測到，無法校正"));
    } else {
      Serial1.print(F("校正陀螺零偏（機身請保持靜止）..."));
      imuCalibrateGyro();
      Serial1.println(F("完成"));
    }
  } else if (cmd == "HOME") {
    tableWalkSafeStop();
    moveAllServosToHome();
  } else if (cmd == "FREE ALL") {
    tableWalkSafeStop();
    processFreeCommand("FREE ALL");
  }
  else if (cmd.startsWith("WALK ")) {
    String params = cmd.substring(5);
    params.trim();

    char dir = params.charAt(0);

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
  } else if (cmd == "STOP") {
    tableWalkSafeStop();
  }
  // ===== table_walk 參數調整命令（取代原本嘅 IK SET）=====
  else if (cmd == "WALKSET" || cmd.startsWith("WALKSET ")) {
    String params = cmd.length() > 7 ? cmd.substring(8) : "";
    params.trim();
    int spacePos = params.indexOf(' ');
    if (spacePos > 0) {
      String paramName = params.substring(0, spacePos);
      int value = params.substring(spacePos + 1).toInt();
      if (paramName == "LEN") {
        tableWalkParams.dura_len = constrain(value, 20, 400);
        Serial1.print(F("DuraLen = ")); Serial1.println(tableWalkParams.dura_len);
      } else if (paramName == "ROLL") {
        tableWalkParams.dura_roll = constrain(value, 10, 200);
        Serial1.print(F("DuraRoll = ")); Serial1.println(tableWalkParams.dura_roll);
      } else if (paramName == "PITCH") {
        tableWalkParams.dura_pitch = constrain(value, 100, 1200);
        Serial1.print(F("DuraPitch = ")); Serial1.println(tableWalkParams.dura_pitch);
      } else if (paramName == "TICKMS") {
        Serial1.println(F("TICKMS 需要重新編譯（#define TABLE_WALK_TICK_MS），暫不支援即時調整"));
      } else if (paramName == "STRC" || paramName == "SPD" ||
                 paramName == "CUR" || paramName == "TMP") {
        // 格式: WALKSET <STRC|SPD|CUR|TMP> <HV|MV> <值>
        // 4個參數 parsing+套用邏輯完全一致，只係 label/上限/set function 唔同，
        // 統一交俾 walkSetServoParam() 處理。
        String args = params.substring(spacePos + 1);
        args.trim();
        int argSpacePos = args.indexOf(' ');
        if (argSpacePos > 0) {
          String group = args.substring(0, argSpacePos);
          group.toUpperCase();
          if (group == "HV" || group == "MV") {
            int rawVal = args.substring(argSpacePos + 1).toInt();
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
        Serial1.println(F("未知參數，可用: LEN, ROLL, PITCH, STRC, SPD, CUR, TMP"));
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
  Serial1.println(F("\n=== 命令列表 (括號內為短寫) ==="));
  Serial1.println(F("H, HELP, ?      : 顯示此說明"));
  Serial1.println(F("VOLTAGE (V)     : 顯示當前電池電壓"));
  Serial1.println(F("PMASTAT (PS)    : 顯示 .pma 封包接收統計 (成功/失敗數)"));
  Serial1.println(F("ANGLES (A)      : 一次過顯示全部 25 隻伺服嘅目前角度"));
  Serial1.println(F("?binaryID [STRC|SPD|CUR|TMP] : 讀取伺服參數，例如 ?13 或 ?13 STRC"));
  Serial1.println(F("IMU (G)         : 顯示 IMU (MPU-6050) 當前姿態 (accel/gyro/pitch/roll)"));
  Serial1.println(F("IMU CAL (GC)    : 重新校正陀螺零偏 (機身須靜止)"));
  Serial1.println(F("HOME (HM)       : 所有伺服機回到初始位置"));
  Serial1.println(F("FREE ALL (F)    : 所有伺服機脫力 (關閉扭力)"));
  Serial1.println(F("\n=== 步行與動作指令 ==="));
  Serial1.println(F("WALK F/B/L/R  : 開始向前/後/左轉/右轉行走，直到收到 STOP"));
  Serial1.println(F("STOP (ST)     : 立即安全停止當前動作並回到 Home Point"));
  Serial1.println(F("\n=== table_walk 步態參數調整 ==="));
  Serial1.println(F("WALKSET             : 顯示目前 LEN/ROLL/PITCH 數值"));
  Serial1.println(F("WALKSET LEN <值>    : 調整抬腳/伸展幅度 (預設140)"));
  Serial1.println(F("WALKSET ROLL <值>   : 調整重心左右搖擺幅度 (預設80)"));
  Serial1.println(F("WALKSET PITCH <值>  : 調整步幅前後幅度 (預設600)"));
  Serial1.println(F("WALKSET STRC HV <值>: HV 伺服 Stretch 硬度 (1-127, 原廠60)"));
  Serial1.println(F("WALKSET STRC MV <值>: MV 伺服 Stretch 硬度 (1-127, 原廠100)"));
  Serial1.println(F("WALKSET SPD HV <值> : HV 伺服 Speed 速度 (1-127, 原廠127)"));
  Serial1.println(F("WALKSET SPD MV <值> : MV 伺服 Speed 速度 (1-127, 原廠100)"));
  Serial1.println(F("WALKSET CUR HV <值> : HV 伺服 電流上限 (1-63, 原廠20)"));
  Serial1.println(F("WALKSET CUR MV <值> : MV 伺服 電流上限 (1-63, 原廠40)"));
  Serial1.println(F("WALKSET TMP HV <值> : HV 伺服 溫度上限 (1-127, 原廠80)"));
  Serial1.println(F("WALKSET TMP MV <值> : MV 伺服 溫度上限 (1-127, 原廠40)"));
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
  Serial1.println(F("\n========================================"));
  Serial1.println(F("プリメイドAI - table_walk 步行"));
  Serial1.println(F("========================================"));

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
  // imuInit() 本身唔需要機身靜止，可以隨時做。
  // imuCalibrateGyro() 就要機身真係企定咗先準 —— moveAllServosToHome()
  // 只係發送指令，伺服係自己異步移動過去，MCU 唔會等佢哋到位；
  // 但唔可以用 delay() 等，會卡住成個 setup()（連 LED 狀態都會卡住）。
  // 改為非阻塞：呢度淨係 init，實際校正延後到 loop() 用時間戳判斷。
  Serial1.print(F("\n初始化 IMU (MPU-6050)..."));
  imuOk = imuInit();
  if (imuOk) {
    Serial1.println(F("完成（陀螺零偏將於開機 4 秒後、servo 企定時自動校正）"));
  } else {
    Serial1.println(F("未偵測到（WHO_AM_I 驗證失敗），跳過 IMU 功能"));
  }

  // LED 交返俾 updateActivityLED() 根據 lastActivityMs 自動判斷：
  // servo 郁緊（moveAllServosToHome() 啱啱觸發咗 safeSetPos()）就顯示
  // 綠色（忙碌），實際到位、冇再郁就自動喺 300ms 內跳返做青色（待機）—
  // 唔再用硬編碼嘅幾秒動畫,LED 會真正反映 servo 實際狀態。
  updateActivityLED();

  Serial1.println(F("\n系統就緒"));
  Serial1.println(F("========================================\n"));
}
// ===== loop() =====
void loop() {
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
  }
}