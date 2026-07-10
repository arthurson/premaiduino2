
// STM32duino 嘅 HardwareSerial RX buffer 預設淨係 64 byte，但 Unity/PC
// 端 continuousMode 送嘅 0x18 多軸角度封包（25隻servo全部）係 80 byte，
// 單一個封包已經超過預設 buffer 容量！封包送到一半 buffer 爆滿，
// 之後嘅 byte 會被硬件丟棄，令 parser 讀到殘缺封包，checksum 永遠
// 對唔上——呢個先係 continuousMode 完全冇反應嘅根本原因。
//
// 注意：唔可以淨係喺呢個 .ino 度寫 #define SERIAL_RX_BUFFER_SIZE。
// Arduino IDE 會將 .ino 轉做 .cpp 編譯，但 HardwareSerial.cpp（STM32
// core 本身）係獨立編譯單元，唔會睇到呢度嘅 #define。要生效必須喺
// 同一個 sketch 資料夾新增一個名叫 hal_conf_extra.h 嘅檔案，STM32
// core 會自動 #include 佢去覆蓋核心庫嘅編譯設定。呢個 .ino 已經另外
// 建立咗 hal_conf_extra.h，內容已經加大 buffer 到 256 byte。

#include <Arduino.h>
#include <math.h>  // table_walk.c 需要 fabsf()

// PA3/PB11 為 NC，冇實體 RX 線，ICS bus 係單線半雙工，必須用
// half-duplex 單 pin constructor（唔可以用普通雙 pin full-duplex）
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
IcsHardSerialClass icsHV(&Serial2, EN_HV_PIN, 1250000, 1);
IcsHardSerialClass icsMV(&Serial3, EN_MV_PIN, 1250000, 1);

// ===== 速度定義 =====
#define DEFAULT_SPEED_HV 127  // HV 伺服速度 (原廠設定值)
#define DEFAULT_SPEED_MV 100  // MV 伺服速度 (原廠設定值)
#define DEFAULT_STRETCH_HV 60   // HV 伺服 Stretch 硬度 (原廠設定值)
#define DEFAULT_STRETCH_MV 100  // MV 伺服 Stretch 硬度 (原廠設定值)
#define DEFAULT_CUR_HV 20    // HV 伺服電流制限值 (原廠設定值)
#define DEFAULT_CUR_MV 40    // MV 伺服電流制限值 (原廠設定值)
#define DEFAULT_TMP_HV 80    // HV 伺服溫度制限值 (原廠設定值)
#define DEFAULT_TMP_MV 40    // MV 伺服溫度制限值 (原廠設定值)

// ===== 25軸伺服資訊 =====
struct ServoInfo {
  uint8_t binaryID;
  uint8_t servoID;
  uint16_t homePosition;
  uint16_t currentTunePos;
  uint8_t currentSpeed;
  IcsHardSerialClass *icsPort;
  const char *name;
  uint16_t minAngle;
  uint16_t maxAngle;
  bool isHV;
  bool enabled;  // false = 呢隻伺服未駁線/未安裝，safeSetPos/safeSetSpd 會跳過
  uint16_t baselineCenter;  // 官方協議/Unity 呢隻 servo 嘅「中立值」，用嚟將
                            // .pma/Unity 送嚟嘅絕對角度轉做 offset：
                            // offset = rawAngle - baselineCenter,
                            // target = homePosition + offset。
                            // 大部分 servo 官方中立值係 7500；但肩ロールR/L
                            // （右/左肩側擺）官方本身就唔係 7500——Excel
                            // 解析資料同 Unity PreMaidServo.cs 都證實呢兩隻
                            // 嘅官方 home 值係 9375~9500（右）/5625~5500
                            // （左），因個體差異而定，用返 Unity 嘅預設值。
};

ServoInfo servoList[] = {
  // ===== HV群 (下半身) - binaryID 1-14 =====
  { 1, 1, 10200, 10200, DEFAULT_SPEED_HV, &icsHV, "右肩前後", 4300, 11500, true, true, 7500 },
  { 2, 2, 4700, 4700, DEFAULT_SPEED_HV, &icsHV, "左肩前後", 3500, 10700, true, true, 7500 },
  { 3, 3, 7780, 7780, DEFAULT_SPEED_HV, &icsHV, "右髖轉向", 6530, 9030, true, true, 7500 },
  { 4, 4, 7500, 7500, DEFAULT_SPEED_HV, &icsHV, "左髖轉向", 6250, 8750, true, true, 7500 },
  { 5, 5, 7400, 7400, DEFAULT_SPEED_HV, &icsHV, "右髖側擺", 6700, 8300, true, true, 7500 },
  { 6, 6, 7600, 7600, DEFAULT_SPEED_HV, &icsHV, "左髖側擺", 6700, 8300, true, true, 7500 },
  { 7, 7, 7500, 7500, DEFAULT_SPEED_HV, &icsHV, "右髖前後", 4700, 10200, true, true, 7500 },
  { 8, 8, 7500, 7500, DEFAULT_SPEED_HV, &icsHV, "左髖前後", 4700, 10200, true, true, 7500 },
  { 9, 9, 7500, 7500, DEFAULT_SPEED_HV, &icsHV, "右膝屈伸", 3950, 7600, true, true, 7500 },
  { 10, 10, 7500, 7500, DEFAULT_SPEED_HV, &icsHV, "左膝屈伸", 7400, 11050, true, true, 7500 },
  { 11, 11, 7500, 7500, DEFAULT_SPEED_HV, &icsHV, "右踝前後", 5700, 8300, true, true, 7500 },
  { 12, 12, 7550, 7550, DEFAULT_SPEED_HV, &icsHV, "左踝前後", 6750, 9350, true, true, 7500 },
  { 13, 13, 7825, 7825, DEFAULT_SPEED_HV, &icsHV, "右踝側擺", 6800, 9150, true, true, 7500 },
  { 14, 14, 7450, 7450, DEFAULT_SPEED_HV, &icsHV, "左踝側擺", 6200, 8450, true, true, 7500 },

  // ===== MV群 (上半身) - binaryID 21-31 =====
  { 21, 1, 7500, 7500, DEFAULT_SPEED_MV, &icsMV, "頭部前後", 7200, 8400, false, true, 7500 },
  { 22, 2, 7500, 7500, DEFAULT_SPEED_MV, &icsMV, "頭部轉向", 5000, 10000, false, true, 7500 },
  { 23, 3, 7500, 7500, DEFAULT_SPEED_MV, &icsMV, "頭部側傾", 6900, 8100, false, false, 7500 },  // 未駁線
  { 24, 4, 9800, 9800, DEFAULT_SPEED_MV, &icsMV, "右肩側擺", 7450, 10350, false, true, 9500 },  // 官方中立值非7500
  { 25, 5, 5200, 5200, DEFAULT_SPEED_MV, &icsMV, "左肩側擺", 4550, 7550, false, true, 5500 },   // 官方中立值非7500
  { 26, 6, 7500, 7500, DEFAULT_SPEED_MV, &icsMV, "右臂轉向", 4000, 11000, false, true, 7500 },
  { 27, 7, 7500, 7500, DEFAULT_SPEED_MV, &icsMV, "左臂轉向", 4000, 11000, false, true, 7500 },
  { 28, 8, 7500, 7500, DEFAULT_SPEED_MV, &icsMV, "右肘屈伸", 7100, 11000, false, true, 7500 },
  { 29, 9, 7500, 7500, DEFAULT_SPEED_MV, &icsMV, "左肘屈伸", 4000, 7900, false, true, 7500 },
  { 30, 10, 5000, 5000, DEFAULT_SPEED_MV, &icsMV, "右手轉向", 3500, 11500, false, true, 7500 },
  { 31, 11, 10000, 10000, DEFAULT_SPEED_MV, &icsMV, "左手轉向", 3500, 11500, false, true, 7500 }
};

#define TOTAL_SERVO_NUM (sizeof(servoList) / sizeof(servoList[0]))

// ===== 系統狀態 =====
String inputBuffer = "";

// 最近一次有 servo 動作嘅時間戳（safeSetPos() 更新）。
// 俾 loop() 用嚟判斷而家係咪「有動作進行緊」，決定 LED 顯示忙碌
// 提示定係待機呼吸燈。未來加 .pma player 都會經 safeSetPos()，
// 唔使再逐個功能加 LED code。
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
void safeStop();
void processCommand(String cmd);
void showHelp();

bool processASCIICommand(String cmd);
bool processMultiCommand(String cmd);
bool processFreeCommand(String cmd);
ServoInfo *findServoByBinaryID(uint8_t binaryID);
bool safeSetPos(ServoInfo *servo, uint16_t pos);
bool safeSetSpd(ServoInfo *servo, uint8_t spd);
bool safeSetStrc(ServoInfo *servo, uint8_t strc);
bool safeSetCur(ServoInfo *servo, uint8_t curlim);
bool safeSetTmp(ServoInfo *servo, uint8_t tmplim);
uint16_t applyHomeOffset(ServoInfo *servo, int rawAngle, int baselineCenter);

int setStrc(byte id, unsigned int strc);
int setSpd(byte id, unsigned int spd);
int setCur(byte id, unsigned int curlim);
int setTmp(byte id, unsigned int tmplim);

int getStrc(byte id);
int getSpd(byte id);
int getCur(byte id);
int getTmp(byte id);

// ===== .pma / ICS binary 封包接收 =====
void pmaReceiveUpdate();


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
// ===== .pma / ICS binary 封包播放器 =====
// =========================================================
// 資料來源：藍牙（Serial1）即時串流，唔喺 MCU 度存住成隻 .pma 檔案。
// 對面（PC/App/自家 script）逐個 packet send 過嚟，MCU 收齊一個
// packet 就即刻執行、然後準備接收下一個——「send 一次做一次」。
//
// 封包格式（同 Unity/官方 App 用嘅協議一致，經社群 PMAAnalyze.rb 驗證）：
//   [LEN] [CMD] ... payload ... [XOR checksum]
// LEN 係由呢個 byte 自己開始計、含 checksum 喺內嘅總長度。
// checksum = LEN ^ CMD ^ 全部 payload byte，應等於封包最後一個 byte。
//
// 支援嘅 command：
//   0x18 複數伺服角度指示：00(保留), SPD, {pmaID, LO, HI} xN
//   0x19 複數伺服 SPEED/STRETCH 設定：(00=SPEED|10=STRETCH), {pmaID,值} xN
// 其餘 command（0x02/0x07/0x15/0x17 等，原廠用嚟做內部狀態/迴圈判斷）
// 冇連接任何實體暫存器可供分支，對「郁機械人」冇實質意義，收到照樣
// 驗 checksum、確認冇解析錯位，但唔執行任何動作，直接跳去下一個封包。
//
// 接收用非阻塞狀態機，逐 byte 喺 loop() 度儲，唔用 while(available()<N)
// 阻塞等待，避免藍牙傳輸未到齊時卡住 LED/電壓監察/IMU 等其他工作。

#define PMA_PKT_MAX_LEN 100  // 已實測 10 個真實檔案 LEN 上限 80，留少少餘量
#define PMA_DEBUG_TRACK_ID 0x02  // 診斷用：追蹤呢個 pmaID 嘅角度值(0x02=HV1/右肩ピッチ)，
                                  // 方便同你郁緊嘅 Unity slider 直接對照

enum PmaRecvState { PMA_RECV_IDLE, PMA_RECV_BODY };
PmaRecvState pmaRecvState = PMA_RECV_IDLE;
uint8_t pmaPktBuf[PMA_PKT_MAX_LEN];
uint8_t pmaPktLen = 0;      // 呢個封包嘅目標長度（第一個 byte）
uint8_t pmaPktFilled = 0;   // 已經收到幾多個 byte（含 LEN 自己）
unsigned long lastPmaChecksumWarnMs = 0;  // (保留，暫未使用)
unsigned long pmaPacketOkCount = 0;    // 成功執行嘅封包累計數，用 "PMASTAT" 查
unsigned long pmaPacketFailCount = 0;  // checksum 錯誤嘅封包累計數
uint8_t pmaLastFailLen = 0;      // 最後一個失敗封包嘅 LEN
uint8_t pmaLastFailCmd = 0;      // 最後一個失敗封包嘅 CMD
uint8_t pmaLastFailCalc = 0;     // 最後一個失敗封包計算出嚟嘅 checksum
uint8_t pmaLastFailParity = 0;   // 最後一個失敗封包實際收到嘅 parity byte
uint8_t pmaLastFailBytes[PMA_PKT_MAX_LEN];  // 最後一個失敗封包嘅完整內容快照
uint8_t pmaLastFailBytesLen = 0;            // 快照實際長度
unsigned long pmaPktStartMs = 0;            // 開始接收目前封包嘅時間戳
unsigned long pmaLastFailDurationMs = 0;    // 最後一個失敗封包，由開始到收齊耗用嘅時間
uint8_t pmaLenHistory[5] = {0};             // 最近 5 次收到嘅 LEN 值
uint8_t pmaLenHistoryIdx = 0;               // 循環寫入位置
unsigned long pmaLenientAcceptCount = 0;    // 寬鬆模式接受咗嘅封包數（checksum錯但結構完整）
uint8_t pmaLastCmd18FirstId = 0;      // 最後一次 0x18 封包，第一隻servo嘅 pmaID
uint16_t pmaLastCmd18FirstAngle = 0;  // 最後一次 0x18 封包，第一隻servo嘅角度值（原始，未經home offset）
unsigned long pmaCmd18Count = 0;      // 總共收到幾多次 0x18 封包

// 由 pma-style servoID 轉做角度，經 applyHomeOffset 轉換基準後送出。
// baselineCenter 用返嗰隻 servo 自己嘅官方中立值（大部分係 7500，
// 但肩ロールR/L 例外），咁樣先可以令 Unity/.pma 送嚟嘅絕對角度正確
// 對應到你自己實測嘅 homePosition，唔會因為兩者基準唔一致而錯位。
//
// 官方協議規定 angle==0（即 raw bytes 00 00）係「脫力」特殊指令，唔係
// 正常角度值——如果照樣做 offset 轉換，會將 targetAngle 拉去極端值，
// 25 隻 servo 各自撞向自己極限，造成「保持⇔脫力」一撳落去全身扭曲嘅
// 結果。呢度要優先攔截呢個特殊值。
static void pmaApplyServoAngle(uint8_t pmaId, uint16_t rawAngle) {
  ServoInfo *servo = findServoByPmaId(pmaId);
  if (!servo) return;  // 未定義/未駁線嘅 ID 直接跳過
  if (rawAngle == 0) {
    if (servo->enabled) {
      servo->icsPort->setFree(servo->servoID);
    }
    return;
  }
  uint16_t safeAngle = applyHomeOffset(servo, rawAngle, servo->baselineCenter);
  safeSetPos(servo, safeAngle);
  servo->currentTunePos = safeAngle;
}

// 0x19：payload[0]=00(SPEED)|10(STRETCH)，之後 {pmaID,值} 一路重複
static void pmaHandleCmd19(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen < 1) return;
  uint8_t subType = payload[0];
  for (uint8_t i = 1; i + 1 < payloadLen; i += 2) {
    uint8_t pmaId = payload[i];
    uint8_t value = payload[i + 1];
    ServoInfo *servo = findServoByPmaId(pmaId);
    if (!servo) continue;
    if (subType == 0x00) {
      safeSetSpd(servo, value);
      servo->currentSpeed = value;
    } else if (subType == 0x10) {
      safeSetStrc(servo, value);
    }
  }
}

// 0x18：payload[0]=00(保留), payload[1]=SPD, 之後 {pmaID,LO,HI} 一路重複
// 收到呢個指令即係外部（PC/App）開始接管動作，同 table_walk 嘅
// 持續性步行輸出互斥，故此先確保 table_walk 已停，避免兩者同時
// safeSetPos() 打交叉。
static void pmaHandleCmd18(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen < 2) return;
  if (tableWalkIsWalking()) {
    tw_walker_init(&tableWalker);  // 強制歸零、即時停，唔送 home 指令
                                     // （.pma 封包自己會送角度，唔需要多餘一次 home）
  }
  pmaCmd18Count++;
  for (uint8_t i = 2; i + 2 < payloadLen; i += 3) {
    uint8_t pmaId = payload[i];
    uint16_t lo = payload[i + 1];
    uint16_t hi = payload[i + 2];
    uint16_t angle = (hi << 8) | lo;
    if (pmaId == PMA_DEBUG_TRACK_ID) {  // 追蹤指定嘅 pmaID，方便對照你郁緊嗰條 slider
      pmaLastCmd18FirstId = pmaId;
      pmaLastCmd18FirstAngle = angle;
    }
    pmaApplyServoAngle(pmaId, angle);
  }
}

// 執行一個已經收齊、通過 checksum 驗證嘅封包
static void pmaExecutePacket(const uint8_t *pkt, uint8_t len) {
  uint8_t cmd = pkt[1];
  const uint8_t *payload = pkt + 2;
  uint8_t payloadLen = len - 3;  // 扣埋 LEN(1)+CMD(1)+checksum(1)

  if (cmd == 0x18) {
    pmaHandleCmd18(payload, payloadLen);
  } else if (cmd == 0x19) {
    pmaHandleCmd19(payload, payloadLen);
  }
  // 其他 command：已知但唔影響郁動作，冇對應處理，略過
}

// 由 loop() 每次 iteration call 一次；逐 byte 儲入 buffer，
// 儲夠一個完整封包先驗 checksum 同執行，然後重置去等下一個封包。
void pmaReceiveUpdate() {
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.peek();

    if (pmaRecvState == PMA_RECV_IDLE) {
      // 判斷呢個 byte 係咪合理嘅 .pma 封包 LEN：要睇多一個 byte（CMD）
      // 先可以確定，因為單靠一個 byte 嘅數值範圍會同 ASCII 指令撞埋。
      //
      // 特殊例外：如果呢個 byte 本身就係 '\n'，一定要即刻處理，唔可以
      // 等第二個 byte 先算——因為 '\n' 代表一句 ASCII 指令已經完整，
      // 如果卡住唔處理，佢會一直等到 *下一次* send 嘅第一個字元一齊
      // 到達先被消耗，造成「指令要打兩次先有反應」。
      //
      // 但除咗 '\n' 之外嘅其他情況（例如 binary 封包啱啱好逐 byte
      // 到達，第一個 byte 啱啱好係 LEN），唔可以貿然假設佢係 ASCII
      // 就即刻消耗咗——試過呢種寫法，會令逐 byte 到達嘅 0x18 封包
      // 成個都被拆散當 ASCII 字元吞晒，令 Unity/PC 端完全冧唔到任何
      // binary 封包。所以除 '\n' 外一律維持保守，等夠 2 個 byte 先判斷。
      if (Serial1.available() < 2) {
        if (b == '\n') {
          Serial1.read();
          inputBuffer += '\n';
          processCommand(inputBuffer);
          inputBuffer = "";
        }
        return;  // 唔係 '\n'：保留呢個 byte 唔讀，等下次 loop() 儲夠 2 個先判斷
      }
      uint8_t peekLen = b;
      // peek 唔到第二個 byte，要 read 咗第一個先，用 index 0 存住
      Serial1.read();
      pmaPktBuf[0] = peekLen;
      pmaPktFilled = 1;

      uint8_t secondByte = Serial1.peek();
      // 用已知 CMD 集合而唔淨係 "< 0x20"，避免好似 "?\n" 呢類短
      // ASCII 指令（第二個 byte 係 \n=0x0A，都 < 0x20）被誤判做 binary 封包。
      // 集合根據解析 spreadsheet「一覧表」入面「直接」欄=〇 嘅指令整理：
      // 01=內部變數查詢(電量等) 02=迴圈起點 04=toggle 05=停止
      // 07=迴圈終點 15/17=PMA專用 18=多軸角度 19=Speed/Stretch
      // 1C=Flash dump 1E=LED控制 1F=播放已存動作
      // 漏咗 0x01 曾經令 Unity 嘅 RequestBatteryRemain()（07 01 00 02
      // 00 02 06，每 10 秒定時發一次）被誤判做 ASCII 字元塞入
      // inputBuffer，污染咗接收狀態，係之前「連續送信一開就冇反應」嘅
      // 根本原因。
      bool isPmaCmd = (secondByte == 0x01 || secondByte == 0x02 ||
                       secondByte == 0x04 || secondByte == 0x05 ||
                       secondByte == 0x07 || secondByte == 0x15 ||
                       secondByte == 0x17 || secondByte == 0x18 ||
                       secondByte == 0x19 || secondByte == 0x1C ||
                       secondByte == 0x1E || secondByte == 0x1F);
      bool looksLikePmaPacket = isPmaCmd && (peekLen >= 3 && peekLen <= PMA_PKT_MAX_LEN);

      if (looksLikePmaPacket) {
        pmaPktLen = peekLen;
        pmaRecvState = PMA_RECV_BODY;
        pmaPktStartMs = millis();  // 記低開始收呢個封包嘅時間，診斷傳輸節奏用
        // 記低最近 5 次收到嘅 LEN 值，方便睇係咪每次都固定同一個數值
        // （固定 = Unity 本身送緊呢個長度；隨機/多變 = 傳輸過程有損失）
        pmaLenHistory[pmaLenHistoryIdx % 5] = peekLen;
        pmaLenHistoryIdx++;
      } else {
        // 唔係 binary 封包，peekLen 呢個 byte 其實係普通 ASCII 字元
        char c = (char)peekLen;
        inputBuffer += c;
        if (c == '\n') {
          processCommand(inputBuffer);
          inputBuffer = "";
        }
        pmaPktFilled = 0;  // 復位，冇進入 binary 模式
      }
    } else {  // PMA_RECV_BODY
      pmaPktBuf[pmaPktFilled] = Serial1.read();
      pmaPktFilled++;

      if (pmaPktFilled >= pmaPktLen) {
        // 收齊一個封包，驗 checksum
        uint8_t xorCalc = pmaPktLen;
        for (uint8_t i = 1; i < pmaPktLen - 1; i++) {
          xorCalc ^= pmaPktBuf[i];
        }
        uint8_t parity = pmaPktBuf[pmaPktLen - 1];

        // 寬鬆驗證：已經反覆證實 Unity（kazz 版 continuousMode）本身
        // 冇正確計算 checksum，一律送出 0x00 佔位值，並非傳輸損毀
        // ——多次觀察 pmaLenHistory 全部固定 68，payload 內容完全自洽
        // （21 隻 servo，全部係已知合理 ID，數值都喺正常角度範圍），
        // 純粹係 Unity 端限制。為咗實際可用，容許 recv==0x00 且
        // payload 結構完整（扣除 LEN/CMD/保留位/SPD/checksum 之後,
        // 剩餘 byte 啱啱好整除 3）嘅封包照樣執行，其餘 checksum 錯誤
        // 仍然拒絕。
        bool structurallyValid = (pmaPktLen >= 8) && ((pmaPktLen - 5) % 3 == 0);
        bool passedChecksum = (xorCalc == parity);
        bool passedLenientMode = (parity == 0x00) && structurallyValid;

        if (passedChecksum || passedLenientMode) {
          pmaExecutePacket(pmaPktBuf, pmaPktLen);
          pmaPacketOkCount++;
          if (!passedChecksum) pmaLenientAcceptCount++;
        } else {
          // 唔即時 print——Serial1.print 阻塞式送出，成句 UTF-8 warning
          // 要幾十毫秒，同 continuousMode 高頻封包、電壓監察等其他
          // Serial1.print 撞埋一齊時，阻塞時間疊加，令 RX buffer 嘅
          // 下一個封包到達期間被覆蓋/延誤，造成「一錯就雪崩、成串
          // checksum 錯」。改為淨係計數 + 保留最後一個失敗封包嘅內容
          // 快照，用 "PMASTAT" 指令按需查詢，唔喺 loop() 入面即時噴。
          pmaPacketFailCount++;
          pmaLastFailLen = pmaPktLen;
          pmaLastFailCmd = pmaPktBuf[1];
          pmaLastFailCalc = xorCalc;
          pmaLastFailParity = parity;
          pmaLastFailBytesLen = pmaPktLen;
          pmaLastFailDurationMs = millis() - pmaPktStartMs;
          for (uint8_t d = 0; d < pmaPktLen; d++) {
            pmaLastFailBytes[d] = pmaPktBuf[d];
          }
        }

        pmaRecvState = PMA_RECV_IDLE;
        pmaPktFilled = 0;
      }
    }
  }
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
    Serial1.print(F("⚠️ setSpd 範圍錯誤 (1-127): "));
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
    Serial1.print(F("⚠️ setStrc 範圍錯誤 (1-127): "));
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
    Serial1.print(F("⚠️ setCur 範圍錯誤 (1-63): "));
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
    Serial1.print(F("⚠️ setTmp 範圍錯誤 (1-127): "));
    Serial1.print(servo->name);
    Serial1.print(F(" tmplim="));
    Serial1.println(tmplim);
    return false;
  }
  int result = servo->icsPort->setTmp(servo->servoID, tmplim);
  return (result != ICS_FALSE);
}

int setStrc(byte id, unsigned int strc) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  return servo->icsPort->setStrc(servo->servoID, strc);
}

int setSpd(byte id, unsigned int spd) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  return servo->icsPort->setSpd(servo->servoID, spd);
}

int setCur(byte id, unsigned int curlim) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  return servo->icsPort->setCur(servo->servoID, curlim);
}

int setTmp(byte id, unsigned int tmplim) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  return servo->icsPort->setTmp(servo->servoID, tmplim);
}

int getStrc(byte id) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  return servo->icsPort->getStrc(servo->servoID);
}

int getSpd(byte id) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  return servo->icsPort->getSpd(servo->servoID);
}

int getCur(byte id) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  return servo->icsPort->getCur(servo->servoID);
}

int getTmp(byte id) {
  ServoInfo *servo = findServoByBinaryID(id);
  if (!servo) return ICS_FALSE;
  return servo->icsPort->getTmp(servo->servoID);
}

// ===== 移動到 Home Point =====
void moveAllServosToHome() {
  Serial1.println(F("\n🚀 所有伺服回家，速度32"));

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
    isFreeMode = true;
    Serial1.println(F("✅ 所有伺服已脫力"));
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
  if (speed < 1) speed = 1;      // IcsBaseClass::MIN_1 = 1，唔係 0
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
    ServoInfo *servo = findServoByBinaryID(binaryId);

    if (servo != NULL) {
      safeSetSpd(servo, speed);
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

    ServoInfo *servo = findServoByBinaryID(binaryId);

    if (servo != NULL) {
      if (angle >= servo->minAngle && angle <= servo->maxAngle) {
        safeSetPos(servo, angle);
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

      ServoInfo *servo = findServoByBinaryID(binaryId);

      if (servo != NULL) {
        bool groupMatch = (group == "MV" && !servo->isHV) || (group == "HV" && servo->isHV);

        if (groupMatch) {
          if (angle >= servo->minAngle && angle <= servo->maxAngle) {

            if (fourthSpace > 0) {
              int speed = cmd.substring(fourthSpace + 1).toInt();
              if (speed < 1) speed = 1;      // IcsBaseClass::MIN_1 = 1，唔係 0
              if (speed > 127) speed = 127;

              safeSetSpd(servo, speed);
              servo->currentSpeed = speed;
            }

            safeSetPos(servo, angle);
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
    int thirdSpace = cmd.indexOf(' ', secondSpace + 1);

    if (firstSpace > 0 && secondSpace > 0) {
      String group = cmd.substring(firstSpace + 1, secondSpace);
      int binaryId = cmd.substring(secondSpace + 1, (thirdSpace > 0 ? thirdSpace : cmd.length())).toInt();
      String field = (thirdSpace > 0) ? cmd.substring(thirdSpace + 1) : "";
      field.trim();

      group.toUpperCase();

      ServoInfo *servo = findServoByBinaryID(binaryId);

      if (servo != NULL) {
        bool groupMatch = (group == "MV" && !servo->isHV) || (group == "HV" && servo->isHV);

        if (groupMatch) {
          if (!servo->enabled) {
            Serial1.print(F("⚠️ "));
            Serial1.print(servo->name);
            Serial1.println(F(" 未駁線 (enabled=false)，略過"));
            return true;
          }

          if (field == "STRC" || field == "SPD" || field == "CUR" || field == "TMP") {
            int result = ICS_FALSE;
            const char *label = "";
            if (field == "STRC") { result = getStrc(binaryId); label = "Stretch"; }
            else if (field == "SPD")  { result = getSpd(binaryId);  label = "Speed"; }
            else if (field == "CUR")  { result = getCur(binaryId);  label = "電流上限"; }
            else if (field == "TMP")  { result = getTmp(binaryId);  label = "溫度上限"; }

            if (result != ICS_FALSE) {
              Serial1.print(group);
              Serial1.print(F(" ID "));
              Serial1.print(binaryId);
              Serial1.print(F(" "));
              Serial1.print(label);
              Serial1.print(F(": "));
              Serial1.println(result);
            } else {
              Serial1.println(F("❌ 讀取失敗（通訊錯誤）"));
            }
            return true;
          }

          // 冇第4個參數：沿用原本行為，查詢目前角度
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

  Serial1.print(F("設定伺服速度 (HV="));
  Serial1.print(DEFAULT_SPEED_HV);
  Serial1.print(F(", MV="));
  Serial1.print(DEFAULT_SPEED_MV);
  Serial1.print(F(")..."));
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    int spdVal = servoList[i].isHV ? DEFAULT_SPEED_HV : DEFAULT_SPEED_MV;
    safeSetSpd(&servoList[i], spdVal);
  }
  Serial1.println(F("完成"));

  Serial1.print(F("設定伺服 Stretch (HV="));
  Serial1.print(DEFAULT_STRETCH_HV);
  Serial1.print(F(", MV="));
  Serial1.print(DEFAULT_STRETCH_MV);
  Serial1.print(F(")..."));
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    int strcVal = servoList[i].isHV ? DEFAULT_STRETCH_HV : DEFAULT_STRETCH_MV;
    safeSetStrc(&servoList[i], strcVal);
  }
  Serial1.println(F("完成"));

  Serial1.print(F("設定伺服電流上限 (HV="));
  Serial1.print(DEFAULT_CUR_HV);
  Serial1.print(F(", MV="));
  Serial1.print(DEFAULT_CUR_MV);
  Serial1.print(F(")..."));
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    int curVal = servoList[i].isHV ? DEFAULT_CUR_HV : DEFAULT_CUR_MV;
    if (!safeSetCur(&servoList[i], curVal)) {
      Serial1.print(F(" [失敗:")); Serial1.print(servoList[i].name); Serial1.print(F("]"));
    }
  }
  Serial1.println(F(" 完成"));

  Serial1.print(F("設定伺服溫度上限 (HV="));
  Serial1.print(DEFAULT_TMP_HV);
  Serial1.print(F(", MV="));
  Serial1.print(DEFAULT_TMP_MV);
  Serial1.print(F(")..."));
  for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
    int tmpVal = servoList[i].isHV ? DEFAULT_TMP_HV : DEFAULT_TMP_MV;
    if (!safeSetTmp(&servoList[i], tmpVal)) {
      Serial1.print(F(" [失敗:")); Serial1.print(servoList[i].name); Serial1.print(F("]"));
    }
  }
  Serial1.println(F(" 完成"));

  Serial1.println(F("\n🏠 發送 home 指令（servo 移動中）..."));
  moveAllServosToHome();
  Serial1.println(F("✅ Servo home 指令已發送，正在移動中..."));

  voltageData.currentVoltage = readBatteryVoltage();
  Serial1.print(F("\n🔋 當前電壓: "));
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
    Serial1.println(F("⚠️ 未偵測到（WHO_AM_I 驗證失敗），跳過 IMU 功能"));
  }

  // LED 交返俾 updateActivityLED() 根據 lastActivityMs 自動判斷：
  // servo 郁緊（moveAllServosToHome() 啱啱觸發咗 safeSetPos()）就顯示
  // 綠色（忙碌），實際到位、冇再郁就自動喺 300ms 內跳返做青色（待機）—
  // 唔再用硬編碼嘅幾秒動畫,LED 會真正反映 servo 實際狀態。
  updateActivityLED();

  Serial1.println(F("\n✅ 系統就緒"));
  Serial1.println(F("========================================\n"));
}

// ===== loop() =====
void loop() {
  checkVoltage();
  updateActivityLED();
  updateManualBreathLED();

  // table_walk 用固定 tick 驅動（唔似原本 WalkGenerator 咁連續時間積分）
  unsigned long nowTick = millis();
  if (nowTick - lastWalkTickMs >= TABLE_WALK_TICK_MS) {
    lastWalkTickMs = nowTick;
    table_walk_update(&tableWalker, &tableWalkParams);
    sendTableWalkAngles();
  }

  // .pma / ICS binary 封包接收（非阻塞，逐 byte 儲，收齊即行）
  pmaReceiveUpdate();

  // IMU 開機自動校正（非阻塞）：等開機滿 IMU_AUTO_CAL_DELAY_MS（servo
  // 應已行到 home 企定）先做一次，唔用 delay() 卡住 loop()，LED/電壓/
  // 伺服通信全部正常運作，唔會再出現「LED 卡住幾秒」嘅問題。
  // 校正嗰刻閃白燈 + 清晰 log，方便唔開 Serial Monitor 都睇到發生咗。
  if (imuOk && !imuGyroCalibrated && millis() >= IMU_AUTO_CAL_DELAY_MS) {
    imuGyroCalibrated = true;  // 即刻標記，防止呢段重複觸發
    ledManualOverride = true;
    setLEDWhite();
    Serial1.print(F("\n⚪ [IMU] 開機自動校正陀螺零偏（機身請保持靜止）... t="));
    Serial1.print(millis());
    Serial1.println(F("ms"));
    imuCalibrateGyro();
    Serial1.print(F("⚪ [IMU] 校正完成 offsetX=")); Serial1.print(gyroOffsetX, 2);
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

// ===== 命令處理 =====
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

  if (cmd == "H" || cmd == "HELP" || cmd == "?") {
    showHelp();
  } else if (cmd == "VOLTAGE") {
    Serial1.print(F("🔋 當前電壓: "));
    Serial1.print(voltageData.currentVoltage);
    Serial1.println(F("V"));
  } else if (cmd == "PMASTAT") {
    Serial1.print(F("📊 .pma 封包統計: 成功="));
    Serial1.print(pmaPacketOkCount);
    Serial1.print(F(" 失敗="));
    Serial1.print(pmaPacketFailCount);
    Serial1.print(F(" 寬鬆接受="));
    Serial1.println(pmaLenientAcceptCount);
    Serial1.print(F("最近5次LEN: "));
    for (uint8_t k = 0; k < 5; k++) {
      Serial1.print(pmaLenHistory[k]);
      Serial1.print(' ');
    }
    Serial1.println();
    Serial1.print(F("0x18封包總數="));
    Serial1.print(pmaCmd18Count);
    Serial1.print(F(" 追蹤pmaID=0x"));
    Serial1.print(PMA_DEBUG_TRACK_ID, HEX);
    Serial1.print(F(" 最後角度="));
    Serial1.println(pmaLastCmd18FirstAngle);
    if (pmaLastFailBytesLen > 0) {
      Serial1.print(F("最後失敗封包 LEN="));
      Serial1.print(pmaLastFailLen);
      Serial1.print(F(" CMD="));
      Serial1.print(pmaLastFailCmd, HEX);
      Serial1.print(F(" calc="));
      Serial1.print(pmaLastFailCalc, HEX);
      Serial1.print(F(" recv="));
      Serial1.print(pmaLastFailParity, HEX);
      Serial1.print(F(" 接收耗時="));
      Serial1.print(pmaLastFailDurationMs);
      Serial1.println(F("ms"));
      Serial1.print(F("bytes="));
      for (uint8_t d = 0; d < pmaLastFailBytesLen; d++) {
        if (pmaLastFailBytes[d] < 0x10) Serial1.print('0');
        Serial1.print(pmaLastFailBytes[d], HEX);
        Serial1.print(' ');
      }
      Serial1.println();
    }
  } else if (cmd == "IMU") {
    if (!imuData.present) {
      Serial1.println(F("⚠️ IMU 未偵測到"));
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
      Serial1.println(F("⚠️ IMU 未偵測到，無法校正"));
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
  // ===== LED 測試指令 =====
  else if (cmd == "LED RED") {
    ledManualOverride = true;
    manualBreathColorIndex = 0;
    Serial1.println(F("✅ LED = 紅（呼吸）"));
  } else if (cmd == "LED GREEN") {
    ledManualOverride = true;
    manualBreathColorIndex = 1;
    Serial1.println(F("✅ LED = 綠（呼吸）"));
  } else if (cmd == "LED BLUE") {
    ledManualOverride = true;
    manualBreathColorIndex = 2;
    Serial1.println(F("✅ LED = 藍（呼吸）"));
  } else if (cmd == "LED YELLOW") {
    ledManualOverride = true;
    manualBreathColorIndex = 3;
    Serial1.println(F("✅ LED = 黃（呼吸）"));
  } else if (cmd == "LED CYAN") {
    ledManualOverride = true;
    manualBreathColorIndex = 4;
    Serial1.println(F("✅ LED = 青（呼吸）"));
  } else if (cmd == "LED PURPLE") {
    ledManualOverride = true;
    manualBreathColorIndex = 5;
    Serial1.println(F("✅ LED = 紫（呼吸）"));
  } else if (cmd == "LED WHITE") {
    ledManualOverride = true;
    manualBreathColorIndex = 6;
    Serial1.println(F("✅ LED = 白（呼吸）"));
  } else if (cmd == "LED OFF") {
    ledManualOverride = true;
    manualBreathColorIndex = -1;
    setLEDOff();
    Serial1.println(F("✅ LED = 關"));
  } else if (cmd == "LED AUTO") {
    ledManualOverride = false;
    manualBreathColorIndex = -1;
    Serial1.println(F("✅ LED 恢復自動模式（電壓狀態顯示）"));
  } else if (cmd.startsWith("LED BREATH ")) {
    // 格式: LED BREATH <RED|GREEN|BLUE> <speed(ms)> [次數]
    String args = cmd.substring(11);
    args.trim();
    int sp1 = args.indexOf(' ');
    if (sp1 < 0) {
      Serial1.println(F("❌ 格式錯誤，用法: LED BREATH <RED/GREEN/BLUE> <速度ms> [次數]"));
    } else {
      String colorName = args.substring(0, sp1);
      String rest = args.substring(sp1 + 1);
      rest.trim();
      int sp2 = rest.indexOf(' ');
      int speed = (sp2 > 0) ? rest.substring(0, sp2).toInt() : rest.toInt();
      int cycles = (sp2 > 0) ? rest.substring(sp2 + 1).toInt() : 1;
      speed = constrain(speed, 1, 100);
      cycles = constrain(cycles, 1, 20);

      int pin = ledPinByName(colorName);
      if (pin < 0) {
        Serial1.println(F("❌ 顏色錯誤，用: RED/GREEN/BLUE"));
      } else {
        ledManualOverride = true;
        manualBreathColorIndex = -1;  // 呢個測試用阻塞式 breathLED()，唔經過手動呼吸機制
        Serial1.print(F("✅ 呼吸燈測試: ")); Serial1.print(colorName);
        Serial1.print(F(" speed=")); Serial1.print(speed);
        Serial1.print(F(" x")); Serial1.println(cycles);
        for (int i = 0; i < cycles; i++) {
          breathLED(pin, speed);
        }
        setLEDOff();
      }
    }
  } else if (cmd.startsWith("LED BRIGHT ")) {
    // 格式: LED BRIGHT <RED|GREEN|BLUE> <0-255>
    String args = cmd.substring(11);
    args.trim();
    int sp1 = args.indexOf(' ');
    if (sp1 < 0) {
      Serial1.println(F("❌ 格式錯誤，用法: LED BRIGHT <RED/GREEN/BLUE> <0-255>"));
    } else {
      String colorName = args.substring(0, sp1);
      int brightness = constrain(args.substring(sp1 + 1).toInt(), 0, 255);
      int logicalColor = ledPinByName(colorName);
      if (logicalColor < 0) {
        Serial1.println(F("❌ 顏色錯誤，用: RED/GREEN/BLUE"));
      } else {
        ledManualOverride = true;
        manualBreathColorIndex = -1;  // 清走手動呼吸選色，避免覆蓋固定亮度
        // 測試單一顏色亮度前，先關晒另外兩隻，避免混色
        setLEDOff();
        int physPin = ledPhysicalPin(logicalColor);
        analogWrite(physPin, 255 - brightness);  // active-low: 反轉 duty cycle
        Serial1.print(F("✅ ")); Serial1.print(colorName);
        Serial1.print(F(" 亮度 = ")); Serial1.println(brightness);
      }
    }
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
        Serial1.println(F("❌ 方向錯誤 (F/B/L/R)"));
        return;
    }
    Serial1.println();

    tableWalker.lt_held = 1;

    Serial1.println(F("✅ 步行指令已接收（輸入 STOP 停止）"));
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
        Serial1.print(F("✅ DuraLen = ")); Serial1.println(tableWalkParams.dura_len);
      } else if (paramName == "ROLL") {
        tableWalkParams.dura_roll = constrain(value, 10, 200);
        Serial1.print(F("✅ DuraRoll = ")); Serial1.println(tableWalkParams.dura_roll);
      } else if (paramName == "PITCH") {
        tableWalkParams.dura_pitch = constrain(value, 100, 1200);
        Serial1.print(F("✅ DuraPitch = ")); Serial1.println(tableWalkParams.dura_pitch);
      } else if (paramName == "TICKMS") {
        Serial1.println(F("❌ TICKMS 需要重新編譯（#define TABLE_WALK_TICK_MS），暫不支援即時調整"));
      } else if (paramName == "STRC") {
        // 格式: WALKSET STRC <HV|MV> <值>
        String strcArgs = params.substring(spacePos + 1);
        strcArgs.trim();
        int strcSpacePos = strcArgs.indexOf(' ');
        if (strcSpacePos > 0) {
          String strcGroup = strcArgs.substring(0, strcSpacePos);
          strcGroup.toUpperCase();
          int strc = constrain(strcArgs.substring(strcSpacePos + 1).toInt(), 1, 127);

          if (strcGroup == "HV" || strcGroup == "MV") {
            bool wantHV = (strcGroup == "HV");
            for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
              if (servoList[i].isHV == wantHV) {
                safeSetStrc(&servoList[i], strc);
              }
            }
            Serial1.print(F("✅ ")); Serial1.print(strcGroup);
            Serial1.print(F(" 伺服 Stretch = ")); Serial1.println(strc);
          } else {
            Serial1.println(F("❌ 格式錯誤，用法: WALKSET STRC HV <值> 或 WALKSET STRC MV <值>"));
          }
        } else {
          Serial1.println(F("❌ 格式錯誤，用法: WALKSET STRC HV <值> 或 WALKSET STRC MV <值>"));
        }
      } else if (paramName == "SPD") {
        // 格式: WALKSET SPD <HV|MV> <值>
        String spdArgs = params.substring(spacePos + 1);
        spdArgs.trim();
        int spdSpacePos = spdArgs.indexOf(' ');
        if (spdSpacePos > 0) {
          String spdGroup = spdArgs.substring(0, spdSpacePos);
          spdGroup.toUpperCase();
          int spd = constrain(spdArgs.substring(spdSpacePos + 1).toInt(), 1, 127);

          if (spdGroup == "HV" || spdGroup == "MV") {
            bool wantHV = (spdGroup == "HV");
            for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
              if (servoList[i].isHV == wantHV) {
                safeSetSpd(&servoList[i], spd);
              }
            }
            Serial1.print(F("✅ ")); Serial1.print(spdGroup);
            Serial1.print(F(" 伺服 Speed = ")); Serial1.println(spd);
          } else {
            Serial1.println(F("❌ 格式錯誤，用法: WALKSET SPD HV <值> 或 WALKSET SPD MV <值>"));
          }
        } else {
          Serial1.println(F("❌ 格式錯誤，用法: WALKSET SPD HV <值> 或 WALKSET SPD MV <值>"));
        }
      } else if (paramName == "CUR") {
        // 格式: WALKSET CUR <HV|MV> <值>
        String curArgs = params.substring(spacePos + 1);
        curArgs.trim();
        int curSpacePos = curArgs.indexOf(' ');
        if (curSpacePos > 0) {
          String curGroup = curArgs.substring(0, curSpacePos);
          curGroup.toUpperCase();
          int curlim = constrain(curArgs.substring(curSpacePos + 1).toInt(), 1, 63);

          if (curGroup == "HV" || curGroup == "MV") {
            bool wantHV = (curGroup == "HV");
            for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
              if (servoList[i].isHV == wantHV) {
                safeSetCur(&servoList[i], curlim);
              }
            }
            Serial1.print(F("✅ ")); Serial1.print(curGroup);
            Serial1.print(F(" 伺服 電流上限 = ")); Serial1.println(curlim);
          } else {
            Serial1.println(F("❌ 格式錯誤，用法: WALKSET CUR HV <值> 或 WALKSET CUR MV <值>"));
          }
        } else {
          Serial1.println(F("❌ 格式錯誤，用法: WALKSET CUR HV <值> 或 WALKSET CUR MV <值>"));
        }
      } else if (paramName == "TMP") {
        // 格式: WALKSET TMP <HV|MV> <值>
        String tmpArgs = params.substring(spacePos + 1);
        tmpArgs.trim();
        int tmpSpacePos = tmpArgs.indexOf(' ');
        if (tmpSpacePos > 0) {
          String tmpGroup = tmpArgs.substring(0, tmpSpacePos);
          tmpGroup.toUpperCase();
          int tmplim = constrain(tmpArgs.substring(tmpSpacePos + 1).toInt(), 1, 127);

          if (tmpGroup == "HV" || tmpGroup == "MV") {
            bool wantHV = (tmpGroup == "HV");
            for (int i = 0; i < TOTAL_SERVO_NUM; i++) {
              if (servoList[i].isHV == wantHV) {
                safeSetTmp(&servoList[i], tmplim);
              }
            }
            Serial1.print(F("✅ ")); Serial1.print(tmpGroup);
            Serial1.print(F(" 伺服 溫度上限 = ")); Serial1.println(tmplim);
          } else {
            Serial1.println(F("❌ 格式錯誤，用法: WALKSET TMP HV <值> 或 WALKSET TMP MV <值>"));
          }
        } else {
          Serial1.println(F("❌ 格式錯誤，用法: WALKSET TMP HV <值> 或 WALKSET TMP MV <值>"));
        }
      } else {
        Serial1.println(F("❌ 未知參數，可用: LEN, ROLL, PITCH, STRC, SPD, CUR, TMP"));
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
      Serial1.println(F("❓ 未知命令，輸入 H 查看說明"));
    }
  }
}

// ===== 顯示幫助選單 =====
void showHelp() {
  Serial1.println(F("\n=== 命令列表 (括號內為短寫) ==="));
  Serial1.println(F("H, HELP, ?      : 顯示此說明"));
  Serial1.println(F("VOLTAGE (V)     : 顯示當前電池電壓"));
  Serial1.println(F("PMASTAT (PS)    : 顯示 .pma 封包接收統計 (成功/失敗數)"));
  Serial1.println(F("IMU (G)         : 顯示 IMU (MPU-6050) 當前姿態 (accel/gyro/pitch/roll)"));
  Serial1.println(F("IMU CAL (GC)    : 重新校正陀螺零偏 (機身須靜止)"));
  Serial1.println(F("HOME (HM)       : 所有伺服機回到初始位置"));
  Serial1.println(F("FREE ALL (F)    : 所有伺服機脫力 (關閉扭力)"));
  Serial1.println(F("\n=== LED 測試指令 ==="));
  Serial1.println(F("LED RED/GREEN/BLUE/YELLOW/CYAN/PURPLE/WHITE/OFF : 切換LED顏色 (7色+關)"));
  Serial1.println(F("LED AUTO   : 恢復自動模式（電壓狀態顯示，藍=正常/紅=低電壓）"));
  Serial1.println(F("LED BREATH <色> <速度ms> [次數] : 測試呼吸燈速度 (例: LED BREATH RED 4 3)"));
  Serial1.println(F("LED BRIGHT <色> <0-255>        : 測試固定亮度 (例: LED BRIGHT BLUE 128)"));
  Serial1.println(F("\n=== 步行與動作指令 ==="));
  Serial1.println(F("WALK F/B/L/R  : 開始向前/後/左轉/右轉行走，直到收到 STOP"));
  Serial1.println(F("STOP (ST)     : 立即安全停止當前動作並回到 Home Point"));
  Serial1.println(F("\n=== 單軸/多軸微調 (ASCII) ==="));
  Serial1.println(F("S [群組] [ID] [角度] [速度] : 設定單一伺服 (例: S HV 1 8000 64)"));
  Serial1.println(F("S MULTI [速度] [數量] [群組ID 角度 ...] : 多軸同步"));
  Serial1.println(F("? [群組] [ID] : 查詢伺服當前位置"));
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
  Serial1.println(F("? [群組] [ID] [STRC/SPD/CUR/TMP] : 讀取伺服參數 (冇第3項=查角度)"));
}