

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
  { 24, 4, 9500, 9500, DEFAULT_SPEED_MV, &icsMV, "右肩側擺", 7450, 10350, false, true, 9500 },  // 官方中立值非7500
  { 25, 5, 5500, 5500, DEFAULT_SPEED_MV, &icsMV, "左肩側擺", 4550, 7550, false, true, 5500 },   // 官方中立值非7500
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

#define PMA_PKT_MAX_LEN 140  // 原本 100 只夠一般 .pma streaming 封包（LEN 上限 80）；
                              // 加入 Flash-based WriteMotionData 之後，最大封包可以係
                              // 134 bytes（128-byte data chunk + 6 bytes overhead：
                              // LEN+CMD+reserved+addrLo+addrHi+checksum），呢度留少少 margin

enum PmaRecvState { PMA_RECV_IDLE, PMA_RECV_BODY };
PmaRecvState pmaRecvState = PMA_RECV_IDLE;
uint8_t pmaPktBuf[PMA_PKT_MAX_LEN];
uint8_t pmaPktLen = 0;      // 呢個封包嘅目標長度（第一個 byte）
uint8_t pmaPktFilled = 0;   // 已經收到幾多個 byte（含 LEN 自己）
unsigned long pmaPacketOkCount = 0;    // 成功執行嘅封包累計數，用 "PMASTAT" 查
unsigned long pmaPacketFailCount = 0;  // checksum 錯誤（且結構唔完整）嘅封包累計數

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
  for (uint8_t i = 2; i + 2 < payloadLen; i += 3) {
    uint8_t pmaId = payload[i];
    uint16_t lo = payload[i + 1];
    uint16_t hi = payload[i + 2];
    uint16_t angle = (hi << 8) | lo;
    pmaApplyServoAngle(pmaId, angle);
  }
}

// 前置宣告：pmaExecutePacket 實際定義喺下面（0x18/0x19 dispatch 之後），
// 但 Flash playback engine（喺呢個位置之後、pmaExecutePacket 定義之前）
// 需要 call 佢嚟執行由 Flash 讀出嚟嘅封包，所以要 forward-declare。
static void pmaExecutePacket(const uint8_t *pkt, uint8_t len, bool fromFlashPlayback = false);

// =========================================================
// ===== STM32 內部 Flash 底層讀寫（motion storage 專用） =====
// =========================================================
// STM32F103C8/CB (128KB Flash)，page size = 1024 bytes（low/medium
// density devices）。喺 Flash 尾部劃一個 60KB 嘅 motion storage 區
// （跟原裝 60-page 上限），用固定地址（Flash 尾 60KB），因為標準
// STM32duino linker script 冇提供 "code end" 符號畀我哋自動計。
// 只要 firmware .bin 大細冇超過 68KB（128KB - 60KB），呢個劃法
// 就唔會同 code 重疊——你依家 firmware 大約 65KB，仲有 3KB margin，
// 加完呢批 code 之後要留意實際編譯後大細（Arduino IDE 編譯完會
// 顯示 "Sketch uses xxx bytes"，要細過 69632 bytes 先安全）。
#define STM32_FLASH_BASE            0x08000000UL
#define STM32_FLASH_TOTAL_SIZE      (128UL * 1024UL)   // 128KB chip
#define MOTION_FLASH_PAGE_SIZE      1024u               // 1 page = 1KB，跟原裝協議一致
#define MOTION_FLASH_PAGE_COUNT     60u                 // 跟原裝 60-page 上限 = 60KB
#define MOTION_FLASH_RESERVED_SIZE  (MOTION_FLASH_PAGE_COUNT * MOTION_FLASH_PAGE_SIZE)  // 60KB
#define MOTION_FLASH_START_ADDR     (STM32_FLASH_BASE + STM32_FLASH_TOTAL_SIZE - MOTION_FLASH_RESERVED_SIZE)
// = 0x08000000 + 0x20000 - 0xF000 = 0x08011000

// 每個 page 一份 1024-byte 嘅 SRAM staging buffer：host 用
// WriteMotionData 逐 128-byte 寫入呢度，收齊一個 page 先用
// SaveMotionData 觸發真正 erase+program 落 Flash（STM32 Flash 唔可以
// 隨意逐 byte 覆寫，一定要成個 page 先 erase 先寫）。
static uint8_t motionPageStaging[MOTION_FLASH_PAGE_SIZE];

static inline uint32_t motionFlashPageAddr(uint8_t pageIndex) {
  return MOTION_FLASH_START_ADDR + (uint32_t)pageIndex * MOTION_FLASH_PAGE_SIZE;
}

// 由 Flash 讀 — Flash 本身係 memory-mapped，直接讀指標就得。
static void motionFlashRead(uint32_t relOffset, uint8_t *outBuf, uint16_t len) {
  const uint8_t *src = (const uint8_t *)(MOTION_FLASH_START_ADDR + relOffset);
  memcpy(outBuf, src, len);
}

// 將指定 page 嘅 staging buffer 真正 erase+program 落 Flash。
// STM32F1 flash program 單位係 half-word (2 bytes)，staging buffer
// 固定 1024 bytes（偶數）冇問題。
static bool motionFlashProgramPage(uint8_t pageIndex, const uint8_t *data1024) {
  if (pageIndex >= MOTION_FLASH_PAGE_COUNT) return false;
  uint32_t pageAddr = motionFlashPageAddr(pageIndex);

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

  FLASH_EraseInitTypeDef eraseInit;
  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.PageAddress = pageAddr;
  eraseInit.NbPages = 1;
  uint32_t pageError = 0;

  bool ok = (HAL_FLASHEx_Erase(&eraseInit, &pageError) == HAL_OK);

  if (ok) {
    for (uint16_t i = 0; i < MOTION_FLASH_PAGE_SIZE; i += 2) {
      uint16_t halfWord = data1024[i] | ((uint16_t)data1024[i + 1] << 8);
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, pageAddr + i, halfWord) != HAL_OK) {
        ok = false;
        break;
      }
    }
  }

  HAL_FLASH_Lock();
  return ok;
}

// =========================================================
// ===== Motion ID 對照表（motionId -> startPage/totalLen） =====
// =========================================================
// (struct MotionIdEntry 定義已搬去檔案最頭，見開頭 comment 解釋)
#define MOTION_ID_MAX_ENTRIES 8
static MotionIdEntry motionIdTable[MOTION_ID_MAX_ENTRIES];

static MotionIdEntry *motionIdFind(uint16_t motionId) {
  for (uint8_t i = 0; i < MOTION_ID_MAX_ENTRIES; i++) {
    if (motionIdTable[i].used && motionIdTable[i].motionId == motionId) {
      return &motionIdTable[i];
    }
  }
  return nullptr;
}

static MotionIdEntry *motionIdAllocOrFind(uint16_t motionId) {
  MotionIdEntry *existing = motionIdFind(motionId);
  if (existing) return existing;
  for (uint8_t i = 0; i < MOTION_ID_MAX_ENTRIES; i++) {
    if (!motionIdTable[i].used) {
      motionIdTable[i].used = true;
      motionIdTable[i].motionId = motionId;
      motionIdTable[i].startPage = 0;
      motionIdTable[i].totalLen = 0;
      return &motionIdTable[i];
    }
  }
  return nullptr;  // 表滿咗
}

// =========================================================
// ===== Motion playback engine — MCU 自己由 Flash 讀出嚟播 =====
// =========================================================
// 同檔案開頭嗰個「即時 0x18/0x19 streaming」係兩種唔同模式：
// - Streaming 模式：host 逐個 packet send，MCU 收到即刻做（原有邏輯，
//   繼續保留，適合 debug/單次測試動作）
// - Flash 模式（呢度）：host 跟原裝協議將成隻 .pma 嘅 binary 內容
//   分 page 寫入 Flash，之後淨係送一次 StartMotion(page)，MCU 之後
//   喺 loop() 自己逐個 packet 由 Flash 度讀出嚟解碼、控制節奏、
//   call pmaExecutePacket()，host 完全唔使再理，同原裝 app 行為
//   一致，唔會再受 host 送 packet 節奏／UART buffer 影響。
//
// 兩者共用 pmaExecutePacket()（0x18/0x19 嘅實際 servo 操作邏輯）。

enum MotionPlaybackState { MOTION_PLAY_IDLE, MOTION_PLAY_RUNNING };
static MotionPlaybackState motionPlayState = MOTION_PLAY_IDLE;
static uint32_t motionPlayOffset = 0;      // 現正播放緊嗰個動作，喺 Flash storage 區入面嘅相對 byte offset
static uint32_t motionPlayEndOffset = 0;   // 呢個動作資料結束嘅相對 offset 上限（motionPlayOffset 追到就停）
static unsigned long motionPlayNextTickMs = 0;  // 下一個封包幾時可以送出（millis）

// 由 Flash 讀一個完整封包出嚟。傳返實際封包長度（含 checksum），
// 如果遇到結尾標記或者資料播完就傳返 0。
static uint8_t motionFlashReadNextPacket(uint8_t *outBuf) {
  if (motionPlayOffset >= motionPlayEndOffset) return 0;

  uint8_t lenByte;
  motionFlashRead(motionPlayOffset, &lenByte, 1);
  if (lenByte == 0xFF || lenByte == 0x00) return 0;  // 檔案結尾標記
  if (motionPlayOffset + lenByte > motionPlayEndOffset) return 0;  // 資料唔完整，當結尾

  motionFlashRead(motionPlayOffset, outBuf, lenByte);
  return lenByte;
}

// 由 loop() 每次 call 一次。非阻塞：如果而家仲喺度等緊上一個封包
// 嘅 SPD tick 時間就直接 return，時間到先讀下一個封包執行。
static void motionPlaybackUpdate() {
  if (motionPlayState != MOTION_PLAY_RUNNING) return;
  if ((long)(millis() - motionPlayNextTickMs) < 0) return;  // 未到時間

  uint8_t pkt[MOTION_FLASH_PAGE_SIZE > 255 ? 255 : MOTION_FLASH_PAGE_SIZE];
  uint8_t len = motionFlashReadNextPacket(pkt);
  if (len == 0) {
    motionPlayState = MOTION_PLAY_IDLE;  // 動作播放完（讀到 0xFF/0x00 結尾標記）
    return;
  }

  pmaExecutePacket(pkt, len, /*fromFlashPlayback=*/true);
  motionPlayOffset += len;

  // SPD tick 喺 0x18 封包嘅 payload[1]（即 pkt[3]，1 tick = 15ms）
  // 決定下一個封包幾時送；其他封包用最短 15ms 間距。
  uint8_t cmd = (len > 1) ? pkt[1] : 0;
  uint8_t spdTicks = (cmd == 0x18 && len > 3) ? pkt[3] : 1;
  if (spdTicks == 0) spdTicks = 1;
  motionPlayNextTickMs = millis() + (uint32_t)spdTicks * 15u;
}

static void motionPlaybackStart(uint8_t startPage, uint16_t totalLen) {
  motionPlayOffset = (uint32_t)startPage * MOTION_FLASH_PAGE_SIZE;
  motionPlayEndOffset = motionPlayOffset + totalLen;
  motionPlayNextTickMs = millis();
  motionPlayState = MOTION_PLAY_RUNNING;
}

static void motionPlaybackStop() {
  motionPlayState = MOTION_PLAY_IDLE;
  moveAllServosToHome();  // 同 tableWalkSafeStop() 一致嘅安全停法
}

// =========================================================
// ===== Flash-based motion command dispatch（原裝協議） =====
// =========================================================
// CMD byte / payload 格式跟返反編譯原裝 APK
// (com.robotyuenchi.premaidai.bluetooth.command.*) 逐個 class 對照：
//
//   0x1D WriteMotionData: payload = [reserved=0x00][addrLo][addrHi][data...]
//        (reserved 固定 0x00，addr = 呢個 page 入面嘅 16-bit
//        little-endian byte offset，最多 128 bytes data；
//        LEN byte = data.length + 6)
//        response: 04 1D <errorBitmask> <checksum>
//
//   0x1E SaveMotionData:  payload = [reserved=0x00][page]
//        response: 04 1E <errorBitmask> <checksum>
//        效果：將 SRAM staging buffer erase+program 落 Flash 第
//        page 個 sector。
//
//   0x1C ReadFlashBuffer: payload = [reserved=0x00][page]
//        response: FF 1C <1024 bytes flash data> <checksum>
//        (原裝期望 response.length==1024 做完整頁校驗；1024+4=1028
//        超過 8-bit LEN 上限，呢度用 0xFF 做 "large payload" sentinel,
//        HTML sender 對應要識别呢個特殊 case)
//
//   0x02 WriteMotionId:   payload = [00][08][00][contentIdLo][contentIdHi]
//        (前 3 bytes 係原裝固定 header，淨係要攞後面 2 bytes contentId)
//        response: 04 02 <errorBitmask> <checksum>
//        contentId==0xFFFF 代表「清空目前 motion id」（傳輸開始前用）。
//
//   0x04 SaveMotionId:    payload = (無)
//        response: 04 04 <errorBitmask> <checksum>
//        原裝呢個 command 本身冇帶 motionId payload，MCU 靠住
//        「上一個成功處理嘅 WriteMotionId(真正 contentId)」呢個
//        module-level 狀態嚟知道應該 save 邊一個 entry。
//
//   0x1F StartMotion:     payload = [reserved=0x00][startPage]
//        response: 04 1F <errorBitmask> <checksum>
//   0x1F StopMotion (借用同一 CMD): payload = [0x08][0x01]
//        (呢個冇 reserved byte，直接就係 sub-command flag 0x08 0x01)
//        response: 04 1F <errorBitmask> <checksum>
//
// errorBitmask 跟原裝 BaseCommand.Error 對應（bit0=ADDRESS,
// bit1=LITERAL, bit2=COMMAND, bit3=SIZE, bit4=COUNT, bit5=DEVICE,
// bit6=CHECK_SUM, bit7=FLASH）；0x00 = 成功。

#define MOTION_ERR_NONE      0x00
#define MOTION_ERR_ADDRESS   0x01
#define MOTION_ERR_SIZE      0x08
#define MOTION_ERR_COUNT     0x10
#define MOTION_ERR_FLASH     0x80

static void sendFlashResponse4(uint8_t cmd, uint8_t errorBitmask) {
  uint8_t resp[4];
  resp[0] = 4; resp[1] = cmd; resp[2] = errorBitmask;
  resp[3] = resp[0] ^ resp[1] ^ resp[2];
  Serial1.write(resp, 4);
}

// 記住「而家傳輸緊嘅係邊個 motionId」——由 handleWriteMotionId 喺
// 收到真正 contentId（非 0xFFFF 清空指令）嗰刻設定，等隨後嗰個
// 冇帶 payload 嘅 SaveMotionId 知道應該 save 邊個 entry。
static uint16_t motionPendingId = 0xFFFF;  // 0xFFFF = 冇 pending 緊嘅真正 id
static bool motionPendingIsClear = false;  // true = 啱啱 WriteMotionId(0xFFFF) 清空咗，
                                            // 下一個 SaveMotionId 應該當做「清空確認」，
                                            // 而唔係當做「冇對應 pending id」嘅錯誤
static uint8_t motionCurrentWritePage = 0; // 而家 WriteMotionData 寫緊邊個 page 嘅 staging
static uint8_t motionTransferStartPage = 0; // 呢輪傳輸嘅起始 page（暫時固定 0）

static void handleWriteMotionData(const uint8_t *payload, uint8_t payloadLen) {
  // 原裝封包格式: [LEN][CMD=0x1D][reserved=0x00][addrLo][addrHi][data...][checksum]
  // payload = pkt+2，即 payload[0]=reserved(固定0x00)，
  // payload[1]/[2]=addrLo/addrHi，payload[3:]=實際 data
  if (payloadLen < 3) { sendFlashResponse4(0x1D, MOTION_ERR_SIZE); return; }
  uint16_t inPageOffset = payload[1] | ((uint16_t)payload[2] << 8);
  uint8_t dataLen = payloadLen - 3;
  const uint8_t *data = payload + 3;

  if ((uint32_t)inPageOffset + dataLen > MOTION_FLASH_PAGE_SIZE) {
    sendFlashResponse4(0x1D, MOTION_ERR_ADDRESS);
    return;
  }

  memcpy(motionPageStaging + inPageOffset, data, dataLen);
  sendFlashResponse4(0x1D, MOTION_ERR_NONE);
}

static void handleSaveMotionData(const uint8_t *payload, uint8_t payloadLen) {
  // 原裝封包格式: [LEN][CMD=0x1E][reserved=0x00][page][checksum]
  // payload = pkt+2，即 payload[0]=reserved(固定0x00)，payload[1]=page
  if (payloadLen < 2) { sendFlashResponse4(0x1E, MOTION_ERR_SIZE); return; }
  uint8_t page = payload[1];

  if (page >= MOTION_FLASH_PAGE_COUNT) { sendFlashResponse4(0x1E, MOTION_ERR_ADDRESS); return; }

  bool ok = motionFlashProgramPage(page, motionPageStaging);
  motionCurrentWritePage = page + 1;  // 下一份 128-byte chunk 預設寫去下一個 page
  memset(motionPageStaging, 0xFF, MOTION_FLASH_PAGE_SIZE);  // 清返 staging，避免殘留舊資料

  sendFlashResponse4(0x1E, ok ? MOTION_ERR_NONE : MOTION_ERR_FLASH);
}

static void handleReadFlashBuffer(const uint8_t *payload, uint8_t payloadLen) {
  // 原裝封包格式: [LEN][CMD=0x1C][reserved=0x00][page][checksum]
  if (payloadLen < 2) return;  // 冇齊夠參數，直接唔回應（等 host timeout）
  uint8_t page = payload[1];
  if (page >= MOTION_FLASH_PAGE_COUNT) return;

  static uint8_t respBuf[MOTION_FLASH_PAGE_SIZE + 4];
  respBuf[0] = 0xFF;  // sentinel：代表 1024-byte large payload（唔係真實長度，
                      // 因為 8-bit LEN byte 表達唔到 1028）
  respBuf[1] = 0x1C;
  motionFlashRead((uint32_t)page * MOTION_FLASH_PAGE_SIZE, respBuf + 2, MOTION_FLASH_PAGE_SIZE);
  uint8_t xorCalc = 0;
  for (uint16_t i = 0; i < MOTION_FLASH_PAGE_SIZE + 2; i++) xorCalc ^= respBuf[i];
  respBuf[MOTION_FLASH_PAGE_SIZE + 2] = xorCalc;
  Serial1.write(respBuf, MOTION_FLASH_PAGE_SIZE + 3);
}

static void handleWriteMotionId(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen < 5) { sendFlashResponse4(0x02, MOTION_ERR_SIZE); return; }
  uint16_t contentId = payload[3] | ((uint16_t)payload[4] << 8);

  if (contentId == 0xFFFF) {
    // 清空流程：傳輸即將開始，reset write-page 追蹤器同 staging buffer。
    // 跟住嚟緊嘅 SaveMotionId 應該當做「清空確認」處理，唔係當做
    // 冇對應 pending id 嘅錯誤。
    motionCurrentWritePage = 0;
    motionTransferStartPage = 0;
    motionPendingId = 0xFFFF;
    motionPendingIsClear = true;
    memset(motionPageStaging, 0xFF, MOTION_FLASH_PAGE_SIZE);
    sendFlashResponse4(0x02, MOTION_ERR_NONE);
    return;
  }

  MotionIdEntry *entry = motionIdAllocOrFind(contentId);
  if (!entry) { sendFlashResponse4(0x02, MOTION_ERR_COUNT); return; }
  entry->startPage = motionTransferStartPage;
  motionPendingId = contentId;  // 記住呢個 id，等 SaveMotionId 嚟緊填 totalLen
  motionPendingIsClear = false;

  sendFlashResponse4(0x02, MOTION_ERR_NONE);
}

static void handleSaveMotionId(const uint8_t *payload, uint8_t payloadLen) {
  (void)payload; (void)payloadLen;  // 原裝 SaveMotionId 本身冇帶 payload

  if (motionPendingIsClear) {
    // 對應啱啱嗰個 WriteMotionId(0xFFFF) 清空指令，呢個 SaveMotionId
    // 純粹係「清空確認」，唔需要搵任何 entry，直接回成功。
    motionPendingIsClear = false;
    sendFlashResponse4(0x04, MOTION_ERR_NONE);
    return;
  }

  if (motionPendingId == 0xFFFF) {
    // 冇對應緊嘅 pending id（可能傳輸流程亂咗序），回錯誤
    sendFlashResponse4(0x04, MOTION_ERR_COUNT);
    return;
  }

  MotionIdEntry *entry = motionIdFind(motionPendingId);
  if (!entry) { sendFlashResponse4(0x04, MOTION_ERR_COUNT); return; }

  entry->totalLen = (uint16_t)((uint32_t)motionCurrentWritePage * MOTION_FLASH_PAGE_SIZE);
  motionPendingId = 0xFFFF;  // 完成，清走 pending 狀態

  sendFlashResponse4(0x04, MOTION_ERR_NONE);
}

static void handleStartMotion(const uint8_t *payload, uint8_t payloadLen) {
  // 原裝封包格式有兩種、共用 CMD=0x1F：
  //   StartMotion: [LEN][CMD][reserved=0x00][startPage][checksum]
  //                -> payload[0]=reserved(0x00), payload[1]=startPage
  //   StopMotion:  [LEN][CMD][0x08][0x01][checksum]  (sub-command flag，
  //                冇 reserved byte，直接係 0x08 0x01)
  if (payloadLen >= 2 && payload[0] == 0x08 && payload[1] == 0x01) {
    motionPlaybackStop();
    sendFlashResponse4(0x1F, MOTION_ERR_NONE);
    return;
  }

  if (payloadLen < 2) { sendFlashResponse4(0x1F, MOTION_ERR_SIZE); return; }
  uint8_t startPage = payload[1];
  if (startPage >= MOTION_FLASH_PAGE_COUNT) { sendFlashResponse4(0x1F, MOTION_ERR_ADDRESS); return; }

  // 直接由 page 播放，總長度用「storage 區尾」做上限，實際結束
  // 由 motionFlashReadNextPacket() 遇到 0xFF/0x00 結尾標記自然停止。
  motionPlaybackStart(startPage, (uint16_t)MOTION_FLASH_RESERVED_SIZE);
  sendFlashResponse4(0x1F, MOTION_ERR_NONE);
}

// 執行一個已經收齊、通過 checksum 驗證嘅封包
static void pmaExecutePacket(const uint8_t *pkt, uint8_t len, bool fromFlashPlayback) {
  uint8_t cmd = pkt[1];
  const uint8_t *payload = pkt + 2;
  uint8_t payloadLen = len - 3;  // 扣埋 LEN(1)+CMD(1)+checksum(1)

  if (cmd == 0x18) {
    pmaHandleCmd18(payload, payloadLen);
    return;
  }
  if (cmd == 0x19) {
    pmaHandleCmd19(payload, payloadLen);
    return;
  }

  // Flash 管理 command（0x02/0x04/0x1C/0x1D/0x1E/0x1F）只應該由 host
  // 透過 pmaReceiveUpdate() 即時 serial 觸發，絕對唔應該由 Flash
  // playback engine（motionPlaybackUpdate）觸發——因為 Flash 入面
  // 實際存住嘅係一連串 0x18/0x19 servo 動作封包，呢啲 CMD byte 空間
  // 同 Flash 管理協議完全撞晒（例如原裝 streaming 協議入面 0x02 本身
  // 已經有「迴圈起點」嘅意思），如果照樣 dispatch，播放緊嘅動作
  // packet 一旦個 CMD byte 啱啱好係 0x02/0x04 等數值，就會被錯誤咁
  // 觸發 WriteMotionId/SaveMotionId 呢類管理邏輯，仲會經 Serial1 送
  // 一堆意外嘅 response 出嚟，令播放頓卡、唔順。
  if (fromFlashPlayback) {
    return;  // 其他 CMD 對 Flash playback 冇意義，直接跳過
  }

  if (cmd == 0x1D) {
    handleWriteMotionData(payload, payloadLen);
  } else if (cmd == 0x1E) {
    handleSaveMotionData(payload, payloadLen);
  } else if (cmd == 0x1C) {
    handleReadFlashBuffer(payload, payloadLen);
  } else if (cmd == 0x02) {
    handleWriteMotionId(payload, payloadLen);
  } else if (cmd == 0x04) {
    handleSaveMotionId(payload, payloadLen);
  } else if (cmd == 0x1F) {
    handleStartMotion(payload, payloadLen);
  }
  // 其他 command：已知但唔影響郁動作，冇對應處理，略過
}

// 由 loop() 每次 iteration call 一次；逐 byte 儲入 buffer，
// 儲夠一個完整封包先驗 checksum 同執行，然後重置去等下一個封包。
void pmaReceiveUpdate() {
  // 注意：呢度故意唔用 "while (Serial1.available() > 0)" 一路清到冧晒為止。
  //
  // 根本原因：一個 0x18 封包最多可以帶 21 隻 servo，pmaExecutePacket()
  // 會逐隻 servo 真實做 ICS bus 讀寫（半雙工 + 10ms timeout + retry-once），
  // 21 隻走完隨時要 20~40ms。如果呢段時間內 sender 端（例如網頁版 BT
  // sender）已經連續送緊下一個封包嘅 byte，STM32 硬件 UART RX FIFO
  // 得幾個 byte 深，好快爆滿，導致中間 byte 被丟棄或者覆蓋。
  //
  // 一旦漏咗一個 byte，狀態機可能停留喺 PMA_RECV_BODY 永久等一個
  // 唔會再嚟嘅 byte（因為 pmaPktFilled 永遠追唔上 pmaPktLen），
  // 令機械人「行到一半完全定住」——呢個唔係死機，而係 UART 協議層面
  // 卡死，冇任何 ASCII 指令可以打斷佢，睇落好似要重開電源先解決。
  //
  // 修法：每次 call pmaReceiveUpdate() 最多完整處理一個 packet 就
  // return，將處理 servo write 嘅時間攤分返去下一次 loop() iteration，
  // 中間畀 UART 硬件 FIFO 有機會被清走，避免爆 buffer。
  uint8_t packetsProcessedThisCall = 0;
  const uint8_t MAX_PACKETS_PER_CALL = 1;

  while (Serial1.available() > 0 && packetsProcessedThisCall < MAX_PACKETS_PER_CALL) {
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
      // 01=內部變數查詢(電量等) 02=WriteMotionId/迴圈起點 04=SaveMotionId/toggle
      // 05=停止 07=迴圈終點 15/17=PMA專用 18=多軸角度 19=Speed/Stretch
      // 1C=ReadFlashBuffer 1D=WriteMotionData 1E=SaveMotionData/LED控制
      // 1F=StartMotion/StopMotion/播放已存動作
      // 漏咗 0x01 曾經令 Unity 嘅 RequestBatteryRemain()（07 01 00 02
      // 00 02 06，每 10 秒定時發一次）被誤判做 ASCII 字元塞入
      // inputBuffer，污染咗接收狀態，係之前「連續送信一開就冇反應」嘅
      // 根本原因。
      //
      // 2026-07 補充：加入 Flash-based motion 協議之後，發現 0x1D
      // (WriteMotionData) 冇被列入呢個集合，導致成份 134-byte 嘅
      // WriteMotionData 封包完全冧唔到 binary 判斷，被逐 byte 當做
      // ASCII 字元塞入 inputBuffer，令機械人隨機郁咗然後卡死——
      // 呢個係「上傳未完成就郁咗一下就停」呢個徵狀嘅根本原因。
      bool isPmaCmd = (secondByte == 0x01 || secondByte == 0x02 ||
                       secondByte == 0x04 || secondByte == 0x05 ||
                       secondByte == 0x07 || secondByte == 0x15 ||
                       secondByte == 0x17 || secondByte == 0x18 ||
                       secondByte == 0x19 || secondByte == 0x1C ||
                       secondByte == 0x1D || secondByte == 0x1E ||
                       secondByte == 0x1F);
      bool looksLikePmaPacket = isPmaCmd && (peekLen >= 3 && peekLen <= PMA_PKT_MAX_LEN);

      if (looksLikePmaPacket) {
        pmaPktLen = peekLen;
        pmaRecvState = PMA_RECV_BODY;
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

        // 寬鬆驗證：Unity 官方 App（continuousMode）本身唔會正確計算
        // checksum，固定送出 0x00 佔位值，並非傳輸損毀——payload 內容
        // 本身完全自洽（合理嘅 servo ID、正常角度範圍）。容許
        // recv==0x00 且 payload 結構完整（扣除 LEN/CMD/保留位/SPD/
        // checksum 之後，剩餘 byte 啱啱好整除 3）嘅封包照樣執行，
        // 其餘 checksum 唔啱嘅封包仍然拒絕。
        bool structurallyValid = (pmaPktLen >= 8) && ((pmaPktLen - 5) % 3 == 0);
        bool passedChecksum = (xorCalc == parity);
        bool passedLenientMode = (parity == 0x00) && structurallyValid;

        if (passedChecksum || passedLenientMode) {
          pmaExecutePacket(pmaPktBuf, pmaPktLen);
          pmaPacketOkCount++;
        } else {
          pmaPacketFailCount++;
        }

        pmaRecvState = PMA_RECV_IDLE;
        pmaPktFilled = 0;
        packetsProcessedThisCall++;  // 呢個 packet（可能耗時 20~40ms）處理完，
                                      // 即刻 return 返去 loop()，等 UART FIFO 有機會被清

        // 回送 1-byte ACK：0x06（ASCII ACK）代表呢個 packet 已經完整
        // 執行完（包括所有 servo write）。Sender 端可以揀跟呢個 ACK
        // 嚟做 flow control（收到先送下一個），而唔係淨係計 SPD tick
        // 估時間——因為真實 servo write 所需時間可能同 SPD tick
        // 假設唔一致，盲目照送會累積令 RX buffer 谷爆。
        // 冇支援 ACK 嘅 sender（例如舊版 python script）唔會理呢個
        // byte，唔影響相容性。
        //
        // 注意：Flash-related command（0x1D/0x1E/0x1C/0x02/0x04/0x1F）
        // 已經跟原裝協議自己送咗一個完整嘅 4-byte（或 ReadFlashBuffer
        // 嘅 1028-byte large payload）response，唔可以再喺呢度追加呢個
        //額外 0x06 byte——如果加埋，host 端會將呢個 0x06 當做下一個
        // response 嘅 LEN byte，令成個 response parser 錯位（例如
        // SaveMotionId 本身應該回 4-byte 但因為前一個 command 遺留低
        // 嘅 0x06 byte，被誤判做一個 6-byte packet，扯埋自己 response
        // 嘅頭 2 個 byte 當做 payload，最終 errorBitmask 顯示錯亂嘅
        // COMMAND(0x04) 之類假錯誤）。
        uint8_t executedCmd = (pmaPktLen > 1) ? pmaPktBuf[1] : 0xFF;
        bool isFlashCmd = (executedCmd == 0x1D || executedCmd == 0x1E ||
                           executedCmd == 0x1C || executedCmd == 0x02 ||
                           executedCmd == 0x04 || executedCmd == 0x1F);
        if (!isFlashCmd) {
          Serial1.write((uint8_t)0x06);
        }
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

// ===== 處理ASCII指令 =====
bool processASCIICommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("FREE ")) return processFreeCommand(cmd);

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
    Serial1.println(pmaPacketFailCount);
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
  Serial1.println(F("\n=== 步行與動作指令 ==="));
  Serial1.println(F("WALK F/B/L/R  : 開始向前/後/左轉/右轉行走，直到收到 STOP"));
  Serial1.println(F("STOP (ST)     : 立即安全停止當前動作並回到 Home Point"));
  Serial1.println(F("\n=== 單軸微調 (ASCII) ==="));
  Serial1.println(F("S [群組] [ID] [角度] [速度] : 設定單一伺服 (例: S HV 1 8000 64)"));
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