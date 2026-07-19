#ifndef PMA_PROTOCOL_H
#define PMA_PROTOCOL_H

#include <Arduino.h>
#include <string.h>
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_flash_ex.h>

// ===== .pma / ICS binary 封包協議 + Flash motion storage =====
// 呢個檔案包含兩大子系統：
//   1. .pma / ICS binary 封包播放器（即時 streaming，pmaReceiveUpdate()
//      逐 byte 非阻塞接收，解碼 0x18/0x19 等指令直接郁伺服）
//   2. STM32 內部 Flash motion storage（host 將成隻 .pma 檔案分 page
//      寫入 Flash，之後 MCU 自己逐個 packet 讀出嚟播放，同原裝協議
//      一致，唔再受 host 送 packet 節奏／UART buffer 影響）
//
// 依賴（include 呢個檔案之前，主 .ino 要已經定義咗）：
//   - ServoInfo, findServoByPmaId(), applyHomeOffset()
//   - safeSetPos(), safeSetSpd(), safeSetStrc()
//   - isFreeMode（全域變數）
//   - tableWalker（tw_walker_t）, tableWalkIsWalking(), tw_walker_init()
//   - moveAllServosToHome()（forward-declare 都得，實際定義喺 .ino）
//   - voltageData（VoltageMonitor.h 嘅 VoltageData 實例，0x01 電壓
//     查詢用；必須喺 include 呢個檔案之前已經 #include "VoltageMonitor.h"）

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
    isFreeMode = true;  // 同 FREE ALL 一樣，令 LED 轉紫色呼吸提示脫力中
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
// STM32F102CBT6 (128KB Flash)，page size = 1024 bytes（medium-density
// devices）。喺 Flash 尾部劃一個 40KB 嘅 motion storage 區，用固定
// 地址（Flash 尾 40KB），因為標準 STM32duino linker script 冇提供
// "code end" 符號畀我哋自動計。
// 只要 firmware .bin 大細冇超過 88KB（128KB - 40KB），呢個劃法
// 就唔會同 code 重疊（Arduino IDE 編譯完會顯示 "Sketch uses xxx
// bytes"，要細過 90112 bytes 先安全）。
//
// 注意：呢個 40-page 上限唔淨止影響 reserve 幾多 flash，仲會影響
// handleSaveMotionData()/handleStartMotion() 入面嘅有效 page 範圍
// 檢查（page >= MOTION_FLASH_PAGE_COUNT 就拒絕）。如果將來想再加
// 新嘅 .pma 檔案令總大細逼近 40KB，記得同步調大呢個數值同重新編譯，
// 否則 host 端會收到 MOTION_ERR_ADDRESS 錯誤。
#define STM32_FLASH_BASE            0x08000000UL
#define STM32_FLASH_TOTAL_SIZE      (128UL * 1024UL)   // 128KB chip
#define MOTION_FLASH_PAGE_SIZE      1024u               // 1 page = 1KB，跟原裝協議一致
#define MOTION_FLASH_PAGE_COUNT     40u
#define MOTION_FLASH_RESERVED_SIZE  (MOTION_FLASH_PAGE_COUNT * MOTION_FLASH_PAGE_SIZE)  // 40KB
#define MOTION_FLASH_START_ADDR     (STM32_FLASH_BASE + STM32_FLASH_TOTAL_SIZE - MOTION_FLASH_RESERVED_SIZE)
// = 0x08016000

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

  // ⚠ Flash erase/program 期間必須關閉中斷：HAL_FLASHEx_Erase()/
  // HAL_FLASH_Program() 執行緊嗰陣，Flash controller 會鎖死，CPU
  // 冧可以由 Flash 讀取任何指令（包括中斷向量表/ISR code）。如果
  // 呢段時間有中斷觸發（最大嫌疑：UART RX interrupt），CPU 跳去
  // interrupt handler 攞指令但讀唔到，會 bus fault/hard fault，
  // 甚至寫壞緊嘅 Flash sector（可能波及 bootloader 或主程式
  // code），令晶片開唔到機，要重新燒錄先返生。用
  // __disable_irq()/__enable_irq() 包住成個 erase+program 過程，
  // 確保唔會被中斷打斷。
  //
  // 副作用：關閉中斷期間 UART RX 停止接收（硬件 FIFO 短時間緩衝
  // 通常夠用，除非 erase+program 耗時過長），風險遠細過 Flash
  // 損壞，屬可接受取捨。
  __disable_irq();

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
  __enable_irq();
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

// ===== Servo 全速轉速估算（防止 SPD tick 太短、servo 未郁完就催下一個）=====
// KRS-3301 datasheet: 7.4V 時 0.14s/60°（全速，ICS speed=127 情況下嘅上限估算，
// 唔同型號/電壓實際會有落差，呢度取一個保守嘅參考值）。
// pulse↔degree: 8000 pulse ≈ 270 度（同 LegIK.h IK_PULSE_PER_DEGREE 一致）。
#define SERVO_DEG_PER_MS (60.0f / 140.0f)             // ≈0.4286 度/ms
#define SERVO_PULSE_PER_DEGREE (8000.0f / 270.0f)     // ≈29.63
#define SERVO_PULSE_PER_MS (SERVO_DEG_PER_MS * SERVO_PULSE_PER_DEGREE)  // ≈12.7 pulse/ms

// 讀一個 0x18 封包入面所有 servo 嘅目標角度，同佢哋各自
// currentTunePos（上一次送出嘅目標）比較，攞最大移動距離，
// 換算做「全速走完呢段距離最少要幾多 ms」。
// 呢個唔係計「應該用邊個 speed」，而係計「就算 servo 已經全速
// (DEFAULT_SPEED_HV/MV) 郁緊，都仲要幾耐先真係到得切」，用嚟做
// SPD tick 換算出嚟嘅等待時間嘅下限，避免 SPD 太短時 firmware
// 唔理 servo 有冇到就催緊下一個目標，造成「跳格」。
static uint32_t pmaMinTravelMsForPacket(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen < 2) return 0;
  uint16_t maxPulseDelta = 0;
  for (uint8_t i = 2; i + 2 < payloadLen; i += 3) {
    uint8_t pmaId = payload[i];
    uint16_t lo = payload[i + 1];
    uint16_t hi = payload[i + 2];
    uint16_t rawAngle = (hi << 8) | lo;
    if (rawAngle == 0) continue;  // 脫力指令，冇實際移動距離

    ServoInfo *servo = findServoByPmaId(pmaId);
    if (!servo) continue;

    uint16_t targetPos = applyHomeOffset(servo, rawAngle, servo->baselineCenter);
    uint16_t delta = (targetPos > servo->currentTunePos)
                      ? (targetPos - servo->currentTunePos)
                      : (servo->currentTunePos - targetPos);
    if (delta > maxPulseDelta) maxPulseDelta = delta;
  }
  if (maxPulseDelta == 0) return 0;
  return (uint32_t)((float)maxPulseDelta / SERVO_PULSE_PER_MS);
}

static void motionPlaybackUpdate() {
  if (motionPlayState != MOTION_PLAY_RUNNING) return;
  if ((long)(millis() - motionPlayNextTickMs) < 0) return;  // 未到時間

  uint8_t pkt[MOTION_FLASH_PAGE_SIZE > 255 ? 255 : MOTION_FLASH_PAGE_SIZE];
  uint8_t len = motionFlashReadNextPacket(pkt);
  if (len == 0) {
    motionPlayState = MOTION_PLAY_IDLE;  // 動作播放完（讀到 0xFF/0x00 結尾標記）
    return;
  }

  // 下限保護要喺 pmaExecutePacket() 之前計算，因為執行完之後
  // servo->currentTunePos 已經俾呢個 packet 嘅新目標蓋咗，再計
  // 距離會變成「新目標 vs 新目標」=0，失去保護意義。
  uint8_t cmd0 = (len > 1) ? pkt[1] : 0;
  uint32_t minTravelMs = (cmd0 == 0x18) ? pmaMinTravelMsForPacket(pkt + 2, len - 2) : 0;

  pmaExecutePacket(pkt, len, /*fromFlashPlayback=*/true);
  motionPlayOffset += len;

  // SPD tick 喺 0x18 封包嘅 payload[1]（即 pkt[3]，1 tick = 15ms）
  // 決定下一個封包幾時送；其他封包用最短 15ms 間距。
  uint8_t cmd = (len > 1) ? pkt[1] : 0;
  uint8_t spdTicks = (cmd == 0x18 && len > 3) ? pkt[3] : 1;
  if (spdTicks == 0) spdTicks = 1;
  uint32_t waitMs = (uint32_t)spdTicks * 15u;

  // SPD tick 換算出嚟嘅等待時間，唔可以短過 servo 全速走完呢個
  // packet 最大移動距離所需嘅時間，否則 servo 未到就催下一個
  // 目標，變成跳格。
  if (minTravelMs > waitMs) waitMs = minTravelMs;

  motionPlayNextTickMs = millis() + waitMs;
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

// 共用 helper：對 buf[0..len-2] 做 XOR checksum，寫入 buf[len-1]，
// 再經 Serial1 送出成個 buf[0..len-1]。封包尾 byte=checksum 呢個
// pattern喺 sendFlashResponse4/handleReadFlashBuffer/
// pmaHandleQueryServoInfo 三處重複，抽出嚟做一個共用 function。
static void sendWithXorChecksum(uint8_t *buf, uint16_t len) {
  uint8_t xorCalc = 0;
  for (uint16_t i = 0; i < len - 1; i++) xorCalc ^= buf[i];
  buf[len - 1] = xorCalc;
  Serial1.write(buf, len);
}

static void sendFlashResponse4(uint8_t cmd, uint8_t errorBitmask) {
  uint8_t resp[4] = { 4, cmd, errorBitmask, 0 };
  sendWithXorChecksum(resp, 4);
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
  sendWithXorChecksum(respBuf, MOTION_FLASH_PAGE_SIZE + 3);
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

// =========================================================
// ===== 0x01：讀取內部變數（監視／查詢）=====
// =========================================================
// 依據社群解析メモ「01コマンド詳細」／「一覧表」兩個工作表逆解
// 出嚟嘅官方協議，格式：
//   要求：[LEN=07][CMD=01][00][引數1][引數2][回覆長度LEN][checksum]
//   回覆：因應引數1 而不同，見下面各 sub-handler 註解。
//
// 呢個 command 原本淨係 Unity/官方 App／pMaitPlay.exe 用嚟定期
// 監視機體狀態（pMaitPlay.exe 嘅 timerPower 每 10 秒送一次
// "07 01 00 02 00 02 06" 查電壓）。你隻 firmware 之前只有
// receive-side 嘅 binary-detect 集合包含咗 0x01（見
// pmaReceiveUpdate() 嘅 isPmaCmd 判斷），但 pmaExecutePacket()
// 本身冇實現任何 0x01 handler，收到會直接落去「已知但唔處理」
// 嗰個 fall-through，靜靜哋冇回覆。而家補返呢一截，等原裝 App／
// pMaitPlay.exe 都可以查到你隻自製機嘅電量同伺服狀態。
//
// 只實現咗 xlsx 已經解讀出嚟、有實際意義嘅兩個 sub-command
// （0x02=電壓、0x05=伺服資訊）；其餘 sub-command（0x00/0x01/
// 0x03/0x04/0x06/0x07）喺原廠都仲未完全解讀（回覆固定值或者
// 用途未知嘅內部變數），冇連接你嘅硬件任何實際狀態，所以冇實現，
// 收到會直接冇回覆（同「未知 CMD」一致嘅保守做法）。

// ---- 0x02：HV 電壓查詢 ----
// 回覆：[LEN=06][CMD=01][00][rawLo][rawHi][checksum]
// 電壓(V) = raw / 216.0（xlsx 實測平均值喺 215.3~217.2 之間浮動，
// 216 係官方/社群公用嘅預設換算常數，同 pMaitPlay.exe 反編譯
// 出嚟嘅 PushRcvCmd() 用嘅除數一致）。
//
// 你隻機用緊嘅係 PA0 分壓 ADC（VoltageMonitor.h，
// VOLTAGE_DIVIDER_RATIO=22.9），同官方協議呢條 216 完全係唔同
// 硬件量測路徑，兩者冇任何算式關係——呢度純粹係將你已經量到嘅
// 真實電壓，反推一個「假設用官方比例會讀到幾多」嘅 raw 值，
// 令回覆封包格式同官方一致，等 pMaitPlay.exe/Unity 端可以直接
// 顯示啱嘅電壓數字，而唔使理解你內部用緊另一種 ADC 換算方式。
static void pmaHandleQueryVoltage() {
  uint16_t raw = (uint16_t)(voltageData.currentVoltage * 216.0f + 0.5f);
  uint8_t resp[6];
  resp[0] = 6;
  resp[1] = 0x01;
  resp[2] = 0x00;
  resp[3] = (uint8_t)(raw & 0xFF);        // rawLo
  resp[4] = (uint8_t)((raw >> 8) & 0xFF); // rawHi
  resp[5] = resp[0] ^ resp[1] ^ resp[2] ^ resp[3] ^ resp[4];
  Serial1.write(resp, 6);
}

// ---- 0x05：伺服資訊查詢（單一 ID）----
// 要求 payload：[引數1=05][引數2=目標pma-style ID]
// （xlsx 話 LEN 用 14 嘅倍數可以一次過攞多個連續 ID，但呢個
// 「連續區塊一次過讀」係假設伺服狀態儲存喺一段連續記憶體，
// 你隻 firmware 嘅 servoList[] 唔一定同 pma ID 順序連續排列
// -- 見 findServoByPmaId() 嘅 HV/MV 分段換算 -- 貿然照抄「一次
// 過畀幾嚿」會讀錯落隔籬伺服，所以呢度保守咁淨係實現「一次查
// 一隻」，host 想查多隻就分開幾次 0x01 request，行為上同官方
// 協議完全相容（官方 App 本身都可以逐隻查），淨係冇官方嗰個
// batch 捷徑。
//
// 回覆：[LEN][CMD折返=01][完成狀態=00][14-byte 伺服資訊][checksum]
// 14-byte 內容完全跟返 xlsx「01コマンド詳細」右側逐 byte 解構：
//   +00 伺服ID (pma-style, 1~0x27)
//   +01 有效旗標：0x80=有效／0x00=無效
//   +02..03 編碼器值（目前實際位置，LE）—— 用 currentTunePos
//   +04..05 Trim值（機體固有校正，2的補數）—— 我哋冇獨立 trim
//           概念，homePosition 已經包含咗呢個校正，故填 0
//   +06..07 指令值（收到嘅目標角度，脫力=0 或 3500~11000 範圍）
//           —— 同樣用 currentTunePos，因為你隻 firmware 冇分開
//           保存「指令值」同「編碼器值」兩個獨立狀態
//   +08..09 陀螺方向旗標：FF FF=不適用／02 05=pitch／02 07=roll
//           —— 淨係 HV5-14 呢類走路/平衡相關伺服先有意義，其餘
//           填 FF FF（不適用）
//   +0A..0B 陀螺倍率（2的補數）—— 你隻 firmware 冇呢個概念，填 0
//   +0C 速度(Speed)：0x00~0x7F，未接線=0
//   +0D 拉伸(Stretch)：0x00~0x7F
static void pmaHandleQueryServoInfo(uint8_t pmaId) {
  ServoInfo *servo = findServoByPmaId(pmaId);

  uint8_t body[14];
  body[0] = pmaId;
  body[1] = (servo && servo->enabled) ? 0x80 : 0x00;

  uint16_t pos = servo ? servo->currentTunePos : 0;
  body[2] = (uint8_t)(pos & 0xFF);
  body[3] = (uint8_t)((pos >> 8) & 0xFF);

  body[4] = 0x00;  // trim（冇獨立保存，homePosition 已內含校正）
  body[5] = 0x00;

  body[6] = body[2];  // 指令值：同編碼器值一致（冇分開保存兩種狀態）
  body[7] = body[3];

  body[8] = 0xFF;  // 陀螺方向旗標：預設不適用
  body[9] = 0xFF;
  body[10] = 0x00; // 陀螺倍率：冇此概念，填0
  body[11] = 0x00;

  body[12] = servo ? (servo->currentSpeed & 0x7F) : 0x00;
  body[13] = 0x00;  // stretch：你隻 firmware 冇獨立保存 stretch 狀態

  uint8_t resp[3 + 14 + 1];
  uint8_t totalLen = 3 + 14 + 1;
  resp[0] = totalLen;
  resp[1] = 0x01;  // CMD 折返
  resp[2] = 0x00;  // 完成狀態：0=成功（未搵到 servo 都當「查詢完成」，
                    // enabled旗標=0x00 已經表達咗「呢個ID冇嘢」）
  memcpy(resp + 3, body, 14);
  sendWithXorChecksum(resp, totalLen);
}

// ---- 0x01 總 dispatcher：payload[0]=引數1, payload[1]=引數2 ----
static void pmaHandleCmd01(const uint8_t *payload, uint8_t payloadLen) {
  if (payloadLen < 2) return;
  uint8_t arg1 = payload[0];
  uint8_t arg2 = payload[1];

  if (arg1 == 0x02 && arg2 == 0x00) {
    pmaHandleQueryVoltage();
  } else if (arg1 == 0x05) {
    pmaHandleQueryServoInfo(arg2);
  }
  // 其餘 sub-command（accel/gyro/WORD變數/DWORD變數等）未實現，
  // 冇對應你隻 firmware 任何實際狀態，故意唔回覆——同「未知 CMD」
  // 一致嘅保守做法，等日後真係需要先再逐個補。
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
  if (cmd == 0x01) {
    // 監視／查詢類指令喺 Flash playback 過程中冇意義（見下面
    // fromFlashPlayback 判斷嘅解釋），但同 0x18/0x19 一樣，係
    // 「隨時可以由 host 主動查詢」嘅指令，唔受 fromFlashPlayback
    // 隔離規則影響——只要唔係嚟自 Flash playback engine 就處理。
    if (!fromFlashPlayback) {
      pmaHandleCmd01(payload, payloadLen);
    }
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
  // 注意：呢度故意唔用 "while (Serial1.available() > 0)" 一路清到冧晒。
  // 一個 0x18 封包最多帶 21 隻 servo，pmaExecutePacket() 逐隻做 ICS
  // bus 讀寫，21 隻走完可能要 20~40ms。如果呢段時間 sender 持續送
  // 緊下一個封包，STM32 UART RX FIFO 好快爆滿、中間 byte 被丟棄，
  // 令狀態機永久卡喺 PMA_RECV_BODY 等一個唔會再嚟嘅 byte（機械人
  // 行到一半定住，要重開電源）。
  // 修法：每次 call 最多處理一個 packet 就 return，將 servo write
  // 時間攤分去下一次 loop() iteration，畀 UART FIFO 有機會被清走。
  uint8_t packetsProcessedThisCall = 0;
  const uint8_t MAX_PACKETS_PER_CALL = 1;

  while (Serial1.available() > 0 && packetsProcessedThisCall < MAX_PACKETS_PER_CALL) {
    uint8_t b = Serial1.peek();

    if (pmaRecvState == PMA_RECV_IDLE) {
      // 判斷呢個 byte 係咪合理嘅 .pma 封包 LEN：要睇多一個 byte（CMD）
      // 先可以確定，因為單靠一個 byte 嘅數值範圍會同 ASCII 指令撞埋。
      //
      // 例外：'\n' 要即刻處理，唔等第二個 byte——'\n' 代表一句 ASCII
      // 指令已完整，卡住唔處理會令指令要打兩次先有反應。除 '\n' 外
      // 一律保守咁等夠 2 個 byte 先判斷，避免逐 byte 到達嘅 binary
      // 封包被拆散當 ASCII 字元吞晒。
      if (Serial1.available() < 2) {
        if (b == '\n') {
          Serial1.read();
          inputBufferAppend('\n');
          processCommand(inputBuffer);
          inputBufferClear();
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
        inputBufferAppend(c);
        if (c == '\n') {
          processCommand(inputBuffer);
          inputBufferClear();
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

        // 回送 1-byte ACK：0x06 代表呢個 packet 已經完整執行完
        // （包括所有 servo write）。Sender 可以揀跟呢個 ACK 做
        // flow control，唔支援 ACK 嘅 sender 唔會理呢個 byte。
        //
        // 注意：Flash-related command（0x1D/0x1E/0x1C/0x02/0x04/0x1F）
        // 已經跟原裝協議自己送咗一個完整嘅 response，唔可以再喺呢度
        // 追加呢個額外 0x06 byte，否則 host 端 response parser 會錯位。
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

#endif