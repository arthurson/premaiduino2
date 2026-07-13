#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <Arduino.h>
#include <IcsHardSerialClass.h>

// ===== 伺服參數預設值 =====
// 下面數值係「原廠出廠設定值」，唔係範圍上下限；合法範圍已喺 ICS3.5
// 官方 Software Manual 確認（Stretch/Speed: 1~127，電流制限: 1~63，
// 溫度制限: 1~127），同 safeSetSpd/Strc/Cur/Tmp() 入面嘅範圍檢查一致。
#define DEFAULT_SPEED_HV 127  // HV 伺服速度 (原廠設定值，範圍 1~127)
#define DEFAULT_SPEED_MV 100  // MV 伺服速度 (原廠設定值，範圍 1~127)
#define DEFAULT_STRETCH_HV 60   // HV 伺服 Stretch 硬度 (原廠設定值，範圍 1~127)
#define DEFAULT_STRETCH_MV 100  // MV 伺服 Stretch 硬度 (原廠設定值，範圍 1~127)
#define DEFAULT_CUR_HV 20    // HV 伺服電流制限值 (原廠設定值，範圍 1~63)
#define DEFAULT_CUR_MV 40    // MV 伺服電流制限值 (原廠設定值，範圍 1~63)
#define DEFAULT_TMP_HV 80    // HV 伺服溫度制限值 (原廠設定值，範圍 1~127，數值越細代表溫度越高)
#define DEFAULT_TMP_MV 40    // MV 伺服溫度制限值 (原廠設定值，範圍 1~127，數值越細代表溫度越高)

// ===== 25軸伺服資訊 =====
// 呢個係純資料表，唔含任何邏輯。firmware（.ino）係呢份資料嘅唯一權威——
// homePosition/minAngle/maxAngle 呢啲數值係手動實測校準，唔可以被 HTML
// 工具或者其他檔案獨立覆寫。要改任何一隻伺服嘅規格，直接改呢度就得。
//
// include 呢個檔案之前，主 .ino 要已經定義咗：
//   - icsHV / icsMV（IcsHardSerialClass 物件）
// 伺服參數預設值（DEFAULT_SPEED/STRETCH/CUR/TMP）已經喺呢個檔案自己定義。
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

inline ServoInfo servoList[] = {
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
  { 23, 3, 7500, 7500, DEFAULT_SPEED_MV, &icsMV, "頭部側傾", 6900, 8100, false, true, 7500 },  // 未駁線
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

#endif