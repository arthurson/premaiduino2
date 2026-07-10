#ifndef VOLTAGE_MONITOR_H
#define VOLTAGE_MONITOR_H

#include <Arduino.h>
#include "LEDControl.h"

// ===== 電壓檢查相關定義 =====
#define VOLTAGE_PIN PA0
#define VOLTAGE_WARNING 9.5    // 低過此值 = 黃色警告
#define VOLTAGE_SHUTDOWN 9.0   // 低過此值 = 紅色 + 自動關機
#define VOLTAGE_CHECK_INTERVAL 10000
#define VOLTAGE_SAMPLES 10
#define VOLTAGE_DIVIDER_RATIO 22.9

// 主檔（Pre-maiduino2.ino）定義，緊急關機時要即停 table_walk
extern void tableWalkSafeStop();

// 主檔（Pre-maiduino2.ino）定義：safeSetPos() 每次真正驅動 servo 都會
// 更新呢個時間戳，俾 updateActivityLED() 判斷而家係咪「有動作進行緊」
extern unsigned long lastActivityMs;
#define ACTIVITY_LED_HOLD_MS 300  // 動作停咗之後，忙碌LED仲維持幾耐先切返待機

// 主檔（Pre-maiduino2.ino）定義：FREE ALL 之後 = true，
// 下次有 safeSetPos() 動作先自動解除
extern bool isFreeMode;

// ===== 電壓數據結構 =====
struct VoltageData {
  float currentVoltage;
  float minVoltage;
  float maxVoltage;
  unsigned long lastCheckTime;
  bool warningActive;
  bool shutdownInitiated;
};

inline VoltageData voltageData;

// 手動 LED 測試模式開關：true 時 checkVoltage() 唔會自動改動 LED 顏色，
// 俾 LED 測試指令 (LED RED/GREEN/BLUE/...) 可以穩定顯示，唔會被電壓
// 監測每 5 秒一次嘅自動 setLEDBlue()/setLEDRed() 覆蓋。
// 由 processCommand() 嘅 LED 指令 set 做 true；打 LED AUTO 先返做 false。
inline bool ledManualOverride = false;

// ===== 電壓檢查函式 =====
inline void initVoltageCheck() {
  pinMode(VOLTAGE_PIN, INPUT_ANALOG);
  voltageData.currentVoltage = 0.0;
  voltageData.minVoltage = 99.0;
  voltageData.maxVoltage = 0.0;
  voltageData.lastCheckTime = 0;
  voltageData.warningActive = false;
  voltageData.shutdownInitiated = false;
}

inline float readBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
    sum += analogRead(VOLTAGE_PIN);
    // [FIX 4] 移除 delay(1)：STM32 ADC 夠快，唔需要等待，省 10ms 阻塞
  }
  float average = (float)sum / VOLTAGE_SAMPLES;
  float voltageAtPin = (average / 4095.0) * 3.3;
  float actualVoltage = voltageAtPin * VOLTAGE_DIVIDER_RATIO;
  return actualVoltage;
}

inline void checkVoltage() {
  unsigned long now = millis();
  if (now - voltageData.lastCheckTime < VOLTAGE_CHECK_INTERVAL) return;

  voltageData.lastCheckTime = now;
  voltageData.currentVoltage = readBatteryVoltage();

  if (voltageData.currentVoltage > voltageData.maxVoltage) voltageData.maxVoltage = voltageData.currentVoltage;
  if (voltageData.currentVoltage < voltageData.minVoltage) voltageData.minVoltage = voltageData.currentVoltage;

  if (voltageData.currentVoltage < VOLTAGE_SHUTDOWN) {
    // ---- 危急：紅色 + 自動關機 ----
    if (!voltageData.warningActive) {
      Serial1.print(F("\n⚠️ 低電壓警告: "));
      Serial1.print(voltageData.currentVoltage);
      Serial1.println(F("V"));
      voltageData.warningActive = true;
    }
    if (!voltageData.shutdownInitiated) {
      Serial1.println(F("\n🚨 電壓過低！自動關機..."));
      voltageData.shutdownInitiated = true;
      tableWalkSafeStop();
      setLEDOff();
      while (1) delay(1000);
    }
  } else if (voltageData.currentVoltage < VOLTAGE_WARNING) {
    // ---- 警告：未到關機門檻 ----
    if (!voltageData.warningActive) {
      Serial1.print(F("\n⚠️ 電壓偏低: "));
      Serial1.print(voltageData.currentVoltage);
      Serial1.println(F("V"));
      voltageData.warningActive = true;
    }
  } else {
    // ---- 正常 ----
    if (voltageData.warningActive) {
      Serial1.println(F("✅ 電壓恢復正常"));
      voltageData.warningActive = false;
    }
  }
}

// ===== LED 狀態指示（優先順序由高到低）=====
// 喺 loop() 入面每次都要 call，非阻塞。全部狀態色都用呼吸效果顯示。
// - 電壓危急（<VOLTAGE_SHUTDOWN）：紅色（checkVoltage() 已經處理，呢度跳過）
// - 電壓警告（<VOLTAGE_WARNING）：黃色呼吸
// - 脫力模式（FREE ALL 之後）：紫色呼吸
// - 電壓正常 + 有動作進行緊（.pma/table_walk/FREE/S 等）：綠色呼吸（忙碌）
// - 電壓正常 + 靜止：青色呼吸（待機）
// LED 測試指令 (ledManualOverride=true) 時完全跳過，唔會覆蓋手動顏色。
#define STATUS_BREATH_PERIOD_MS 3000  // 狀態呼吸燈嘅完整週期（暗->光->暗）

inline void updateActivityLED() {
  if (ledManualOverride) return;
  if (voltageData.shutdownInitiated) return;  // 關機中，交返俾 checkVoltage() 嘅紅燈

  if (voltageData.currentVoltage < VOLTAGE_SHUTDOWN) {
    setLEDRed();
    return;
  }
  if (voltageData.currentVoltage < VOLTAGE_WARNING) {
    breathLEDYellowNB(STATUS_BREATH_PERIOD_MS);
    return;
  }
  if (isFreeMode) {
    breathLEDPurpleNB(STATUS_BREATH_PERIOD_MS);
    return;
  }

  bool isBusy = (millis() - lastActivityMs) < ACTIVITY_LED_HOLD_MS;
  if (isBusy) {
    breathLEDGreenNB(STATUS_BREATH_PERIOD_MS);
  } else {
    breathLEDCyanNB(STATUS_BREATH_PERIOD_MS);
  }
}

#endif