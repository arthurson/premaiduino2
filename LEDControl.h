#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>

// ===== LED 腳位定義 =====
// 名稱直接反映 schematic 腳位（PA7/PB0/PB1），唔叫 RED/GREEN/BLUE，
// 因為呢三隻腳實際駁嘅顏色同腳位編號冇對應（見下面顏色校正表）。
#define LED_PIN_PA7 PA7
#define LED_PIN_PB0 PB0
#define LED_PIN_PB1 PB1

// ===== 呼吸燈速度定義 =====
#define BREATH_SPEED 4

// ===== LED 硬件校正表（實測窮舉 8 種 HIGH/LOW 組合確認，2026-07）=====
// active-low：LOW = 開，HIGH = 熄。
// 物理顏色對應腳位：
//   物理 R (紅) <- LED_PIN_PB0
//   物理 G (綠) <- LED_PIN_PB1
//   物理 B (藍) <- LED_PIN_PA7
// 呢個係 setLEDRaw() 入面「叫乜色見乜色」嘅唯一根據，
// 如果之後換板/換LED，只需要改呢三行。
#define LED_PHYS_R_PIN LED_PIN_PB0
#define LED_PHYS_G_PIN LED_PIN_PB1
#define LED_PHYS_B_PIN LED_PIN_PA7

// ===== LED 控制 =====
inline void initLED() {
  pinMode(LED_PIN_PA7, OUTPUT);
  pinMode(LED_PIN_PB0, OUTPUT);
  pinMode(LED_PIN_PB1, OUTPUT);
  digitalWrite(LED_PIN_PA7, HIGH);   // active-low: HIGH = 熄
  digitalWrite(LED_PIN_PB0, HIGH);
  digitalWrite(LED_PIN_PB1, HIGH);
}

// 底層 helper：真正驅動三隻物理腳，處理 active-low
// r_on/g_on/b_on: 1 = 想要嗰隻色開, 0 = 想要嗰隻色熄
// 每次都強制 pinMode(OUTPUT)：breathLED() 用 analogWrite() 會將
// 個 pin 切去 timer output-compare(PWM) mode，如果之後淨係
// digitalWrite() 而冇重設 pinMode，個 pin 有機會卡喺 PWM mode
// 唔生效，所以呢度每次都強制拉返做純 GPIO output。
inline void setLEDRaw(bool r_on, bool g_on, bool b_on) {
  pinMode(LED_PHYS_R_PIN, OUTPUT);
  pinMode(LED_PHYS_G_PIN, OUTPUT);
  pinMode(LED_PHYS_B_PIN, OUTPUT);
  digitalWrite(LED_PHYS_R_PIN, r_on ? LOW : HIGH);
  digitalWrite(LED_PHYS_G_PIN, g_on ? LOW : HIGH);
  digitalWrite(LED_PHYS_B_PIN, b_on ? LOW : HIGH);
}

inline void setLEDRed() {
  setLEDRaw(true, false, false);
}

inline void setLEDGreen() {
  setLEDRaw(false, true, false);
}

inline void setLEDBlue() {
  setLEDRaw(false, false, true);
}

inline void setLEDPurple() {
  setLEDRaw(true, false, true);
}

inline void setLEDCyan() {
  setLEDRaw(false, true, true);
}

inline void setLEDYellow() {
  setLEDRaw(true, true, false);
}

inline void setLEDWhite() {
  setLEDRaw(true, true, true);
}

inline void setLEDOff() {
  setLEDRaw(false, false, false);
}

// 非阻塞式呼吸燈：每次 call 淨係更新一格 brightness，唔會 delay()。
// 用喺 loop() 入面持續 call，唔會擋住 command 處理。
// r_on/g_on/b_on: 邊隻 channel 參與呼吸（同 setLEDRaw() 同一套介面），
// 例如 (true,false,false)=紅呼吸、(true,true,false)=黃呼吸、
// (true,true,true)=白呼吸，7 色（R/G/B/黃/青/紫/白）都可以用呢個做。
// period_ms：一次完整呼吸（暗->光->暗）嘅總時間
// 呼吸最暗都唔會跌到全黑，維持喺 BREATH_MIN_BRIGHTNESS 或以上，
// 咁樣先睇得出顏色，唔會有一刻好似熄咗咁。
#define BREATH_MIN_BRIGHTNESS 40  // 0-255，數值越大最暗嗰刻越光
inline void breathLEDRaw(bool r_on, bool g_on, bool b_on, unsigned long period_ms) {
  unsigned long t = millis() % period_ms;
  float phase = (float)t / (float)period_ms;  // 0.0 ~ 1.0
  // 三角波：0->1->0，令光暗上落對稱
  float triangle = (phase < 0.5f) ? (phase * 2.0f) : (2.0f - phase * 2.0f);
  // 將 0~1 重新映射去 BREATH_MIN_BRIGHTNESS~255，確保唔會跌到全黑
  int brightness = BREATH_MIN_BRIGHTNESS +
      (int)(triangle * (float)(255 - BREATH_MIN_BRIGHTNESS));
  int duty = 255 - brightness;  // active-low: 反轉 duty cycle

  // 冇參與呼吸嘅 channel 要維持全熄（255=active-low下嘅"熄"）
  analogWrite(LED_PHYS_R_PIN, r_on ? duty : 255);
  analogWrite(LED_PHYS_G_PIN, g_on ? duty : 255);
  analogWrite(LED_PHYS_B_PIN, b_on ? duty : 255);
}

inline void breathLEDRedNB(unsigned long period_ms)    { breathLEDRaw(true,  false, false, period_ms); }
inline void breathLEDGreenNB(unsigned long period_ms)  { breathLEDRaw(false, true,  false, period_ms); }
inline void breathLEDBlueNB(unsigned long period_ms)   { breathLEDRaw(false, false, true,  period_ms); }
inline void breathLEDYellowNB(unsigned long period_ms) { breathLEDRaw(true,  true,  false, period_ms); }
inline void breathLEDCyanNB(unsigned long period_ms)   { breathLEDRaw(false, true,  true,  period_ms); }
inline void breathLEDPurpleNB(unsigned long period_ms) { breathLEDRaw(true,  false, true,  period_ms); }
inline void breathLEDWhiteNB(unsigned long period_ms)  { breathLEDRaw(true,  true,  true,  period_ms); }

#endif