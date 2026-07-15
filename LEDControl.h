#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include <Arduino.h>

// ===== LED 腳位定義 =====
#define LED_PIN_PA7 PA7
#define LED_PIN_PB0 PB0
#define LED_PIN_PB1 PB1

// ===== 呼吸燈速度定義 =====
#define BREATH_SPEED 4
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