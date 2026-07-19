#ifndef BALANCE_GYRO_H
#define BALANCE_GYRO_H

#include <Arduino.h>
#include "LegIK.h"

// ===== Gyro 自動平衡 =====
// 總體流程沿用 Bioloid Premium 官方 gyro tutorial 嘅骨架（開機校正、
// joint offset 疊加落 homePosition、UseGyro 總開關、企定先套用）。
//
// 「幾多修正」由 PID 計（見下面 BalanceGains），「點樣分配落各關節」
// 由 LegIK.h 嘅 ankle strategy 全鏈 IK 反解（髖-膝-踝三關節）——
// 膝主動配合屈伸，關節之間嘅角度關係由腿部幾何
// （THIGH_LENGTH/SHIN_LENGTH/ANKLE_HEIGHT，見 LegIK.h）算出。
//
// 呢個模組只負責「算 offset」，唔負責送 servo——由呼叫方（.ino）
// 攞返呢啲 offset 之後，加落 homePosition 度，再用 safeSetPos() 送出去，
// 同 table_walk/.pma 的送法一致。

// ===== 總開關（對應官方 UseGyro）=====
// 企定時可以開；WALK 行走中建議關閉，避免同 table_walk 本身嘅
// x2/y2 姿態控制、以及行走中嘅 ROLL/PITCH 波形疊加打架。
inline bool balanceEnabled = false;

// ===== PID 輸出 → 水平位移換算 =====
// PID 輸出嘅「度」睇成「軀幹想歪返嘅角度」，用小角度近似
// (offset ≈ LEG_LENGTH * tan(θ) ≈ LEG_LENGTH * θ_rad) 轉做
// 「腳應該喺水平面移動幾多 mm」嚟抵消，然後交俾 LegIK.h 反解
// 關節角度。
//
// 用 LEG_LENGTH（腿總長）做槓桿臂長度，假設軀幹重心大約喺
// 髖部高度；如果實測跟得唔夠貼／太貼，可以獨立調呢條轉換嘅
// 比例，唔使動 PID gain 本身。
// 用 LegIK.h 已定義嘅 IK_PI_F（float 精度，非 Arduino 核心嘅
// double PI），確保全程停留喺 float，唔拉入 double soft-float
// routine。IK_PI_F/180.0f*LEG_LENGTH 都係編譯期常數，預先摺埋
// 等 BALANCE_DEG_TO_MM 每次展開淨係一個乘法。
#define BALANCE_MM_PER_DEG ((IK_PI_F / 180.0f) * (float)LEG_LENGTH)
#define BALANCE_DEG_TO_MM(deg) ((deg) * BALANCE_MM_PER_DEG)

// ===== PID Gain 參數（可用 WALKSET BALANCE 指令即時調） =====
// P：跟原本邏輯一樣，gain=1.0 即傾幾多度、個關節就跟住轉幾多度。
// I：消除穩態誤差（例如機械組裝唔完全對稱、long-term 靠某個方向傾）；
//    純 P 會停喺一個「非零殘留傾斜」度數唔郁，I 會慢慢將呢個殘留谷走。
// D：用陀螺角速度（imuData.gyroX/gyroZ，已經扣咗零偏）做阻尼，
//    唔對 pitch/roll 做數值微分——差分本身會放大雜訊，陀螺讀數
//    本身已經係「現成、乾淨」嘅角速度，直接用更穩。
//    D 項嘅作用：傾斜「正在快速惡化」時提早加大修正、
//    「傾斜正在自行變好」時提早減力，抑制純 P 嘅 overshoot / 持續震盪。
struct BalanceGains {
  float pitchKp = 0.5f;   // 前後傾 P：誤差角度 → 髖前後 + 踝前後（經 IK 反解）
  float pitchKi = 0.0f;   // 前後傾 I：預設 0，穩定後先慢慢加（建議由 0.01 開始試）
  float pitchKd = 0.0f;   // 前後傾 D：用 gyroZ 阻尼，預設 0，建議由 0.02~0.05 開始試

  float rollKp = 0.5f;    // 左右傾 P：誤差角度 → 髖側擺 + 踝側擺（經 IK 反解）
  float rollKi = 0.0f;    // 左右傾 I：預設 0
  float rollKd = 0.0f;    // 左右傾 D：用 gyroX 阻尼，預設 0

  // ---- 膝屈曲獨立 PID（控制 kneeBendDeg）----
  // 誤差訊號用 |pitch|（絕對值——膝屈曲唔分方向，淨係「傾斜幅度
  // 大就屈多啲」），獨立 gain 可以分開整定「側移跟得幾貼」同
  // 「膝屈得幾快」。預設全部 0（kneeBendDeg 恆常 0，等同完全打直）。
  float kneePitchKp = 0.0f;  // 前後傾幅度 → 膝屈曲角度（建議由 0.1~0.3 開始試）
  float kneePitchKi = 0.0f;
  float kneePitchKd = 0.0f;

  // 注意：冇 kneeRollKp/Ki/Kd —— roll 平面（HV5-HV13）結構上冇膝
  // 可屈（見 LegIK.h solveRollIK() 説明）。
};
inline BalanceGains balanceGains;

// ===== I term anti-windup 上限 =====
// I term 本身係「誤差隨時間累積」，如果機身長時間傾斜（例如企定
// 前傾未修正回來、或者 balance 啱啱先 enable 嗰陣仲未追到），
// 個累積值會不斷谷大，等到誤差終於反方向時，個 overshoot 會非常誇張
// （即 "integral windup"）。用呢個上限 clamp 住個積分值嘅範圍，
// 單位同 pitch/roll 一樣係「度」（clamp 喺乘 gain/轉 pulse 之前），
// 相當於「最多俾 I term 假設機身傾咗幾多度嘅誤差歷史」。
#define BALANCE_INTEGRAL_CLAMP_DEG 10.0f

// ===== PID 內部狀態（積分項）=====
// 呢四個值要喺 balanceEnabled 關閉／行走中／未企定時清零，否則
// enable 前/行走中累積落嘅舊誤差會喺下次 enable 一開波就爆疊落去。
inline float pitchIntegral = 0.0f;
inline float rollIntegral = 0.0f;
inline float kneePitchIntegral = 0.0f;

// 喺 .ino 度，行走開始/balance 關閉時 call 呢個，確保 I term 唔會
// 帶住舊嘅、唔相關語境嘅累積誤差入返新一輪企定修正。
inline void resetBalanceIntegral() {
  pitchIntegral = 0.0f;
  rollIntegral = 0.0f;
  kneePitchIntegral = 0.0f;
}

// ===== 方向翻轉開關（實機測試用） =====
// table_walk.c 嘅正負號係喺「行走波形」語境下驗證，唔保證同「靜態
// 平衡修正」語境嘅符號需求一致（例如「傾右要郁邊個方向先叫企返直」
// 仍然要實機驗證）。如果實測方向反咗，將對應 *_INVERT 改做 -1 即可，
// 唔使動下面公式同 table_walk.c 本身。
#define BALANCE_PITCH_INVERT 1   // 髖前後/踝前後 (HV7/8/11/12) 方向，反咗改做 -1
#define BALANCE_ROLL_INVERT  1   // 髖側擺/踝側擺 (HV5/6/13/14) 方向，反咗改做 -1

// ===== 除錯用：逐 servo 實際套用 log 開關 =====
// 開咗會喺每次 IMU tick 套用 balance offset 嗰刻，經 Serial1 印出
// 每隻 servo 實際送咗嘅目標角度。預設關閉，因為 IMU tick 頻率密，
// 長開會洗版；用 "BALANCE LOG ON/OFF" 指令切換。
inline bool balanceLogEnabled = false;

// ===== 除錯用：Arduino IDE Serial Plotter 格式輸出開關 =====
// BALANCE LOG 用嘅係人手可讀格式（"HV5=7500"，用 = 號），Arduino
// Serial Plotter 認唔到 = 號，畫唔到圖。呢個開關輸出 Plotter 接受
// 嘅 "label:數值,label:數值,..." 格式，用 "BALANCE PLOT ON/OFF"
// 指令切換，同 BALANCE LOG 獨立、可以分開開關。輸出邏輯喺 .ino
// 入面（同 BALANCE LOG 同一個位置，跟住 balancePlotEnabled 判斷）。
inline bool balancePlotEnabled = false;

// ===== 計算結果 =====
// hip-knee-ankle 全鏈 IK，膝主動屈伸做到「軀幹水平位移、垂直
// 高度不變」。
struct BalanceOffsets {
  int hipFBR;    // 右髖前後 (HV7)
  int hipFBL;    // 左髖前後 (HV8)
  int kneeFBR;   // 右膝屈伸 (HV9)
  int kneeFBL;   // 左膝屈伸 (HV10)
  int ankleFBR;  // 右踝前後 (HV11)
  int ankleFBL;  // 左踝前後 (HV12)
  int hipLRR;    // 右髖側擺 (HV5)
  int hipLRL;    // 左髖側擺 (HV6)
  int ankleLRR;  // 右踝側擺 (HV13)
  int ankleLRL;  // 左踝側擺 (HV14)
};

// pitch/roll 單位：度，讀自 imuData.pitch / imuData.roll。
// gyroPitchRate/gyroRollRate 單位：度/秒，直接讀 imuData.gyroZ（pitch軸）
// 同 imuData.gyroX（roll軸）——同 MPU6050IMU.h 入面 complementary filter
// 用嘅係同一組軸對應（Z軸角速度→pitch，X軸角速度→roll），保持一致。
// dt 單位：秒，由 caller（.ino）傳入呢次同上次 IMU tick 之間嘅實際間隔，
// 用嚟做 I term 積分；理論上同 IMU_UPDATE_INTERVAL_MS 一致（50Hz≈0.02s），
// 但傳實際量度值而唔係寫死常數，可以避免 loop 偶爾延遲時 I term 算錯。
//
// 內部流程：PID（誤差角度 → 控制輸出角度）→ 換算做水平位移 mm →
// LegIK.h 反解做髖/膝/踝角度 → 轉返做 pulse offset。
inline void computeBalanceOffsets(float pitch, float roll,
                                   float gyroPitchRate, float gyroRollRate,
                                   float dt, BalanceOffsets &out) {
  if (!balanceEnabled) {
    out.hipFBR = out.hipFBL = 0;
    out.kneeFBR = out.kneeFBL = 0;
    out.ankleFBR = out.ankleFBL = 0;
    out.hipLRR = out.hipLRL = 0;
    out.ankleLRR = out.ankleLRL = 0;
    resetBalanceIntegral();  // 冇 enable 時保持清零，防止 enable 一開波就有殘留 I
    return;
  }

  // dt 唔可靠（例如剛啟動、或者呢次 tick 間隔異常大）就跳過積分，
  // 只做 P+D，避免用一個唔準嘅 dt 去谷大/谷細 I term。
  bool integrateOk = (dt > 0.0f && dt < 0.5f);

  // ---- 前後傾 (pitch) PID：控制水平位移 ----
  if (integrateOk) {
    pitchIntegral += pitch * dt;
    pitchIntegral = constrain(pitchIntegral, -BALANCE_INTEGRAL_CLAMP_DEG, BALANCE_INTEGRAL_CLAMP_DEG);
  }
  float pitchPidDeg = pitch * balanceGains.pitchKp
                     + pitchIntegral * balanceGains.pitchKi
                     + gyroPitchRate * balanceGains.pitchKd;
  pitchPidDeg *= BALANCE_PITCH_INVERT;

  // ---- 左右傾 (roll) PID：控制水平位移 ----
  if (integrateOk) {
    rollIntegral += roll * dt;
    rollIntegral = constrain(rollIntegral, -BALANCE_INTEGRAL_CLAMP_DEG, BALANCE_INTEGRAL_CLAMP_DEG);
  }
  float rollPidDeg = roll * balanceGains.rollKp
                    + rollIntegral * balanceGains.rollKi
                    + gyroRollRate * balanceGains.rollKd;
  rollPidDeg *= BALANCE_ROLL_INVERT;

  // ---- 膝屈曲 PID：獨立控制 kneeBendDeg（LegIK.h v2 新增輸入）----
  // 誤差輸入用 |pitch|（絕對值）——膝屈曲呢個動作本身冇方向性
  // （唔會「向前傾就屈多啲、向後傾就屈少啲」咁分方向），淨係
  // 「傾斜幅度幾大就屈幾多」，所以食絕對值嚟做誤差量，D 項用
  // 陀螺角速度嘅絕對值做阻尼（傾斜速度越快，膝提早屈多啲幫手
  // 穩住），gain 預設 0。
  // 注：冇 roll 方向嘅版本——roll 平面（HV5-HV13）結構上冇膝可
  // 屈，computeLegIK() 嘅 kneeBendRollDeg 參數純粹係函式簽名
  // 相容，傳 0 即可（見 LegIK.h solveRollIK() 説明）。
  float absPitch = fabsf(pitch);
  if (integrateOk) {
    kneePitchIntegral += absPitch * dt;
    kneePitchIntegral = constrain(kneePitchIntegral, 0.0f, BALANCE_INTEGRAL_CLAMP_DEG);
  }
  float kneeBendPitchDeg = absPitch * balanceGains.kneePitchKp
                          + kneePitchIntegral * balanceGains.kneePitchKi
                          + fabsf(gyroPitchRate) * balanceGains.kneePitchKd;

  // ---- PID 輸出角度 → 水平位移 (mm) → IK 反解 ----
  float dx = BALANCE_DEG_TO_MM(pitchPidDeg);  // 前後位移
  float dy = BALANCE_DEG_TO_MM(rollPidDeg);   // 左右位移

  LegIKAngles ik;
  computeLegIK(dx, dy, kneeBendPitchDeg, 0.0f, ik);

  // ---- IK 角度 → pulse offset ----
  out.hipFBR   = ikDegToPulse(ik.hipPitchR);
  out.hipFBL   = ikDegToPulse(ik.hipPitchL);
  out.kneeFBR  = ikDegToPulse(ik.kneePitchR);
  out.kneeFBL  = ikDegToPulse(ik.kneePitchL);
  out.ankleFBR = ikDegToPulse(ik.anklePitchR);
  out.ankleFBL = ikDegToPulse(ik.anklePitchL);

  out.hipLRR   = ikDegToPulse(ik.hipRollR);
  out.hipLRL   = ikDegToPulse(ik.hipRollL);
  out.ankleLRR = ikDegToPulse(ik.ankleRollR);
  out.ankleLRL = ikDegToPulse(ik.ankleRollL);
}

#endif