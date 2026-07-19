#ifndef LEG_IK_H
#define LEG_IK_H

#include <Arduino.h>
#include <math.h>

// ===== Leg IK Solver =====
// 目的：由 BalanceGyro.h 嘅 PID 輸出（軀幹應該點郁先企返直嘅
// 控制量），反解做髖、膝、踝三隻關節嘅目標角度。
//
// 輸入「水平位移 offset（mm）」同「膝屈曲角度 kneeBendDeg（度）」
// 兩個獨立值（分別由 BalanceGyro.h 嘅兩組獨立 PID 提供），用餘弦
// 定理反解 reach、height、髖角、踝角。offset=0 且 kneeBendDeg=0
// 精準對應 home 姿態，數學上冇奇異點。
//
// 呢層淨係負責「幾何」：輸入 offset + kneeBendDeg，輸出關節角度
// 修正量（度，相對 home 姿態）。唔負責讀 IMU、唔負責 PID 計算，
// 呢啲交返 BalanceGyro.h 做，兩層職責分開。

// ===== IK Solver 常數（2026-07 實機量度，沿腿直線量度各 servo
//      轉軸之間嘅距離：HV5-HV7-HV9-HV11-HV13-地面）=====
//
// 重要結構事實：呢五隻 servo 唔係「pitch/roll 各自對稱嘅 2-link
// chain」。HV7/HV9/HV11 三隻全部係 pitch（前後）軸，HV5/HV13
// 先係 roll（左右）軸。即係話：
//   - Pitch 平面：HV7→HV9→HV11 先係真正可屈伸嘅 2-link（膝喺呢度），
//     真.可以獨立收縮/伸展 reach。
//   - Roll 平面：HV5→HV13 之間隔住嘅 35+65+65+50=215mm 全部係
//     pitch 軸，對 roll 嚟講係固定剛體，唔存在一隻可以喺 roll
//     平面屈曲嘅「膝」。Roll 方向淨係得 HV5、HV13 兩隻 servo 郁，
//     中間係死長度。
//
// 呢個結構性分別令 pitch 同 roll 唔可以用同一組
// THIGH/SHIN/ANKLE 常數，亦唔可以用同一種「2-link + 膝屈曲」
// 幾何處理——roll 冇膝可屈，要用單軸 + 固定臂長嘅簡單三角函數。
#define PITCH_THIGH_LENGTH 65.0f   // HV7(髖前後)→HV9(膝)
#define PITCH_SHIN_LENGTH  65.0f   // HV9(膝)→HV11(踝前後)
#define PITCH_FOOT_LENGTH  60.0f   // HV11→地面（50 底板 + 10 腳掌），唔屈曲，跟住 ankleDeg 保持垂直

#define ROLL_LEVER_LENGTH  215.0f // HV5(髖側擺)→HV13(踝側擺)：35+65+65+50，固定剛體，冇膝
#define ROLL_FOOT_LENGTH   10.0f  // HV13→地面，唔屈曲，跟住 ankleRollDeg 保持垂直

#define HIP_WIDTH 40.0f
#define LEG_LENGTH (PITCH_THIGH_LENGTH + PITCH_SHIN_LENGTH + PITCH_FOOT_LENGTH)  // pitch 方向全長，BalanceGyro.h 槓桿臂用

// solve2LinkIK() 下面用咗「大腿==小腿」嘅等腰三角形閉式解嚟慳返
// cos+sqrt+acos 三個 transcendental 做一個 cos。呢個 static_assert
// 確保如果將來改咗 PITCH_THIGH_LENGTH/PITCH_SHIN_LENGTH 令兩者唔
// 再相等，編譯期就會報錯提醒，而唔係靜默計錯（等腰簡化式喺
// THIGH != SHIN 時唔再成立）。
static_assert(PITCH_THIGH_LENGTH == PITCH_SHIN_LENGTH,
              "solve2LinkIK() isosceles-triangle simplification requires THIGH == SHIN; "
              "revert to general law-of-cosines formula if these differ");

// ===== Pulse ↔ 角度換算 =====
// 同 BalanceGyro.h 用返同一條轉換（8000 pulse ≈ 270 度），
// 全部伺服共用一條，唔分關節。
#define IK_PULSE_PER_DEGREE (8000.0f / 270.0f)  // ≈ 29.63

// ===== 膝屈曲角度上限 =====
// 防止 kneeBendDeg PID 輸出失控時，膝解出一個機械上唔可能/唔安全
// 嘅屈曲角度（例如摺埋）。60° 純粹係「留返一線」嘅保守上限，
// 實機應該以 ServoConfig.h 嘅 HV9/HV10 minAngle/maxAngle 做最終
// 把關（呢層 IK 唔知道 servo 嘅實際物理極限，只做幾何合理性
// clamp；真正嘅安全上限一定要喺 .ino 度用 constrain() 對住
// servo 嘅 minAngle/maxAngle 再做多一次）。
#define IK_KNEE_BEND_MAX_DEG 60.0f

// Arduino 核心嘅 PI 係 double 常數，喺 float 運算式入面用佢會令
// 嗰條運算式暫時提升做 double 精度計算，變相都係拉入 double
// soft-float routine（同 float 版本並存，浪費 flash）。呢度定義
// 一個 float 版本嘅 PI，確保呢個檔案入面所有三角函數運算全程
// 停留喺 float 精度，唔會偷偷跳去 double。
#define IK_PI_F 3.14159265f

// ===== 左右膝機械方向表 =====
// 由 ServoConfig.h 嘅 range 推出：
//   HV9（右膝）range 3950~7600，home=7500 接近上限
//     → pulse 減少 = 膝彎（屈曲）增加
//   HV10（左膝）range 7400~11050，home=7500 接近下限
//     → pulse 增加 = 膝彎（屈曲）增加
// 即右膝、左膝係鏡射關係：同一個「屈膝」動作，兩邊 pulse
// 變化方向相反。IK 輸出「膝彎角度」（正值=屈曲）之後，套用
// 呢兩個常數轉返做各自嘅 pulse 方向，唔使關節公式本身分左右。
#define IK_KNEE_RIGHT_INVERT -1  // 右膝：屈曲角度增加 → pulse 減少
#define IK_KNEE_LEFT_INVERT  1   // 左膝：屈曲角度增加 → pulse 增加

// ===== 計算結果（三關節 × 雙腳 = 6 個角度修正量，單位：度，相對 home）=====
struct LegIKAngles {
  float hipPitchR, hipPitchL;      // 髖前後角修正 (HV7/HV8)
  float kneePitchR, kneePitchL;    // 膝屈伸角修正 (HV9/HV10，已經套用左右 invert)
  float anklePitchR, anklePitchL;  // 踝前後角修正 (HV11/HV12，補償令腳掌貼地)

  float hipRollR, hipRollL;        // 髖側擺角修正 (HV5/HV6)
  float ankleRollR, ankleRollL;    // 踝側擺角修正 (HV13/HV14)
};

// ---- 內部 helper：單一 sagittal（前後）或 frontal（左右）平面嘅
//      2-link IK。輸入水平位移 offset（mm）同膝屈曲角度
//      kneeBendDeg（度，正值=屈曲，0=完全打直），兩者係獨立輸入
//      （側移量交俾 pitch/roll PID，屈膝量交俾另一組獨立 PID），
//      呢層淨係做幾何反解。
//
//      幾何簡化（等腰三角形恆等式）：PITCH_THIGH_LENGTH ==
//      PITCH_SHIN_LENGTH（都係 65mm），大腿同小腿形成等腰三角形，
//      有精確閉式解：
//        1) reach = 2·T·|cos(kneeBendDeg/2)|
//        2) thighReachAngle = kneeBendDeg / 2
//      （T=THIGH=SHIN，已經 Python 逐點數值驗證 0°~60° 同通用餘弦
//      定理+acos 做法一致）。呢個簡化將 cos+sqrt+acos 三個
//      transcendental function call 減到一個 cos，係專為
//      THIGH==SHIN 呢個特定幾何度身定造，如果將來兩者長度唔再
//      相等，要改返用通用餘弦定理版本。
//
//      height（垂直分量）冇閉式簡化捷徑，仍要用 sqrt(reach²-offset²)。
//      offset=0 且 kneeBendDeg=0 時，reach=height=THIGH+SHIN，
//      hipDeg=ankleDeg=0，精準對應 home。
inline void solve2LinkIK(float offset, float kneeBendDeg,
                          float &hipDeg, float &ankleDeg) {
  float kneeBendClamped = constrain(kneeBendDeg, 0.0f, IK_KNEE_BEND_MAX_DEG);

  // 等腰三角形閉式解：reach = 2T|cos(bend/2)|，thighReachAngle = bend/2。
  // 淨係一個 cosf() call，代替原本 cos+sqrt+acos 三個 transcendental。
  float halfBendRad = kneeBendClamped * (IK_PI_F / 360.0f);  // (bend/2) 轉 rad
  float reach = 2.0f * PITCH_THIGH_LENGTH * fabsf(cosf(halfBendRad));
  float thighReachAngle = kneeBendClamped * 0.5f;

  // reach 係斜邊，offset 係其中一隻直角邊，offset 嘅大小唔可以
  // 超過 reach（否則另一隻直角邊 height 會變負數/無實數解）。
  // 呢種情況代表「想要嘅側移量超過咗依家膝屈曲角度所能達到嘅
  // 最大伸展」，clamp offset 落 reach 嘅範圍之內（留一線緩衝
  // 防止除零/NaN），令 height 趨近於 0（髖同踝幾乎同一水平面，
  // 幾何上嘅極限狀態，唔會產生無效解）。
  float reachLimit = reach - 0.01f;
  float offsetClamped = constrain(offset, -reachLimit, reachLimit);
  float heightSq = reach * reach - offsetClamped * offsetClamped;
  float height = sqrtf(heightSq > 0.0f ? heightSq : 0.0f);

  float reachTiltFromVertical = atan2f(offsetClamped, height) * 180.0f / IK_PI_F;
  float hipDegLocal = reachTiltFromVertical + thighReachAngle;
  hipDeg = hipDegLocal;

  // 踝角：令腳掌保持水平貼地，補償量 = -(髖角度造成嘅傾斜) +
  // (膝屈曲帶嚟嘅反向補償)，令腳掌相對地面嘅淨旋轉 = 0。
  ankleDeg = -(hipDegLocal - kneeBendClamped);
}

// ---- Roll（左右）平面專用 helper：HV5(髖側擺)→HV13(踝側擺)
//      之間 215mm 全部係 pitch 軸，對 roll 嚟講係固定剛體，
//      冇膝可屈——即係話 roll 平面淨係得 HV5 一隻 servo 真正
//      負責「傾側幾多度」，唔存在 solve2LinkIK() 嗰種
//      「膝屈曲令 reach 縮短」嘅自由度。
//
//      幾何：輸入水平位移 offsetY（mm），用固定臂長
//      ROLL_LEVER_LENGTH（215mm）做直角三角形嘅鄰邊，
//      hipRollDeg = atan2(offsetY, ROLL_LEVER_LENGTH)，即髖側擺
//      要轉幾多度先令腳掌側移 offsetY mm。
//
//      踝側擺角：令腳掌保持水平貼地，中間冇其他 roll 關節分薄
//      呢個轉角，所以補償量就係髖轉角嘅完整反向：
//      ankleRollDeg = -hipRollDeg。
//
//      offsetY 唔可以喺物理上超過 ROLL_LEVER_LENGTH，呢度留返
//      一線 clamp 防止輸入離譜數值。
//
//      注意符號慣例：呢度輸出嘅 ankleRollDeg 已經係「送去踝關節
//      嘅局部轉角」（同 hipRollDeg 反號），computeLegIK() 出面
//      直接原值賦落 out.ankleRollR/L，唔可以再 negate 一次。
inline void solveRollIK(float offsetY, float &hipRollDeg, float &ankleRollDeg) {
  float leverLimit = ROLL_LEVER_LENGTH - 0.01f;
  float offsetClamped = constrain(offsetY, -leverLimit, leverLimit);

  hipRollDeg = atan2f(offsetClamped, ROLL_LEVER_LENGTH) * 180.0f / IK_PI_F;
  ankleRollDeg = -hipRollDeg;
}

// ---- 主入口：輸入軀幹想要嘅水平位移 dx（前後，mm，正=向前）、
//      dy（左右，mm，正=向右），同埋想要嘅膝屈曲角度
//      kneeBendPitchDeg（前後方向屈膝，度），輸出六隻關節角度
//      修正量。呢啲輸入嘅來源：BalanceGyro.h 嘅獨立 PID（一組管
//      側移、一組管屈膝），呢層淨係做純幾何反解，唔理輸入嘅
//      物理意義嚟源。
//
//      簽名保留第四個參數（原 kneeBendRollDeg）純粹做呼叫介面
//      相容——roll 平面（HV5-HV13）結構上冇膝可屈（見上面
//      solveRollIK() 説明），呢個值喺呢度完全唔會被使用。
//      BalanceGyro.h 已經直接傳 0.0f，冇再計算呢個值嘅 PID。
inline void computeLegIK(float dx, float dy,
                          float kneeBendPitchDeg, float /* unused: roll plane has no knee */,
                          LegIKAngles &out) {
  float hipP, ankleP;   // sagittal 平面（前後，pitch，HV7-HV9-HV11 有膝可屈）
  float hipR, ankleR;   // frontal 平面（左右，roll，HV5-HV13 冇膝，單軸+固定臂長）

  solve2LinkIK(dx, kneeBendPitchDeg, hipP, ankleP);
  solveRollIK(dy, hipR, ankleR);

  // ---- Pitch（前後）----
  // 沿用 BalanceGyro.h 舊 LINKDEF_PITCH 嘅左右對稱慣例：
  // 右髖/右踝 = +號基準，左髖/左踝 = 反號（雙腳前後移動方向相反，
  // 令身體整體前後平移，唔係雙腳各自各樣）。
  out.hipPitchR   =  hipP;
  out.hipPitchL   = -hipP;
  out.anklePitchR =  ankleP;
  out.anklePitchL = -ankleP;

  // 膝屈伸：兩腳同時屈曲同一個量（前後平移時雙腳同步屈膝落，
  // 淨係 pulse 方向要用 invert 常數反，機械上鏡射）。
  out.kneePitchR = kneeBendPitchDeg * IK_KNEE_RIGHT_INVERT;
  out.kneePitchL = kneeBendPitchDeg * IK_KNEE_LEFT_INVERT;

  // ---- Roll（左右）----
  // Roll 平面淨係得 HV5(髖)/HV13(踝) 兩個獨立轉軸，中間 215mm 係
  // 固定剛體。腳掌絕對角度 = hipRollDeg（剛體傾側角度）+
  // ankleRollDeg（踝關節局部轉角），要令腳掌保持水平（絕對角度=0），
  // ankleRollDeg 必須同 hipRollDeg 反號——呢個 negate 已經喺
  // solveRollIK() 做咗，呢度直接賦值，唔可以再 negate 多一次。
  out.hipRollR   =  hipR;
  out.hipRollL   =  hipR;
  out.ankleRollR =  ankleR;
  out.ankleRollL =  ankleR;
}

// ---- 角度 → pulse 修正量（int），供 .ino 直接加落 homePosition ----
inline int ikDegToPulse(float deg) {
  return (int)(deg * IK_PULSE_PER_DEGREE);
}

#endif