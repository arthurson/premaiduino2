#ifndef LEG_IK_H
#define LEG_IK_H

#include <Arduino.h>
#include <math.h>

// ===== Leg IK Solver =====
// 目的：由 BalanceGyro.h 嘅 PID 輸出（軀幹應該點郁先企返直嘅
// 控制量），反解做髖、膝、踝三隻關節嘅目標角度，取代舊版
// LINKDEF 符號表（+1/-1/+2/-2）嗰種「膝完全唔郁、憑經驗比例
// 分配」嘅做法。
//
// ---- 設計沿革（重要：呢個 v2 版本修正咗 v1 嘅一個根本錯誤）----
// v1 嘗試「保持垂直高度固定，淨係水平位移」，但 home 姿態本身
// 已經接近完全打直（膝彎<10°），打直嘅腿已經係 THIGH+SHIN 嘅
// 最大伸展長度，冇「額外長度」可以攞嚟做斜向位移，結果數學上
// reach 一定要 > height，逼到膝角公式撞落 clamp 上限，變成
// 「膝完全唔郁、修正全部落晒喺髖度」——同冇做 IK 前嘅症狀一樣，
// 只係換咗個包裝。
//
// v2 做法：唔再假設垂直高度固定，改為將「水平位移 offset」同
// 「膝屈曲角度 kneeBendDeg」當做兩個獨立輸入（分別由 BalanceGyro.h
// 嘅兩組獨立 PID 提供），用餘弦定理由呢兩個值直接反解 reach、
// height、髖角、踝角。數學上冇奇異點（offset=0 且 kneeBendDeg=0
// 精準對應 home），亦唔使憑空估「膝要屈幾快」呢條無經驗根據嘅
// 比例常數。
//
// 呢層淨係負責「幾何」：輸入 offset（mm）+ kneeBendDeg（度），
// 輸出關節角度修正量（度，相對 home 姿態）。唔負責讀 IMU、
// 唔負責 PID 計算，呢啲交返 BalanceGyro.h 做，兩層職責分開。

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
#define PITCH_THIGH_LENGTH 65.0   // HV7(髖前後)→HV9(膝)
#define PITCH_SHIN_LENGTH  65.0   // HV9(膝)→HV11(踝前後)
#define PITCH_FOOT_LENGTH  60.0   // HV11→地面（50 底板 + 10 腳掌），唔屈曲，跟住 ankleDeg 保持垂直

#define ROLL_LEVER_LENGTH  215.0 // HV5(髖側擺)→HV13(踝側擺)：35+65+65+50，固定剛體，冇膝
#define ROLL_FOOT_LENGTH   10.0  // HV13→地面，唔屈曲，跟住 ankleRollDeg 保持垂直

#define HIP_WIDTH 40.0
#define LEG_LENGTH (PITCH_THIGH_LENGTH + PITCH_SHIN_LENGTH + PITCH_FOOT_LENGTH)  // pitch 方向全長，BalanceGyro.h 槓桿臂用

// ===== Pulse ↔ 角度換算 =====
// 同 BalanceGyro.h 用返同一條轉換（8000 pulse ≈ 270 度），
// 全部伺服共用一條，唔分關節。
#define IK_PULSE_PER_DEGREE (8000.0 / 270.0)  // ≈ 29.63

// ===== 膝屈曲角度上限 =====
// 防止 kneeBendDeg PID 輸出失控時，膝解出一個機械上唔可能/唔安全
// 嘅屈曲角度（例如摺埋）。60° 純粹係「留返一線」嘅保守上限，
// 實機應該以 ServoConfig.h 嘅 HV9/HV10 minAngle/maxAngle 做最終
// 把關（呢層 IK 唔知道 servo 嘅實際物理極限，只做幾何合理性
// clamp；真正嘅安全上限一定要喺 .ino 度用 constrain() 對住
// servo 嘅 minAngle/maxAngle 再做多一次）。
#define IK_KNEE_BEND_MAX_DEG 60.0

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
//      kneeBendDeg（度，正值=屈曲，0=完全打直），兩者係獨立輸入，
//      唔互相假設對方嘅值——側移量交俾 pitch/roll PID，屈膝量
//      交俾另一組獨立 PID，呢層淨係做幾何反解。
//
//      幾何：
//        1) 由 kneeBendDeg 用餘弦定理直接攞髖到踝嘅斜邊長度 reach
//           （呢個唔再假設垂直高度固定，reach 完全由膝屈曲角度
//           決定，膝屈得越多、reach 越短）。
//        2) reach 攞到之後，配合 offset（水平分量）反解 height
//           （垂直分量，係結果，唔係輸入——膝屈得越多、offset
//           越大，height 自然跟住縮）。
//        3) 用 reach 嘅方向（atan2）加返大腿相對 reach 嘅偏角，
//           攞到髖角度。
//        4) 踝角度 = 補償「髖偏 + 膝屈」兩者疊加喺腳掌方向嘅
//           總旋轉，令腳掌保持水平貼地。
//
//      offset=0 且 kneeBendDeg=0 時，reach=height=THIGH+SHIN，
//      hipDeg=ankleDeg=0，精準對應 home，冇 v1 版本嗰種「home
//      都解出非零角度」嘅問題。
inline void solve2LinkIK(double offset, double kneeBendDeg,
                          double &hipDeg, double &ankleDeg) {
  double kneeBendClamped = constrain(kneeBendDeg, 0.0, IK_KNEE_BEND_MAX_DEG);
  double kneeInteriorRad = (180.0 - kneeBendClamped) * PI / 180.0;

  // 餘弦定理：reach^2 = THIGH^2 + SHIN^2 - 2*THIGH*SHIN*cos(kneeInterior)
  // kneeInterior = 大腿同小腿之間嘅內角（180°=完全打直）。
  // 呢個 helper 專門用喺 pitch 平面（HV7-HV9-HV11 先有膝可屈），
  // roll 平面用另一條 solveRollIK()，唔經呢度。
  double reachSq = PITCH_THIGH_LENGTH * PITCH_THIGH_LENGTH + PITCH_SHIN_LENGTH * PITCH_SHIN_LENGTH
                  - 2.0 * PITCH_THIGH_LENGTH * PITCH_SHIN_LENGTH * cos(kneeInteriorRad);
  double reach = sqrt(reachSq > 0.0 ? reachSq : 0.0);

  // reach 係斜邊，offset 係其中一隻直角邊，offset 嘅大小唔可以
  // 超過 reach（否則另一隻直角邊 height 會變負數/無實數解）。
  // 呢種情況代表「想要嘅側移量超過咗依家膝屈曲角度所能達到嘅
  // 最大伸展」，clamp offset 落 reach 嘅範圍之內（留一線緩衝
  // 防止除零/NaN），令 height 趨近於 0（髖同踝幾乎同一水平面，
  // 幾何上嘅極限狀態，唔會產生無效解）。
  double reachLimit = reach - 0.01;
  double offsetClamped = constrain(offset, -reachLimit, reachLimit);
  double heightSq = reach * reach - offsetClamped * offsetClamped;
  double height = sqrt(heightSq > 0.0 ? heightSq : 0.0);

  double reachTiltFromVertical = atan2(offsetClamped, height) * 180.0 / PI;
  double cosThighReachAngle = (PITCH_THIGH_LENGTH * PITCH_THIGH_LENGTH + reach * reach - PITCH_SHIN_LENGTH * PITCH_SHIN_LENGTH)
                             / (2.0 * PITCH_THIGH_LENGTH * reach);
  cosThighReachAngle = constrain(cosThighReachAngle, -1.0, 1.0);
  double thighReachAngle = acos(cosThighReachAngle) * 180.0 / PI;
  hipDeg = reachTiltFromVertical + thighReachAngle;

  // 踝角：令腳掌保持水平貼地，補償量 = -(髖角度造成嘅傾斜) +
  // (膝屈曲帶嚟嘅反向補償)，令腳掌相對地面嘅淨旋轉 = 0。
  ankleDeg = -(hipDeg - kneeBendClamped);
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
//      ankleRollDeg = -hipRollDeg（同 solve2LinkIK() 入面
//      kneeBendClamped=0 個特殊 case 一致，但呢度冇膝呢個
//      choice，成條式恆常都係咁）。
//
//      offsetY 唔可以喺物理上超過 ROLL_LEVER_LENGTH（否則
//      atan2 都仲有效，但代表側移量已經超過成隻腳打直側擺
//      90° 都仲唔夠嘅極端情況），呢度都留返一線 clamp 防止
//      user 輸入離晒譜嘅數值，同 solve2LinkIK() 做法一致。
//      注意符號慣例：呢度嘅 ankleRollDeg 要輸出「同 hipRollDeg
//      反號」（即係 -hipRollDeg），因為 computeLegIK() 出面
//      仲有一次 out.ankleRollR/L = -ankleR 嘅 negate，兩層夾埋
//      先等於「最終 ankleRoll 輸出同 hipRoll 同號」——同
//      solve2LinkIK() 嘅 ankleDeg 輸出慣例（kneeBendDeg=0 特例下
//      = -hipDeg，出面再 negate 一次變返同 hipDeg 同號）保持一致。
//      如果呢度輸出同 hipRollDeg 同號，會令兩層 negate 冚唔返，
//      最終方向會反咗——之前一個修正版本正是咁，已經改返。
inline void solveRollIK(double offsetY, double &hipRollDeg, double &ankleRollDeg) {
  double leverLimit = ROLL_LEVER_LENGTH - 0.01;
  double offsetClamped = constrain(offsetY, -leverLimit, leverLimit);

  hipRollDeg = atan2(offsetClamped, ROLL_LEVER_LENGTH) * 180.0 / PI;
  ankleRollDeg = -hipRollDeg;
}

// ---- 主入口：輸入軀幹想要嘅水平位移 dx（前後，mm，正=向前）、
//      dy（左右，mm，正=向右），同埋想要嘅膝屈曲角度
//      kneeBendPitchDeg（前後方向屈膝，度）、kneeBendRollDeg
//      （左右方向屈膝，度，通常較少用，預設可傳 0），輸出六隻
//      關節角度修正量。呢啲輸入嘅來源：BalanceGyro.h 嘅兩組
//      獨立 PID（一組管側移、一組管屈膝），呢層淨係做純幾何
//      反解，唔理輸入嘅物理意義嚟源。
inline void computeLegIK(double dx, double dy,
                          double kneeBendPitchDeg, double kneeBendRollDeg,
                          LegIKAngles &out) {
  double hipP, ankleP;   // sagittal 平面（前後，pitch，HV7-HV9-HV11 有膝可屈）
  double hipR, ankleR;   // frontal 平面（左右，roll，HV5-HV13 冇膝，單軸+固定臂長）

  solve2LinkIK(dx, kneeBendPitchDeg, hipP, ankleP);
  solveRollIK(dy, hipR, ankleR);
  // kneeBendRollDeg 保留做函式簽名相容（BalanceGyro.h 仍然會傳入），
  // 但 roll 平面結構上冇膝可屈，呢個參數喺呢度冇實際幾何意義，
  // 淨係避免要改埋 BalanceGyro.h 嘅呼叫介面。如果將來想完全
  // 移除，要同步清理 BalanceGyro.h 嘅 kneeRollKp/Ki/Kd 呢組
  // gain（依家全部預設 0，即係冇被使用緊）。

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
  // 固定剛體（冇其他關節分薄角度）。腳掌絕對角度 = hipRollDeg
  // （剛體傾側嘅角度）+ ankleRollDeg（踝關節局部轉角），要令腳掌
  // 保持水平（絕對角度=0），踝關節局部轉角必須同 hipRollDeg 反號，
  // 即 ankleRollDeg = -hipRollDeg——呢個 negate 已經喺 solveRollIK()
  // 入面做咗，ankleR 呢度已經係「送去踝關節嘅局部轉角」，唔可以
  // 出面再 negate 多一次（之前試過 out.ankleRollR = -ankleR，
  // 令兩次 negate 互相抵消，變成同 hipRollR 同號，導致腳掌側傾
  // 唔貼地——已經改正）。
  out.hipRollR   =  hipR;
  out.hipRollL   =  hipR;
  out.ankleRollR =  ankleR;
  out.ankleRollL =  ankleR;
}

// ---- 角度 → pulse 修正量（int），供 .ino 直接加落 homePosition ----
inline int ikDegToPulse(double deg) {
  return (int)(deg * IK_PULSE_PER_DEGREE);
}

#endif