#ifndef MPU6050_IMU_H
#define MPU6050_IMU_H

#include <Arduino.h>
#include <Wire.h>

// ===== MPU-6050 硬件接線（見解析メモ CPU工作表）=====
// CPU 同 IC2(MPU-6050) 之間係 I2C1：
//   PB6 = I2C1_SCL
//   PB7 = I2C1_SDA
//   PB8 = INT（呢個模組暫時唔用中斷，純 polling）
// STM32duino 嘅 Wire.h 硬件 I2C 會自動用返呢兩隻腳（I2C1 default pins），
// 唔使自己 pinMode()。

#define MPU6050_ADDR 0x68  // AD0 接 GND 時嘅預設地址

// ===== 暫存器位址 =====
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

// ===== 感度換算常數（預設 ±2g / ±250deg/s，同原廠 PWR_MGMT_1=0 後嘅預設一致）=====
#define MPU6050_ACCEL_LSB_PER_G   16384.0f
#define MPU6050_GYRO_LSB_PER_DPS  131.0f

// 除法喺 soft-float（呢個 MCU 冇 FPU）通常比乘法貴，而
// imuUpdate() 係 hot path（50Hz 定期 call）。呢兩條轉換每次都係
// 「raw / 常數」，改用預先算好嘅倒數做乘法，等 imuUpdate() 每次
// tick 少做兩次除法。MPU6050_ACCEL_LSB_PER_G 係 2 的冪，倒數精確
// 冇損失；MPU6050_GYRO_LSB_PER_DPS(131) 唔係 2 的冪，倒數會有極
// 微小捨入誤差（float 精度下遠低於陀螺本身嘅讀數雜訊，可忽略）。
#define MPU6050_ACCEL_G_PER_LSB (1.0f / MPU6050_ACCEL_LSB_PER_G)
#define MPU6050_GYRO_DPS_PER_LSB (1.0f / MPU6050_GYRO_LSB_PER_DPS)

// Arduino 核心嘅 PI 係 double 常數，喺 float 運算式用佢會令嗰條
// 運算式暫時提升做 double 精度，變相拉入 double soft-float
// routine（同 float 版本並存，浪費 flash）。呢度定義 float 版本，
// 同 LegIK.h 做法一致，確保 imuUpdate() 全程停留喺 float 精度。
#define IMU_PI_F 3.14159265f

// ===== 資料結構 =====
struct IMUData {
  int16_t accelRawX, accelRawY, accelRawZ;
  int16_t gyroRawX, gyroRawY, gyroRawZ;

  float accelX, accelY, accelZ;  // 單位: g
  float gyroX, gyroY, gyroZ;     // 單位: deg/s

  float pitch;  // 由加速度計算，單位: deg
  float roll;   // 由加速度計算，單位: deg

  bool present;      // WHO_AM_I 驗證成功先 true
  unsigned long lastReadMs;
};

inline IMUData imuData;

// ===== 加速度計安裝零偏（Z軸） =====
// 主板竪直安裝，MPU-6050 喺板上焊接角度冇完全對齊，企定時
// accelZ 實測唔係 0（企定/前倾/左倾三組數據交叉驗證確認），
// 呢個係固定安裝誤差，同陀螺零偏（每次開機飄）性質唔同 ——
// 呢個值理論上裝好之後係固定嘅，唔使每次開機校正，
// 除非拆裝過感測器先需要重新量測更新。
#define MPU6050_ACCEL_Z_MOUNT_OFFSET 0.303f

// ===== 陀螺零偏補正值（開機時量測，假設靜止）=====
// 對應解析メモ「01コマンド詳細」入面「測定值-補正值=補正後測定值」嘅概念，
// 加速度計原廠備注「未使用因此補正值=0」，但陀螺我哋自己實作補正，
// 因為 MPU-6050 零偏喺唔同板/唔同溫度下都唔一樣，開機量一次準過寫死。
inline float gyroOffsetX = 0.0f;
inline float gyroOffsetY = 0.0f;
inline float gyroOffsetZ = 0.0f;

// ===== 底層 I2C helper =====
inline bool imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

inline bool imuReadBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // repeated start
  uint8_t got = Wire.requestFrom((uint8_t)MPU6050_ADDR, len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ===== 初始化 =====
// 用 400kHz Fast Mode（MPU-6050 支援），初始化失敗會將
// imuData.present 設做 false，之後所有 imuUpdate() 都會直接跳過，
// 唔會拖慢 loop()（例如冇焊 IC2 或者未來拆咗個 sensor 都唔會卡住）。
inline bool imuInit() {
  Wire.begin();
  Wire.setClock(400000);

  uint8_t whoAmI = 0;
  if (!imuReadBytes(MPU6050_REG_WHO_AM_I, &whoAmI, 1) || whoAmI != 0x68) {
    imuData.present = false;
    return false;
  }

  // 喚醒 sensor（原廠開機預設 sleep=1，要清返 0 先會開始轉換）
  if (!imuWriteReg(MPU6050_REG_PWR_MGMT_1, 0x00)) {
    imuData.present = false;
    return false;
  }
  delay(10);  // 俾內部震盪器穩定

  // 明確設定 ±2g / ±250deg/s（原廠開機值本身就係呢個，寫多次做保險）
  imuWriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00);
  imuWriteReg(MPU6050_REG_GYRO_CONFIG, 0x00);

  imuData.present = true;
  imuData.lastReadMs = 0;
  return true;
}

// 開機校正：機身企定唔郁時 call 呢個，量 N 次陀螺值攞平均做零偏。
// 唔好喺行走/郁動緊嘅時候 call，否則零偏會不準。
inline void imuCalibrateGyro(uint16_t samples = 200) {
  if (!imuData.present) return;

  float sumX = 0, sumY = 0, sumZ = 0;
  uint8_t buf[14];
  uint16_t ok = 0;

  for (uint16_t i = 0; i < samples; i++) {
    if (imuReadBytes(MPU6050_REG_ACCEL_XOUT_H, buf, 14)) {
      int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
      int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
      int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
      sumX += gx * MPU6050_GYRO_DPS_PER_LSB;
      sumY += gy * MPU6050_GYRO_DPS_PER_LSB;
      sumZ += gz * MPU6050_GYRO_DPS_PER_LSB;
      ok++;
    }
    delay(2);
  }

  if (ok > 0) {
    gyroOffsetX = sumX / ok;
    gyroOffsetY = sumY / ok;
    gyroOffsetZ = sumZ / ok;
  }
}

// ===== 讀取姿態（每次 call 讀一次 14 bytes: accel+temp+gyro 連續暫存器）=====
// ===== Complementary Filter 參數 =====
// 純 accelerometer 算出嚟嘅 pitch/roll 對震動/雜訊好敏感——機身/servo
// 郁動產生嘅瞬間震動會令 atan2() 讀數跳動，喺 balance 疊加落 servo
// 嗰刻，呢種跳動會被放大成明顯震盪（表現為「B ON 就不斷震」）。
// Complementary filter 用陀螺角速度做短時間積分（反應快、無雜訊，
// 但會長期漂移），再用 accelerometer 嘅角度做長期修正（無漂移，但
// 短時間內嘈），兩者以 alpha 混合，取兩家之長。
// alpha 越接近 1，越信任陀螺（跟得快但長遠會漂移）；
// 越接近 0，越信任 accel（穩定不漂移但短時間內震盪明顯）。
#define IMU_FILTER_ALPHA 0.98f

// 濾波後嘅角度是否已經初始化——開機/校正完第一次 imuUpdate() 要
// 用純 accel 讀數做起點，之後先開始用濾波器遞歸更新。
inline bool imuFilterInitialized = false;

// 淨係做讀取＋簡單 pitch/roll 計算，唔做任何控制決策 —— 呢個模組只負責
// 感測，行走/平衡邏輯要用呢啲值嘅話由 caller (.ino) 自己決定點用。
inline bool imuUpdate() {
  if (!imuData.present) return false;

  uint8_t buf[14];
  if (!imuReadBytes(MPU6050_REG_ACCEL_XOUT_H, buf, 14)) {
    return false;  // 讀取失敗：唔更新數據，保留上一次有效值
  }

  unsigned long nowMs = millis();
  float dt = (imuData.lastReadMs == 0) ? 0.0f : (nowMs - imuData.lastReadMs) / 1000.0f;
  // dt 過大（例如剛開機、或者 imuUpdate() 好耐冇被 call 過）代表積分
  // 會唔準，呢種情況跳過陀螺積分，淨係用返 accel 讀數做起點。
  if (dt > 0.5f) dt = 0.0f;

  imuData.accelRawX = (int16_t)((buf[0] << 8) | buf[1]);
  imuData.accelRawY = (int16_t)((buf[2] << 8) | buf[3]);
  imuData.accelRawZ = (int16_t)((buf[4] << 8) | buf[5]);
  // buf[6]/buf[7] = 溫度，暫時唔用

  imuData.gyroRawX = (int16_t)((buf[8]  << 8) | buf[9]);
  imuData.gyroRawY = (int16_t)((buf[10] << 8) | buf[11]);
  imuData.gyroRawZ = (int16_t)((buf[12] << 8) | buf[13]);

  imuData.accelX = imuData.accelRawX * MPU6050_ACCEL_G_PER_LSB;
  imuData.accelY = imuData.accelRawY * MPU6050_ACCEL_G_PER_LSB;
  imuData.accelZ = imuData.accelRawZ * MPU6050_ACCEL_G_PER_LSB;

  imuData.gyroX = (imuData.gyroRawX * MPU6050_GYRO_DPS_PER_LSB) - gyroOffsetX;
  imuData.gyroY = (imuData.gyroRawY * MPU6050_GYRO_DPS_PER_LSB) - gyroOffsetY;
  imuData.gyroZ = (imuData.gyroRawZ * MPU6050_GYRO_DPS_PER_LSB) - gyroOffsetZ;

  // 主板竪直安裝，實測軸向同標準「Z軸朝上」假設唔同：
  //   Y 軸 = 重力軸（企定時 accelY≈1.0g）
  //   X 軸 = 左右傾（左倾時 accelX 明顯變負）
  //   Z 軸 = 前後傾（前倾時 accelZ 明顯變負，但要扣走安裝零偏先準）
  float accelZCorrected = imuData.accelZ - MPU6050_ACCEL_Z_MOUNT_OFFSET;
  float accelPitch = atan2f(-accelZCorrected,
                             sqrtf(imuData.accelX * imuData.accelX +
                                   imuData.accelY * imuData.accelY)) * 180.0f / IMU_PI_F;
  float accelRoll = atan2f(imuData.accelX, imuData.accelY) * 180.0f / IMU_PI_F;

  if (!imuFilterInitialized || dt <= 0.0f) {
    // 第一次讀數，或者 dt 唔可靠：直接用 accel 角度做起點，
    // 唔做混合（冇歷史值可以積分）。
    imuData.pitch = accelPitch;
    imuData.roll = accelRoll;
    imuFilterInitialized = true;
  } else {
    // Complementary filter：
    //   新角度 = alpha * (舊角度 + 陀螺角速度*dt) + (1-alpha) * accel角度
    // gyroX 對應 accelX 呢條軸（roll），gyroZ 對應前後傾（pitch）—
    // 因為 accelZ 驅動 pitch、accelX 驅動 roll，MPU6050 陀螺同加速度計
    // 三軸物理上對齊同一組軸，故沿用同軸對應：X軸角速度→roll積分，
    // Z軸角速度→pitch積分。
    imuData.pitch = IMU_FILTER_ALPHA * (imuData.pitch + imuData.gyroZ * dt)
                     + (1.0f - IMU_FILTER_ALPHA) * accelPitch;
    imuData.roll = IMU_FILTER_ALPHA * (imuData.roll + imuData.gyroX * dt)
                    + (1.0f - IMU_FILTER_ALPHA) * accelRoll;
  }

  imuData.lastReadMs = nowMs;
  return true;
}

#endif