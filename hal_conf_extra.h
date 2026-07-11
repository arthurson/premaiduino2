// hal_conf_extra.h
//
// 呢個檔案要放喺 sketch 資料夾入面（同 .ino 同一層），Arduino_Core_STM32
// 先會揀到。單單喺 .ino 度 #define SERIAL_RX_BUFFER_SIZE 冇用——
// HardwareSerial.cpp 係獨立編譯單元，佢 include 呢個檔案嘅時機早過
// .ino 嘅 define，所以必須用呢個機制先蓋得到預設值。
//
// 背景：一個 .pma binary packet 最大 100 bytes（PMA_PKT_MAX_LEN），
// 而 Arduino_Core_STM32 預設 SERIAL_RX_BUFFER_SIZE 淨係 64 bytes。
// 當 sender（例如網頁版 BT sender）連續送 packet，MCU 呢邊如果因為
// 逐隻 servo 做 ICS bus read/write（每隻都可能要 1ms timeout + retry）
// 而黎唔切喺 loop() 入面清 UART data，software ring buffer 好快
// 爆滿，導致中間 byte 被丟棄——協議狀態機（pmaRecvState）可能永久
// 卡喺 PMA_RECV_BODY 等一個唔會再嚟嘅 byte，令機械人「行到一半完全
// 定住」，睇落好似死機，其實只係 UART 協議層面卡死。
//
// 加大到 512 bytes：足夠緩衝多個 packet 排隊等處理，爭取返
// pmaReceiveUpdate() 追得上嘅時間差。

#pragma once

#define SERIAL_RX_BUFFER_SIZE 512
#define SERIAL_TX_BUFFER_SIZE 512