// hal_conf_extra.h
//
// 呢個檔案要同 Pre-maiduino2.ino 放喺同一個 sketch 資料夾。
// STM32duino core 嘅 stm32yyxx_hal_conf.h 會自動 __has_include 呢個
// 檔案名並 #include 佢，令下面嘅 #define 喺編譯 HardwareSerial.cpp（核心
// 庫本身，獨立編譯單元）嗰陣真正生效——喺 .ino 度寫 #define 冧唔到，
// 因為 .ino 會被轉做另一個 .cpp 檔案編譯，同核心庫唔係同一個編譯單元。
//
// 背景：STM32duino 嘅 HardwareSerial RX buffer 預設淨係 64 byte，但
// Unity/PC 端 continuousMode 送嘅 0x18 多軸角度封包（25隻servo全部）
// 可達 80 byte，單一個封包已經超過預設 buffer 容量，封包送到一半
// buffer 爆滿，之後嘅 byte 被硬件丟棄，令接收端讀到殘缺封包、
// checksum 永遠對唔上。加大 buffer 到 256 byte，足夠緩衝 3 個完整
// 封包，解決呢個問題。

#if !defined(SERIAL_TX_BUFFER_SIZE)
#define SERIAL_TX_BUFFER_SIZE 256
#endif

#if !defined(SERIAL_RX_BUFFER_SIZE)
#define SERIAL_RX_BUFFER_SIZE 256
#endif
