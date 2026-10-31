# I2C資源の先行確保と起動メモリー診断

利用者の8e2dccb実機では両I2C資源の確保成功を確認したが、Wi-Fi接続はタイムアウト。
`after_wifi` は internal_free=2308 / internal_largest=2036。
現在は [CSVコピーの必要時確保](csv-snapshot-lifetime.md) で常駐領域を減らす。
以下の詳細な段階別ログは `HTTP_MEMORY_STAGE_DIAGNOSTICS=1` の診断ビルドで有効。
通常はOFFで、起動結果と `STARTUP memory result=...` の残量を常時表示する。

## ed9c4ed実機ログの評価

利用者の実機では `after_wifi` が `heap_free=20888 internal_free=4236 internal_largest=3956`、
続いて `I2C_1 rxBuffer` の確保失敗となった。
従来はWeb正常化後、`sensor1.begin()` → `sensor2.begin()` が初めて各 `TwoWire::begin()` を呼ぶ。
そのためWi-Fi/Bluetoothが内部RAMを使用した後で、I2C_0のRX/TX・ドライバー等、
続いてI2C_1のRX/TX・ドライバー等を確保していた。
`after_wifi` とI2C_1の失敗は異なる時刻なので、3956バイトの連続領域が
RX確保時にも残っていた、または128バイトだけの不足だったとは結論できない。

Arduino ESP32 3.0.3の `Esp.cpp` では `ESP.getFreeHeap()` は `MALLOC_CAP_INTERNAL`。
今回の `internal_free/largest` は `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT`。
前者にはバイト配列に使えない領域も含み得るため、両者を同じ空き量として扱わない。
静的RAMのビルド表示も実行時の内部ヒープ残量ではない。

## 確認した実装

固定のpioarduino 51.03.03 / Arduino ESP32 3.0.3 / ESP-IDF 5.1.4を対象とする。

| バス | ピン/用途 | Wire RX | Wire TX | 設定 |
|---|---|---:|---:|---|
| I2C_0 (`I2Cone`) | SDA19/SCL18、右AS5600とMPU6050 | 128 B | 128 B | 400 kHz、timeout 5ms |
| I2C_1 (`I2Ctwo`) | SDA23/SCL5、左AS5600 | 128 B | 128 B | 同上 |

- `Wire.h` の `I2C_BUFFER_LENGTH=128` とコンストラクターの既定値を使用する。
  `setBufferSize()` やビルドフラグによるサイズ変更は行わない。2バス計512 Bは配列本体のみ。
- `Wire.cpp` の `begin()` はmutex、RX/TXの `malloc`、HALの `i2cInit()` を実行し、boolを返す。
  TX失敗時はRXも解放し、begin失敗時もWireバッファを解放する。
- `setBufferSize()` は未初期化時にはサイズ指定だけで、RX/TXの確保成功確認にはならない。
  今回は両バスの `begin()` 成功で確認し、後段で `end()` して予約を失うことも避ける。
- HALの `esp32-hal-i2c.c` は追加のmutexと `i2c_driver_install(master,0,0,0)` を使う。
  [ESP-IDF v5.1.4実装](https://github.com/espressif/esp-idf/blob/v5.1.4/components/driver/i2c/i2c.c)では
  オブジェクト、指令mutex、イベントキュー、割り込み資源等を確保する。
  マスターモードでスレーブ用RX/TXリングバッファは作らない。
  512 Bだけを必要量として見積もらず、確保前後の実機差分を測る。
- MPU6050はI2C_0を共有し、別のWire RX/TXを確保しない。初期化/校正/読み取りはWeb確認後に行う。

Wi-Fiの `WiFiGeneric.cpp` は `WiFi.mode()` 内から `Network.begin()`、`esp_wifi_init()`、
ネットワークインターフェース生成、Wi-Fi開始を行う。
本設定は `useStaticBuffers()` の既定falseを維持し、実際の初期化設定はstatic RX=4、
dynamic RX=32、dynamic TX=32等となる。sdkconfigの既定値だけで実消費を算定しない。
Bluetoothにも既存のタスク/バッファ確保があり、接続・IP取得・通信時にも空きは変動する。
Wi-Fi/BTのバッファ・スタック・設定は今回変更しない。各段階の実消費バイト数はまだ未測定。

## 起動経路

1. 両輪ENをLOW。シリアル開始後、I2C_0とI2C_1をそれぞれ `begin()`、成否を保存する。
   バスの準備だけで、AS5600読み取り、MPU設定、サーボ移動、モーター/FOCは実行しない。
2. PS4ライブラリ開始、HTTPキュー/静的タスク、LittleFS、Wi-Fi/IP、HTTP待受を準備する。
3. I2C両バス準備とWeb必須機能が正常な場合だけ、制御ループでセンサー初期化/校正、
   車輪ドライバー・モーター・FOCを既存順序で実行する。
   `CheckedWheelSensor::beginPreparedBus()` は確保済みバスで従来のSensor初期サンプルだけを取得する。
4. 両輪初期化・事後サンプルが正常なら、従来の起動確定と一度だけのメニューへ進む。

I2C資源準備失敗は自動再試行せず、`I2C0_RESOURCES_FAILED` / `I2C1_RESOURCES_FAILED` と
個別結果を表示する。以後も両輪出力を遮断し、Webが正常でもFOCへ進まない。
Web/PS4診断は引き続き起動を試みるが、それら自体のメモリー確保成功を保証するものではない。
成功した片側資源も保持し、再起動まで解放/再アラインメントはしない。

I2C準備結果はHTTP起動前に確定し、それ以後変更しない。
setupと制御ループは同じArduinoタスク、HTTPはキューの状態コピーだけを配信する。
HTTP/PS4コールバックからI2Cにアクセスしない。無線接続待ちを制御ループへ追加しない。
実行中のPID・FOCパラメーター・PS4処理・停止条件は変更しない。

## 追加ログと実機での比較

すべて既存の `HTTP memory stage=...` 形式で `heap_free`、内部8bitの空きと最大連続領域を記録。
接頭辞は互換性のため維持し、HTTP以外の確保段階にも使用する。

| stage | 比較対象 |
|---|---|
| `before_i2c0` / `after_i2c0` | I2C_0 RX/TX・ロック・マスタードライバー等の一括準備 |
| `before_i2c1` / `after_i2c1` | I2C_1の同上 |
| `before_ps4` / `after_ps4` | Bluetooth/PS4開始 |
| `before_queues` / `after_status_queue` | HTTP状態キュー |
| `before_command_queue` / `after_command_queue` | HTTP指令キュー |
| `before_task` / `after_task` | 静的8KiB HTTPタスク。スタックはBSSに確保済み |
| `before_filesystem` / `after_filesystem` | LittleFS mount、自動フォーマットなし |
| `before_wifi_mode` / `after_wifi_mode` | Wi-Fi低レベル初期化/開始 |
| `before_wifi_begin` / `after_wifi` | 接続要求直後。IP取得完了ではない |
| `wifi_ip_ready` | 非ゼロIP取得/変更時 |
| `before_http_listen` / `after_http_listen` | HTTP TCP待受開始要求 |
| `before_wheel_align_timer` / `after_wheel_align_timer` | 既存アラインメントタイマー |
| `before_mpu6050` / `after_mpu6050` | MPU設定と既存校正 |
| `before/after_right/left_driver` | 個別車輪ドライバー初期化（例 `before_right_driver`） |
| `before/after_right/left_motor` | 個別モーター初期化 |
| `before/after_right/left_foc` | 個別FOC初期化 |
| `after_both_wheels_initialized` | 両輪初期化・事後センサー確認成功後 |

各I2C準備後にはバス番号、ピン、クロック、RX/TX要求サイズ、READY/FAILEDも表示する。
公開APIの単位である `begin()` 全体を前後から計測するため、内部のRX/TX別消費は分離しない。
ログの差分は同時に動く無線タスク等の影響も含む。失敗してもafterログを出す。
既存のメモリー診断と静的8KiBスタックを保持し、バッファ/スタックは削減していない。
メモリー行・I2C結果行・起動失敗行は固定長のローカル配列に整形してSerialへ渡す。
このcoreの `Print::printf()` は64文字以上で一時mallocするため、これらの診断行では
その経路を避け、空き量取得後の診断整形が新しいヒープ確保を必要としないようにした。

## 検証の限界

ホストテストで実Wire関数にRX/TXそれぞれの失敗、mutex/ドライバー失敗を注入し、
失敗判定・解放・予約保持を確認する。起動テストは片側失敗でWebを維持しFOC/メニューを
抑止する経路を確認する。ホストのmallocを用いるため実ESP32のヒープ量は検証できない。

ESP32ビルド成功は実機でのI2C・Wi-Fi・FOC資源の確保成功を意味しない。
今回の先行確保で全体の必要RAMを減らすわけではなく、必要なI2C資源の確保順序を優先する。
Wi-Fi開始後に残る余裕、長時間通信、両輪FOC完了、停止遅延、静止倒立・走行/減速/停止・
旋回・脚操作は変更後の実機未検証。PID/FOC値やスタックを根拠なく縮小していない。
