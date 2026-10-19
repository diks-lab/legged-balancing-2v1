# UARTログの事前準備と確保失敗記録

対象: Arduino core 3.0.3、SDK v5.1.4-497-gdc859c1e67-dirty。使用パッケージ表記は5.1.0+sha.dc859c1e67。SDK内部ロック/abort処理は変更しません。

## 起動時の準備

Serial/Serial2開始後、Bluetooth/Wi-Fi開始前に公開write(STDOUT_FILENO,...)でUART VFSを通し、esp_log_writeで実SDK標準ログ経路を通してfflushします。writeの実戻り値、flush/stream errorを表示します。Serialだけの出力ではVFSのwrite_lockを事前初期化できないためです。ログレベル/ログ出力先は変更しません。

初回VFS write_lockは内部8bit RAMから84 B＋管理分を遅延確保します。前倒しはこの時期を変えるだけで、Wi-Fi/PS4全体の余裕や、SDK各タスクの後続確保成功を保証しません。準備中にSDK内部でabortしても、ロックを無視して続行する変更はありません。既存のモーター出力/起動ゲートも維持します。

## コールバックとRTCの寿命

heap_caps_register_failed_alloc_callbackを無線開始前に登録。各coreの最初の確保失敗だけを固定長56 Bレコードへ保存します。要求サイズ/属性、core、ISR判定、PS、CPU cycle counterを記録します。function_nameの文字列/ポインターは保存しません。失敗時刻cyclesはcoreの生CPUカウンターであり、ミリ秒や他core間の同一時刻として比較しません。

コールバックはロックフリー32bit CASを1回使ってslotを取得し、保存するだけです。出力、動的確保、待機、ヒープ走査、キュー操作、SDK内部ロックの変更は行いません。各coreの重複失敗は破棄します。RTCは2スロット＋起動世代/検証ヘッダーで124 B固定です。checksumを書いた後、markerを最後に公開します。途中書込みは無効として扱います。

起動時にヘッダー、非ゼロ世代、record version、core、checksum、直前起動の世代一致を検証します。ESP_RST_PANIC/INT_WDT/TASK_WDT/WDTの場合だけ、検証済み記録をprevious_bootとして表示します。電源投入/外部reset/software reset/deepsleep/brownout/不明では失敗内容を表示せず破棄します。これは電源断時のRTC保持を保証するものではありません。

旧セッションのmarkerはslot読出し/消費より先に無効化し、旧slotも出力より前に消費します。消費途中のresetで、残った旧slotを再生しません。出力中の再resetや次回起動で同じ記録を再生しません。表示は「直前起動でリセット前に確保失敗を観測、原因確定ではない」と明示します。ロック生成失敗以外の確保失敗も記録対象であり、古いOOMを今回のabort原因と断定しません。

## 実際の失敗要求と直近の残量

requested/capsは実際に失敗した要求です。sample_ms/internal_free/internal_largestは通常のsetup/HTTP側が採取した直近値で、失敗瞬間のヒープ値ではありません。HTTPの既存処理後/5秒採取を共用し、制御ループに走査を追加しません。

sampleはatomicの世代付きで公開します。コールバックは一度だけ読み、書込み途中/世代不一致/未取得ならsampleValid=0と数値0を保存し、再試行しません。時刻を混同せず、直近値から失敗時の枯渇/断片化を確定しません。メモリー破損や、後続の別のSDK確保失敗の可能性も残ります。

## 検証

tests/uart_allocation_host/run.pyは実装本文をコンパイルし、ターゲットのPS読出し命令だけを置換します。公開API準備順、出力/確保/ヒープ走査禁止、task/ISR、並行core、先着保持、checksum/部分適用/version/core/世代不一致、各reset理由、消費/再起動/世代折返し、途中sampleを検証します。malloc/calloc/realloc/newにもコールバック禁止assertを入れています。ASan/UBSanと既存の起動/操作/HTTP集計の回帰も成功しました。

ESP32ビルド成功。静的DRAMは106604→106636 B（32 B増、28 Bの状態＋配置差4 B）。RTC_NOINITは別途124 B追加（2×56 B＋12 B）。コールバック本体はIRAM 275 Bで、リテラル48 BもIRAMに配置します（計323 B、配置調整前）。Flash1900753 B。事前準備のVFS mutexは実行時に84 B＋管理分を確保するため、この動的分は静的DRAM差分に含みません。新しいタスク/スタック予約はありません。実機のVFS準備成功、失敗記録のRTC保持、再接続/併用時のヒープ、ppTaskログでのabort解消は未確認です。PID/FOC/PS4操作/停止条件、FIFO、全トレース、タスクスタックは変更しません。
