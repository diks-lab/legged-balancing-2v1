# UART初期化と継続送信予算の検証

変更は2コミットに分離。基準7ea05d0、head codex/2208-web-ps4、PR #13のbase codex/2208-short-drive / dfd4d0cを維持します。

| 構成 | 静的DRAM | 直前比 | RTC_NOINIT | Flash |
|---|---:|---:|---:|---:|
|7ea05d0|106604 B|—|0 B|1898921 B|
|UART事前準備/確保失敗観測 7884122|106636 B|+32 B|124 B|1900753 B|
|継続送信予算修正|106668 B|+32 B|124 B|1901205 B|

7ea05d0比の静的DRAM増は64 B、RTC追加124 Bを含むデータ領域増は188 Bです。DRAMのdata28852 Bは不変、bss77752→77816 B。UART観測はDRAM状態28 B＋配置差4 B、RTCは12＋2×56 Bです。UARTコールバックのIRAM配置は本体275 B＋literal48 B（323 B、配置調整前）。送信側のコンテキストは368→400 B、固定payload256 Bとリング32件・HTTPスタック8192 Bは不変です。

UART VFS mutexの84 B＋ヒープ管理分は従来の遅延確保を前倒しする動的領域であり、静的RAM値に含みません。stdout/SDKログ経路の後続の追加確保やネットワークヒープの余裕も実機で測定します。新しいタスク/スタック予約・常駐CSVコピー・大量ログ配列を追加しません。

## 実施した確認

- UART実装本文: callbackにmalloc/calloc/realloc/new・出力・ヒープ走査を禁止、task/ISR/並行core、最初の要求保持、RTC marker/version/checksum/世代/core、通常reset破棄とpanic/WDT表示、消費後の重複表示なし、途中sampleの一度読みでの拒否、cycle counterとsample_msの区別。ASan/UBSan成功。
- ESP32の実オブジェクト: callbackはIRAM配置、claimは単一s32c1i命令で再試行ループなし。外部呼出はISR判定のみ。checksumは固定12語の処理です。SDK内部ロックは未変更。
- 継続送信: 実3ファイルのLF/CRLF、10/30/100 msの遅いpollでHTML完送、部分送信/短いread/先読み禁止、10 ms未満のEAGAIN待機では再送なし、20回連続EAGAIN後の完送、永久busyで500 ms無進捗、進捗を挟む累積64エラー、累積128 send、進捗中でも2000 ms全体期限、read異常/切断/待機中の切断/Wi-Fi取消、時刻折返し、二重終了/再利用/全所有者解放。ASan/UBSan成功。
- 実SDKのNetworkClientコンストラクター/デストラクター/stopとFile::closeを取り込んだ参照寿命テストで、WebServer相当の所有者を先に破棄しても継続し、最後にsocket/RX/Fileを解放します。
- 統合: 実HTTP transportにEAGAINを注入した待機中に、PS4接続/切断/再接続/×停止とコピー済みPS4診断の送出が進むことを確認。既存Wi-Fi監視テストはpending中の要求受付抑止と切断検出を確認します。終了時のreason/calls128/retries64とTIMING/PROGRESSのSerial整形を検証します。
- 各コミットでESP32ビルド成功。検証はホストとビルドであり、実機成功を示すものではありません。

## 実機未確認と範囲

まずSDK UART preflightの実戻り値と表示を確認し、ブラウザーなしのPS4接続/切断/再接続、次にWebのHTML/CSS/JS完送・状態更新で比較します。失敗時はRTCのprevious_boot/reset/requested/capsと、preceding_sampleを分けて確認します。HTTP終了理由、max_gap_ms、retry_wait_ms、進捗時刻、実内部空き/最大連続領域も同一起動で比較します。

UART初期化後のppTask abort解消、RTCの実保持、Wi-Fi/PS4共存の余裕、制御周期/最大遅延、TCP解放時期は未検証です。2秒は応答の寿命であり、その間制御を待たせる処理ではありません。ただしOS send/closeやFile.read内部の時間まで厳密保証するものではありません。

今回の予算変更は正常なHTML/CSS/JS応答だけです。状態API/小エラー/操作受付等の既存上限付き同期応答と、正常CSVの旧core送信は維持します。PID/FOC/PS4操作/停止条件/全トレース/FIFO/スタックは不変。マージ/実機書き込みは行いません。
