> 最新画面・制御値の対応・RAM・更新手順は [制御グラフ](control-graph.md) を参照。以下のRAM比較は当初移行時の記録。

# WebSocket診断とブラウザー履歴への移行

基準: codex/2208-web-ps4 / 7056ad6（取得したFETCH_HEADとローカルHEADの一致を確認）。
PR #13の既存変更を保持した更新。PS4を基本操作とし、WebSocketには走行指令・PID設定を持たせない。

## 通信と上限

同一LANのHTTP `http://<IP>/` から、CSS→JSの直列取得完了後に
`ws://<IP>:81/diagnostics` を開く。外部CDN/新しいWebSocketライブラリに依存しない。
使用Arduino core/SDKの公開socket APIとmbedTLSのSHA1/Base64でRFC6455のhandshakeを行う。
SDK/TCP内部へのパッチやTCP容量変更はない。HTTP要求解析の照合付きWDT対策も維持する。

1セッション、20 Hz（50 ms）固定。listener backlog=1。接続中の第2クライアントは即時close。
listener確保/bind/listen失敗は資源をcloseし、5秒以上あけて再試行する。
これは診断側だけの再試行で、FOC初期化条件や走行停止条件を追加しない。
ソケットはHTTPのWebServerとは独立して通信タスクだけが所有する。
NetworkClientの受信キャッシュを作らず、recv/send(MSG_DONTWAIT)で処理する。

- RX領域1025 B（最大入力1024 B＋終端）。handshake全体1秒/無進捗200 ms。
- JSON本文1000 B以下、masked/FINの単一text frame。分割frame、binary、64bit長は拒否してclose。
  ping/pong/closeを扱う。ping最大125 B。未完frameも全体1秒/無進捗200 msでclose。
- TX領域2052 B、JSON生成に使うのは2048 B（終端を含む）、frame header最大4 B。
- 送信中はその1枠と送信位置を保持し、未送信部分を上書きしない。
  全体2秒、無進捗500 ms、累積128 send/64 retry、EAGAIN/ENOMEM/EINTR後10 ms以上待つ。
  HTTPの既存送信予算を緩めず、WSも同じ有限予算で扱う。
- 1周はrecv最大128 Bまたはsend最大256 Bを1回。SHA/購読解析も有限入力だけを処理。
  毎周、既存のWi-Fi監視/PS4診断/HTTP配信/イベント出力へ戻り、末尾のRTOS待機を維持する。
- 送信枠が埋まっている間の配信予定点は破棄してdroppedへ加算。スケジュール遅れの飛ばした点、
  終了時の未完sampleも加算する。未完JSONを完了させる前に次サンプルを継ぎ足さない。
  全失敗・切断・Wi-Fi喪失で接続をclose。待機キューや測定履歴を機体に蓄積しない。

購読例:

```json
{"type":"subscribe","request_id":1,"period_ms":50,"fields":["angle_error_deg","right_velocity_rad_s"]}
```

応答:

```json
{"type":"subscribed","boot_id":"0123456789abcdef","request_id":1,"subscription_id":1,"period_ms":50,"streaming":true,"fields":["angle_error_deg","right_velocity_rad_s"]}
```

JSONのtype/request_id/period_ms/fields以外のキー、重複キー・項目、未知項目、50以外の周期、
空リスト、過大数値、不正構文を拒否。設定は全部検査した後だけ一括更新する。
文字列はこのASCIIスキーマの直接表記を受理し、escape表記や別の汎用JSON形式は未対応。
エラーは `{"type":"error","reason":"invalid_subscription"}`。現在の購読は不変。
`{"type":"stop","request_id":2}` は診断配信だけを止め、世代を進めてstreaming=false/fields=[]を応答する。
車輪の停止・PS4停止操作とは無関係。Web切断でも走行要求を解除しない。

測定sampleは選択項目だけをvaluesへ生成する。全項目を送ってブラウザーで隠す方式ではない。
固定メタデータはboot_id（無線起動後の64bit乱数）、subscription_id（受理ごとに更新）、
seq（配信予定点ごと、破棄点を含め増加）、source_seq（制御側snapshotごと）、sample_ms、
foc_capture_us、age_ms、dropped、stop_reason/stop_ms、faults、running/disabled/ps4/foc_ready。
非有限値はnull。配信停止中/購読前は測定values={}のstatusを1 Hzで送り、最新状態を表示する。
停止理由・時刻は再開始で消さず最新1件を固定長保存する（電源断/再起動で失われる）。

seqの飛びとdroppedからブラウザーが欠落を表示し、boot_id変更を再起動/新セッションとして扱う。
サーバーsend成功はブラウザー受信確認ではないため、最後の切断区間の全欠落数は確定できない。
ブラウザーは送信破棄累計も別表示する。停止・切断時にも最新理由/異常の最後の表示を残し、
通信状態に切断/更新停止を表示する。自動再接続はせず、手動ボタンで再接続できる。

## 取得経路・項目

制御ループ末尾/初期化待ち経路で50 msごとに、既存変数から216 BのSnapshotを作り、
短いcritical sectionで最新1個へコピーする。追加I2C読み取り、制御側の動的確保、JSON整形、
通信待ちはない。通信タスクは同じlockでコピーしてからメモリー情報・JSONを生成する。
従来のHTTP状態キューは残し、既存の状態API/PS4シリアル表示も維持する。

| キー | 単位・符号 | 取得時点 |
|---|---|---|
|angle_deg / reference_deg / angle_error_deg|deg、既存AngleY / 基準角 / その差|最新制御更新後|
|gyro_deg_s|deg/s、既存GyroY|同上|
|right_position_rad / left_position_rad|rad、各shaft_angleに既存車体前進符号を適用|直近loopFOCの既存センサーキャッシュ|
|right_velocity_rad_s / left_velocity_rad_s|rad/s、各shaft_velocity（SimpleFOC LPF後）に既存車体前進符号を適用|同上|
|mean_velocity_rad_s|rad/s、実際のdrive_speed_for_control|通常LPF後／PS4走行時は既存速度窓。既に平均化済みで再度÷2しない|
|angle_term_v / gyro_term_v|V、既存姿勢/角速度項|最新制御更新後|
|position_term_v|V、使用モードの符号補正済み位置項、未使用時null|同上|
|speed_term_v|V、使用モードの符号補正済み直接速度項、未使用時null|同上|
|total_output_v|V、test_LQR_u（左右制限/旋回の前）|同上|
|right_command_v / left_command_v|V、制限・旋回適用後|その周期のmove直前。停止強制記録時はゲート遮断後|
|right_foc_q_v / left_foc_q_v|V、既存motor.voltage.q|loopFOC後・その周期move前。foc_capture_us付き|
|control_dt_ms|ms、既存control_dt_ms|直近制御更新|
|loop_us / right_foc_us / left_foc_us / imu_us / move_us|us、既存の区間測定|最新ループの計測済み区間（新規I2Cなし）|
|internal_free_b / internal_largest_b|B、INTERNAL\|8BIT|通信タスクの送信準備時（制御snapshotとは時点が異なる）|

sample_msは制御snapshotの起動後millis、32bit周回をブラウザー側で展開する。
source_seqやage_msで同じキャッシュの再送/制御更新停止も判別できる。
取得タイミングの違うFOC/指令/メモリーを同時刻の物理測定値と扱わない。
faultsはbit1=右センサーfault、bit2=左センサーfault、bit4=FOC初期化失敗。
出力遮断表示は既存のemergency latchと既存ENキャッシュから作る。

## 画面と履歴の置換

初期選択は制御角度誤差・制御角速度・制御平均速度・左右車輪速度・左右指令電圧の7項目。
単一Canvasへ重ね描きし、単位ごとに独立した縦軸と共通のsample_ms横軸で直近60秒を表示する。
観測選択・詳細は初期状態で閉じ、凡例で線を切り替える。プリセットと追加6項目は[制御グラフ](control-graph.md)を参照。
項目変更は「配信開始」で送信し、受理した世代・項目・周期を画面に表示する。
配信停止はグラフ/履歴を残し、表示一時停止は受信・記録・異常表示を継続する。
ブラウザーの履歴消去・直近履歴CSV保存を提供。履歴は60秒かつ1200点以下で、永続保存しない。
再起動のboot_id変更時は異なる時刻軸を混ぜず履歴を消す。CSVには受信メタデータと選択値を保存する。
ブラウザー未接続・未購読・送信破棄区間の過去履歴は機体から復元できない。

機体のVelocity/LoopTiming/LqrControl/DriveSign/Stopの5測定履歴と、CSVコピー/同期要求/ダウンロード処理を撤去。
旧/trace/*.csvは上限付きの410を返し、ブラウザー記録への移行を案内する。
シリアルの旧履歴コマンドも保存先変更を案内し、通常の現在値診断・停止手段は残す。
短い解放イベント出力リング、RTC確保失敗、左右FOC観測は既存不具合の切り分け用として維持。
PS4速度窓の履歴は速度制御に必要なので維持。PS4 FIFO/Batch・スタックは削減しない。

## RAM（7056ad6との比較）

最終ビルドの値はPR検証結果にも記載する。確保サイズはESP32 ELF/DWARFとシンボルで確認。

| 撤去する履歴 | 1点 | 個数 | 領域 |
|---|---:|---:|---:|
|Velocity|60 B|128|7680 B|
|LoopTiming|24 B|64|1536 B|
|LqrControl|84 B|128|10752 B|
|DriveSign|84 B|60|5040 B|
|Stop|64 B|50|3200 B|
|合計|||28208 B|

旧CSV用10776 Bコピーは元々取得時だけの動的領域であり、常駐削減量には足さない。
CSV撤去により、その一時確保自体も不要になる。
新規主要領域: 最新Snapshot192 B、最新停止理由64 B＋時刻4 B、WS固定コンテキスト3168 B。
WSコンテキストにはRX1025 B/TX2052 Bと送信位置・期限・購読mask/世代/seq等の管理を含む。
listener/接続fdは通信タスクだけで保持。TCP内部の動的資源はこれと別で、接続時の実機測定が必要。
既存HTTP状態キューは2052 B×1＋RTOS管理分、HTTPスタック8192 B、PS4 FIFOオブジェクト2144 Bを維持。

| 領域 | 7056ad6 | 今回 | 差分 |
|---|---:|---:|---:|
|.dram0.data|28852 B|29124 B|+272 B|
|.dram0.bss|77864 B|52800 B|-25064 B|
|静的DRAM計|106716 B|81924 B|-24792 B|
|RTC_NOINIT|124 B|124 B|0 B|
|IRAM text|113079 B|113155 B|+76 B|
|Flash|1898153 B|1894805 B|-3348 B|

常駐差分は、5配列-28208 B、WS3168 B＋Snapshot192 B＋最新停止68 B、
その他の追加/撤去された管理値・項目名ポインター・配置調整等の純差-12 Bで計-24792 B。
送信中のSnapshotコピー192 BやJSON整形の局所値は既存HTTPスタック内で使用し、
新しいタスク/スタック予約は追加しない。局所スタック使用量の実機高水位は未測定。

## 検証と制限

- 実protocol/formatter/WS状態機械: 選択項目だけの生成、全26項目、最大float/null/escape、
  不正/過大JSON・重複/未知キー・原子的更新・購読変更/停止/世代、固定領域不足、
  部分送信、EAGAIN/ENOMEM、低速時の破棄、期限・未完要求・不正frame/mask、ping、切断/再接続。
  poll中のmalloc/calloc/realloc/newを禁止した試験も行う。
- 実socket adapter/listenerをsyscall doubleで検証: socket/fcntl/bind/listen失敗の解放、
  1クライアント/第2拒否、Wi-Fi喪失/再試行、snapshot排他、RFC handshake入力の構成。
- 実HTML/JS: HTML→CSS本文→JS準備→WSの順、失敗時の後段抑止、購読JSON・受理世代、
  boot/欠落表示、一時停止中の状態更新、履歴消去、60秒/1200点・時計周回・CSV。
- 実制御関数: 既存のPID/符号/開始・中立・停止・転倒/センサーfault・PS4入力/切断/再接続、
  WSのEAGAIN待機中もPS4通知と×停止が進み、最新停止記録が再開始で消えないこと。
- HTTP要求解析WDT対策、既存送信期限/所有権、FOC観測、UART/確保観測、I2C/起動の回帰も実施。

実機未検証: ブラウザー完送、無線TCPとPS4の併用、20 Hzの実効周期/制御周期への負荷、
実メモリーfree/largest・スタック高水位、SDK内のclose/crypto実行時間、WDT再発有無。
ホストの非ブロッキングモデルとビルド成功を実機成功とは扱わない。
PID/検証済み符号/FOC/配線/センサー取得・PS4ペアリング/操作/停止条件は変更しない。

## ビルドと書込み手順

プロジェクトルートで（PORTはCOM番号等に置換）:

```sh
pio run -e esp32dev
pio run -e esp32dev -t buildfs
# 以下は利用者が停止・安全な状態を確認して行う。今回は実行していない。
pio run -e esp32dev -t upload --upload-port PORT
pio run -e esp32dev -t uploadfs --upload-port PORT
```

今回はファームとdataの両方が変わるため両方の書込みが必要。
ファームだけでは画面は更新されず、dataだけではWebSocket配信は追加されない。
キャッシュを残した古いタブを閉じて新しいタブで再読込みし、1台/1タブで比較する。
まずPS4未接続・倒立未開始で購読6項目/26項目、停止/再開、切断・再接続、CSV保存を確認し、
次にPS4接続/入力/切断・再接続とRAMを比較する。静止倒立・走行等の成功を通信試験から推定しない。
マージ・実機書込みは行っていない。

実施コマンド:

```sh
python tests/telemetry_host/run.py
python tests/telemetry_host/socket_lifecycle.py
node tests/web_serial_host/run.js
python tests/drive_host/run.py
TELEMETRY_SANITIZERS=1 ASAN_OPTIONS=detect_leaks=0 python tests/telemetry_host/run.py
CSV_TEST_SANITIZERS=1 ASAN_OPTIONS=detect_leaks=0 python tests/drive_host/run.py
```

HTTP core照合/要求解析/固定ファイル/継続配信/固定応答、解放・UART確保観測、
操作、HTTP起動、PS4イベント/HID/Classic、FOC観測/パッチ、I2C資源、新規telemetryの
16系統とsocket lifecycle/ブラウザー試験が成功。telemetry/操作/HTTP送信2系統はASan/UBSanでも成功。
ESP32ビルド・LittleFSイメージ生成成功。実機書込み・実機通信成功の確認は行っていない。

## 初回sample切断の追試版

購読受理後seq=0/1006への対処と、通常ログ/診断設定を
[websocket-first-sample.md](websocket-first-sample.md)へ記載。
通常は周期MEM/詳細HTTP/PS4/FOC表示と長い自動メニューを出さず、WS接続/切断と初回sampleの最小観測を残す。
手動h/?は維持する。表中RAMはd4633ba時点の値で、追試版の値は同文書/PRを参照。
