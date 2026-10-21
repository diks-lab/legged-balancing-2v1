> 最新の初期選択・最低軸範囲・選択自動反映は [姿勢中心グラフ](attitude-graph.md) を参照。以下はd367acf時点の設計・検証記録。

# 制御試験用の重ね描きグラフ（PR #13）

取得した `codex/2208-web-ps4` の先頭 `8f9fe98` を基に更新。
PRの基準は引き続き `codex/2208-short-drive` / `dfd4d0c`。
PID・検証済み符号・FOC・センサー処理・PS4操作・停止条件は変更しない。

## 画面と操作

上から状態、重ね描きCanvas、色付き凡例と基本操作、閉じた観測選択、閉じた詳細。
グラフの高さを残りの表示領域に合わせる。ページの縮小や文字の極小化はしない。
凡例のボタンは線の表示だけを切り替え、ESP32への選択変更は送らない。
色は項目に固定。左右車輪速度と左右指令電圧にも別の色を割り当てる。
1つのグラフで単位別の独立した縦軸を左右に配置し、各軸に単位と上下値を表示する。
同じ単位の信号だけが尺度を共有する。横軸はESP32取得時刻、直近60秒。

選択プリセット（選択後も個別変更可、適用は「配信開始」）：

| プリセット | 項目 |
|---|---|
|自立静止（初期）|制御角度誤差、制御角速度、制御平均速度、左右車輪速度、左右指令電圧（7）|
|位置保持|自立静止＋基準位置誤差（8）|
|PS4走行・旋回|制御角度誤差、制御角速度、目標速度、制御平均速度、目標傾斜角、旋回指令、左右車輪速度、左右指令電圧（10）|

- 「配信停止」は測定配信だけを止め、状態は1 Hzで受信。走行停止ではない。
- 「表示一時停止」は描画だけを止め、受信、60秒／1200点の履歴保持、異常・停止理由・欠落表示は続く。
- 履歴消去、手動再接続を基本操作に置く。CSV保存は詳細欄の補助機能。
- seqの飛び、dropped増加、世代変更、再接続、150 msを超える取得間隔で線を切る。
  null／NaN／未選択値の間も結ばない。補間・平滑化フィルターは追加しない。
- 再起動（boot_id変更）は履歴を消去。未接続・未配信・欠落区間の履歴は復元不可。
- 選択数32に増やしても、送るvaluesは受理した選択項目だけ。1クライアント・20 Hzは不変。
  WSに走行指令／PID変更を追加せず、切断による新しい停止条件も設けない。

## 制御値との対応

`updateLqrDiagnostic()` の実際のPID入力直前に、固定24 Bの観測値をコピーする。
制御に渡す式の値と、表示のために似た式をブラウザーで再計算した値を混同しない。
姿勢のみ・位置無効・速度直接項無効・未開始などの未使用制御項はnullにする。
未開始時の制御入力／項は描画されない。車輪値、姿勢角などの観測値は選択可能。

| JSONキー | 単位・符号・取得元 |
|---|---|
|angle_deg|deg、既存MPU6050 getAngleY（既存相補フィルター後）→LQR_angle|
|reference_deg|deg、既存angle_zeropoint|
|angle_error_deg|deg、LQR_angle−angle_zeropoint。**基準角との差**で、走行時の制御角度誤差と別項目|
|control_angle_error_deg|deg、実際の姿勢PID入力lqr_angle_error。LQR_angle−(angle_zeropoint＋既存PS4目標傾斜)−既存走行バイアス。姿勢のみモードも実入力を捕捉|
|gyro_deg_s|deg/s、実際に角速度PIDが使うLQR_gyro＝既存getGyroY。新しいLPFはない|
|right_position_rad / left_position_rad|rad、既存shaft_angleに既存kRightForwardSign／kLeftForwardSignを適用。車体前進が正。車体姿勢角ではない|
|right_velocity_rad_s / left_velocity_rad_s|rad/s、既存shaft_velocityに同じ車体前進符号を適用。SimpleFOCのsensor_directionとLPF_velocity適用後|
|mean_velocity_rad_s|rad/s、実際に速度PIDの誤差計算で選ばれたdrive_speed_for_control。通常はlqr_speed_for_control（既存車体速度LPF後）、PS4走行要求中はlqr_ps4_window_speed（既存制御用速度窓）|
|speed_raw_rad_s|rad/s、lqr_speed_raw_for_control。選択済みSimpleFOC／10 ms診断速度の車体LPF前比較値。SimpleFOC内部LPF前のセンサー生値ではなく、PS4専用速度窓とも異なる|
|position_error_rad|rad、実際の位置PID入力distance_zeropoint−LQR_distance。位置項無効時はnull|
|target_speed_rad_s|rad/s、速度PIDの誤差基準lqr_target_speed。前進要求が正|
|target_tilt_deg|deg、姿勢PIDに適用した既存PS4目標傾斜＋走行バイアス。正の目標を姿勢角から減算|
|yaw_command_v|V、applyPs4Steeringの既存ヘッドルーム制限後yaw。右から減算、左へ加算して既存モーター方向符号を適用。旋回不適用時0。後段の安全遮断は最終指令電圧に反映|
|angle_term_v / gyro_term_v|V、既存angle_control／gyro_control。未開始時null|
|position_term_v|V、FULLの検証済み反転・既存リカバリーランプを含むdistance_control。未使用時null|
|speed_term_v|V、既存speed_direct_control。モード未使用／PS4外側傾斜ループ／リカバリーで直接項不使用時null|
|total_output_v|V、既存test_LQR_u。左右電圧制限・旋回前。未開始時null|
|right_command_v / left_command_v|V、制限・旋回・安全判定後のmove直前。モーター側の既存符号を保持（前進正に変換しない）|
|right_foc_q_v / left_foc_q_v|V、既存motor.voltage.q。loopFOC直後・同周期move前のキャッシュ。foc_capture_usを併送。指令とは時点が異なる|
|control_dt_ms|ms、既存control_dt_ms|
|loop_us / right_foc_us / left_foc_us / imu_us / move_us|us、既存の最新計測区間|
|internal_free_b / internal_largest_b|B、INTERNAL\|8BITの空き量／最大連続領域。通信タスクが送信準備時に採取|

SimpleFOC v2.4.0の `FOCMotor::shaftVelocity()` は
`sensor_direction * LPF_velocity(sensor->getVelocity())`。
位置も既存shaftAngleの方向／オフセット補正後を利用する。
左右の車体符号は制御と同じ既存係数から取り、追加I2C読み取りを行わない。
`m1_direction = 0.5 * kRightForwardSign`、`m2_direction = 0.5 * kLeftForwardSign` は
既に平均化を含む。以前の表示専用の再度の÷2を撤去し、実際に選択した制御速度を使う。
**制御式・LPF係数・速度源の選択条件は変更していない。**

snapshotは従来どおり制御ループ末尾で50 msごとに整合コピーする。
sample_msはそのコピー時刻であり、新たな同時センサー測定の時刻ではない。
FOC取得時刻、送信時メモリー、取得値のage_msの違いも維持する。

## ファイル配信と更新

HTML解析後、CSS本文受信・適用 → signals.js → history.js → plot.js → app.js
の本文受信・実行 → WebSocket接続の順。一度に1件だけ取得し、失敗時は対象を表示して後段へ進まない。
CDN・無限再試行はない。各手編集ソースに改行とインデントを付ける。
全6ファイルを既存の上限付き継続送信で配信。8 KiB／ファイル、全体2秒、無進捗500 ms、
send128回／retry64回／EAGAIN後10 ms、要求解析WDT対策は維持する。

| ファイル | LFのバイト数 | CRLFのバイト数 |
|---|---:|---:|
|index.html|5686|5793|
|style.css|1853|1916|
|signals.js|2614|2677|
|history.js|1608|1657|
|plot.js|4503|4633|
|app.js|5860|6027|

初回取得は3件12813 Bから6件22124 B（LF、HTTPヘッダー除外）へ増加。
整形とグラフ機能による増加で、各応答の期限は緩めない。実機で各ファイルの完送を確認する。

**ファームとLittleFSの両方を更新する必要がある。** 通常uploadだけではdataは更新されない。
新JSと新ファームのフィールド定義を揃え、古いタブを閉じて再読み込みする。

```sh
pio run -e esp32dev
pio run -e esp32dev -t buildfs
# 以下は利用者が実機を安全に停止・保持し、対象ポートを確認して行う。
# 今回はエージェントによる実行なし。
pio run -e esp32dev -t upload --upload-port PORT
pio run -e esp32dev -t uploadfs --upload-port PORT
```

LittleFS失敗時の自動フォーマットなし。既存の起動順・FOCゲート・停止手段を維持する。

## RAMと通信量

通常設定SERIAL_DIAGNOSTICS=0のESP32 ELF同士を比較。

| 領域 | 8f9fe98 | 今回 | 増減 |
|---|---:|---:|---:|
|.dram0.data|29124 B|29172 B|+48 B|
|.dram0.bss|51768 B|51768 B|0 B|
|静的DRAM合計|80892 B|80940 B|**+48 B**|
|Flash|1892425 B|1893685 B|+1260 B|
|IRAM text|113155 B|113155 B|0 B|
|RTC_NOINIT|124 B|124 B|0 B|

最新snapshotは192→216 B（float 6項目追加、+24 B）。PID入力等の固定観測が24 B追加。
WSコンテキスト3176 B、RX1025 B／TX2052 B、JSON2048 B、HTTPファイルコンテキスト400 B、
HTTPスタック8192 B、PS4 FIFO2144 B／Batch2128 Bは不変。新規履歴・タスク・送信キューはない。
ローカルsnapshotとマッピング配列は各24 B増えるが、予約スタックは削減／拡張していない。
既に移行済みの機体側5履歴28208 Bを再導入しない。今回さらに撤去する履歴領域はない。

静的DRAMとは別に、分割JSのためWebServerのルート3件が増える。
対象ESP32 ELFのDWARFでFunctionRequestHandlerは76 B、Uriは20 B。
3×(76＋20)＝**288 Bの起動時ヒープ要求＋アロケーター管理／丸め分**が増える。
3パスともStringのSSOに収まる。キャプチャなし関数の登録で新しい測定履歴は持たない。
TCP/LittleFSの動的確保、最大連続領域と実スタック高水位は実機で比較する。
静的+48 Bだけを実行時の総消費増加とは扱わない。

同じ代表値0.1234567、seq123、sample_ms45600、停止理由NONE等を実formatterへ渡した比較：

| 選択 | JSON本文 | WSヘッダー込み | 20 HzのWS bytes/s |
|---|---:|---:|---:|
|旧初期6項目|448 B|452 B|9040|
|自立静止7項目|488 B|492 B|9840|
|位置保持8項目|519 B|523 B|10460|
|走行・旋回10項目|573 B|577 B|11540|
|全32項目|1142 B|1146 B|22920|

TCP/IP等を除く例示値で、実データの桁・null・理由・連番によって変動する。
全32項目の最大有限float、メタデータ最大値、停止理由escapeも既存2048 BのJSON上限内で試験。
選択を増やすと通信量・整形負荷は増えるが、送信待ちが1枠を超えて蓄積することはない。

## 検証範囲

実Chromium（模擬WS相手、deviceScaleFactor=1、100%）で確認：

- 1366×768、1920×1080の両方。3プリセットと全32項目選択、折りたたみ閉。
  document scrollWidth/scrollHeightと各要素の矩形がviewport内。
  グラフ実領域は1366×768で自立／位置1332×448、走行1332×416、
  1920×1080で各1886×760。スクロールなし。14–16 pxの通常テキスト。
- 1 Canvas、単位軸、固定色凡例の切替、選択変更が配信開始まで送られないこと。
- 表示一時停止中のグラフ不変／履歴・異常更新、欠落表示、配信停止と手動再接続、
  再起動時の履歴分離、CSV補助保存、全段の直列取得と取得失敗時の有限終了。
- 画面の目視も1366幅の走行プリセット等のスクリーンショットで実施。

ホスト試験（実機を使わない）：

- 実制御関数でPID入力、通常LPF速度／PS4専用窓速度、左右方向、位置誤差、目標傾斜、
  未使用項nullを比較。従来の開始・中立・停止・転倒・PS4再接続等の回帰も維持。
- 実JSON／WS状態機械：32項目、選択限定、世代、不正・過大JSON、部分送信、
  EAGAIN／ENOMEM、低速破棄、切断・再接続、時間／回数／容量上限、処理中のPS4配送。
- 履歴60秒／1200点、時計周回、seq／drop／無効値／世代／再接続による線分分離。
- HTTP実core照合・要求解析WDT対策・継続送信の所有権／解放／LF・CRLF全6ファイル。
- 関連16系統＋socket lifecycle成功。telemetry、制御、HTTP継続送信はASan/UBSanでも成功。
- ESP32ビルド、LittleFSイメージ生成成功。これらは実機成功を意味しない。

主要コマンド：

```sh
node tests/web_serial_host/run.js
node tests/web_serial_host/browser.js    # PlaywrightとChromiumが必要
python tests/drive_host/run.py
python tests/telemetry_host/run.py
python tests/telemetry_host/socket_lifecycle.py
python tests/http_continuation_host/run.py
python tests/http_file_host/run.py
pio run -e esp32dev
pio run -e esp32dev -t buildfs
```

実機未検証：新6ファイルの完送、WS20 Hz実効更新、制御周期／PS4応答への負荷、
Wi-Fi/PS4併用RAM・スタック余裕、新画面での静止倒立・位置保持・走行・旋回・脚操作。
マージ・実機書き込みは行っていない。

利用者の実機確認手順：

1. まずPS4未接続・倒立未開始・1ブラウザー／1タブで、6ファイル完送と
   世代／50 ms／選択数、seqの進行、WS_FIRST_SAMPLE、切断理由を確認。
2. 観測項目を開いて自立／位置／走行プリセットを選び、閉じて配信開始。
   実際の使用モードにない項目は線が出ないこと、単位軸と左右色を確認。
3. PS4接続／切断／再接続、表示一時停止中の異常・欠落更新、配信停止との違いを確認。
   詳細診断ONで必要時のみfree/largest・制御周期・スタックを比較。
4. 機体を保持できる条件で、既存PS4手順に従って自立→位置保持→走行／旋回／停止を段階確認。
   通信・ビルドの成功を機体制御の実機検証の代わりにしない。
