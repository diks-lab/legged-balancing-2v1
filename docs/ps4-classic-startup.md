# Classic専用起動と実機確認

この変更はBluetooth起動モードだけを対象とする。WebSocket、ブラウザー記録、
100 Hz化、機体側トレース削減は含まない。PID/FOC/ペアリング/操作/停止条件は維持する。

## 公開APIによる起動

使用環境はArduino ESP32 3.0.3 / IDF 5.1.4、platformio.iniで既存プラットフォームを固定。
`include/Ps4ClassicBluetooth.h` は初回IDLE状態を確認し、公開API
`btStartMode(BT_MODE_CLASSIC_BT)` で起動する。使用中coreの実装は、
controller設定modeをClassicにし、初期化前に未使用BLE controllerメモリーの解放を呼び、
同じClassic modeでenableする。その後、引数なしの `PS4.begin()` を呼ぶ。
PS4ライブラリはbtStartedが真なので既定のBTDM起動を呼ばない。
MAC/ベースMAC/ペアリング情報の変更や削除は行わない。

既にINITED/ENABLEDならモード不明として拒否し、停止/再初期化を試みない。
core開始失敗/ENABLED未到達でもPS4.beginを呼ばず、失敗を表示する。
自動再試行は追加しない。PS4ライブラリ初期化と実際の接続は既存通り別状態。
PS4失敗/未接続をFOC開始の新しい必須条件にはせず、Web診断は引き続き起動を試みる。
I2C先行確保→Bluetooth→Web必須機能→FOCの順序は変えない。

表示 `PS4 Bluetooth startup: CLASSIC_ONLY_READY` はcontroller開始成功を示す。
BLE解放量、PS4接続や入力成功を保証する表示ではない。core内のmem_release戻り値は
公開APIが直接返さないので、回収量は実機の前後値で評価する。
Classic専用でもコンパイル済みBluedroid host側のBLE関連領域すべてが消えるわけではない。

## ログと実機確認の順序

必要なら診断用ビルドだけ `-DHTTP_MEMORY_STAGE_DIAGNOSTICS=1` を使う。
既定OFFは維持。既存before_ps4/after_ps4に加え、before_classic_bt/after_classic_btで
内部空き/最大連続領域を記録する。通常時はモード結果・既存起動結果/IP/URL/最終メモリーを表示。
Web `/api/status` には従来通り現在の内部空き/最大連続領域がある。

利用者によるファーム書き込み後、次を同じ起動で記録する。

1. Classic開始結果、PS4ライブラリ結果、Wi-Fi/IP/HTTP結果、FOCと最終起動結果。
   起動後のWeb表示が利用でき、PS4未接続でも診断できること。
2. PS4接続前、接続成立後、Web状態更新併用中の内部空き/最大連続領域。
   PS4の接続カウンターと実接続表示が変わること。
3. 左右スティック/トリガー、ボタン押下/解放をWeb/Serialで観測。
   接続だけや保持した開始ボタンで倒立/走行を開始しないこと。
4. 切断/再接続を繰り返し、切断イベント・保持要求解除・中立/解放待ちを確認。
   再接続だけで開始しないこと。メモリーが毎回減り続けないこと。
5. 通常のPS4開始/中立停止/×全停止は既存手順で別途確認。
   静止倒立確認済みの基準と、この版の操作検証を混同しない。

初回はCSV取得を併用せずPS4＋Wi-Fi＋Web状態表示を単独評価する。
その後CSVも別試験とし、10,776 B＋管理分の一時確保が必要なことを考慮する。
HID失敗時のrequested/free/largestログは維持。確保失敗後にLED送信や接続を続けない。
評価は内部8bitの残量で行い、heap_free全体だけで判断しない。
以前の接続時free1624/largest1460との比較は同一条件で行う。
以前提案した16 KiB/最大連続6 KiBは暫定の余裕目標で、API保証値や新しい停止条件ではない。

実機の接続/入力/切断/再接続とWi-Fi併用時の余裕が確認されるまでWebSocketは追加しない。
確認後の初期仕様は診断専用・1クライアント・20 Hz・ブラウザー側グラフ/記録/CSV。
100 Hz化と既存トレース削減は比較結果を見て別途判断する。
