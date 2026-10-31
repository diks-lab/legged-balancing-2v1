# HTTP要求解析のWDT対策

## 照合結果

対象はArduino core 3.0.3 / 使用SDK v5.1.4-497-gdc859c1e67-dirty。
実機ELF SHA先頭7ebec3082cea7811の解析結果は利用者が照合済み。
Stream::readStringUntil → WebServer::_parseRequest → handleClient → httpTaskで停止した。
ホストで生成したELFが実機クラッシュELFと同一とは扱わない。

- WebServer::handleClientはStream::setTimeout(HTTP_MAX_SEND_WAIT)を呼ぶ。5000はミリ秒。
- Stream::timedReadはread()が負値でもmillis()まで待機なしで反復する。
  readStringUntilはこれを文字ごとに呼び、全要求の期限を持たない。
- NetworkClientには別の_timeoutフィールドがあり、Streamの待ち時間とソケット設定は同一ではない。
- NetworkClientRxBuffer::fillBufferの1436 B malloc失敗は0を返し、readは-1へ変換する。
  RX確保失敗やreadエラーで進捗しなくてもStream側が回り続け、Idle0を飢餓状態にできる。
- legacy解析はタイムアウトで返った部分行を受け付ける場合があり、不完全な要求から操作を実行してはならない。

## 修正と適用の確認

scripts/bounded_http_request_overlay.pyはインストール済みParsing.cppを変更せず、
プロジェクトのビルド領域へ対象照合済みの置換ソースを生成する。
版とLF正規化全体SHAを検査し、不一致なら停止。CRLF/LFで同じ生成結果になる。
PlatformIO middlewareでそのファイルをコンパイルし、ELF生成後にbounded readerと
唯一のWebServer::_parseRequestがリンクされていることをnmで検査する。
SDKアーカイブ、TCP内部、全体のStream、WDT設定にはパッチを当てない。

要求解析全体1000 ms、無進捗200 ms。read負値は即時失敗。
受信待ちはdelay(1)、入力が続く場合も32バイトまたは処理2 msごとにdelay(1)。
CRLF完結を必須とし、1行1024 B、ヘッダー全体8192 B、要求行＋64ヘッダー＋空行まで。
本文は4096 B以下、Content-Lengthを厳密検査し、重複・負数・不一致を拒否する。
途中の本文・String生成失敗でも接続を終了し、ハンドラーへ渡さない。
失敗時はparser所有接続をstopし、WebServer側の既存終了処理も維持する。
成功時は従来どおり応答ハンドラーへ渡す。送信側の所有権・予算は変更しない。

GET/HEAD、本文なしの既存操作POST、通常のplain/urlencoded POSTを維持。
アプリに登録されていないmultipart/raw uploadとTransfer-Encodingは拒否する。
将来uploadを追加する場合は独立した有限・協調型の受信実装が必要。
解析は同期処理なのでHTTPタスクの監視サービス復帰には最大約1秒かかり得るが、
各待機でIdleへ譲り、別coreの制御ループを待たせる同期処理は追加していない。
RTOS tick丸め・実行スケジュールによる遅れは実機で確認する。

## 観測と2308 B要求

HTTP_REQUESTには終了reason・実際の読取バイト数・解析経過msを記録する。
reason: 0成功、1全体期限、2無進捗、3read失敗、4切断、5String確保失敗、
6行上限、7総量上限、8行数上限、9構文、10本文上限、11本文確保失敗、
12未対応形式、13その他の解析拒否。
既存固定リングを使い、欠落はring_lostで表示する。
HTTP_ALLOC route=0は解析期間、route=1等は従来どおり応答期間。
開始前のaccept/availableは解析窓に含まれない。RTC観測・欠落表示を維持する。

2308 B / caps=0x1800は確保元未確定。
使用SDKのheap_caps_malloc_defaultは通常mallocにもDEFAULT|INTERNAL(0x1800)を使う。
RXキャッシュ本体の要求は1436 Bであり2308 Bと同一ではない。
core0・応答窓内・同一size/capsという記録だけでTCP/Wi-Fi由来とは断定できない。
ブラウザーでは51/40件、順次curlでは0件という差は併用時の資源圧迫を示唆するが、
2308 B失敗が後続RX確保失敗を起こしたという因果や呼出元は証明していない。
RTC preceding_sampleは過去のヒープ観測であり失敗瞬間の残量ではない。
今回TCP内部観測を追加しておらず、事前ビルド済みSDKにソース編集が反映されるとも扱わない。

## 検証とRAM

実coreの置換_parseRequestと実NetworkClientRxBufferをホストでコンパイルし、
RX malloc失敗、readエラー（途中を含む）、String/body OOM、不完全ヘッダー、
途中停止、遅いトリクル入力、各入力上限、切断、時刻周回、正常GET/POST、
失敗後の次接続を検証。失敗後の有限終了・閉鎖・delayによるIdle機会を検査する。
ASan/UBSanも実施。これは実機WDTや無線TCPの成功を証明する試験ではない。

ESP32ビルドの静的DRAMは106716 B（data28852 / bss77864）、基準8e86128と差0 B。
RTC124 B・IRAM text113079 Bも不変。新しい記録配列・タスクは追加しない。
helperはELFのDWARFで36 Bと確認した解析時のローカル状態で、既存タスクのスタック予約サイズは変更しない。
要求StringとRXキャッシュは引き続き動的だが入力上限を設け、本文確保は最大4097 B。
Flashは1902213 → 1898153 B（4060 B減）。

実機未検証: ブラウザーの並列取得完送、低RAM時のWDT再発防止、PS4併用、
実際のスケジュールとヒープ推移。PS4操作、PID/FOC、停止条件、FIFO、トレースは変更しない。
書込み・マージは行っていない。

再現コマンド（PlatformIOのcore配置は環境に合わせる）:

```sh
PLATFORMIO_CORE_DIR=/workspace/pio-core python tests/http_request_host/run.py
HTTP_REQUEST_SANITIZERS=1 ASAN_OPTIONS=detect_leaks=0 python tests/http_request_host/run.py
PLATFORMIO_CORE_DIR=/workspace/pio-core /workspace/pio-venv/bin/pio run -e esp32dev
```

関連回帰: HTTP core照合/固定応答/ファイル/継続所有権、解放観測、UART確保観測、
操作/CSV、起動、PS4イベント/HID/Classic、FOC観測/パッチ、I2C資源の15系統が成功。
ホストのdelay呼出検査はIdle実行機会の確認であり、実機でIdleが動いた測定ではない。
