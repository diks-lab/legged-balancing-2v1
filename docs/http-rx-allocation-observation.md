# 解析済みGETのRX参照と応答単位の確保失敗

基準55406fd、head codex/2208-web-ps4、PR #13。PS4切断後のfree17972/largest13300でもHTMLは460 ms・retry34で完送、CSSは256/440 B、JSはヘッダーのみで無進捗501 ms終了した実機報告を受けた変更です。RAM不足やACK待ちを原因確定したものではありません。

## TX専用の共有参照

正常なファイルGETで、NetworkClientをコピーしたHttpTransmitClientが自分の_rxBuffer参照だけをresetします。ソケットのshared_ptrはそのままコピーし、stop/closeは呼びません。ContinuingHttpFileにはこのTX専用コピーのbaseを渡します。WebServerのRX参照は解析・ハンドラー処理中に維持され、ハンドラー後の参照解除で最後のRX所有者が消えると受信領域を回収します。socket/Fileはコンテキストに残り、EAGAINや部分送信後も継続します。

core 3.0.3の受信キャッシュは実際の読み出しでmalloc(1436)します。公開clear()は領域を解放せず、stop()は共有ソケットも閉じるため使いません。今回のアダプターはprotectedメンバーへの依存です。scripts/check_http_client_core.pyでバージョン3.0.3とNetworkClient.h/cpp全体の既知SHA-256をビルド前に読み取り専用で検証します。CRLF→LFだけを正規化し、未知の版/本文ならビルドを停止します。core/SDKソースは一切書き換えません。

撤去条件: 上流にRXだけを解放して共有ソケットを保つ公開APIが提供された場合、またはWebServerの解析/所有権設計を置換した場合に再評価します。現版での当て推量の互換運用はしません。

成功、各予算上限、送信/読出し失敗、切断、Wi-Fi喪失、取消の最終stop/closeは既存の共通終了に維持します。RX参照解除時と応答終了時を区別し、RXを返すためにsocketを閉じません。現在の正常ファイルGETだけが対象で、状態/小エラー/CSV等の受信参照方式は変更しません。

## 応答中の確保失敗

新しいAllocationResponseObserverは56 B固定です。既存RTC124 Bとは別で、元の各core最初のRTC記録を上書き/消費しません。RTC slotが使用済みでもライブ集計は継続します。

HTTPタスクが登録ハンドラー開始でwindowを開き、同期応答はハンドラー終了、継続ファイル応答は接続/File解放後に閉じます。期間中の両coreの確保失敗callbackを数え、各coreで最初に観測できた要求のsize/capsを保持します。TCP限定ではなく、BT等の他タスクも含む同時期のシステム全体の失敗です。先行するaccept/要求解析、window終了後、coreの自動応答等は集計範囲外です。境界の帰属はcallbackがslotを取得してactive windowを読む時点で決まります。

callbackではcore slotのロックフリーCASを一度だけ試みます。競合した場合は再試行/待機せず、欠落フラグを立てます。出力、動的確保、ヒープ走査、キュー操作はしません。HTTP側の開始/終了でもslotを一度だけ試み、終了時に取得できなかった場合は値を推測せずunavailableを表示します。世代が折り返しても旧件数を再生しません。競合境界では保守的にdroppedが残る場合があります。

終了時だけ既存32件リングへcoreごと1レコードを保存し、既存の最大48 B/回・空きUART範囲の送出を共用します。リングの容量、タスク/スタック予約は増やしません。

```
HTTP_ALLOC ms=... route=... core=0 count=... first_size=... first_caps=0x... saturated=0 unavailable=0 dropped=0 ring_lost=...
HTTP_ALLOC ms=... route=... core=1 count=... first_size=... first_caps=0x... saturated=0 unavailable=0 dropped=0 ring_lost=...
```

同一のroute/msを持つ2行が同じ応答の結果です。HTTPタスクは直列で処理します。通常のヒープ残量や直近sampleとは別の、実際の失敗要求です。first_size/capsは最初に記録できた1件だけで、全要求サイズの内訳ではありません。

| 項目 | 意味 |
|---|---|
|count|記録できた件数。下位29bit、最大536870911|
|saturated|件数が飽和。以後の正確な数は不明|
|unavailable|終了時にslotを取得できず、件数/要求値を確定できなかった|
|dropped|callbackまたは開始時の競合で観測欠落の可能性あり。欠落数そのものではない|
|ring_lost|既存の全観測リングで捨てた累積件数。ライブsummary自体も破棄対象|

両行が揃い、各フラグが0で、リング欠落もない場合だけ「このwindowで記録されたcallbackが0件」と読めます。欠落時のcount=0を失敗なしとは扱いません。またcallbackが0でも、TCPキュー制約やACK待ち、mallocを伴わないERR_MEMがないことを意味しません。

event22/23=MemHttpAllocCore0/1、cid=route、ms=終了時刻、free欄=first_size、largest欄=first_caps（ヒープ値ではない）、arg=件数とflagsです。bit29=saturated、30=unavailable、31=dropped。

## テストとRAM

- 実coreのNetworkClient constructor/destructor/stop、NetworkClientRxBuffer本体、File::closeを取り込み、実際に1436 B受信キャッシュを確保してから解析完了相当のコピーを作ります。TXコピー生成時には元のRXとsocketが残り、WebServer相当の参照解除後はRXだけが消え、socket/Fileが有効なことをassertします。
- LF/CRLFの実3ファイル、成功、部分送信、EAGAIN継続、無進捗/期限/回数、切断、読出し失敗、取消、全参照解放、再利用、確保禁止中の継続を確認。ASan/UBSan成功。
- 実確保失敗callbackで、RTC記録の不変、以後のライブ件数、応答外除外、連続window、両core並行、競合drop、終了snapshot失敗、飽和、世代折返しを注入。callback中のmalloc/calloc/realloc/new/出力/ヒープ走査を禁止して確認。ASan/UBSan成功。
- 同期応答の終了と継続応答の遅延終了、EAGAIN待機中のPS4処理/診断、summaryの実リング投入と整形、欠落表示を回帰。core照合のLF/CRLF/繰返し/未知本文拒否も確認。

ESP32ビルド成功。55406fdとの比較:

| 領域 | 前 | 今回 | 差分 |
|---|---:|---:|---:|
|静的DRAM(data+bss)|106668 B|106716 B|+48 B|
|IRAM text|112927 B|113079 B|+152 B|
|RTC_NOINIT|124 B|124 B|0 B|
|送信コンテキスト|400 B|400 B|0 B|
|Flash|1901205 B|1902213 B|+1008 B|

ライブ集計の実オブジェクトは56 B、配置調整を含めたDRAM総増は48 Bです。受信済みGETの最後のRX参照が消える場合、1436 Bの動的payloadとRXオブジェクト/管理領域を従来より早く返せます。payload分だけと静的DRAM増だけの差は1388 Bですが、実機のヒープ差分や最大連続領域は未測定です。要求解析前のピーク消費やブラウザーの他TCP接続をなくす変更ではありません。

全体2秒/無進捗500 ms/EAGAIN後10 ms/send128回/error64回、TCP容量、256 B payload、スタック、FIFO、全トレース、PID/FOC/PS4操作/停止条件は不変です。dataとブラウザーの並列読込みも変更しません。

## 今回含めない変更と実機確認

TCP内部の観測パッチ、tcp_write/tcp_output wrapper、SDKソース/事前ビルドarchiveの変更は含めません。将来必要なら、対象SDKとlwIPの版/ビルド設定/バイナリー一致、観測を含むオブジェクトの実リンクを確認できる方法を先に示します。ソースだけの変更を実機反映済みと扱いません。

ビルド成功は実機完送成功を意味しません。PS4未接続で単独GETとブラウザー取得を比較し、HTTP_ALLOC両行/欠落フラグ、実内部free/largest、HTML/CSS/JSの完送とPS4再接続を確認してください。RX回収の実ヒープ差分、ACK待ち/確保不足の因果、制御周期と遅延は実機未検証です。マージ/実機書き込みは行いません。
