# 1応答限定の継続型Webファイル送信

9472ad2で本文失敗、7ea05d0でHTML/JSの50 ms期限とCSSの累積3 EAGAINによる終了を実機確認しました。継続型は維持し、1回の処理量と応答全体の許容時間を分離して予算を修正します。バッファ容量は変更しません。

## 所有権と進行

正常なHTML/CSS/JSの応答を静的コンテキスト1個で保持します。NetworkClientとFileの共有参照、256 B共用バッファ、未送信位置、開始時刻、全応答の計数を所有します。ヘッダー整形にも同じ256 Bを使い、有効長は192 B未満です。上限8192 B・実際のfile.size()に基づくContent-Lengthを維持します。

使用中のArduino core 3.0.3ではWebServerのハンドラー後の_currentClientリセットは共有参照を手放します。NetworkClientのデストラクターは明示stopを呼びません。コンテキストのコピーがソケット/RX領域を保持し、Fileもローカル参照消滅後に有効です。終了時にstop/closeしてから空の所有者を代入し、参照も解放します。

通信タスクの1周で非ブロッキングsend(MSG_DONTWAIT)を1回、またはFile.readを1回だけ実行します。部分送信では位置を保持し、残りを送り終えるまで次のreadをしません。EAGAIN/EWOULDBLOCK/EINTRでも位置を保持して戻り、同じ呼出内で再試行しません。EAGAIN/EWOULDBLOCK/EINTRの後は10 ms以上空け、待機中のpollはread/sendをせず戻ります。新しいsendをする時だけ回数を消費します。待機中でも期限/切断は確認し、待機終了を待たず解放します。

ヘッダー開始から全体2000 ms・無進捗500 ms・最大128 send・累積64回の再試行可能エラーで終了します。正のsendだけが無進捗時刻を更新し、read/EAGAINは更新しません。進捗で全体期限/回数を戻しません。毎pollで全体期限、無進捗期限、接続、呼出回数を確認します。同時成立はこの順序で判定します。読出しは最大256 Bで、0/異常値なら終了。Wi-Fi喪失でも終了します。期限検査はアプリ側の上限で、File/OS/スケジューラー内部まで厳密な時間保証をするものではありません。

応答中は新たなhandleClientだけ抑止し、Wi-Fi監視、PS4診断出力、解放時系列集計は同じタスクで継続します。制御側に待機や確保処理を追加しません。成功/各上限/切断/ソケット・読出し異常/取消で同じ終了関数を通り、接続とファイルを閉じます。二重終了は無操作です。終了確定後にHTTP activeを減らし、送信失敗数と処理後メモリーを更新します。

1応答限定はアプリのコンテキスト数です。TCP待受のbacklog、既存core受信領域やTCP終了後の回収待ちをゼロにするものではありません。

## 終了理由とバイト数

既存20 B×32件のリングを拡張せず、完了時だけ5件を保存します。満杯時は既存のlost計数を使います。コールバック内の出力はありません。

| event | 表示 | 保存値 |
|---|---|---|
|17|HTTP_END|route、reason、phase、elapsed_ms、send calls、累積retry、最後のerrno|
|18|HTTP_BYTES kind=body|本文sent/total|
|19|HTTP_BYTES kind=wire|ヘッダー込みsent/total|
|20|HTTP_TIMING|poll回数、最大処理間隔ms、retry待機合計ms|
|21|HTTP_PROGRESS|最初/最後の正のsendの開始相対ms、終了時の無進捗ms|

17のfree欄=elapsed、largest欄=最後のerrno、arg=reason|(phase<<8)|(calls<<16)|(retries<<24)。18/19のfree/largest欄=sent/total。20ではfree=polls、largest=max_gap_ms、arg=retry_wait_ms。21ではfree=first_ms、largest=last_ms、arg=idle_ms。これらはヒープ残量ではありません。時刻/cidは通常通り終了時刻/ルートIDです。elapsedは解放前に取得します。最後のerrnoは途中回復後も保持するので、終了理由そのものとは区別してください。

reason: 0 Complete、1 Deadline、2 CallLimit、3 RetryLimit、4 Disconnected、5 SocketError、6 ReadError、7 Invalid、8 SizeLimit、9 WifiLost、10 Cancelled、11 NoProgress。
HTTP_TIMINGは待機で即時returnしたpollも含み、最大間隔には開始から最初のpollまでを含みます。retry_wait_msはEAGAIN検出から次回試行または終了までの実経過合計で、10 msより長いタスク遅延も含みます。first_ms/last_msはヘッダーも含む正のsend時刻（応答開始からの相対値）です。wireSent=0なら未進捗で両方0。成功時刻0も有効なのでwireSentと合わせて解釈します。各計数は終了後に既存の最大48 B/回のSerial処理で出力し、測定途中には出しません。

phase: 0 Idle、1 Header、2 Read、3 Body、4 Finished。終了直前の段階を記録します。初回event14/15/16も互換維持します。

## 検証とRAM

`tests/http_continuation_host/run.py`はインストール済みcoreのNetworkClientコンストラクター/デストラクター/stop、File::close本文を取り込みます。ハンドラー相当の参照を先に消し、保持された資源の生存と終了後の解放数を検査します。

- 実HTML/CSS/JSのLF/CRLF計6種類、正確なContent-Length/本文、確保禁止中の進行。
- 部分ヘッダー/本文、短いread、先読み禁止、EAGAIN後の次回継続、進捗後も累積するretry。
- 2000 ms、500 ms無進捗、128 calls、64 errorsの個別終了、時刻折返し、切断/送信エラー、Wi-Fi取消、read異常、空ファイル、サイズ/整形上限。
- 二重終了、使用中begin拒否、終了通知後の再利用、デストラクター取消、ソケット/RX/File参照解放。
- 既存起動テストでは応答中もWi-Fi喪失検出が進むこと、実制御テストでは応答保留中にPS4接続/切断/再接続/×停止が進むことを検証。既存PS4診断のサービス呼出はHTTPタスク内で継続します。

リポジトリHTMLは1243 B、LF10個。CRLFなら1253 Bで実機報告と一致します。両形式で本文一致を確認しますが、実機データの同一性や送信失敗の直接原因をこの差だけで確定しません。dataは変更しません。

ESP32ビルド成功。ELFのコンテキスト実サイズは**400 B**（7ea05d0の368 Bから32 B増）。UART観測コミット7884122の106636 Bから、この予算/計数修正後は静的DRAM **106668 B（32 B増）**です。data28852 B不変、bss77784→77816 B。Flash1901205 B。7ea05d0比ではDRAM合計64 B増、UART側のRTC_NOINIT124 Bは別途です。新たなタスク/スタック予約/動的確保はありません。既存File.openやTCP/RXの動的領域は残ります。ASan/UBSanも成功しました。

## 範囲と実機確認

今回の継続型は正常なHTML/CSS/JSファイル応答です。状態API・404/503・操作受付・CSV拒否は従来の固定長/上限付き同期小応答を保持します。正常CSVのcoreヘッダー/sendContent/chunk終端とcore要求解析は旧経路のままです。経路一覧は[bounded-web-files.md](bounded-web-files.md)を参照してください。

PID・FOC・PS4操作・停止条件・トレース・FIFO・スタックは変更しません。WebSocket未追加。トップページ完送、CSS/JS/状態更新、PS4併用/再接続、実内部RAM/最大連続領域、制御周期とTCP回収時期は実機未検証です。ビルド成功は実機通信成功を意味しません。マージ・実機書き込みは行いません。

後続変更で解析済みファイルGETのRX参照をTXコピーから外し、応答単位の確保失敗を集計しました。[http-rx-allocation-observation.md](http-rx-allocation-observation.md)に寿命・互換性チェック・集計範囲/欠落・最新RAM値を記載します。期限やTCP容量は変更しません。
