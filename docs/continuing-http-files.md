# 1応答限定の継続型Webファイル送信

9472ad2実機では本文送信失敗まで判明しましたが、従来event16では期限/呼出/再試行上限を区別できませんでした。今回これらを分離し、EAGAINで次の通信タスク処理へ戻します。期限の延長やバッファ削減は行いません。

## 所有権と進行

正常なHTML/CSS/JSの応答を静的コンテキスト1個で保持します。NetworkClientとFileの共有参照、256 B共用バッファ、未送信位置、開始時刻、全応答の計数を所有します。ヘッダー整形にも同じ256 Bを使い、有効長は192 B未満です。上限8192 B・実際のfile.size()に基づくContent-Lengthを維持します。

使用中のArduino core 3.0.3ではWebServerのハンドラー後の_currentClientリセットは共有参照を手放します。NetworkClientのデストラクターは明示stopを呼びません。コンテキストのコピーがソケット/RX領域を保持し、Fileもローカル参照消滅後に有効です。終了時にstop/closeしてから空の所有者を代入し、参照も解放します。

通信タスクの1周で非ブロッキングsend(MSG_DONTWAIT)を1回、またはFile.readを1回だけ実行します。部分送信では位置を保持し、残りを送り終えるまで次のreadをしません。EAGAIN/EWOULDBLOCK/EINTRでも位置を保持して戻り、同じ呼出内で再試行しません。

ヘッダー開始から50 ms・最大64 send・累積3回の再試行可能エラーで終了します（再試行最大2回）。進捗で予算を戻しません。毎pollで期限、呼出回数、接続を確認します。読出しは最大256 Bで、0/異常値なら終了。Wi-Fi喪失でも終了します。期限検査はアプリ側の上限で、File/OS/スケジューラー内部まで厳密な時間保証をするものではありません。

応答中は新たなhandleClientだけ抑止し、Wi-Fi監視、PS4診断出力、解放時系列集計は同じタスクで継続します。制御側に待機や確保処理を追加しません。成功/各上限/切断/ソケット・読出し異常/取消で同じ終了関数を通り、接続とファイルを閉じます。二重終了は無操作です。終了確定後にHTTP activeを減らし、送信失敗数と処理後メモリーを更新します。

1応答限定はアプリのコンテキスト数です。TCP待受のbacklog、既存core受信領域やTCP終了後の回収待ちをゼロにするものではありません。

## 終了理由とバイト数

既存20 B×32件のリングを拡張せず、完了時だけ3件を保存します。満杯時は既存のlost計数を使います。コールバック内の出力はありません。

| event | 表示 | 保存値 |
|---|---|---|
|17|HTTP_END|route、reason、phase、elapsed_ms、send calls、累積retry、最後のerrno|
|18|HTTP_BYTES kind=body|本文sent/total|
|19|HTTP_BYTES kind=wire|ヘッダー込みsent/total|

17のfree欄=elapsed、largest欄=最後のerrno、arg=reason|(phase<<8)|(calls<<16)|(retries<<24)。18/19のfree/largest欄=sent/totalで、ヒープ残量ではありません。時刻/cidは通常通り終了時刻/ルートIDです。elapsedは解放前に取得します。最後のerrnoは途中回復後も保持するので、終了理由そのものとは区別してください。

reason: 0 Complete、1 Deadline、2 CallLimit、3 RetryLimit、4 Disconnected、5 SocketError、6 ReadError、7 Invalid、8 SizeLimit、9 WifiLost、10 Cancelled。
phase: 0 Idle、1 Header、2 Read、3 Body、4 Finished。終了直前の段階を記録します。初回event14/15/16も互換維持します。

## 検証とRAM

`tests/http_continuation_host/run.py`はインストール済みcoreのNetworkClientコンストラクター/デストラクター/stop、File::close本文を取り込みます。ハンドラー相当の参照を先に消し、保持された資源の生存と終了後の解放数を検査します。

- 実HTML/CSS/JSのLF/CRLF計6種類、正確なContent-Length/本文、確保禁止中の進行。
- 部分ヘッダー/本文、短いread、先読み禁止、EAGAIN後の次回継続、進捗後も累積するretry。
- 50 ms、64 calls、3 errorsの個別終了、時刻折返し、切断/送信エラー、Wi-Fi取消、read異常、空ファイル、サイズ/整形上限。
- 二重終了、使用中begin拒否、終了通知後の再利用、デストラクター取消、ソケット/RX/File参照解放。
- 既存起動テストでは応答中もWi-Fi喪失検出が進むこと、実制御テストでは応答保留中にPS4接続/切断/再接続/×停止が進むことを検証。既存PS4診断のサービス呼出はHTTPタスク内で継続します。

リポジトリHTMLは1243 B、LF10個。CRLFなら1253 Bで実機報告と一致します。両形式で本文一致を確認しますが、実機データの同一性や送信失敗の直接原因をこの差だけで確定しません。dataは変更しません。

ESP32ビルド成功。ELFのコンテキスト実サイズは**368 B**。別途pendingフラグ1 Bを追加しますが配置調整により静的RAM総量は**106236→106604 B（368 B増）**です。data28852 B不変、bss77384→77752 B。Flash1898921 B。新たなタスク/スタック予約/動的確保はありません。既存File.openやTCP/RXの動的領域は残ります。ASan/UBSanも成功しました。

## 範囲と実機確認

今回の継続型は正常なHTML/CSS/JSファイル応答です。状態API・404/503・操作受付・CSV拒否は従来の固定長/上限付き同期小応答を保持します。正常CSVのcoreヘッダー/sendContent/chunk終端とcore要求解析は旧経路のままです。経路一覧は[bounded-web-files.md](bounded-web-files.md)を参照してください。

PID・FOC・PS4操作・停止条件・トレース・FIFO・スタックは変更しません。WebSocket未追加。トップページ完送、CSS/JS/状態更新、PS4併用/再接続、実内部RAM/最大連続領域、制御周期とTCP回収時期は実機未検証です。ビルド成功は実機通信成功を意味しません。マージ・実機書き込みは行いません。
