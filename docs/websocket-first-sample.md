# 購読受理後の初回sample切断と通常ログ

基準はd4633ba。取得したブランチ先頭とローカルHEADの一致を確認した。
利用者の実機ではHTML4211 B/CSS619 B/JS8048 Bがretry=0で完送し、観測HTTP期間の確保失敗も0件。
画面は世代1・50 ms・6項目を受理した後、seq=0のままcode=1006で切断した。
約91秒後の右I2C短読取り/timeoutは別の異常として扱う。RAM不足/I2Cを切断原因と断定しない。

## 再現した不具合と修正

poll開始時のnowを使っている間に、snapshot取得/JSON整形が時刻を進める。
その後frameがtxAtを新しいmillisで設定し、同じpoll内で古いnowとの差を符号なし計算していた。

例: poll now=1000 → snapshot/整形 → frame txAt=1001 →
`uint32_t(1000-1001)=4294967295`。2秒経過していなくてもWRITE_DEADLINEと判定し、
sendを1回も呼ぶ前に生ソケットを閉じ得る。購読ACKは次のpollで送るためACKだけは到着できる。
close frameなしの終了はブラウザーに1006と見える。

送信予算の判定直前にnowを取り直す。snapshot採取後のage_msと受信後の時刻も取り直し、
異なる時点の時計を差し引かない。全体2秒/無進捗500 ms/128 send/64 retry/10 ms backoffは不変。
時計の32bit周回を扱う差分計算も維持する。

ホストでsnapshotに1/5 msの処理時間を加え、時計API自身も進む条件を追加した。
修正後は初回sampleが生成・部分送信・完了まで進む。
送信前の時刻取り直しだけを除いた変異版は同じ条件で切断し、回帰試験が失敗した。
これはソース上の不具合の再現であり、今回の実機1006が同一原因だったかは新ログで確認する。

## 今回の最小観測（通常モードでも有効）

同じ固定32件×20 Bの既存イベントリングを再利用し、後送り・1回最大48 BでUART出力する。
新しい長い記録領域・毎sample出力は追加しない。

- WS ACCEPTED / UPGRADED / SUBSCRIBED: 接続、handshake送信、購読受理。
- FIRST_PREPARED: 初回sampleを固定領域へ生成した時点（世代/seq/総bytes）。
- FIRST_WRITE: 初回sampleで最初に正のsendが返った時点（送信済/総量/calls/retry）。
- FIRST_SENT: 初回frame全体をsocketへ渡した時点。ブラウザー到着の証明とは区別する。
- CLOSED: 終了理由・時刻・世代/seq・経過時間・送信済/総量・累積回数・最後のerrno。

初回観測は接続/購読世代ごとに1回だけ。再接続/再購読では再採取する。
理由はWRITE_DEADLINE/WRITE_IDLE/CALL_LIMIT/RETRY_LIMIT/WRITE_ERROR、
READ_ERROR/PEER_CLOSED/FRAME_INVALID、handshake/RX期限、JSON_ERROR、WIFI_LOST等を区別する。
elapsed_msは処理中の送信枠の経過時間で、枠がなければ0。最後のerrnoは失敗した最後のI/Oの情報であり、必ずしも切断操作自体のerrnoではない。
リング欠落はlostで表示する。欠落があれば、行がないことだけで未実行と判断しない。

ブラウザーは受信JSONを処理・履歴へ保存し（表示中なら描画した）後に、Consoleへ
`WS_FIRST_SAMPLE <世代> <seq> <sample_ms>` を1回表示する。
画面のseq/グラフとこのConsoleを見て、サーバーsend完了とブラウザー処理を切り分ける。
ブラウザーからの新たな指令/ACKメッセージは追加していない。
ボタンと説明は「配信開始」に統一した。

## 通常ログと診断設定

`include/SerialDiagnostics.h`の `SERIAL_DIAGNOSTICS=0` が既定。
通常は起動結果/IP/URL・異常/停止理由・WS接続/切断と上記初回観測を中心にする。
周期MEM HTTP、HTTP_REQUEST/ALLOC/END/BYTES/TIMING/PROGRESS、PS4自動詳細、
FOC観測の詳細表示、自動の長い操作メニューは通常OFF。
FOC観測そのもの、RTC確保失敗記録、内部ヒープの直近sample、イベント集計は保持する。
手動h/?と明示的なシリアル診断/停止コマンドは維持する。
FOC/SDKの既存初期化結果・エラー出力も無効化しない。

必要時はplatformio.iniの対象build_flagsへ以下を設定してファームを再ビルドする。

```ini
build_flags =
  -DSERIAL_DIAGNOSTICS=1
```

この設定で詳細HTTP/MEM/PS4/FOC表示と自動メニューを有効にする。
HTTP_MEMORY_STAGE_DIAGNOSTICSも既定でこの値を引き継ぎ、必要なら従来どおり個別上書き可能。
ログ出力を止めて不具合を解決した扱いにはしない。時計不整合の修正と独立した変更である。

## 検証と実機追試

実protocol/状態機械の進む時計・millis周回・初回3段階の1回性、部分送信・EAGAIN・OOM・低速・
不正/過大JSON・購読変更・切断/再接続を検査。
実C++生成JSONを実JSのホストDOMへ渡し、受信/seq更新・初回Console・グラフ処理まで確認。
通常/詳細モードのUART出力とリング排出、通常/詳細の起動/一度だけのREADY、
既存PS4・制御・FOC・HTTP解析/送信の回帰も実施する。

ファームとdataの両方が変わるため、**両方の更新が必要**。

```sh
pio run -e esp32dev
pio run -e esp32dev -t buildfs
# 利用者が停止・安全な状態を確認して行う。今回は未実行。
pio run -e esp32dev -t upload --upload-port PORT
pio run -e esp32dev -t uploadfs --upload-port PORT
```

追試はPS4未接続・倒立未開始・1タブで行い、古いタブを閉じて再読込みする。
SUBSCRIBED→FIRST_PREPARED→FIRST_WRITE→FIRST_SENTとConsole/画面seqの順を確認し、
切断すればCLOSEDと直後のWS_TXを保存する。毎sampleのSerial出力を追加する必要はない。
実機での継続更新/1006解消、PS4併用、20 Hz周期、RAM/スタック余裕は未検証。
右I2C異常の原因も未確定。制御・PID・FOC設定・PS4操作・停止条件は変更していない。

## ビルド結果とRAM

通常・診断ONの両方でESP32ビルド成功。LittleFSイメージ生成も成功した。

| 構成 | 静的DRAM | d4633baとの差 | Flash |
|---|---:|---:|---:|
|d4633ba|81924 B|—|1894805 B|
|今回・通常（既定）|80892 B|-1032 B|1892425 B|
|今回・SERIAL_DIAGNOSTICS=1|81940 B|+16 B|1897317 B|

通常のdata=29124 B、bss=51768 B。WSコンテキストは3168→3176 B（+8 B）。
RX1025 B/TX2052 B、既存イベントリング680 B、snapshot192 B、HTTPスタック8192 B、
PS4 FIFOオブジェクト2144 B、RTC124 Bは維持。新しいログ用の長い配列は追加しない。
通常では未参照の自動詳細出力用バッファ等がリンクから除去される。
スタック/FIFO容量を減らした結果ではない。実機のfree/largestと高水位は未測定。

関連16系統に加え、socket lifecycle、ブラウザーのホストDOM/実C++JSON連携、
通常/詳細UART・起動、通常モードの実PS4/制御関数試験が成功。
telemetry/制御/イベント出力のASan/UBSanも成功した。

```sh
TELEMETRY_SAMPLE_OUTPUT=work/native-samples.json python tests/telemetry_host/run.py
WS_SAMPLE_FIXTURE=work/native-samples.json node tests/web_serial_host/run.js
DRIVE_SERIAL_DIAGNOSTICS=0 python tests/drive_host/run.py
python tests/release_memory_host/run.py
python tests/http_startup_host/run.py
```

上記ホストテストの時計/ソケット/DOMはモデルであり、実機での1006解消・実ブラウザー継続描画の確認ではない。
