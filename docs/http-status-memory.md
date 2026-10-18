# 低メモリー時の状態応答と送信上限

## 確認済みの障害経路

利用者がクラッシュ時ELFのSHA先頭 `0df2cef97c2d747d` の一致と、
handleWifiStatusPage:4961 → WebServer::send(int,const char*,const char*):590 → strlen(NULL)
を確認した。PS4切断と同時期でも、この障害の実行経路はHTTPタスクである。
PS4切断が直接このNULLを作ったと断定しない。

使用中Arduino coreは3.0.3、IDF 5.1.4。WebServer.cppのconst char*版sendは、
入力を一度Stringへ変換してから、元の入力をNULL確認せずstrlenする。
WString.cppのoperator+はconcat失敗時にinvalidateする。無効Stringのc_strはNULLとなり得る。
以前の状態応答はIP.toStringと複数String連結を使い、そのc_strをsendへ渡していた。
連結でメモリーを消費した後にsend内でもStringを作る構成だった。
String cast failedログは変換失敗の証拠だが、最後のNULL経路ではstrlenの時点で停止するため、
そのログが最後の呼び出しそのものから出たとは扱わない。

NetworkClient.cppのwriteは既定10回、select待ち1秒/回。部分送信成功で回数を10に戻す。
EAGAIN(errno11)ではstopせず繰り返すため、部分送信を伴う応答全体の時間に固定上限はない。
今回の状態APIはこれを呼ばない。core全体や他HTTP経路を変更するパッチは追加しない。

## 修正

- HttpStatusコピー、IPの4オクテット、起動前提、resourcesを固定長領域で生成する。
  String連結、IP.toString、WebServer.send/sendHeader/_prepareHeaderを状態APIでは使わない。
- 最大8断片、本文最大4096 B、固定長192 Bヘッダー。Content-Lengthを明記し、
  Cache-Control:no-store、Connection:closeを返す。本文は断片から直接送り大きな合成領域を作らない。
  snprintfの失敗/切詰めと、断片のNULL/長さを検査する。状態/診断の既存情報は保持する。
- 公開NetworkClient.fd()からHTTPソケットへsend(MSG_DONTWAIT)し、1回最大512 B。
  NetworkClient.writeを経由しない。TLS用の経路ではなく、現在の平文HTTP専用である。
- 応答全体で送信呼出最大64回、再試行可能エラー最大3回（再試行は最大2回）。
  部分成功で上限を戻さない。応答開始から50 msを各呼出前に確認する。
  EAGAIN/EWOULDBLOCK/EINTRでだけHTTPタスクを1 tick譲り、他エラー/0送信は直ちに打ち切る。
- 成功、失敗、切断、形式不正のすべてでNetworkClient.stopを呼ぶ。
  送信後に別の503を重ねたり、同じ応答を再送したりしない。途中失敗は短い本文となり、
  ブラウザーは取得失敗として前回値を維持する。
- キュー未準備時の503も同じ固定長送信経路を使う。
  HTTP request parserや他経路にはcore側のString/ヒープ使用が残るので、HTTP全体が無確保とは主張しない。

stopは使用中coreで共有socket handleをcloseし、client側の参照と状態を解除する。
TCPスタックの未送達データがいつ完全に回収されるかはACK/タイムアウト等にも依存する。
送信成功は全データがソケットへ受理された意味であり、相手の受信/保存完了を保証しない。
50 msはアプリ内で検査する送信期限で、OSスケジューリング、send/close内部、
共有Wi-Fi/BTの影響まで含む厳密な実時間上限を保証するものではない。

## 制御との分離と検証

制御側は従来通り状態を非待機キューへ上書きする。変更した既存関数は
handleWifiStatusPageのみ。HTTPの送信待ちで制御/PS4を待たせるロックや通知待ちは追加しない。
PID/FOC、PS4初期化/操作/切断、停止条件、CSV寿命、全トレース、FIFO/スタック予約は不変。
WebSocketは追加していない。

- tests/http_response_host/run.py：実送信ヘルパーでmalloc/calloc/reallocとC++ newを失敗させても
  応答生成/送信が成功すること、部分送信で同一本文/Content-Length、EAGAIN累積上限、
  微小進捗時の回数上限、50ms判定、切断/送信ENOMEM/0、NULL、長さ超過、503、時刻折返しを確認。
  全経路でcloseを確認。ASan/UBSanでも成功。環境制約でLeakSanitizerは無効。
- tests/drive_host/run.py：実HTTPハンドラーを別スレッドの送信で待たせ、その間に実PS4関数で
  接続・入力・切断・再接続・×全停止を処理できることを確認。既存操作/CSVの回帰も成功。
- HTTP起動は診断設定0/1で成功。Classic専用起動、PS4通知FIFOとHID確保失敗の回帰も成功。

実機での低メモリー応答、TCP回収、遅いクライアント/再読込連打、PS4/Wi-Fi併用時の余裕と
制御周期/停止遅延は未検証。実機確認では通常/接続中/切断直後の状態取得を繰り返し、
メモリーが戻ること、前回状態への失敗表示、PS4処理/停止の遅延、HTTPスタック最小空きを記録する。
