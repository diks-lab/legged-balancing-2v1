# ライブラリパッチの対象・理由・撤去条件

公開APIを優先し、アプリ側の指令処理/停止処理をライブラリ内で重複実装しない。
依存版の更新だけを理由に補正を撤去せず、問題が解消した上流コードと回帰テストで判断する。
調査時のPS4上流HEADは0cc4cf6ad085405d87ec2c0812e6d2bfafa38535、表示版は1.1.1。
この上流にも下記PS4不具合が残っていることを確認した。

| ファイル / 対象 | 維持する理由と照合 | 撤去条件 |
|---|---|---|
| scripts/patch_ps4_notifications.py / PS4_Controller_Host 1.1.1のps4.c | 公開attachOnDisconnectが呼ばれない、最初の入力レポートが接続通知に置換される問題。対象2関数の元/適用済み本文を照合して配送のみ補正 | 上流で切断通知と最初の入力配送が修正され、短い×、押下/解放、重複切断、再接続の回帰が通ること。各補正が不要になれば該当分だけ撤去 |
| scripts/patch_ps4_l2cap.py / PS4_Controller_Host 1.1.1のps4_l2cap.c | HID確保失敗後NULLへ書く、失敗/中断した接続を継続する問題。再接続の解放時系列を調べる出力なし観測フックも追加。版と元/適用済み全体SHAを照合。4,112 Bの要求は変更しない | 上流でNULL、後続enable/LED送信、中断後データ、設定失敗、所有権と再接続が適切に処理され、OOM/輻輳/切断回帰が通ること |
| scripts/patch_foc_observation.py / SimpleFOC 2.4.0、4f072b365f6e0185adca544071e595834405babcのFOCMotor.cpp | 左FOC失敗切り分け用の固定長観測。元/適用済み全体SHA、UTF-8/改行/生成結果を照合。制御式・待ち・電圧は不変 | 実機切り分け後、観測が不要と確認された場合、または上流の同等観測APIで代替できた場合に観測本体と対応して撤去。PS4改善やWeb移行だけを理由に撤去しない |

Classic専用化はArduino公開APIとアプリ側の初回起動ゲートで行い、
PS4/Arduinoライブラリの追加パッチを作らない。[起動手順・実機確認](ps4-classic-startup.md)。
PS4のattach/attachOnConnect/attachOnDisconnect、引数なしbegin、入力アクセサーは公開APIを使用。
FIFO/Batchは通知を制御ループへ渡すアプリ側の仕組みであり、上流パッチとは区別する。

将来のライブラリ更新では、更新前後の対象本文とAPIを確認してから補正を整理する。
未知ソースの照合を緩めたり、接続ポーリングで短い切断/入力の配送問題を隠したりしない。
別PS4ライブラリへの切替はペアリング/Arduino基盤/操作回帰を含む独立変更とする。

解放時系列の観測フック・固定RAM量・実機比較手順は[解放時系列](release-memory-timeline.md)を参照。
調査完了後は観測フックだけを撤去できる。安全補正の撤去条件は上表のままとする。

## HTTP送信用のRX参照アダプター（coreは未変更）

HttpTransmitClientはArduino core 3.0.3のprotected _rxBufferへ依存します。共有socketを保持し、解析済みファイルGETの不要な受信キャッシュだけを早期回収する目的です。scripts/check_http_client_core.pyがNetworkClient.h/cpp全体のLF正規化SHAとversionを読み取り専用で検証し、未知版ではビルドを停止します。SDK/coreへのソースパッチではありません。上流で同等の公開APIが提供された時、または解析/所有権の置換時に撤去・再評価します。詳細は[http-rx-allocation-observation.md](http-rx-allocation-observation.md)。TCP内部の観測やSDK再ビルドは今回含めません。
