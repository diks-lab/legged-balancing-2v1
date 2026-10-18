# PS4 HID送信の確保失敗と接続中断

## 実機報告とELF照合

ae0c84dのPS4接続試行で `allocating buffer for sending the command failed` の後に
StoreProhibited / EXCVADDR=0x00000004。報告のELF SHA先頭は `316041d836927826`。
起動完了時の内部8bit空き7296 B・最大連続7156 Bは、接続処理中の確保失敗時刻の値ではない。
Bluetooth接続処理自体の追加消費があるため、起動完了値だけで4112 Bの確保成功を保証しない。

こちらのae0c84d ELF全体SHAは
`35079f6d64a4683f708efdd0117bdde633b35763db382f0206b34b69cf115acc`。
報告SHAと異なるため、このELFで元バックトレースの関数/行を確定していない。
同じGitコミットでもビルド環境・パス等によりELFが変わり得る。
Windows側で保存済みの `firmware-crash-20261018.elf` の一致確認・解析結果は確認待ち。

保存したELFのあるフォルダーで実行する読み取り専用PowerShellコマンド：

```powershell
Get-FileHash .\firmware-crash-20261018.elf -Algorithm SHA256

& "$env:USERPROFILE\.platformio\packages\toolchain-xtensa-esp32\bin\xtensa-esp32-elf-addr2line.exe" -pfiaC `
  -e .\firmware-crash-20261018.elf `
  0x400d88bb 0x400d8633 0x400d867f 0x400d87f9 `
  0x4015133f 0x40153e08 0x4014393c 0x4015e629 0x400972b6
```

SHA先頭が報告値と一致したELFの出力だけを確定解析に用いる。
ツールが存在しない場合はインストール済みPlatformIOのpackagesディレクトリーを確認し、
該当するESP32用addr2lineを指定する。新しいビルドや別ELFで代用しない。
解析出力のローカルパスに含まれるユーザー名等は共有前に伏せてよい。

## ソースから確認できた不具合

PS4_Controller_Host **1.1.1** の `ps4_l2cap_send_hid()` は、
`osi_malloc(BT_DEFAULT_BUFFER_SIZE)` がNULLでもログだけで処理を続け、
`BT_HDR` のlength/offsetへ書き込んでいた。NULL参照が存在することはソースで確定している。
EXCVADDR=4はoffsetの配置と整合するが、障害PCの命令位置の確定は一致ELF待ちである。

呼び出しは `config_cfm → ps4ConnectEvent(true) → ps4Enable()`。
ps4Enableはenable（4 BのHID内容）を送り、続けてps4SetLed→ps4Cmd（79 B）を送る。
低層送信がvoidなので、単純なreturn追加だけでは呼び出し元が次のLED送信へ進み、
後続レポートから接続済みとされる可能性が残る。
元のconfig確認も結果コードを判定せず、interrupt CIDだけで接続を判定していた。

| 対象 | 使用中の値/扱い |
|---|---|
| BT_DEFAULT_BUFFER_SIZE | 4096+16 = **4112 B**、今回変更なし |
| BT_HDR / L2CAP_MIN_OFFSET | 8 B / 13 B、今回変更なし |
| osi_malloc | 同梱osi/allocator.hの通常分岐はmalloc(size) |
| メモリー属性 | mallocのMALLOC_CAP_DEFAULT（8bitアクセス可能な通常ヒープ）。DMAや32bit専用領域を要求しない |
| 使用中の構成 | HEAP_MEMORY_DEBUG / HEAP_ALLOCATION_FROM_SPIRAM_FIRSTの有効定義なし。esp32devでPSRAM追加設定なし |
| DataWriteの所有権 | 呼び出し後はBluetoothスタックへ移る。FAILEDでもスタックが解放するため呼出側で再解放しない |

IDFヘッダーの版は **5.1.4**。
所有権は [v5.1.4 l2c_main.cのl2c_data_write](https://github.com/espressif/esp-idf/blob/v5.1.4/components/bt/host/bluedroid/stack/l2cap/l2c_main.c)
の不明CID・MTU超過・既存輻輳時のfreeと正常時の状態機械への引き渡しで確認した。
CONGESTEDは受理済みとして扱い、再送しない。
4112 Bを減らす変更、タスクスタックやPS4 FIFOの削減は行わない。

## 修正した失敗処理

- NULL時は書き込み/DataWriteを行わない。確保失敗直後に1行だけ、要求サイズ、
  MALLOC_CAP_DEFAULTの数値、同属性の空き/最大連続領域、内部8bit空き/最大連続領域を記録する。
  正常送信時の新しい周期ログはない。アドレス・ペアリング情報を記録しない。
- セッションを先に中断済みにし、接続状態・両CID・設定完了フラグをクリア。
  既存のps4ConnectEvent(false)を通し、両L2CAPチャネルの切断を要求する。
  既に接続通知済みなら既存の切断イベントが制御側へ届く。直接モーター/停止処理は呼ばない。
- 初回enable失敗後のLED送信は入口で抑止する。LED失敗も同じ中断処理となる。
  DataWrite失敗も中断し、渡したバッファを二重解放しない。
- 両チャネルの設定成功を確認して一度だけenableを実行。
  設定失敗、ConfigReq失敗、切断通知/確認時も有効な接続を残さない。
- 中断後の古い設定確認/データは接続を復活させず、受信バッファは解放する。
  新しいcontrolチャネルとinterruptチャネルの確立・設定成功を経てのみ再接続を受理する。
  切断要求自体が失敗してもローカルの中断を取り消さず、自動再試行ループを追加しない。
- 設定待ちのタイムアウトを新設せず、既存Bluetoothスタックの接続/設定タイムアウトに委ねる。
  設定が揃わない間は入力を受理せず、既存の未接続状態を維持する。
- 既存connectログのbinary BD_ADDRを%sで扱う出力は削除した。
  アドレスを文字列として範囲外読み取りする危険と不要なペアリング情報表示を避ける。

## 対象照合と検証

`pre:scripts/patch_ps4_l2cap.py` はlibrary.propertiesの版1.1.1を確認し、
CRLF→LFだけで正規化した**ファイル全体SHA-256**で未適用/適用済みを照合する。
各置換条件と生成後全体SHAも検証し、未知ソースは書き換えずビルドを停止する。
UTF-8/バイナリで読み書きし、LF/CRLFを保持。適用済みは更新時刻も変えない。
既存の通知修正は別ファイルps4.cが対象なので相互に再適用を妨げない。

`python3 tests/ps4_l2cap_host/run.py` は実低層関数と実ps4Enable/LED/通知関数をコンパイルし、
enable/LED/接続後の確保失敗、NULL後の送信抑止、接続中断/再接続、設定失敗、切断途中、
古いCID、DataWrite失敗/輻輳、所有権、LF/CRLF再適用と未知ソース拒否を検証する。

PID・FOC・ペアリング設定・操作割り当て・既存停止条件・FIFO/スタックを変更しない。
接続確保が失敗しなくなること、実際の失敗時メモリー量、切断要求と再接続の実機動作は未検証。
NULL参照の修正はメモリー不足の解消やPS4接続成功を保証するものではない。
