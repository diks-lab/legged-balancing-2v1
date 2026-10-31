# 単体診断：保存FOCによる固定電圧比較

基準は`codex/2208-web-ps4`の`6b8b1e6`。
AGENTS.md、fixed-voltage-comparison.md、upstream-origin.mdを確認した。
利用者の実機記録では通常ファームの±0.5 V／500 msで右は約±164°、
左は+3.6°／−0.7°で停止し、q電圧は要求どおり、異常記録なし。
一方、右→左初期化の速度診断は両輪±6 rad/sに追従する。
速度PIDは電圧を変えるため、この結果だけで固定電圧の回転を保証しない。
今回追加した単体固定電圧試験の実機結果は未取得。

## 最短比較

車輪を浮かせて機体を支持し、同じ電源・負荷条件を保つ。
単体診断を新規起動し、USB Serial 115200 baud・LFで一行ずつ送る。

| 単体診断コマンド | 通常ファームの比較指令 | 右の生指令 | 左の生指令 |
|---|---|---:|---:|
| `voltage right +` | `d`→`J`→`5` | +0.5 V | 0 V |
| `voltage right -` | `d`→`J`→`6` | −0.5 V | 0 V |
| `voltage left +` | `d`→`J`→`7` | 0 V | +0.5 V |
| `voltage left -` | `d`→`J`→`8` | 0 V | −0.5 V |

最短は新規起動→`voltage right +`→`FIXED RESULT`待ち→`status`で両EN無効確認、
続けて`voltage left +`→結果待ち。負符号も各一行で同様に比較する。
初期化条件まで揃える比較では各試行を新規起動から行い、先に`motor`を実行しない。
初期化中の追加入力は試験取消しになるため、コマンドをまとめて送信しない。
固定試験は再校正せず、通常ファームと同じ保存FOCを利用する。
未保存・不整合なら拒否する。この試験のために保存値を作り直さない。
`stop`は途中でも両輪遮断。自動再試行・昇圧なし。

## 保存値・初期化・モード

`Preferences.begin("wheel-foc", true)`で読取専用に開き、通常と同じ
`wheelcal::load`で二つのslot、52 byte ABI、CRC、世代、構成指紋、
両輪の方向・有限なゼロ角・7極対・保存済みpp合格を検証する。
読取失敗も拒否する。新しい有効slotが別構成なら古い構成へ戻らない。
診断にNVS書込APIはなく、保存値の変更・再登録・再校正を行わない。
構成指紋の順序と定数はmain.cppのcalibrationConfigと一致する。

片輪指定でも両輪を右→左に初期化する。方向・ゼロ角・pp合格を
initFOC前にRAMへ適用し、適用後の値が一致することを確認する。
固定SimpleFOC 2.4.0は保存方向・ゼロ角があれば位置合わせを省略する。
実ライブラリーを使うホスト試験でもinitFOC中の相電圧書込がないことを確認した。
通常と同じSinePWM（表示値0）、centered=1、8.3 V電源／driver limit、
駆動Uq上限1 V、torque/voltageを明示する。試験要求は±0.5 Vのみ。
速度PIDを出力計算に使わず、固定モードの速度LPF設定も通常の既定値0.005 sへ戻す。

既存の`motor left/right/both +/-`は±6 rad/s・上限1 V・1.5秒を維持する。
速度試験で測定したFOCはRAMに分離保持し、固定試験の保存FOCと混在させない。
速度へ戻るときは速度モード・従来設定・RAM校正を復元する。
速度校正が未実施なら、明示した`motor`コマンドは従来どおり位置合わせを行う。
固定試験へ戻るたび保存値を読み直す。モード切替でPWMを再割当しない。
先に左から割当した起動では固定試験を拒否し、再起動を求める。

## 停止・出力

既存の独立esp_timerを最初のenable前に500 msで開始し、各enable前後と
ループ内にも期限・両センサーを確認する。開始失敗ではenableしない。
コールバックはGPIOで両ENを遮断するだけで、ループ停止中も働く。
PWM／指令ゼロ化・motor.disable・結果整形はループ復帰後に行う。
500 msにはタイマータスク遅延とGPIO処理時間が加わり得る。
初期化は既存の同期motor.initと6秒initFOC監視を利用し、試験500 msとは別区間。
motor.initの待機は既存FOC監視の対象外で、初期化中のSerial stopは即時処理できない。

両輪ともゼロ指令からenableし、非選択輪は0 V。
センサー異常・非有限FOC/速度/角度・手動停止・期限で両輪遮断し、再開しない。
動作中の追加コマンドはstop以外拒否し、ストリームと周期DRIVEログも抑止する。
入力処理は従来どおり64 byte/loop以下。不正行を拒否し、終了時の残入力・途中行を破棄して再生しない。

出力中は固定長の左右一件だけを更新し、追加I2C読取・同期Serial・動的確保を行わない。
遮断後に一度だけ`FIXED RESULT`と左右各3行を既存ログリングへ入れる。
理由、経過時間、拒否件数、終了EN、保存世代、適用方向・ゼロ角・変調・pp、
requested／最後のmove指令／q、連続角の開始・最終・差（deg）、速度（rad/s）、
有効サンプル数・最終時刻、センサー異常を表示する。
EN_after=0/0を確認する。samples=0は印加成功を意味しない。
qは最後のFOC後、終了角はそのセンサーキャッシュ、速度は続くmoveで計算された値。
角度は保存sensor_directionとsensor_offsetを適用した通常と同じshaft座標で、RAW_ANGLEの符号とは区別する。
commandは最後のmove指令で、qはそれより前のFOCなので開始直後は一周期ずれる。
実端子電圧・相電流・遮断後の惰性回転を測った値ではない。

## 結果の切り分け

単体固定電圧でも左が動かなければ、通常アプリ固有の処理だけでは説明できない。
同一保存値・生指令・q・極性・負荷条件の一致を確認し、左右の低電圧トルク、
機械抵抗、電源／配線／ドライバの実出力を次に比較する。故障・校正不良とは未確定。

単体で左が回った場合に残るソース上の差は以下。

- 通常は両センサー／IMU更新→右FOC→左FOC→両move、単体は左FOC→move→右FOC→move。
  周期と指令適用時刻が異なる。双方moveの要求は次FOCでqに適用される。
- 通常のCheckedWheelSensorは通信異常の一時停止・回復を持つ。
  単体CheckedAS5600は異常ラッチ・停止速度補正を持つ。RAW_ANGLEレジスターとI2Cピンは同じ。
- 通常には同一I2C上のIMU処理、Wi-Fi/Web、PS4、脚UART、タスク負荷がある。
  単体にはそれらがなく、500 msの実際のFOCサンプル数／経過時間が異なり得る。

保存FOC・変調・PWM割当・qが一致して左の回転だけ異なる実機記録が得られたら、
まず両結果の時刻／回転量／サンプル数と通常の単発出力APIを照合し、
FOC順・センサー更新間隔・実端子PWMを比較する。未取得の実機結果を前提に原因を断定しない。
通常ファーム・脚・方向係数・ゲインは変更していない。

## 検証コマンド

リポジトリのルートで実行（SIMPLEFOC_SRC未指定なら既存.pio/libdepsを使用）。

```sh
bash tests/wheel_host/run.sh
ASAN_OPTIONS=detect_leaks=0 WHEEL_SANITIZERS=1 WHEEL_MCPWM_METADATA=1 bash tests/wheel_host/run.sh
python3 tests/wheel_calibration_host/run.py
bash tests/attitude_host/run.sh
python3 tests/wheel_output_host/run.py
ASAN_OPTIONS=detect_leaks=0 SANITIZERS=1 python3 tests/wheel_output_host/run.py
python3 tests/drive_host/run.py
ASAN_OPTIONS=detect_leaks=0 SANITIZERS=1 python3 tests/drive_host/run.py
pio run -e wheel_diagnostic
git diff --check
```

速度57＋固定43ケース、保存FOC、センサー／姿勢、実SimpleFOC／MCPWM allocator、
通常driveの通常設定／production roll設定、上記ASan/UBSanと車輪ESP32ビルドが成功。
固定ケースには全4指令、保存不整合・読取失敗、右→左、片輪／二輪目初期化失敗、
タイマー失敗、499/500 ms、独立コールバック、enable中期限切れ、左右センサー、
非有限値、時刻周回、停止、入力拒否・残入力破棄、結果一回、双方向モード切替を含む。
LeakSanitizerは環境制約のため無効で、リーク検査成功とはしない。
車輪診断ESP32ビルドの静的RAMは24,688 B、Flashは367,265 B。
通常ファームのビルド対象・ソース・脚・係数・ゲインは変更していない。

環境のHTTPプロキシ接続失敗でgit fetchが使えなかったため、GitHub APIで最新SHAを確認し、
そのSHAのソース／Git tree／commitを取得した。正確な6b8b1e6をローカルの浅い履歴として復元し、
変更を同じブランチへGitHub Git Data API（create_tree/create_commit/update_ref、旧SHA一致条件付き）で反映する。
実機書込み・マージは実施しない。
