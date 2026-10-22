# 再組付け後のアライメントと片輪確認

基準先頭は `b231c8a`。`codex/2208-web-ps4` の既存変更を保持。
**起動時の自動倒立は無効。再起動しても開始しない。**
ただし、MPU校正・FOCアライメントは従来どおり起動時に行い、車輪が動く。
PS4／シリアルの手動開始経路は残るため、方向確認が終わるまで開始操作をしない。

## 保存値／固定値の確認

- アプリ内にNVS／Preferences／EEPROMから方向・電気ゼロ角を復元する経路はない。
  数値を固定した代入や、引数付きinitFOCで渡す経路もない。
- 従来も左右の `BLDCMotor(7)` はSimpleFOC 2.4.0の
  `sensor_direction=UNKNOWN`、`zero_electric_angle=NOT_SET` から起動。
  今回、ガードタイマー開始成功後、initFOC直前に両値とPP結果を明示的にリセットする。
  以前の校正値が同一オブジェクトに残っていても、方向走査／ゼロ角測定を省略しない。
- 右motor1はPWM 13/12/14、EN27、AS5600はI2C 19/18。
  左motor2はPWM 26/25/33、EN32、AS5600はI2C 23/5。
  RIGHT／LEFTはこの論理チャネル名。物理左右の入れ替えを自動判別する機能ではない。
- SimpleFOCは方向をshaftAngle／shaftVelocityとelectricalAngleへ適用し、
  電気角からzero_electric_angleを引いて正規化する。
  `src/wheel_diagnostic.cpp` の別ファームも標準初期値から校正し、結果を表示する。
  本変更では別ファームや、その速度PID等は変更しない。
- `kRightForwardSign=kLeftForwardSign=+1`、`m1_direction=m2_direction=+0.5` は維持。
  後者には平均化が含まれる。センサー方向CW／CCWと車体の前進方向は別。
  PID、制御式の符号、フィルター、極対数、電圧、FOC設定は変更しない。

## 起動ログの判定

両輪とも、出力遮断・観測終了後に次を通常ログへ一度表示する。

```text
BOOT AUTO: DISABLED; manual start only. FOC alignment still moves wheels.
FOC ALIGN wheel=RIGHT direction=... zero_rad=... zero_measured=... poles=7 pp=... estimated_pp=... raw=... accepted=... expired=... sensors_ok=...
FOC ALIGN wheel=LEFT direction=... zero_rad=... zero_measured=... poles=7 pp=... estimated_pp=... raw=... accepted=... expired=... sensors_ok=...
```

- `direction` はCW／CCW／UNKNOWN。`zero_rad` は電気角[rad]。
  `zero_measured=0` の値は校正結果として使わない（NOT_SETの数値が出る場合もある）。
- `pp=PASS/FAIL/NOT_RUN`、`estimated_pp=2π/moved`。未測定はnan。
  ライブラリの既存判定 `abs(moved*7−2π) <= 0.5` を変更しない。
- `raw` はinitFOCの生戻り値。PP不合格でもraw=1となり得るため、
  `accepted` は別表示。raw成功・PP合格・タイムアウトなし・両センサー正常の全てが必要。
- 既存6秒ガード・異常時両輪遮断を維持。右が失敗したら起動を止め、
  左のために失敗を飛び越えない。右成功時に左へ進む。
- `SERIAL_DIAGNOSTICS=0` でも上記結果は表示する。
  走査詳細は従来どおり診断ONのみ。測定中の追加I2Cや出力はない。

## 更新・実機での確認（未実施）

今回のdataは無変更。ファーム更新が必要で、直前版の画面が書込み済みならLittleFS更新不要。
画面も古い場合のみ、通常のLittleFS書込みを行う。自動フォーマットはしない。

```sh
pio run -e esp32dev
pio run -e esp32dev -t buildfs
# 以下は利用者が対象ポート・保持を確認して実施。エージェントは未実施。
pio run -e esp32dev -t upload --upload-port PORT
# 必要な場合だけdataを更新:
pio run -e esp32dev -t uploadfs --upload-port PORT
```

1. 機体を保持し両輪を浮かせ、磁石・各センサーと対応モーターの機械的対応を確認する。
   MPU校正中は静止。起動時のFOC走査には動ける余地を用意する。
2. 自動倒立無効の表示と両輪の `accepted=1`／`pp=PASS`、方向・ゼロ角を保存する。
   不合格時は電圧・極対数・許容幅を変更せず、配線／機械対応を確認する。
3. 既存シリアル `j` で極性試験へ。片輪パルスは `5`=右正、`6`=右負、
   `7`=左正、`8`=左負。初期0.15 V／500 ms、既存の傾斜制限を維持。
   一回ずつ完了を待ち、実際に動いた物理左右と機体前進／後退方向を記録する。
4. 各 `POLARITY RESULT` のcommand、right/left_angle_delta、velocity_avg／peakを保存。
   **物理車輪・前進方向・位置差の符号・速度の符号**をセットで確認する。
   この実測が済むまで倒立試験を再開せず、PID／方向係数は変更しない。
5. 中止は既存USB `d`／PS4×の全停止。既存の解除・再開始条件を飛び越えない。

## 検証・資源

ホスト：両輪に古い方向／ゼロ角／PP合格値を与えても再走査し、全観測点を取得。
通常ログ／診断ログ両設定、PP不合格、移動不足、生戻り値、6秒タイマー／Web中断／
センサー異常遮断、タイマー開始失敗、測定回数・電圧指令列・待ち時間不変を確認。
本番の自動開始OFFで10秒間開始しないことと、手動開始・PS4停止／再接続等を実制御関数で確認。
旧自動開始はテストだけ明示opt-inし既存回帰を維持。UI等の変更・ブラウザー再試験はなし。

ESP32ビルド・LittleFS生成成功。静的RAMは前後とも81,092 B
（data 29,284 B／bss 51,808 B）、差0 B。RTC 124 B、IRAM text 113,155 Bも不変。
Flashは1,898,057→1,898,649 B（+592 B）。新規常駐バッファ・タスク・ヒープ確保なし。
結果整形用320 Bは既存スタック上の一時領域。スタック設定・FIFO・通信バッファ・配信量は不変。

実機のアライメント成功、物理左右、車体前進と位置／速度の符号、静止倒立・走行は未検証。
ビルド／ホスト試験成功を実機成功とは扱わない。マージ・実機書込みなし。
