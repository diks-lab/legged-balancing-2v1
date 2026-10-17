# 2208: 指令からFOC出力までの観測

基準はPR #12の53f155d。USBなしFullで転倒し、AttitudeOnlyでも支えが必要で、
再校正でも改善しないという実機報告に対し、今回は観測項目だけを追加する。
PID・電圧・基準角取得・制御式・停止条件は変更しない。

## 確認したライブラリ

ビルドに使うArduino-FOCを確認した。
`git describe --tags --exact-match` は `v2.4.0`、HEADは
`4f072b365f6e0185adca544071e595834405babc`。
以下はその実装に基づく（別版の一般的説明ではない）。

- [FOCMotor.cpp](https://github.com/simplefoc/Arduino-FOC/blob/4f072b365f6e0185adca544071e595834405babc/src/common/base_classes/FOCMotor.cpp):
  move（673行付近）、loopFOC（580行付近）、updateVoltageLimit（466行付近）。
- [BLDCMotor.cpp](https://github.com/simplefoc/Arduino-FOC/blob/4f072b365f6e0185adca544071e595834405babc/src/BLDCMotor.cpp):
  init（66行付近）、enable/disable（111行付近）、setPhaseVoltage（SinePWMの逆Park/Clarke変換）。
- [BLDCDriver3PWM.cpp](https://github.com/simplefoc/Arduino-FOC/blob/4f072b365f6e0185adca544071e595834405babc/src/drivers/BLDCDriver3PWM.cpp):
  enable/disable、setPwm。

## torque/voltage経路と単位

```text
mainのright_output / left_output [V]
 → move(value): target=value
 → モーター有効時、torque分岐で current_sp=target
 → 次のloopFOC(): センサー更新、有効フラグ確認、電気角計算
 → voltage.q = constrain(current_sp, ±motor.voltage_limit) + feed_forward_voltage.q
    voltage.d = feed_forward_voltage.d
 → setPhaseVoltage(q,d,electrical_angle)
 → 既定SinePWM: 逆Park/Clarke変換 → Ua/Ub/Uc
 → 中央配置: 各相へ driver.voltage_limit / 2 を加算
 → driver.setPwm(): 各相を0..driver.voltage_limitに制限
 → 各相 / voltage_power_supply を0..1へ制限 → _writeDutyCycle3PWM
```

このモードの `current_sp` は名前に反して電圧指令[V]であり、実測電流[A]ではない。
mainはfeed_forward_voltageを設定しておらず、静的オブジェクトの初期値0のまま。
フィードフォワードが設定される一般の場合は、加算がUq制限の後なので、
`voltage.q` がmotor上限を超え得るが、今回の設定では該当しない。
moveはmotion_downsample/校正中判定の後に進み、enabled=falseならcurrent_spを更新しない。
現行設定はmotion_downsample既定0。loopFOCも校正中／enabled=falseならPWM計算を行わない。

電圧制限は別々の段階にある。

| 段階 | 現行設定／動作 |
|---|---|
| 倒立出力のアプリ側制限 | ±1 V |
| motor初期化時 | motor.voltage_limit=2 V、voltage_sensor_align=2 V |
| motor.init内 | motor上限をdriver上限以下、align電圧をmotor上限以下へ制限 |
| 初期化完了後 | updateVoltageLimit(1 V)。PID関連の上限も更新するが、torqueでは速度PIDを駆動に使用しない |
| loopFOCのUq制限 | current_spを±1 Vへ制限（今回feed-forward=0） |
| driver相電圧範囲 | 0..8.3 V |
| driver供給換算 | 8.3 V。SinePWM中心は4.15 V、dutyは0..1 |

これらは電流制限ではない。`voltage.q` はFOCが計算した電圧値であり、相電圧・
PWMピン・電源電圧の物理計測値ではない。EN遮断中は数値だけから通電状態を判断できない。

## 記録時点

通常のmainのループは変更していない。

```text
loop n: 右loopFOC → 左loopFOC → FOC値をキャッシュ → MPU更新 → 状態取得・制御計算
        → 制御トレース BEFORE_MOVE → 右move(U_n) → 左move(U_n)
loop n+1: loopFOCがU_nを反映
```

通常行の既存 `right_output/left_output` は **今回の新しい計算指令**。
追加のq/current_spは **今回冒頭のFOCが消費した既存の指令と計算結果**。
FOC直後にq/current_sp/有効状態をキャッシュし、後続の制御処理で生のフィールドが
クリアされても、そのFOC値を通常行に残す。
つまり、通常は同じ行のqを同じ行の新しいoutputへ一致させる必要はない。
同じ行のqとcurrent_spを比較し、上限と有効状態を確認する。
左右のFOCは順次実行であり、物理的な同時測定ではない。

採取は既存の10 ms間隔（最大128行のリングバッファ）を維持する。
**直前のCSV行が直前の制御ループとは限らない**ので、単に1行ずらすだけでは照合できない。

| 追加列 | 内容 |
|---|---|
| right_voltage_q_V / left_voltage_q_V | 各motor.voltage.q |
| right_current_sp_V / left_current_sp_V | 各motor.current_sp（このモードではV） |
| right_motor_enabled / left_motor_enabled | SimpleFOCのソフトウェア有効フラグ |
| right_driver_en / left_driver_en | EN GPIOの読み返し。外部配線先や実電流の計測ではない |
| sample_phase | BEFORE_MOVE、または転倒停止のAFTER_DISABLE |
| foc_capture_us | FOC値キャッシュ時点のmicros()。起動からのµsで、約71.6分で周回。試験開始からのelapsed_msとは別 |

AFTER_DISABLEはEN遮断・指令クリア後の強制最終行。
q/current_spは停止処理により0へ書き換え済みで、遮断直前の最後のFOC値ではない。
停止前の通常行とは意味が異なる。停止後の保持と次回開始時消去は維持する。
既存14列の後へ10列を追加し、シリアルCSVとWeb CSVをそろえた。
制御中のI2C読み取りや新たなシリアル出力は追加していない。

## driverとmotorの有効状態

ライブラリのdriver.enable/disableはENとPWMだけを扱い、motor.enabledを変更しない。
motor.enable/disableはdriverを操作し、motor.enabledも1/0に変更する。
motor.enableはさらにPIDリセットを伴う。

既存mainはmotor.initでmotor.enabled=1になった後、driver.disable/enableを使う。
転倒・通信異常停止でもdriverだけを遮断するため、停止後に
**motor.enabled=1、EN=0** となり得る。次のloopFOCがソフトウェア上でPWM値を
再計算しても、既定SinePWMのsetPwm自体は単一ENを再有効化しない。
ゼロ停止ではcurrent_spもクリアしており、これだけで意図しない再通電や符号反転が
起きる根拠は見つかっていない。

状態表示の落とし穴は明確なので、今回はmotor.enabledとENを併記する最小の観測追加を行う。
両者を常に一致させる別の修正案は、停止をmotor.disable、再開をmotor.enableへ統一すること。
ただしPIDリセット・有効化経路に影響するため、今回の「制御・停止動作を変えない」範囲では
実施しない。出力経路の実装から、今回の転倒・後退の原因を断定できる不具合は未確定。

## Webで見る状態

既存Web画面に、キャッシュ済みAngleY・基準角・差分、system initialized、
左右driver初期化済みフラグ、sensor.ready/fault/error、motor_status、motor.enabled、ENを追加。
motor_statusの主な値は0=未初期化、2=未校正、3=校正中、4=FOC ready、14=校正失敗、15=初期化失敗。
`motor_status=4` はEN有効の意味ではない。

追加表示はキャッシュとGPIOを読むだけで、HTTP要求でIMUやAS5600を再読み取りしない。
fault後の角度は最後のキャッシュであり、正常な最新測定と解釈しない。
既存構造ではWebサーバーは初期化成功後に開始するため、起動途中／初期化失敗でsetupが
早期returnする場合の画面取得はできない。この起動経路も今回は変更していない。

## 検証と実機での取得

- 実制御トレース関数で、新指令と異なるq/current_spを保持し、制御で生フィールドをクリアした後やmove後もFOCキャッシュと行が不変であることを確認。
- Web CSVの24列、正常行BEFORE_MOVEと停止行AFTER_DISABLE、motor.enabled=1/EN=0、
  Webの角度・基準・差・初期化/fault表示を回帰確認。既存の走行・停止・保持テストも実施。
- ESP32ビルドを実施。PID、全設定定数、基準角取得、制御・停止関数、ライブラリは不変更。

USBなし試験では、停止後・次のSTARTや電源断の前に制御CSVを保存し、Webの角度・基準角・
fault状態も記録する。FullとAttitudeOnlyの行を混ぜず、sample_phase、EN、motor.enabled、
current_sp、qの順に確認する。本変更の実機観測は未検証。マージ・書き込みは行わない。
