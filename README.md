# Legged balancing 2v1

## 2208 車輪単体診断 (`wheel_diagnostic`)

`src/wheel_diagnostic.cpp` だけをビルドする独立環境です。既存の
`esp32dev` / `bus_diagnostic` のソース選択は変更していません。
SimpleFOC は通常環境と同じ **v2.4.0**、ESP32 Arduino は同じ指定プラットフォームです。
LQR、脚サーボ、C3、MPU6050、Wi-Fi、ゲームパッドを初期化・操作しません。
C3 の実機確認結果やレーザーの調査は、この診断とは別に扱ってください。

### 配線・電圧・起動時の状態

左右は `main.cpp` の対応をそのまま使用しています。2208 という型番から
極対数を判断せず、既存値 **7極対を暫定使用**しています。

| 車輪 | PWM A/B/C | enable (HIGH 有効) | AS5600 SDA/SCL | I²C |
| --- | --- | --- | --- | --- |
| 右 (`right`, 旧 motor1) | GPIO13/12/14 | GPIO27 | GPIO19/18 | bus 0, 400 kHz, 0x36 |
| 左 (`left`, 旧 motor2) | GPIO26/25/33 | GPIO32 | GPIO23/5 | bus 1, 400 kHz, 0x36 |

起動時は **EN の出力ラッチを LOW にしてから OUTPUT に切り替え**、
AS5600 のみ初期化します。Arduino-ESP32 3.x は `pinMode()` 前の
`digitalWrite()` を受け付けないため、ラッチ設定には IDF の
`gpio_set_level()` を使います。起動時には PWM ドライバ・モーターの
`init()`、`initFOC()`、自動回転を行いません。リセットから `setup()` までの
高インピーダンス期間はファームウェアで制御できません。EN の外部プルダウンと
実際の HIGH 有効配線を確認してください。

定数は専用ソース冒頭にあります。実機で適切な値とはまだ確認できていません。

| 定数 | 初期値 | 理由・範囲 |
| --- | --- | --- |
| `kSupplyVoltage` | 8.0 V | 既存設定を踏襲。実際の電源電圧と一致させること。PWM の電圧換算に使う |
| `kAlignVoltage` | 0.5 V | 既存の 2 V より低く開始。初回の位置合わせ・方向検出用 |
| `kDriveVoltage` | 0.3 V | ±の電圧トルク指令。回転速度の指定ではない |
| `kRunMs` | 1500 ms | 通常駆動を2秒未満にし、確認を短い単発動作に限定 |
| `kAlignMs` | 6000 ms | `initFOC()` 内の約3秒以上の待ち・測定に余裕を設ける EN 遮断期限 |
| `kI2cTimeoutMs` | 5 ms | 各 I²C トランザクションの待ち時間を制限 |
| `kStreamMs` | 200 ms | 約5 Hzで表示。センサーの更新自体は毎ループ行う |

ドライバの `voltage_limit` は電源電圧（PWM の相電圧範囲）、モーターの
`voltage_limit` は位置合わせ0.5 V／通常駆動0.3 Vです。これで PWM の中心を
実電源の半分に保ちます。いずれも電流制限ではありません。2208 の巻線抵抗・
KV・ドライバ仕様により低電圧でも発熱し得ます。電流制限付き電源で短時間確認し、
回らない／位置合わせに失敗する場合は配線・磁石・極対数を調べてください。
**自動昇圧、自動再試行、連続運転はありません。** 極対数チェックが不一致なら、
SimpleFOC が警告だけで続行できても、この診断では通常駆動を拒否します。

### コマンド（改行終端、大小文字・空白を含め完全一致）

LF または CRLF で終端します。末尾に余計な空白を付けないでください。
入力は最大63バイト（終端前の CR を含む）です。長すぎる行、制御文字、
不正な引数は行全体を拒否し、モーターを開始しません。

| コマンド | 動作 |
| --- | --- |
| `help` | メニューと初期化中の制約を表示 |
| `status` | センサー状態、実 EN 状態、PWM 初期化・FOC 成否、選択輪、指令、電圧・期限・ログ破棄数 |
| `read left` / `read right` / `read both` | 0–360度の角度、連続角度、連続角度/360の回転数、境界通過数、サンプル経過時間 |
| `stream on` / `stream off` | 両輪のセンサー表示を約5 Hzで開始／停止（モーターは始動しない） |
| `motor left +` / `motor left -` | 左だけを +0.3 V／−0.3 V で最大1.5秒駆動 |
| `motor right +` / `motor right -` | 右だけを +0.3 V／−0.3 V で最大1.5秒駆動 |
| `stop` | 両輪の EN を LOW にし、初期化済み PWM と電圧指令をゼロにする |

**各輪の初回 `motor` は、実行前に警告を送信してから、その輪だけの
ドライバ・モーター初期化と FOC 位置合わせを行います。位置合わせでも車輪が動きます。**
反対側の EN は常に LOW です。成功後の校正値は RAM 内で再使用し、リセットすると失われます。
`+` / `-` は指令の符号です。車体の前進・後退と対応すると断定しません。
通常駆動中の追加 `motor` は拒否し、予約しません。停止後に改めて1コマンドずつ送ってください。

### センサー異常・停止の制約

SimpleFOC 2.4.0 の `MagneticSensorI2C::getRawCount()` は
`endTransmission()` の結果を保存しますが、`requestFrom()` の受信数を
検証せず2バイトを読んで角度に変換します。また `Sensor::update()` は負の値を
受け取ると前回角度を保持し、呼出元へエラーを通知しません。
このため専用 `CheckedAS5600` は、アドレス／レジスタ送信結果、受信3バイト、
データ範囲、AS5600 の磁石検出・弱磁場・強磁場フラグを毎回確認します。
未検出、NACK、短い受信、通信タイムアウト、磁石状態異常はラッチし、
**両輪 EN を即座に LOW** にします。FOC 初期化内で呼ばれるセンサー更新にも適用します。
異常後は古い角度を正常値として表示せず `INVALID` とし、原因を直して再起動するまで
駆動を拒否します。安全側に倒すため、反対側センサーの異常でも駆動できません。
正しい長さの壊れたデータやセンサー内部のフリーズなど、I²C の ACK と値範囲だけで
検出できない故障まで検出するものではありません。

連続角度は0/360度の境界を最短の符号付き変化で追跡します。表示 OFF 中も更新します。
最初の値は起動時の絶対角度で、そこへ境界通過分を足した値です。電源断を跨いで
回転数を保持しません。サンプル間に半回転以上動く場合や通信欠落後の回転数は保証できません。

`BLDCMotor::init()` は合計約1秒の待ちを持ち途中で EN を有効にします。
`initFOC()` も方向検出の往復とゼロ位置保持を同期的に行うため、初回コマンドは
通常約4秒以上（初期化＋位置合わせ）ブロックします。**この間のシリアル `stop` は
即時処理できず、通常駆動の1.5秒期限も位置合わせ時間には含まれません。**
入力を受信した場合は初期化から戻った時点で通常駆動を取り消し、その入力を破棄します。
途中まで受信した行も次の改行まで捨てます。初期化中にコマンドを連続送信しないでください。

`initFOC()` の開始直前に独立した `esp_timer` を6秒で起動し、期限になれば
Arduino ループがブロックしていてもタイマータスクから両 EN を LOW にします。
これはライブラリ処理を中断せず、PWM をゼロにする後処理は戻ってから行います。
6秒は `motor.init()` の待ち時間を含みません。期限超過時に通常駆動へ移行しません。
通常駆動は別の1.5秒タイマーとループ内の経過時間確認で停止します。
タイマー生成・起動に失敗した場合も駆動を拒否します。
ソフトウェアタイマーにはタスクの実行遅延があり、CPU停止・ハード故障まで含む厳密な
停止時刻を保証しません。**初期化中に止める必要がある場合は、物理的にモーター電源を切ってください。**

通常の表示は固定長リングバッファに入れ、UART が受け付けられる分だけ送ります。
満杯なら行を破棄して `dropped_logs` を増やし、表示待ちで制御を止めません。
初回警告の送信完了だけは両 EN が LOW の間に待ちます。受信も1ループ64バイトまでとし、
入力の洪水によって停止確認が無期限に遅れないようにしています。

### Windows / PlatformIO の手順

VS Code の PlatformIO 拡張を導入し、このフォルダを開いて PlatformIO Terminal
から実行します。以下は PowerShell の例です。`COM5` は実際のポートに置き換えてください。

```powershell
cd C:\path\to\legged-balancing-2v1
pio run -e wheel_diagnostic
```

既存環境もビルドする場合、通常ファームの `inc/ssid.h` が無ければサンプルを
コピーします（既存ファイルは保持）。これはビルド用で、実 Wi-Fi 認証情報ではありません。
車輪診断自体に `ssid.h` は不要です。

```powershell
if (!(Test-Path inc\ssid.h)) { Copy-Item inc\ssid.example.h inc\ssid.h }
pio run -e esp32dev -e bus_diagnostic -e wheel_diagnostic
```

書き込みは **車輪を浮かせて機体を固定し、モーター電源を切った状態で**、
USB 接続・選択環境を確認した後に手動で行います。

```powershell
pio device list
pio run -e wheel_diagnostic -t upload --upload-port COM5
pio device monitor --port COM5 --baud 115200 --eol LF --echo
```

この作業ではアップロードしていません。USB モニターで Enter を押して1行ずつ送信します。
モニター終了は Ctrl+C。通常環境の書き込みへ戻す際は、その起動時 FOC と両輪動作を
理解した上で環境を明示的に選んでください。

### 最初の実機確認（まだ未実施）

1. **車輪を浮かせて機体を固定**する。周囲と配線を確認し、モーター電源は切って
   USB で診断を起動する。最初は `help` → `status` → `read both` を実行し、
   両ドライバが `DISABLED`、PWM/FOC 未初期化であることを確認する。
2. **AS5600 を手回し確認**する。`stream on` を実行して左右を1輪ずつゆっくり回し、
   左右対応、角度変化、0/360度を跨ぐ連続角度、回転数を確認する。`stream off` で止める。
   磁石・配線異常や `INVALID` があれば、モーターコマンドを送らず原因を直して再起動する。
3. **片輪ずつ短時間駆動**する。電源電圧・電流制限を確認してモーター電源を入れ、
   `motor left +` を1回だけ送る。初回は位置合わせで動き、その後最大1.5秒の通常駆動となる。
   右輪 EN が無効のまま、終了後に両輪 EN が無効になることを `status` と実測で確認する。
   その後 `motor left -` を1回送り、同様に確認する。続いて `motor right +`、
   `motor right -` をそれぞれ1回ずつ実行する。指令符号と実際の回転方向を記録する。
4. 通常駆動中の `stop`、追加 `motor` の拒否、自動停止を確認する。
   `motor both +`、`motor left ++`、`motor left +x` 等の不正入力で動かないことを確認する。
   通信異常は、まずモーター電源 OFF でセンサー未接続を確認する。駆動中の異常確認は
   固定・電流制限と安全な故障注入方法を用意した上で行う（活線配線を短絡させない）。

### ソフトウェア検証

クラウド上で3環境のビルドと、実際の診断ソースを使うホストテストを実行します。
ホストテストは GPIO/I²C/UART/タイマー/モーター API を模擬し、SimpleFOC の実
`Sensor.h` を参照します。電気的な FOC 校正、実電流、実回転、タイマー遅延、
起動過渡、EN 配線の実機検証の代わりにはなりません。

```sh
pio run -e wheel_diagnostic -e esp32dev -e bus_diagnostic
bash tests/wheel_host/run.sh
```

ホストテストは C++17 対応の `c++` が必要です。Windows は WSL/Git Bash 等の
利用可能な C++ 環境で実行してください。テストは、起動時の非駆動・EN 設定順序、
異常入力・長い行・CRLF、角度境界、左右単独動作、追加駆動拒否、`stop`、
独立タイマー／ループ期限、時刻の周回、通信・磁石・初期化・極対数異常、
初期化中の入力破棄、UART 送信不可時の停止を確認します。
倒立制御の調整や C3 機能との統合は対象外です。

## 1 Mbps bus diagnostic firmware

The normal balancing firmware remains the default `esp32dev` environment. The
bus diagnostic is a separate, explicitly selected firmware which does not
initialize or drive either wheel motor, the MPU6050, Wi-Fi, or the gamepad.

```sh
pio run -e bus_diagnostic
pio run -e bus_diagnostic -t upload
pio device monitor -b 115200
```

Do not upload as part of review. Select `bus_diagnostic` explicitly only when
the robot is secured and wheel power is safe. The existing source establishes
the shared bus as 1 Mbps 8N1 with ESP32 RX GPIO16 and TX GPIO17. It contains no
direction-enable GPIO, so the diagnostic preserves that wiring assumption; the
external half-duplex direction/echo behavior must be checked on the machine.

### Menu

Commands are newline-terminated:

| Command | Action / expected result |
| --- | --- |
| `help` | Print the menu; no bus transmission. |
| `ping1`, `ping2` | Ping the right ID1 or left ID2 STS3215 without moving it. A valid, checksummed non-echo status packet is `RESPONDED`; otherwise it times out. |
| `ping3` | Send one standard STS-style Ping to ESP32C3 ID3. A matching, zero-error, length-2 status packet is required; a transmitted-packet echo is logged and ignored. |
| `pingall` | Run one standard Ping each for ID1, ID2, and ID3. |
| `pos1`, `pos2` | Read the current raw position at STS address 56 from ID1 or ID2. |
| `rhome`, `rext` | Move only right ID1 to HOME 2061 or small extension 2044. |
| `lhome`, `lext` | Move only left ID2 to HOME 2026 or small extension 2044. |
| `c3ping` | Send the ID3 `0xA0` ASCII command `PING` (expected payload: `OK PONG`). |
| `c3help` | Send the ID3 `0xA0` ASCII command `HELP` and display its ASCII response. |
| `c3status` | Send the ID3 `0xA0` ASCII command `STATUS` and display its ASCII response. |
| `c3arm` | Send the ID3 `0xA0` ASCII command `ARM` exactly once and display the response. It does not fire and no ARM state is cached locally. |
| `c3fire <n>` | Send the ID3 `0xA0` ASCII command `FIRE <n>` exactly once, where `<n>` is exactly one digit from 1 through 9. Missing, signed, out-of-range, whitespace-containing, or suffixed values are rejected before transmission. There is no automatic ARM or retry. |
| `c3disarm` | Send the ID3 `0xA0` ASCII command `DISARM` exactly once and display the response. |
| `c3stop` | Send the ID3 `0xA0` ASCII command `STOP` exactly once and display the response. |
| `c3tilt <us>` | Send one ID3 `0xA0` ASCII command `TILT <us>` to set the barrel servo pulse. `<us>` must contain only decimal digits and be in the inclusive range 500–2400; invalid input is not transmitted. Success requires the exact ASCII response `OK TILT <us> us`. |
| `c3mute on` | Send the ID3 `0xA0` ASCII command `MUTE ON` exactly once. Success requires the exact ASCII response `OK MUTE ON`. This mutes only the automatic firing audio produced by `FIRE`. |
| `c3mute off` | Send the ID3 `0xA0` ASCII command `MUTE OFF` exactly once. Success requires the exact ASCII response `OK MUTE OFF`. |
| `battery` | Print averaged GPIO36 ADC mV, converted battery voltage, and GPIO22 LED state (`NORMAL`, `LOW`, `CRITICAL`, or `INVALID`). |
| `battery stream on` | Enable the same battery report at 1 Hz; measurement and LED operation are independent of streaming. |
| `battery stream off` | Disable the 1 Hz battery report. |

Leg moves use the values already present in the balancing firmware: speed 150
and acceleration 15. Startup, help, position reads, and Ping never issue a move.
Every move prints its target ID and position and is sent once, with no retry.
The ID3 commands were checked against the source under
`reference/legged-balancing-2c3`; that directory remains reference-only and is
not part of either PlatformIO build. The diagnostic validates the ID, declared
length, checksum, and zero error byte before accepting an ID3 response. A
timeout, malformed packet, TX echo, or packet from another ID is never reported
as success. Response parameters from `0xA0` commands must be printable ASCII and
are displayed on the USB monitor.

The diagnostic does not send either MUTE command at startup, while displaying
help, or during Ping. It neither saves nor infers MUTE state: that state belongs
to the C3's RAM, and restarting the C3 restores its default of audio enabled.
MUTE applies only to the automatic firing sound made by `FIRE`; a manual C3
`SOUND` command still plays while MUTE is on. (The diagnostic does not expose a
manual `SOUND` command.) Missing arguments, values other than exactly `on` or
`off`, extra whitespace, and suffixes are rejected before bus transmission.

### Battery voltage and status LED

The new-board battery divider is monitored only by this diagnostic firmware.
GPIO36 uses the calibrated `analogReadMilliVolts()` API with the Arduino-ESP32
3.x `ADC_ATTEN_DB_12` attenuation setting (the range formerly called 11 dB).
Sixteen samples, spaced 10 ms apart without a blocking wait, are averaged.
Battery voltage is `ADC volts * 133 / 33`; a separate calibration multiplier in
`src/bus_diagnostic.cpp` is initially `1.0`. GPIO22 drives the active-HIGH LED:

| Averaged voltage / condition | LED and state |
| --- | --- |
| At least 7.2 V | Steady on, `NORMAL` |
| At least 7.0 V and below 7.2 V | Toggle every 500 ms, `LOW` |
| Below 7.0 V | Toggle every 125 ms, `CRITICAL` |
| ADC at/below 50 mV or at/above 3000 mV | Off, `INVALID` |

A transition toward a warning must remain below its threshold for one second.
Recovery is immediate only after 0.1 V hysteresis: LOW returns to NORMAL at
7.3 V, while CRITICAL returns to LOW at 7.1 V (or directly to NORMAL at 7.3 V).
Until the first complete valid average, the LED is off and state is `INVALID`.
These are provisional low-voltage warnings, not a state-of-charge or “10%
remaining” measurement. They cannot detect faults not inferable from voltage
and do not inhibit wheels, leg movement, or firing.

#### Meter comparison and threshold test

1. Secure the robot and make the wheel and firing mechanisms safe. Connect a
   multimeter across battery positive and ground.
2. With a normally charged 2S LiPo connected, run `battery` and compare it with
   the meter. At 8.4 V battery voltage, expect about 2.084 V (2084 mV) on
   GPIO36. Do not apply more than the board permits to the ADC pin.
3. If a consistent ratio remains after checking wiring and resistor values,
   calculate `meter voltage / reported voltage` and use it as the diagnostic's
   calibration multiplier. Its checked-in value intentionally remains `1.0`.
4. **Do not discharge a LiPo to test warnings.** Disconnect it and use a
   current-limited adjustable bench supply on the battery input. Sweep around
   7.2 V and 7.0 V, allowing over one second below each threshold, and verify
   states and LED periods. Raise the supply through 7.1 V and 7.3 V to verify
   hysteresis. Disconnect the supply and verify `INVALID` with the LED off.

Sampling, state timing, and blinking use `millis()` with no added delay. The
optional stream writes once per second only when the USB serial buffer has
room, so it does not intentionally hold up the 1 Mbps bus path.

The diagnostic also exposes the existing C3 `ARM`, `FIRE`, `DISARM`, and `STOP`
commands for deliberate manual testing. It never sends `ARM` or `FIRE` during
startup, help, or any Ping, never automatically arms or repeats a fire request,
and does not infer or retain ARM or firing state. Use `c3status` to ask the C3
for its actual state. Even when a FIRE response times out or is rejected, the
diagnostic does not claim that firing did not occur and does not retry it.

The reference-only 2c3 copy implements these command/state rules: it starts
`DISARMED`; `ARM` changes any state except `FIRING` to `ARMED`; `FIRE 1` through
`FIRE 9` is accepted only while `ARMED` and changes the state to `FIRING` after
the firing output starts; completion returns to `ARMED` on success or `ERROR`
on failure. `DISARM` and `STOP` both stop firing and audio and change the state
to `DISARMED`. Its response strings are `OK ARMED`, `OK FIRE`, `OK DISARMED`,
`OK STOPPED`, `ERR BUSY`, `ERR NOT ARMED`, `ERR FIRE START`, and `ERR COMMAND`;
`STATUS` returns `OK STATUS` followed by `DISARMED`, `ARMED`, `FIRING`, or
`ERROR`. Because the checked-in reference may predate the C3 installed on the
robot, the diagnostic displays the received body but does not guess or require
one of those strings as the success condition for these four commands. It only
reports a valid C3 response after checking packet ID, declared length,
checksum, zero error byte, printable ASCII, and excluding a TX echo.

### Recommended on-machine order

1. Secure the robot, keep the wheels clear, select and upload only
   `bus_diagnostic`, and open the 115200 baud USB monitor.
2. Run `help`. Confirm that neither wheel motor is driven at startup.
3. Run `ping1`, then `ping2`, then `ping3`. Confirm each TX echo is labelled and
   ignored rather than counted as success, and that `ping3` accepts only a
   zero-error ID3 status packet with length 2. Run `pingall` to repeat all three
   Pings once each.
4. Run `c3ping`, `c3help`, and `c3status` individually. Confirm each report says
   `ID=3`, `checksum=OK`, and `error=00`, and inspect the printed ASCII response.
   Expected values include `OK PONG`, a help line beginning `OK HELP`, and an
   `OK STATUS ...` line. A timeout or a response classified as another ID,
   malformed, non-ASCII, or nonzero-error is a failed check.
5. **Never aim the laser at the eyes of a person or animal.** Secure the robot,
   point it in a safe direction, and initially observe the firing LED, laser,
   and audio behavior using this exact manual sequence: `c3ping`, `c3status`,
   `c3arm`, `c3status`, `c3fire 1`, `c3status`, `c3disarm`, `c3status`. Inspect
   every printed response. One `c3fire 1` input must produce exactly one TX.
   Also try `c3fire`, `c3fire 0`, `c3fire 10`, `c3fire -1`, `c3fire +1`,
   `c3fire 1x`, and `c3fire 1 2`; each must be rejected without a `TX:` line.
   A timeout or invalid response does not prove the mechanism did not fire, so
   do not repeat FIRE automatically. If anything abnormal occurs, manually
   issue `c3stop` followed by `c3disarm`.
6. With the barrel mechanism secured and clear, run `c3tilt 1500` once. Confirm
   that exactly one `TILT 1500` packet is transmitted and that success is shown
   only after a valid, non-echo ID3 response containing exactly
   `OK TILT 1500 us`. Also try missing, signed, suffixed, and out-of-range inputs
   such as `c3tilt`, `c3tilt +1500`, `c3tilt 1500x`, `c3tilt 499`, and
   `c3tilt 2401`; each must print an error without a `TX:` line or servo motion.
7. Run `pos1`, then `pos2`, and compare the raw readings with the physical legs.
8. Run `c3mute on` and confirm that exactly one `MUTE ON` packet is transmitted
   and that success is shown only for the exact response `OK MUTE ON`. Then run
   `c3mute off` and perform the equivalent checks for `MUTE OFF` and
   `OK MUTE OFF`. Try `c3mute`, `c3mute yes`, `c3mute on `, and `c3mute offx`;
   each must be rejected without a `TX:` line. MUTE affects FIRE's automatic
   audio only; manually requested SOUND playback remains audible. These are
   on-machine checks, not claims that this repository change was hardware-tested.
9. Only after securing each leg, run one command at a time in this order:
   `rhome`, `rext`, `rhome`, then `lhome`, `lext`, `lhome`.

## Unverified hardware/protocol items

- The external half-duplex transmit-enable circuit and whether it echoes TX.
- Any direction-enable pin (none is used by the existing source).
- All responses and physical motion; this repository change has not been tested
  on the robot.
