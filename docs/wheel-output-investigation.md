# 通常ファームの左車輪出力調査

基準: `be45f14cb2f45ceea82973e22e1a494185990862`、`codex/2208-web-ps4`。
`AGENTS.md`・[実装の由来](upstream-origin.md)を確認。SimpleFOCは
v2.4.0 / `4f072b365f6e0185adca544071e595834405babc`。

## 結論と証拠の範囲

**通常倒立時に左だけ回らない原因は未確定。左回転の修復完了とはしない。**
利用者の実機結果では左センサー、単体診断の±6 rad/s、保存FOCの受理・復元は成功している。
通常運転のEN=1、enabled=1、status=4は必要条件だが、q電圧・相電圧・端子波形・トルクの証明ではない。
Webの左+1 V表示は実指令の−1 Vを表す。表示反転や車体方向係数を変更していない。

固定版の実SimpleFOC `FOCMotor.cpp`、`BLDCMotor.cpp`、`BLDCDriver3PWM.cpp`を
ホストでリンクし、右CCW/1.3360963、左CCW/5.8521371、7極対、電圧トルク制御、
左右±1 V、両輪初期化でmove→次FOC→各相dutyまで両符号が伝わることを確認した。
HALだけを代替しており、ESP32端子や実モーターの再現試験ではない。
この試験では左だけ停止するソフト経路を再現しなかった。
ゲイン・電圧増加、方向係数・PID・DLPF・起立／転倒復帰の変更、保存値の削除は行っていない。

別途、**既存の通常ファームのpolarity試験が500 ms終了／傾斜超過後に
目標をゼロにするだけで両輪のENを下げず、結果を同期出力していた不具合**を確認した。
これを修正し、次のFOCより前に期限／既存10°条件を判定し、結果出力より前に
両輪を遮断する。明示終了k、開始要求時の傾斜拒否も遮断する。
再試験にはu/jの明示操作が必要。これは試験終了時の修正であり、通常倒立の左不動の原因とは扱わない。
パルスの500 ms・電圧・傾斜条件は変更しない。独立したハードウェア監視タイマーの追加ではなく、
既存の制御ループによる期限判定であるため、ループ停止中の500 ms遮断を保証しない。

## 通常ファームと単体診断の比較

| 項目 | 通常 `src/main.cpp` | 単体 `src/wheel_diagnostic.cpp` |
|---|---|---|
| 右対応 | motor1 / driver1 / sensor1、I2C0 SDA19/SCL18、AS5600 0x36 | rightMotor / rightDriver / rightSensor、同じバス・ピン |
| 左対応 | motor2 / driver2 / sensor2、I2C1 SDA23/SCL5、AS5600 0x36 | leftMotor / leftDriver / leftSensor、同じバス・ピン |
| 右PWM / EN | A13 B12 C14 / 27 | 同じ |
| 左PWM / EN | A26 B25 C33 / 32 | 同じ |
| 電源・制限 | supply/driver limit=8.3 V、初期化/align=2 V、起動完了後motor limit=1 V | supply/driver limit=8.3 V、motor/align=2 V |
| 制御 | torque / voltage。targetはV、moveがcurrent_spへ転記 | velocity / voltage。targetはrad/s、速度PIDがcurrent_spを生成 |
| ±6の意味 | ±6 Vなら1 Vへ制限される（今回要求しない） | ±6 rad/s。PID出力のq電圧とは別物 |
| 初期化 | EN低→UART/両I2C/Web準備→MPU確認/校正→右driver/motor/FOC→左driver/motor/FOC→両motor limit更新 | EN低→両センサー準備→明示要求した輪のdriver/motor/FOC。both要求時は両輪を順に初期化 |
| センサー | checked RAW_ANGLE、制御周期の先頭で両輪取得後にFOCは同じキャッシュを再利用 | checked RAW_ANGLE、回転中輪はloopFOC内で取得、非駆動輪を別に監視 |
| 校正 | 保存direction/zero/PPをinitFOC直前に適用。明示再校正時だけUNKNOWN/NOT_SETへ戻す | 明示初期化で実測校正 |
| 周期 | 両輪とIMUの安全確認→右/左loopFOC→制御・安全判定→右/左move | 駆動輪loopFOC→move。速度制御でも次FOCに適用 |
| 停止/復帰 | gate→target/current_sp/q/dゼロ→driver.disable。条件を満たす既存復帰でenable。保存方向/zeroは書き換えない | gate→出力ゼロ/disable、タイマーとループ双方の停止経路 |
| 周辺機能 | MPUは右I2Cを共用。脚UART RX16/TX17、LED22、WiFi/PS4/HTTP | それらを起動しない |

通常ファームのMCPWM設定はSimpleFOC `esp32_mcpwm_mcu.cpp::_configure3PWM` →
`esp32_driver_mcpwm.cpp::_findBestGroup/_configurePinsMCPWM`。
既定25 kHz。空きグループを優先する実関数を回帰試験で実行し、
右はgroup0/timer0、続く左はgroup1/timer0になることを確認。
各グループでoperator[0]のcomparator/generator[0,1]をA/B、operator[1]の[0]をCに使う。
単体で左だけを最初に初期化すれば左がgroup0、bothなら初期化順にgroup0/1。
この2輪構成ではshared_timer分岐へ入らない。脚UART・LED・I2Cは上記PWMピンと重複せず、
コード上に別のMCPWM利用もない。PS4/WiFiの時間・メモリー負荷がないことまでは証明しない。
この割当は固定版のソースとallocator実行による確認で、実機レジスター読戻しではない。

SimpleFOCの電気角はnormalize(direction×7×mechanical−zero)。保存値が設定済みなら
alignSensorはその方向・ゼロを再測定せず、通常のmove/loopFOCも書き換えない。
電圧トルクモードではmove(target)→current_sp=target、次loopFOCで
q=constrain(current_sp,±motor_limit)+feed_forward.q、d=feed_forward.d。
本構成はfeed_forward=0。setPhaseVoltageが相電圧を計算し、driverが0～8.3 Vに制限して
供給電圧で割ったdutyをMCPWMへ渡す。motor limitとdriver limitを混同しない。

## 一回だけの要求と観測時点

既存WebSocketの指令値・q・`foc_capture_us`は維持する。ただし別周期の値なので、
それだけで同じ指令の適用を証明しない。既存`captureFocOutputSnapshot()`の直後と、
既存moveの直後に固定長の単発観測を追加した。センサーの追加update/I2C、
制御経路での文字列化・Serial・動的確保・待機はない。

同じ機体のWebアドレスに以下を要求する。再取得は新しいPOSTを明示する。

```sh
curl -X POST http://ROBOT_IP/wheels/output-capture
curl http://ROBOT_IP/api/wheels/output
```

POSTは202、保留中の追加POSTは409。GETは同じ一件を返すだけで取り直さない。
`id`が取得番号。`state=1`要求待ち、2 move取得済み/次FOC待ち、3 FOC取得済み、
4停止・異常等で中断。0は未要求。1/2なら少し後にGETだけを再実行する。
中断時は`applied=0`で、出力欄の初期値を実測ゼロと解釈しない。

| 項目 | 取得時点・意味 |
|---|---|
| request_us | HTTPタスクが単発要求を受け付けた時刻 |
| command_us / command_V | 周期N、安全判定を通した右/左実指令をmoveへ渡す直前。左は生の符号 |
| move_us / move_target / move_current_sp | 周期N、両move完了直後の公開値。current_spはこのモードではV |
| sensor_us | 周期N+1、両prepareControlCycle・MPU・軸状態更新の完了時刻。各I2C転送の開始時刻ではない |
| foc_us=start..end | 周期N+1、左右それぞれのloopFOC呼出し前後 |
| foc_target / foc_current_sp / q,d / electrical | 両FOC完了後、次の制御計算・moveより前 |
| phase_V / duty | そのFOC後のBLDCMotor Ua/Ub/Uc（driver制限前）とdriver dc_a/b/c（制限後、HALへ渡した比率） |
| mechanical / shaft_angle / shaft_velocity | mechanicalは周期N+1のセンサーキャッシュ、shaft値は同周期の既存updateLqrStateが更新した公開キャッシュ。追加速度フィルター呼出しなし |
| motor_limit / driver_limit / supply、mode / torque、status / enabled / driver_init / EN | FOC後のソフト設定・状態、ENはGPIO読取り。driverには独立したenabled公開値がない |
| zero / direction / poles | FOC後の現用電気角ゼロ・センサー方向・極対数 |

左右の読取りは逐次処理で同時刻ではない。32bit microsは約71分で周回するため差はunsignedで扱う。
`applied=1`はFOCを呼び両EN/enableがあり、FOC時current_spと取得したmove値が一致したことを表す。
これは回転や物理出力の保証ではない。**dutyはソフトのPWM設定値であり、端子波形の実測ではない。**
MCPWM比較器の設定成功・カウンター・ゲートドライバー出力・相電流は取得していない。
HALは内部でperiod×dutyの整数compareへ変換するが、この記録はcompareレジスターの読戻しではない。

## 最小限の実機確認

1. 支持して車輪を浮かせ、通常ファームで症状が出ている短い間に一回取得し、停止する。
   左表示+1に対してcommandが−1か、move→次FOCのcurrent_sp/qが一致するかをまず見る。
2. 必要なら通常ファームの既存低電圧試験を使う。支持・明示d停止→u→j、
   5=右正、6=右負、7=左正、8=左負（正負は車体方向係数を通す前のパルス符号）。
   各回500 ms後に両輪遮断するため、次の一回の前にu→jで再許可する。
   必要に応じて各パルス中に上記単発取得。既定0.15 Vで回らなくても故障確定にしない。
   今回、倒立電圧・ゲインや試験電圧を上げる手順は追加しない。最後はd。
3. command→target/current_spに差があれば制御ゲート・状態を、qまで一致して相/dutyに差があれば
   角度・変調・driver制限を調べる。左右とも妥当な相/dutyでも左が止まれば、
   次は端子PWM・EN・電源・相電流等の実測でHAL以降と電気的トルク生成を分ける。

配線変更を必要とする根拠は得ていないため、相線の差替えやFOC再登録を要求しない。
配線確認が別途必要と判明した場合は電源を切って行う。

## 検証・資源・更新

新規`tests/wheel_output_host/run.py`は実SimpleFOCと実MCPWM allocatorで
両符号・保存CCW/zero・1周期遅延・q/d・相/duty・電圧/速度モードの差・制限・無効状態・
中断と単発取得、文字列上限、追加センサー読取りがないことを検証。
既存drive_hostに500 ms前後・左右別両符号・傾斜停止・両EN遮断・HTTP単発/重複拒否を追加。
全アプリループの順序も検査する。HTTP起動試験には新規ハンドラーの代替を追加した。
脚・PS4・保存・配信等の既存ホスト試験、ASan/UBSan、通常/車輪診断ESP32ビルドを実施する。
LeakSanitizerは実行環境のptrace制約で動作せず、`ASAN_OPTIONS=detect_leaks=0`で
ASan/UBSanを実行。リーク検査成功とはしない。

最終結果（同一環境・同一固定依存版、SSIDはexample設定）:

- 基準版と最終版の通常ESP32ビルド成功。車輪診断ESP32ビルドも成功。
- 通常静的RAM 84,772→85,060 B、**+288 B**。
  ELFの観測オブジェクトは284 B（Record276 B＋ESP32ロック8 B）、残りは配置の差4 B。
- Flash 1,955,601→1,959,029 B、**+3,428 B**。
- 既存ホスト26コマンド（車輪・姿勢・脚・UART・保存・PS4・HTTP・配信・Web）、
  drive_hostと新規wheel_output_hostが成功。環境依存パスは
  `PLATFORMIO_CORE_DIR=/workspace/.platformio`を指定して再実行した。
- ASan/UBSan: 実SimpleFOC出力/allocator、drive_host（従来/production roll両設定）、
  leg_host、leg_tx_host、leg_calibration_host、telemetry_hostが成功。
- `git diff --check`成功。実機回転・端子波形・通信負荷下の周期は未検証。
一件だけの固定長記録で履歴・FIFO・タスク・スタック配列の増設なし。
HTTP応答時だけ記録コピーと最大1900 Bの文字列を既存8 KiB HTTPスタックに置く。
新規2ルート分のWebServer起動時確保はある。実機ヒープ・stack high-water・周期負荷は未測定。

**更新対象は通常ファームのみ。LittleFSの更新・NVS校正再登録は不要。**
wheel_diagnosticは比較ビルドのみでソース不変。実機書込み・マージは行わない。
ソフト試験の成功と実機左回転成功を区別し、後者は未検証。
