'use strict';
const MACHINE_NAMES = {
  INITIALIZING: '初期化中', READY_WAIT: '開始待機', BALANCE_WAIT: '開始姿勢の安定待ち',
  AUTO_WAIT: '起立待ち', BALANCING: '倒立中', EXPLICIT_STOP: '明示停止',
  CALIBRATION_REQUIRED: '要校正', CALIBRATING: '校正中（車輪が動きます）',
  FALL_RECOVERY_WAIT: '転倒復帰待ち', FALL_STOP: '転倒停止', FAULT: '異常', STARTUP_FAILED: '異常（起動失敗）'
};
const IMU_REASONS = {
  OK: '正常', NOT_INITIALIZED: 'IMU未初期化', CALIBRATING: 'IMU校正中',
  CALIBRATION_FAILED: 'IMU校正の既存6バイト読取りに失敗', NO_SAMPLE: '測定未取得', RIGHT_WHEEL_FAULT: '右車輪異常で制御ループ停止',
  LEFT_WHEEL_FAULT: '左車輪異常で制御ループ停止',
  BOTH_WHEEL_FAULT: '両輪異常で制御ループ停止', NONFINITE: 'IMU値が非有限',
  LOOP_STALE: 'IMU測定周期が停止（250 ms超）', IMU_I2C_FAILED: 'IMU既存読取りのI2C失敗／14バイト未取得'
};
function renderRecoveryChecks(view, sample, current, imuValid) {
  const fields = [['upright', '直立範囲', 1], ['imu', 'IMU', 2],
    ['wheels', '車輪センサー', 4], ['foc', 'FOC', 8]];
  const live = current && sample.age_ms + Date.now() - view.last <= 250 &&
    Number.isInteger(sample.recovery_checks);
  for (const [key, label, bit] of fields) {
    let node = view.el(`condition-${key}`);
    if (!node) {
      node = document.createElement('span');
      node.id = `condition-${key}`;
      node.className = 'condition';
      node.append(document.createElement('strong'), document.createElement('span'));
      node.firstChild.textContent = label;
      view.el('recovery-checks').appendChild(node);
    }
    const known = live && (key !== 'upright' || imuValid);
    const ok = known && Boolean(sample.recovery_checks & bit) && (key !== 'imu' || imuValid);
    node.dataset.ok = known ? String(ok) : 'unknown';
    let text = known ? ok ? ' ○' : ' ×' : ' ？';
    if (key === 'upright') {
      const angle = sample.values.angle_deg;
      text += ` ${imuValid && Number.isFinite(angle) ? angle.toFixed(2) + '°' : '不明'} / ±2°`;
    }
    node.lastChild.textContent = text;
  }
}
function renderDashboardStatus(view) {
    const s = view.sample, current = view.live && Date.now() - view.last <= 300;
    view.meter('left'); view.meter('right');
    if (!s) return;
    const valid = current && s.imu_valid &&
      ((s.sample_ms - s.imu_ms) >>> 0) + s.age_ms + Date.now() - view.last <= 250;
    const names = [s.faults & 1 ? `右:${s.right_error || '車輪センサー異常'}` : '',
      s.faults & 2 ? `左:${s.left_error || '車輪センサー異常'}` : '',
      s.faults & 4 ? 'FOC初期化失敗' : ''].filter(Boolean);
    const recovery = {
      UPRIGHT_REQUIRED: '直立範囲への進入待ち', ANGLE_OUTSIDE_2_DEG: '姿勢が±2°の範囲外',
      RECOVERED: '転倒から復帰完了',
      EXPLICIT_STOP: '明示停止・復帰解除', HEALTH_FAULT: '異常のため復帰解除', RESTART_REJECTED: '復帰開始を拒否'
    };
    view.el('health').textContent = current ? MACHINE_NAMES[s.state] || '状態不明' : '更新停止／不明';
    view.el('recovery-details').textContent = `${recovery[s.recovery] || s.recovery || '記録なし'}` +
      (names.length ? ` / ${names.join('・')}` : '');
    renderRecoveryChecks(view, s, current, valid);
    const calibration = {
      NOT_LOADED: '確認前', MISSING_OR_NVS_ERROR: '要校正（未保存／保存領域未取得）',
      MISSING_INVALID_CONFIG: '要校正（未保存・破損・設定不一致）', VALID_SAVED: '保存値有効',
      RESTORED: '保存値復元・今回PP未測定／保存PP合格',
      CALIBRATING_WHEELS_MOVE: '測定中・車輪が動きます', SAVING_OUTPUTS_OFF: '出力遮断・保存中',
      SAVED_STOPPED: '保存完了・停止中（今回PP合格）', SAVE_FAILED: '保存失敗・停止中',
      FAILED: '校正失敗・停止中', ABORTED: '校正中断・停止中',
      REJECTED_ACTIVE: '稼働中のため校正拒否', REJECTED_FAULT_OR_WEB: '異常／Web未起動で校正拒否'
    };
    view.el('calibration').textContent = `校正: ${current ? calibration[s.calibration] || s.calibration || '未取得' : '更新停止／不明'}`;
    view.el('stop-history').textContent = `最新停止履歴: ${s.stop_reason} @ ${s.stop_ms} ms / 起動:${s.startup_reason}`;
    view.el('measurement').textContent = `${valid ? '姿勢測定更新中' : '測定更新停止'}: ` +
      `${!current ? '診断配信停止／通信更新停止' : IMU_REASONS[s.imu_reason] || s.imu_reason}` +
      `${s.imu_reason === 'IMU_I2C_FAILED' ? `（tx=${s.imu_tx} rx=${s.imu_rx}/14）` : ''}` +
      ` / 実測 ${s.imu_ms} ms・#${s.imu_seq} / 基準角:${s.reference_valid ? '有効な設定値' : '未確定の設定値'}`;
    renderGyroWindow(view, s, valid);
    const angle = s.values.angle_deg;
    view.el('pose-value').textContent = valid && Number.isFinite(angle)
      ? `${angle > 30 ? '▲ 上限超過 ' : angle < -30 ? '▼ 下限超過 ' : '姿勢角 '}${Math.abs(angle) > 30 ? String(angle) : angle.toFixed(2)}°`
      : '姿勢角：測定更新停止／不明';
}

// Compact metadata accompanies only the selected gyro signal; no display filter.
function renderGyroWindow(view, sample, valid) {
  let node = view.el('gyro-window');
  if (!node) {
    node = document.createElement('p');
    node.id = 'gyro-window';
    view.el('measurement').after(node);
    const note = document.createElement('p');
    note.textContent = '振動比較：内部DLPF後・オフセット補正後の角速度。' +
      '内部DLPF前の値は取得できません。ソフトLPFなし。' +
      'RMS・最大絶対値は制御周期ごとの約200 ms集計（時間加重なし）。' +
      '20 Hz波形では高周波を識別できず、集計でも周波数は特定できません。';
    node.after(note);
  }
  const w = sample.gyro_window;
  const current = valid && Array.isArray(w) && w.length === 9 &&
    ((sample.sample_ms - w[1]) >>> 0) + sample.age_ms + Date.now() - view.last <= 500;
  if (!current || !Number.isFinite(w[7]) || !Number.isFinite(w[8])) {
    node.textContent = '角速度短区間集計：更新停止／未取得／無効（角速度を選択）';
    return;
  }
  node.textContent = `CONFIG=0x${w[0].toString(16).padStart(2, '0')} / ` +
    `RMS ${w[7].toFixed(3)} deg/s・最大絶対 ${w[8].toFixed(3)} deg/s / ` +
    `${(w[2] / 1000).toFixed(1)} ms・${w[3]}点・無効${w[4]} / ` +
    `取得間隔 ${w[5]}～${w[6]} us / 終了 ${w[1]} ms`;
}
