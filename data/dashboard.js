'use strict';
const MACHINE_NAMES = {
  INITIALIZING: '初期化中', READY_WAIT: '開始待機', BALANCE_WAIT: '開始姿勢の安定待ち',
  AUTO_WAIT: '自動開始待ち', BALANCING: '倒立中', EXPLICIT_STOP: '明示停止',
  CALIBRATION_REQUIRED: '要校正', CALIBRATING: '校正中（車輪が動きます）',
  FALL_RECOVERY_WAIT: '転倒復帰待ち', FALL_STOP: '転倒停止', FAULT: 'センサー異常', STARTUP_FAILED: '起動失敗'
};
const IMU_REASONS = {
  OK: '正常', NOT_INITIALIZED: 'IMU未初期化', CALIBRATING: 'IMU校正中',
  CALIBRATION_FAILED: 'IMU校正の既存6バイト読取りに失敗', NO_SAMPLE: '測定未取得', RIGHT_WHEEL_FAULT: '右車輪異常で制御ループ停止',
  LEFT_WHEEL_FAULT: '左車輪異常で制御ループ停止',
  BOTH_WHEEL_FAULT: '両輪異常で制御ループ停止', NONFINITE: 'IMU値が非有限',
  LOOP_STALE: 'IMU測定周期が停止（250 ms超）', IMU_I2C_FAILED: 'IMU既存読取りのI2C失敗／14バイト未取得'
};
function outputReading(sample, key, live, elapsed = 0) {
  if (!live || !sample || sample.type !== 'sample' || sample.age_ms + elapsed > 250) return { state: 'unknown', text: '更新停止／不明' };
  if (sample.disabled) return { state: 'disabled', text: '遮断' };
  if (!sample.command_valid || ((sample.sample_ms - sample.command_ms) >>> 0) +
    sample.age_ms + elapsed > 250 || !Number.isFinite(sample.values[key])) return { state: 'unknown', text: '更新停止／不明' };
  const value = sample.values[key];
  const number = Math.abs(value) > 0 && Math.abs(value) < 0.005
    ? Math.abs(value).toPrecision(2) : Math.abs(value) > 1
      ? String(Math.abs(value)) : Math.abs(value).toFixed(2);
  return { state: value === 0 ? 'zero' : value > 0 ? 'positive' : 'negative',
    text: `${value < 0 ? '−' : '+'}${number} V`, value,
    arc: Math.min(Math.abs(value), 1) * Math.PI, ccw: value < 0,
    over: Math.abs(value) > 1 };
}

class Dashboard {
  constructor(el, history) {
    this.el = el;
    this.history = history;
    this.live = false;
    this.last = 0;
    this.sample = null;
    const poseView = {
      get clock() { return history.clock; },
      get rows() {
        let previous = null;
        return history.rows.filter(row => {
          if (row.imu_valid && row.imu_seq === previous) return false;
          previous = row.imu_valid ? row.imu_seq : null;
          return true;
        }).map(row => ({ ...row,
          time: row.imu_valid ? row.time - ((row.sample_ms - row.imu_ms) >>> 0) : row.time,
          values: { angle_deg: row.imu_valid ? row.values.angle_deg : null,
            reference_deg: row.values.reference_deg }
        }));
      }
    };
    this.pose = new OverlayPlot(el('pose'), el('pose-legend'), poseView);
    this.pose.fixedAttitude = true;
    this.pose.select(['angle_deg', 'reference_deg']);
    this.aux = new OverlayPlot(el('plot'), el('legend'), history);
    for (const side of ['left', 'right']) {
      new ResizeObserver(() => this.meter(side)).observe(el(`${side}-meter`));
    }
    new ResizeObserver(() => this.draw()).observe(el('pose'));
    new ResizeObserver(() => {
      el('auxiliary').style.minHeight = `${el('legend').offsetHeight + 140}px`;
    }).observe(el('legend'));
    setInterval(() => this.status(), 100);
  }

  stop() { this.live = false; this.status(); }
  update(sample) {
    this.sample = sample;
    this.last = Date.now();
    this.status();
  }
  meter(side) {
    const s = this.sample;
    const reading = outputReading(s, `${side}_command_v`, this.live && Date.now() - this.last <= 300, Date.now() - this.last);
    const el = this.el;
    el(`${side}-reading`).textContent = reading.text;
    el(`${side}-limit`).textContent = reading.over ? `表示範囲超過：${reading.text}` : '指令電圧・±1 V＝半周';
    el(`${side}-reading`).dataset.state = reading.state;
    const canvas = el(`${side}-meter`);
    const size = canvas.clientWidth;
    if (!size) return;
    canvas.width = canvas.height = size * (window.devicePixelRatio || 1);
    const c = canvas.getContext('2d');
    c.scale(canvas.width / size, canvas.width / size);
    const radius = size * 0.39, center = size / 2;
    c.lineWidth = size * 0.09;
    c.strokeStyle = '#d5dbe3';
    c.beginPath(); c.arc(center, center, radius, 0, Math.PI * 2); c.stroke();
    c.strokeStyle = '#697789';
    c.lineWidth = 2;
    c.beginPath(); c.moveTo(center, size * 0.04); c.lineTo(center, size * 0.16); c.stroke();
    if (reading.arc > 0) {
      c.lineWidth = size * 0.09;
      c.strokeStyle = reading.ccw ? '#c2255c' : '#1971c2';
      c.beginPath();
      c.arc(center, center, radius, -Math.PI / 2,
        -Math.PI / 2 + (reading.ccw ? -reading.arc : reading.arc), reading.ccw);
      c.stroke();
    }
  }
  status() {
    const s = this.sample, current = this.live && Date.now() - this.last <= 300;
    this.meter('left'); this.meter('right');
    if (!s) return;
    const valid = current && s.imu_valid &&
      ((s.sample_ms - s.imu_ms) >>> 0) + s.age_ms + Date.now() - this.last <= 250;
    const names = [s.faults & 1 ? `右:${s.right_error || '車輪センサー異常'}` : '',
      s.faults & 2 ? `左:${s.left_error || '車輪センサー異常'}` : '',
      s.faults & 4 ? 'FOC初期化失敗' : ''].filter(Boolean);
    const recovery = {
      UPRIGHT_REQUIRED: '直立保持待ち', ANGLE_OUTSIDE_2_DEG: '姿勢が±2°の範囲外',
      GYRO_TOO_FAST: '角速度が安定範囲外', WHEELS_MOVING: '車輪速度が安定範囲外',
      HOLD_1500_MS: '直立を1.5秒保持中', RECOVERED: '転倒から復帰完了',
      EXPLICIT_STOP: '明示停止・復帰解除', HEALTH_FAULT: '異常のため復帰解除', RESTART_REJECTED: '復帰開始を拒否'
    };
    this.el('health').textContent = `${MACHINE_NAMES[s.state] || '状態不明'} / ` +
      (!current ? '通信・配信更新停止 / ' : '') +
      `${recovery[s.recovery] || (s.running ? '制御中' : s.stop_reason === 'NONE' ? s.startup_reason : s.stop_reason)}` +
      (names.length ? ` / ${names.join('・')}` : '');
    const calibration = {
      NOT_LOADED: '確認前', MISSING_OR_NVS_ERROR: '要校正（未保存／保存領域未取得）',
      MISSING_INVALID_CONFIG: '要校正（未保存・破損・設定不一致）', VALID_SAVED: '保存値有効',
      RESTORED: '保存値復元・今回PP未測定／保存PP合格',
      CALIBRATING_WHEELS_MOVE: '測定中・車輪が動きます', SAVING_OUTPUTS_OFF: '出力遮断・保存中',
      SAVED_STOPPED: '保存完了・停止中（今回PP合格）', SAVE_FAILED: '保存失敗・停止中',
      FAILED: '校正失敗・停止中', ABORTED: '校正中断・停止中',
      REJECTED_ACTIVE: '稼働中のため校正拒否', REJECTED_FAULT_OR_WEB: '異常／Web未起動で校正拒否'
    };
    this.el('calibration').textContent = `校正: ${current ? calibration[s.calibration] || s.calibration || '未取得' : '更新停止／不明'}`;
    this.el('stop-history').textContent = `最新停止履歴: ${s.stop_reason} @ ${s.stop_ms} ms / 起動:${s.startup_reason}`;
    this.el('measurement').textContent = `${valid ? '姿勢測定更新中' : '測定更新停止'}: ` +
      `${!current ? '診断配信停止／通信更新停止' : IMU_REASONS[s.imu_reason] || s.imu_reason}` +
      `${s.imu_reason === 'IMU_I2C_FAILED' ? `（tx=${s.imu_tx} rx=${s.imu_rx}/14）` : ''}` +
      ` / 実測 ${s.imu_ms} ms・#${s.imu_seq} / 基準角:${s.reference_valid ? '有効な設定値' : '未確定の設定値'}`;
    const angle = s.values.angle_deg;
    this.el('pose-value').textContent = valid && Number.isFinite(angle)
      ? `${angle > 30 ? '▲ 上限超過 ' : angle < -30 ? '▼ 下限超過 ' : '姿勢角 '}${Math.abs(angle) > 30 ? String(angle) : angle.toFixed(2)}°`
      : '姿勢角：測定更新停止／不明';
  }
  draw() { this.pose.draw(); this.aux.draw(); }
  pause(paused) { this.pose.setPaused(paused); this.aux.setPaused(paused); }
}
