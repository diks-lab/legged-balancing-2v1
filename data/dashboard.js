'use strict';
function outputReading(sample, key, live, elapsed = 0) {
  if (!live || !sample || sample.type !== 'sample' || sample.age_ms + elapsed > 250) return { state: 'unknown', text: '更新停止／不明' };
  if (sample.disabled) return { state: 'disabled', text: '遮断' };
  if (!sample.command_valid || ((sample.sample_ms - sample.command_ms) >>> 0) +
    sample.age_ms + elapsed > 250 || !Number.isFinite(sample.values[key])) return { state: 'unknown', text: '更新停止／不明' };
  // Only the left donut presentation is reversed; never modify the sample.
  const value = sample.values[key] * (key === 'left_command_v' ? -1 : 1);
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
  status() { renderDashboardStatus(this); }
  draw() { this.pose.draw(); this.aux.draw(); }
  pause(paused) { this.pose.setPaused(paused); this.aux.setPaused(paused); }
}
