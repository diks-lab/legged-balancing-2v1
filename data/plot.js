'use strict';
// One plot, independent Y scales per unit. No interpolation or filtering.
function signalSegments(rows, key) {
  const segments = [];
  let segment = [];
  for (const row of rows) {
    const value = row.values[key];
    if (row.gap || !Number.isFinite(value)) {
      if (segment.length) segments.push(segment);
      segment = [];
    }
    if (Number.isFinite(value)) segment.push([row.time, value]);
  }
  if (segment.length) segments.push(segment);
  return segments;
}

const MINIMUM_RANGES = {
  deg: [-20, 20], 'deg/s': [-100, 100], V: [-1, 1], 'rad/s': [-6, 6]
};

function axisRange(rows, keys, unit, autoScale = false) {
  let lo = Infinity, hi = -Infinity;
  for (const row of rows) {
    for (const key of keys) {
      const value = row.values[key];
      if (!Number.isFinite(value)) continue;
      lo = Math.min(lo, value);
      hi = Math.max(hi, value);
    }
  }
  const minimum = MINIMUM_RANGES[unit];
  if (!Number.isFinite(lo)) return minimum ? [...minimum] : [-1, 1];
  if (!autoScale && minimum) {
    const padding = Math.max(hi - lo, minimum[1] - minimum[0]) * 0.08;
    return [
      lo < minimum[0] ? lo - padding : minimum[0],
      hi > minimum[1] ? hi + padding : minimum[1]
    ];
  }
  if (lo === hi) {
    const padding = Math.max(Math.abs(lo) * 0.08, 0.01);
    return [lo - padding, hi + padding];
  }
  const padding = (hi - lo) * 0.08;
  return [lo - padding, hi + padding];
}

class OverlayPlot {
  constructor(canvas, legend, history) {
    this.canvas = canvas;
    this.legend = legend;
    this.history = history;
    this.signals = [];
    this.hidden = new Set();
  }

  select(keys) {
    this.signals = SIGNALS.filter(s => keys.includes(s[0]));
    this.legend.replaceChildren();
    for (const signal of this.signals) {
      const [key, label, unit] = signal;
      const button = document.createElement('button');
      button.textContent = `${label} [${unit}]`;
      button.style.setProperty('--signal-color', COLORS[SIGNALS.indexOf(signal)]);
      button.className = 'legend-item';
      const update = () => {
        button.setAttribute('aria-pressed', String(!this.hidden.has(key)));
      };
      button.onclick = () => {
        if (this.hidden.has(key)) this.hidden.delete(key);
        else this.hidden.add(key);
        update();
        this.draw();
      };
      update();
      this.legend.appendChild(button);
    }
  }

  setPaused(paused) {
    this.pausedClock = paused ? this.history.clock : null;
  }

  draw() {
    const canvas = this.canvas;
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    if (!width || !height) return;
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
    const c = canvas.getContext('2d');
    c.scale(ratio, ratio);
    c.font = '14px sans-serif';
    const visible = this.signals.filter(s => !this.hidden.has(s[0]));
    const units = [...new Set(visible.map(s => s[2]))];
    // Angle/reference share the nearest left axis, regardless of selection order.
    if (units.includes('deg')) {
      units.splice(units.indexOf('deg'), 1);
      units.unshift('deg');
    }
    // Freeze the displayed time, not a second measurement history.
    // Legend changes use only still-retained rows at/before that time.
    const end = this.pausedClock ?? this.history.clock;
    const rows = this.pausedClock == null ? this.history.rows
      : this.history.rows.filter(row => row.time <= end);
    const left = 12 + Math.ceil(units.length / 2) * 76;
    const right = width - 12 - Math.floor(units.length / 2) * 76;
    const top = 40;
    const bottom = height - 30;
    const start = end - 60000;
    const scales = new Map();
    units.forEach((unit, index) => {
      const keys = visible.filter(s => s[2] === unit).map(s => s[0]);
      const [lo, hi] = axisRange(rows, keys, unit, this.autoScale);
      scales.set(unit, { lo, hi });
      const x = index % 2 === 0
        ? left - 8 - Math.floor(index / 2) * 76
        : right + 8 + Math.floor(index / 2) * 76;
      c.fillStyle = '#243247';
      c.textAlign = index % 2 === 0 ? 'right' : 'left';
      c.fillText(unit === 'deg' ? 'deg 主軸' : unit, x, 18);
      c.fillText(hi.toPrecision(3), x, top + 5);
      c.fillText(lo.toPrecision(3), x, bottom);
      c.strokeStyle = '#b5bfcc';
      c.beginPath();
      c.moveTo(x, top + 12);
      c.lineTo(x, bottom - 18);
      c.stroke();
    });
    c.strokeStyle = '#d4dae2';
    c.strokeRect(left, top, right - left, bottom - top);
    c.fillStyle = '#243247';
    c.textAlign = 'left';
    c.fillText(`${(start / 1000).toFixed(1)} s`, left, height - 8);
    c.textAlign = 'right';
    c.fillText(`${(end / 1000).toFixed(1)} s`, right, height - 8);
    c.textAlign = 'center';
    c.fillText('ESP32取得時刻 [s] ／ 単位ごとに独立した縦軸',
      (left + right) / 2, height - 8);
    for (const signal of visible) {
      const [key, , unit] = signal;
      const { lo, hi } = scales.get(unit);
      c.strokeStyle = COLORS[SIGNALS.indexOf(signal)];
      c.lineWidth = 1.8;
      c.beginPath();
      for (const segment of signalSegments(rows, key)) {
        segment.forEach(([time, value], i) => {
          const x = left + (time - start) / 60000 * (right - left);
          const y = bottom - (value - lo) / (hi - lo) * (bottom - top);
          if (i) c.lineTo(x, y);
          else c.moveTo(x, y);
        });
      }
      c.stroke();
    }
  }
}
if (typeof module !== 'undefined') module.exports = { signalSegments, axisRange, MINIMUM_RANGES };
