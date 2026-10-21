// Production modules: bounded history and segment boundaries, without a browser.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = name => fs.readFileSync(`data/${name}`, 'utf8');
const assets = ['index.html', 'style.css', 'panels.css', 'signals.js', 'history.js', 'plot.js', 'subscribe.js', 'dashboard.js', 'app.js'];
const context = vm.createContext({});
for (const name of ['signals.js', 'history.js', 'plot.js', 'dashboard.js']) {
  vm.runInContext(source(name), context);
}
vm.runInContext(`
  const h = new DiagnosticHistory();
  for (let i = 0; i < 4000; i++) {
    h.push({ sample_ms: i * 50, seq: i, subscription_id: 1,
      boot_id: 'a', dropped: 0, values: { angle_error_deg: i },
      stop_reason: '"stop"' });
  }
  if (h.rows.length !== 1200 || h.clock - h.rows[0].time > 60000) throw Error('limit');
  if (!h.csv().includes('""stop""') || h.csv().includes('right_velocity_rad_s')) throw Error('CSV');
  h.clear();
  const sample = (seq, ms, value, generation = 1, dropped = 0) => ({
    seq, sample_ms: ms, subscription_id: generation, boot_id: 'a', dropped,
    values: { x: value }
  });
  h.push(sample(1, 0xfffffff0, 1));
  h.push(sample(2, 34, 2));
  if (h.rows[1].time - h.rows[0].time !== 50 || h.rows[1].gap) throw Error('wrap');
  h.clear();
  h.push(sample(1, 0, 1)); h.push(sample(2, 50, 2));
  h.push(sample(4, 100, 3)); h.push(sample(5, 150, null));
  h.push(sample(6, 200, 4)); h.push(sample(7, 250, 5, 2));
  h.push(sample(8, 300, 6, 2, 1));
  h.breakNext = true; h.push(sample(9, 350, 7, 2, 1));
  const parts = signalSegments(h.rows, 'x');
  if (JSON.stringify(parts.map(p => p.length)) !== '[2,1,1,1,1,1]') throw Error('gaps');
  if (SIGNALS.length !== 32 || new Set(COLORS).size !== 32) throw Error('fields/colors');
  const output = value => ({ type: 'sample', age_ms: 0, disabled: false,
    command_valid: true, values: { left_command_v: value } });
  for (const v of [0, 0.35, -0.2, 1, -1, 1.5, -1.5]) {
    const r = outputReading(output(v), 'left_command_v', true);
    if (r.arc !== Math.min(Math.abs(v), 1) * Math.PI || r.ccw !== (v < 0) ||
      r.over !== (Math.abs(v) > 1)) throw Error('arc');
  }
  if (outputReading({ ...output(0.2), disabled: true }, 'left_command_v', true).text !== '遮断') throw Error('disabled');
  if (outputReading(output(0.2), 'left_command_v', false).state !== 'unknown' ||
    outputReading(output(null), 'left_command_v', true).state !== 'unknown') throw Error('stale');
  if (DEFAULTS.length !== 1 || DEFAULTS[0] !== 'gyro_deg_s' ||
    !PERMANENT.includes('angle_deg') || !PERMANENT.includes('reference_deg')) throw Error('permanent');
  for (const [unit, range] of Object.entries(MINIMUM_RANGES)) {
    const rows = [{ values: { x: 0.0001 } }, { values: { x: -0.0001 } }];
    if (JSON.stringify(axisRange(rows, ['x'], unit)) !== JSON.stringify(range)) throw Error('minimum');
    const auto = axisRange(rows, ['x'], unit, true);
    if (auto[0] >= -0.0001 || auto[1] <= 0.0001 || auto[1] - auto[0] >= 0.001) throw Error('auto');
    const large = axisRange([{ values: { x: range[1] * 3 } }], ['x'], unit);
    if (large[0] > range[0] || large[1] <= range[1] * 3) throw Error('extend');
    const invalid = axisRange([{ values: { x: null } }], ['x'], unit);
    if (JSON.stringify(invalid) !== JSON.stringify(range)) throw Error('invalid scale');
  }
  if (DEFAULTS.length !== 1 || PRESETS.position.length !== 8 || PRESETS.drive.length !== 10) throw Error('presets');
`, context);
for (const file of assets) {
  const text = source(file);
  const crlf = text.replace(/\r?\n/g, '\r\n');
  assert(Buffer.byteLength(crlf) <= 8192, file);
  assert(text.split('\n').length > 20, `${file}: readable source`);
}
const html = source('index.html');
assert(!/<(?:link|script)[^>]+(?:href|src)="\//i.test(html));
assert(!html.includes('<details open'));
assert(!/購読開始|購読適用/.test(assets.map(source).join('')));
const cpp = fs.readFileSync('src/main.cpp', 'utf8');
for (const file of assets.slice(1)) assert(cpp.includes(`"/${file}"`), `route ${file}`);
console.log('PASS: 60s/1200 history, wrap, seq/drop/invalid/generation/reconnect gaps, CSV, presets, 32 stable colors, LF/CRLF size caps and routes');
