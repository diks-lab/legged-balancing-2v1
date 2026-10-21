// Production modules: bounded history and segment boundaries, without a browser.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = name => fs.readFileSync(`data/${name}`, 'utf8');
const assets = ['index.html', 'style.css', 'signals.js', 'history.js', 'plot.js', 'app.js'];
const context = vm.createContext({});
for (const name of ['signals.js', 'history.js', 'plot.js']) {
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
  if (DEFAULTS.length !== 7 || PRESETS.position.length !== 8 || PRESETS.drive.length !== 10) throw Error('presets');
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
