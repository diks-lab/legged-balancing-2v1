// Production renderer with captured canvas paths; no changes to stored samples.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const context = vm.createContext({ window: { devicePixelRatio: 1 } });
for (const file of ['signals.js', 'history.js', 'plot.js']) {
  vm.runInContext(fs.readFileSync(`data/${file}`, 'utf8'), context);
}
vm.runInContext(`
  const operations = [];
  const canvasContext = {
    path: [], scale() {}, fillText() {}, strokeRect() {}, save() {}, restore() {},
    setLineDash() {}, beginPath() { this.path = []; },
    rect(...args) { operations.push({ type: 'clip', args }); }, clip() {},
    moveTo(...args) { this.path.push(['move', ...args]); },
    lineTo(...args) { this.path.push(['line', ...args]); },
    arc(...args) { this.path.push(['point', ...args]); },
    stroke() { operations.push({ type: 'stroke', color: this.strokeStyle, path: [...this.path] }); },
    fill() { operations.push({ type: 'fill', color: this.fillStyle, path: [...this.path] }); }
  };
  const history = new DiagnosticHistory();
  const plot = new OverlayPlot({ clientWidth: 800, clientHeight: 300,
    getContext: () => canvasContext }, {}, history);
  plot.fixedAttitude = true;
  plot.signals = SIGNALS.filter(s => ['angle_deg', 'reference_deg'].includes(s[0]));
  const angleColor = COLORS[SIGNALS.findIndex(s => s[0] === 'angle_deg')];
  const referenceColor = COLORS[SIGNALS.findIndex(s => s[0] === 'reference_deg')];
  function sample(seq, value) {
    history.push({ boot_id: 'a', seq, sample_ms: 10000 + seq * 50,
      subscription_id: 1, dropped: 0, values: { angle_deg: value, reference_deg: 0 } });
  }
  function render() {
    operations.length = 0;
    const original = history.csv();
    plot.draw();
    if (history.csv() !== original) throw Error('renderer mutated history/CSV');
    return operations.filter(o => o.color === angleColor);
  }
  function paths() { return render().filter(o => o.type === 'stroke').flatMap(o => o.path); }
  if (render().some(o => o.path.length)) throw Error('unreceived posture');
  sample(1, -48.9);
  let points = render().filter(o => o.type === 'fill').flatMap(o => o.path);
  if (points.length !== 1 || points[0][2] !== 270) throw Error('first point at lower edge');
  if (history.rows[0].values.angle_deg !== -48.9 || !history.csv().includes('-48.9')) throw Error('raw angle lost');
  sample(2, -49);sample(3, -30);sample(4, -29);
  let line = paths();
  if (line.length !== 4 || line.slice(0, 3).some(p => p[2] !== 270) || line[3][2] <= 40 || line[3][2] >= 270) throw Error('lower edge/reentry');
  sample(5, 48);sample(6, 60);
  line = paths();
  if (line.at(-1)[2] !== 40 || line.at(-2)[2] !== 40) throw Error('upper edge');
  const clip = operations.find(o => o.type === 'clip').args;
  if (!(clip[1] < 38 && clip[1] + clip[3] > 272)) throw Error('edge dots clipped');
  const reference = operations.filter(o => o.color === referenceColor && o.type === 'stroke').flatMap(o => o.path);
  if (referenceColor === angleColor || reference.some(p => p[2] !== 155)) throw Error('reference zero/color');
  sample(7, null);sample(8, undefined);sample(10, -48.9);sample(11, -48.9);
  line = paths();
  if (line.length !== 8 || line.at(-2)[0] !== 'move') throw Error('null/missing/gap bridged');
  history.breakNext = true; // reconnect cannot bridge the disconnected interval
  sample(12, -48.9);
  points = render().filter(o => o.type === 'fill').flatMap(o => o.path);
  if (!points.some(p => p[2] === 270)) throw Error('reconnect point');
  history.clear();
  if (render().some(o => o.path.length)) throw Error('clear invented posture');
  sample(20, -48.9);
  if (render().filter(o => o.type === 'fill')[0].path[0][2] !== 270) throw Error('post-clear first point');
`, context);
console.log('PASS: fixed ±30 pose coordinates, first point, clipping, reentry, gaps/reconnect/clear, raw history/CSV');
