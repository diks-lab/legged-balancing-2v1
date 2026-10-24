// Production modules: bounded history and segment boundaries, without a browser.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = name => fs.readFileSync(`data/${name}`, 'utf8');
const assets = ['index.html', 'style.css', 'panels.css', 'signals.js', 'history.js', 'plot.js', 'subscribe.js', 'diagnosis.js', 'leg-reply.js', 'leg-state.js', 'leg-config.js', 'legs.js', 'status.js', 'dashboard.js', 'app.js'];
const readyFile = process.env.DRIVE_LEG_READY_OUTPUT;
const legReadyJson = readyFile ? fs.readFileSync(readyFile, 'utf8') : JSON.stringify({
  leg_ms: 100, leg_mode: 1, leg_roll_confirmed: false, leg_roll_sign: 1,
  leg_state: [1, 100, 1, 1, 0, 255, 0, 0, 3, 0, 549, 255, 0, 0, 0, 2, []],
  legs: [0, 0, 0, 0, null, null, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 1, 0, 0]
});
const legRxJson = process.env.LEG_RX_JSON_OUTPUT ? fs.readFileSync(process.env.LEG_RX_JSON_OUTPUT, 'utf8') : null;
const legPositionJson = process.env.LEG_POSITION_JSON_OUTPUT ? fs.readFileSync(process.env.LEG_POSITION_JSON_OUTPUT, 'utf8') : null;
const legFlowJson = process.env.DRIVE_LEG_FLOW_OUTPUT ? fs.readFileSync(process.env.DRIVE_LEG_FLOW_OUTPUT, 'utf8') : null;
const context = vm.createContext({ legReadyJson, legRxJson, legPositionJson, legFlowJson });
for (const name of ['signals.js', 'history.js', 'plot.js', 'diagnosis.js', 'leg-reply.js', 'leg-state.js', 'leg-config.js', 'legs.js', 'status.js', 'dashboard.js']) {
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
  h.push({ sample_ms: 200000, seq: 4000, subscription_id: 1, boot_id: 'a',
    dropped: 0, values: { gyro_deg_s: 1 }, gyro_window: [3, 200000, 200000, 101, 0, 2000, 2000, 1.25, 4.5] });
  if (!h.csv().includes('gyro_rms_deg_s') || !h.csv().includes('"1.25"')) throw Error('window CSV');
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
  for (const [v, text] of [[-0.01, '0.0'], [-0, '0.0'], [-48.94, '-48.9'], [42.25, '42.3']]) {
    if (formatAttitude(v) !== text) throw Error('attitude presentation');
  }
  if (legFlowJson) {
    const flow = JSON.parse(legFlowJson);
    flow.legs[10] = 17; flow.leg_mode = 0; // contradictory legacy fields must not drive the new UI
    const view = legPresentation(flow, true, 0, {});
    if (view.values.available !== '上下操作可能' ||
        view.values.blocker !== 'なし' || !view.values.failure.includes('操作範囲外')) throw Error('real failed operation/release is separate');
  }
  const confirmed = [1, 100, 1, 1, 0, 255, 0, 0, 3, 0, 549, 255, 0, 0, 0, 2, []];
  const legPacket = { leg_ms: 1001, leg_age_ms: 0, leg_state: confirmed,
    leg_mode: 0, leg_roll_confirmed: false, leg_roll_sign: 1,
    legs: [15, 2, 1, 0, 16, 14, 2034, 2055, 90, 50, 17, 3, 1, 1, 1, 1, 4, 0, 27] };
  const ready = JSON.parse(legReadyJson);
  const readyView = legPresentation(ready, true, 0, {});
  if (readyView.values.mode !== '支持試験（倒立OFF）' ||
      readyView.values.available !== '上下操作可能' ||
      readyView.values.sync !== '実位置確認前' || readyView.values.blocker !== 'なし' ||
      readyView.values.height !== '指令未送信') throw Error('real Share snapshot consistency');
  const legCache = {};
  const legState = legPresentation(legPacket, true, 0, legCache);
  if (legState.values.mode !== '支持試験（倒立OFF）' || legState.values.available !== '上下操作可能' ||
      legState.values.blocker !== 'なし') throw Error('confirmed tuple beats stale raw fields');
  const progress = { ...legPacket, leg_state: [...confirmed], legs: [...legPacket.legs] };
  progress.leg_state[4] = 1;progress.leg_state[6] = progress.leg_state[7] = 1;
  if (!legPresentation(progress, true, 0, legCache).values.operation.includes('伸長')) throw Error('held input operation');
  for (const legacyTime of [0, 1000, 1002, 0xffffffff]) {
    progress.leg_ms = legacyTime;
    if (legPresentation(progress, true, 0, legCache).values.available !== '上下操作可能') throw Error('separate stream timing');
  }
  progress.leg_state[3] = 0;progress.leg_state[4] = 7;
  progress.leg_state[5] = progress.leg_state[11] = 8;progress.leg_state[12] = 200;
  if (!legPresentation(progress, true, 0, legCache).values.blocker.includes('応答なし')) throw Error('immediate failure');
  progress.leg_state[3] = 1;progress.leg_state[4] = 0;progress.leg_state[5] = 255;
  const afterFailure = legPresentation(progress, true, 0, legCache);
  if (afterFailure.values.blocker !== 'なし' || !afterFailure.values.failure.includes('応答なし')) throw Error('firmware failure separate');
  progress.leg_state[2] = progress.leg_state[3] = 0;progress.leg_state[4] = 6;progress.leg_state[5] = 17;
  if (!legPresentation(progress, true, 0, legCache).values.blocker.includes('Share')) throw Error('explicit stop immediate');
  if (!legPresentation(progress, false, 0, legCache).note.includes('最終受信時')) throw Error('leg disconnect');
  if (legPresentation(progress, true, 1501, legCache).fresh) throw Error('arrival stale');
  progress.leg_age_ms = 1600;
  if (legPresentation(progress, true, 0, legCache).fresh) throw Error('cached server snapshot stale');
  if (legPresentation(null, false, 0, {}).values.mode !== '未取得') throw Error('no packet');
  if (legRxJson) {
    const fromBus = legReplyPresentation(JSON.parse(legRxJson).leg_rx);
    if (fromBus.summary !== '左：ID' || !fromBus.fields.id.includes('期待 0x02 ／ 取得 0x01')) {
      throw Error('production receiver -> snapshot -> JSON -> Web mismatch');
    }
  }
  if (legPositionJson) {
    const position = legPositionPresentation(JSON.parse(legPositionJson).leg_position);
    if (!position.right.includes('2009 tick') || !position.right.includes('D9 07') ||
        !position.right.includes('30.769230%') || !position.right.includes('2010～2061') ||
        !position.right.includes('操作範囲外・駆動拒否') || !position.left.includes('操作範囲内')) {
      throw Error('real position response -> snapshot -> JSON -> UI');
    }
  }
  const positionData = [100, 0, 3, 1,
    [101, 0, 0, 0, 0, 1219.5266, 2010, 2061, false, 2061, 1892],
    [50, 2053, 5, 8, 2053, 14.835165, 2026, 2081, true, 2026, 2208]];
  const positionView = legPositionPresentation(positionData);
  if (!positionView.right.includes('実位置 0 tick') || positionView.right.includes('未取得') ||
      !positionView.left.includes('前回取得・今回未取得')) throw Error('zero/old position distinct');
  positionData[2] = 0;
  if (!legPositionPresentation(positionData).right.includes('未取得')) throw Error('not acquired position');
  const cfgPacket = {
    leg_accept: [0, 1, 2010, 2061, 2026, 2081, 0, 0],
    leg_config: [1000, 1151, 0, 2, 0, 4, 15, 15, 0,
      [1001, 1051, 0, 4095, 0], [1101, 1151, 0, 4095, 1]]
  };
  if (!legAcceptanceLines(cfgPacket).join(' ').includes('無効（既定）')) throw Error('extra default disabled');
  const configText = legConfigurationLines(cfgPacket).join(' ');
  if (!configText.includes('0～4095') || !configText.includes('0（位置モード）') ||
      !configText.includes('1（車輪モード）') || !configText.includes('自動適用しません')) throw Error('mode/angle readout');
  cfgPacket.leg_config[7] = 7;
  if (!legConfigurationLines(cfgPacket).join(' ').includes('前回取得・今回未取得')) throw Error('old mode read separated');
  cfgPacket.leg_accept = [1, 1, 1950, 2100, 2000, 2110, 2, 2];
  if (!legAcceptanceLines(cfgPacket).join(' ').includes('↑×・↓○')) throw Error('return directions');
  const rxFailure = [53587, 8, 2, 56, 2, 8, 26, 8, 171, 0, 0, 0,
    'FFFF010420F407DD', 'FFFF010420F407DD', 221, 1];
  const rxView = legReplyPresentation(rxFailure);
  if (!rxView.summary.includes('左：ID・状態・チェックサム') ||
      !rxView.fields.id.includes('期待 0x02 ／ 取得 0x01') ||
      !rxView.fields.status.includes('期待 0x00 ／ 取得 0x20') ||
      !rxView.fields.checksum.includes('0xAB ／ 取得 0xDD')) throw Error('rx exact mismatches');
  const partialRx = [...rxFailure]; partialRx[6] = 32; partialRx[7] = 3;
  partialRx[13] = 'FFFF01'; partialRx[11] = 1; partialRx[15] = 0;
  const partialView = legReplyPresentation(partialRx);
  if (!partialView.fields.status.includes('未受信') ||
      !partialView.fields.checksum.includes('未算出') || !partialView.fields.raw.includes('記録打切り')) throw Error('rx missing/truncation');
  const oldRxPacket = { ...legPacket, leg_rx: rxFailure };
  const oldRxState = legPresentation(oldRxPacket, true, 120, {});
  if (oldRxState.values.failure.includes('左：ID') || oldRxState.values.blocker !== 'なし') throw Error('rx retained as past');
  const wheelFirst = [53587, 5100, 53000, 0, 0, 0, -1, -1, 12, 2, 69, 3];
  const wheelProbe = [55000, 100, 53000, 0, 2, 2, 2, 3, 12, 2, 4, 1];
  const wheelSample = { diagnosis: [100, 'RIGHT_SENSOR', 2, 3, 2, 2, 2, 0, 0, 0, 1, 26, 3, 3, 0, 1, 3, 0], sample_ms: 55020, wheel0: [...wheelFirst, ...wheelProbe, 1, 1] };
  const wheelLines = healthLines(wheelSample).join(' / ');
  if (!wheelLines.includes('現在の読取り正常') || !wheelLines.includes('53587 ms') ||
      !wheelLines.includes('転倒停止=53000') || !wheelLines.includes('要求=2 受信=0')) throw Error('wheel detail');
  if (healthLines(wheelSample, false).join(' / ').includes('現在の読取り正常')) throw Error('offline wheel state');
  wheelSample.wheel0[23] = 3;wheelSample.wheel0[24] = 0;
  if (!healthLines(wheelSample).join(' / ').includes('現在の読取り失敗')) throw Error('probe failure');
  const output = value => ({ type: 'sample', age_ms: 0, disabled: false,
    command_valid: true, values: { left_command_v: value } });
  for (const v of [0, 0.35, -0.2, 1, -1, 1.5, -1.5]) {
    const raw = output(v);
    raw.values.right_command_v = v;
    const r = outputReading(raw, 'left_command_v', true);
    if (r.value !== -v || raw.values.left_command_v !== v ||
      outputReading(raw, 'right_command_v', true).value !== v) throw Error('presentation only');
    if (r.arc !== Math.min(Math.abs(v), 1) * Math.PI || r.ccw !== (-v < 0) ||
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
  const failure = { diagnosis: [100, 'IMU_CONFIG_FAILED', 3, 2, 2, 4, 4, 5, 0, 0, 2, 26, 3, -1, 2, -1, -1, 2],
    imu_reason: 'NOT_INITIALIZED', right_error: 'OK', left_error: 'OK' };
  let lines = healthLines(failure).join(' / ');
  if (!lines.includes('右FOC 未実行：IMU設定確認失敗のため') ||
      lines.includes('右FOC 実行して失敗') || !lines.includes('期待=0x03 取得=未取得')) throw Error('blocked/config');
  for (const [code, name] of MPU_FAILURES.entries()) {
    const d = [...failure.diagnosis]; d[10] = code;
    if (!healthLines({ ...failure, diagnosis: d }).join(' / ').includes(name)) throw Error('MPU category');
  }
  const unrun = [...failure.diagnosis]; unrun[2] = 0;
  if (!healthLines({ ...failure, diagnosis: unrun }).includes('IMU 未実行')) throw Error('not run');
  if (!diagnosisName('RIGHT_FOC_STATE').includes('readyでない') || !FOC_FAILURES[10].includes('motor_status')) throw Error('FOC state detail');
  const failedFoc = [...failure.diagnosis]; failedFoc[5] = 3; failedFoc[8] = 5;
  if (!healthLines({ ...failure, diagnosis: failedFoc }).join(' / ').includes('右FOC 実行して失敗（タイムアウト）')) throw Error('real FOC failure');
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
