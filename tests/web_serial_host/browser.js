// Real Chromium with production files and a deterministic WebSocket peer.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const assets = ['index.html', 'style.css', 'panels.css', 'signals.js', 'history.js',
  'plot.js', 'subscribe.js', 'diagnosis.js', 'leg-reply.js', 'leg-state.js', 'leg-config.js', 'leg-measure.js', 'leg-calibration.js', 'legs.js', 'status.js', 'dashboard.js', 'app.js'];
(async () => {
  const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH || '/usr/bin/chromium',
    args: ['--no-sandbox'] });
  try {
    for (const size of [{ width: 1366, height: 768 }, { width: 1920, height: 1080 }]) {
      const page = await browser.newPage({ viewport: size, deviceScaleFactor: 1 });
      const requests = [], errors = [], calibrationRequests = [], legReadRequests = [], legMeasureRequests = [], legCalRequests = [];
      page.on('pageerror', e => errors.push(e.message));
      await page.route('http://127.0.0.1:8765/**', async route => {
        const file = new URL(route.request().url()).pathname.slice(1) || 'index.html';
        if (file.startsWith('legs/calibration/')) {
          legCalRequests.push([file.split('/').at(-1), route.request().method()]);
          await route.fulfill({ status: 202, body: 'queued four-point calibration' });return;
        }
        if (file.startsWith('legs/measure/')) {
          legMeasureRequests.push([file, route.request().method()]);
          await route.fulfill({ status: 202, body: 'queued read-only measurement' }); return;
        }
        if (file === 'legs/read-config') {
          legReadRequests.push(route.request().method());
          await route.fulfill({ status: 202, body: 'queued read-only' }); return;
        }
        if (file.startsWith('calibration/')) {
          calibrationRequests.push([file, route.request().method()]);
          await route.fulfill({ status: 202, body: 'queued' }); return;
        }
        requests.push(file);
        await route.fulfill({ body: fs.readFileSync(`data/${file}`),
          contentType: file.endsWith('.html') ? 'text/html' : 'text/plain' });
      });
      await page.addInitScript(() => {
        window.sent = []; window.peers = 0; window.sequence = 0; window.autoAck = true;
        window.axisLabels = []; window.arcCalls = [];
        const fillText = CanvasRenderingContext2D.prototype.fillText;
        CanvasRenderingContext2D.prototype.fillText = function (text, ...args) {
          window.axisLabels.push({ id: this.canvas.id, text }); return fillText.call(this, text, ...args);
        };
        const arc = CanvasRenderingContext2D.prototype.arc;
        CanvasRenderingContext2D.prototype.arc = function (...args) {
          window.arcCalls.push({ id: this.canvas.id, args }); return arc.apply(this, args);
        };
        window.reply = packet => window.peer.onmessage({ data: JSON.stringify({
          type: 'subscribed', request_id: packet.request_id, subscription_id: packet.request_id,
          period_ms: 50, fields: packet.fields || [], streaming: packet.type !== 'stop' }) });
        window.WebSocket = class {
          constructor() { window.peers++; window.peer = this; this.readyState = 0;
            setTimeout(() => { this.readyState = 1; this.onopen(); }, 0); }
          send(text) { const packet = JSON.parse(text); window.sent.push(packet);
            if (window.autoAck) window.reply(packet); }
          close() { this.readyState = 3; this.onclose({ code: 1006 }); }
        };
        window.legState = (reason, mode = 1, rx = null) => {
          if ([7, 8, 9, 10, 11, 14].includes(reason)) window.legFailure = [reason, rx?.[0] || window.sequence * 50];
          const failure = window.legFailure || [255, 0];
          const allowed = reason === 1 || reason === 3 || reason === 14;
          const operation = reason === 2 ? 5 : reason === 3 ? 1 : reason === 17 ? 6 :
            [7, 8, 9, 10, 11].includes(reason) ? 7 : 0;
          return [window.sequence, window.sequence * 50, mode, Number(allowed), operation,
            [1, 2, 3].includes(reason) ? 255 : reason, reason === 3 ? 1 : 0, reason === 3 ? 1 : 0,
            reason === 3 ? 15 : 3, reason === 2 ? 2 : reason === 3 ? 8 : 0,
            549, failure[0], failure[1], 3, 0, 2, []];
        };
        window.sample = (extra = {}) => {
          const seq = ++window.sequence, request = window.sent.at(-1);
          const values = Object.fromEntries((request.fields || []).map(key => [key,
            key === 'angle_deg' ? 8 * Math.sin(seq / 20) : key === 'reference_deg' ? -0.8 : 0.0001]));
          const sample = { type: request.type === 'stop' ? 'status' : 'sample',
            boot_id: 'boot-test', subscription_id: request.request_id, seq, source_seq: seq,
            sample_ms: seq * 50, imu_ms: seq * 50, imu_seq: seq, imu_valid: true,
            imu_reason: 'OK', reference_valid: false, command_valid: true, command_ms: seq * 50,
            age_ms: 0, dropped: 0, recovery_checks: 15, state: 'READY_WAIT', startup_reason: 'READY',
            right_error: '', left_error: '', stop_reason: 'NONE', stop_ms: 0,
            faults: 0, running: false, disabled: false, foc_ready: true, ps4: false, values, ...extra };
          window.peer.onmessage({ data: JSON.stringify(sample) });
        };
      });
      await page.goto('http://127.0.0.1:8765/');
      await page.waitForFunction(() => window.sent.length === 1);
      assert.deepEqual(requests, assets);
      assert.equal(await page.locator('#fields input:checked').count(), 1);
      assert.equal(await page.locator('canvas').count(), 4);
      assert.deepEqual(await page.evaluate(() => window.sent[0].fields),
        ['angle_deg', 'reference_deg', 'right_command_v', 'left_command_v', 'gyro_deg_s']);
      await page.evaluate(() => window.sample({ gyro_window: [3, 50, 200000, 101, 0, 1800, 2200, 1.25, 4.5] }));
      assert((await page.locator('#gyro-window').textContent()).includes('CONFIG=0x03'));
      assert((await page.locator('#gyro-window').textContent()).includes('RMS 1.250'));
      await page.evaluate(() => window.peer.onmessage({ data: JSON.stringify({
        type: 'wheel_diagnostics', boot_id: 'boot-test', leg_ms: window.sequence * 50,
        leg_mode: 1, leg_roll_confirmed: false, leg_roll_sign: 1,
        leg_age_ms: 0, leg_state: window.legState(3),
        legs: [15, 2, 1, 0, 16, 14, 2034, 2055, 40, 30, 3, 3, 1, 1, 1, 1, 4, 0, 27]
      }) }));
      assert((await page.locator('#leg-diagnostics').textContent()).includes('実位置ではありません'));
      assert((await page.locator('#leg-diagnostics').textContent()).includes('応答未確認'));
      assert((await page.locator('#leg-diagnostics').textContent()).includes('可動制限：あり'));
      assert.equal(await page.locator('#leg-mode').textContent(), '支持試験（倒立OFF）');
      await page.evaluate(() => window.peer.onmessage({ data: JSON.stringify({
        type: 'wheel_diagnostics', boot_id: 'boot-test', leg_ms: window.sequence * 50 + 1,
        leg_age_ms: 0, leg_state: window.legState(1), leg_mode: 1,
        leg_roll_confirmed: false, leg_roll_sign: 1,
        legs: [15, 0, 0, 0, null, null, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 1, 0, 0]
      }) }));
      assert.equal(await page.locator('#leg-available').textContent(), '上下操作可能');
      await page.waitForTimeout(120);
      assert.equal(await page.locator('#leg-available').textContent(), '上下操作可能');

      const legResult = await page.evaluate(async () => {
        const section = document.getElementById('leg-diagnostics');
        const rows = [...section.querySelectorAll('.leg-field')];
        const geometry = () => rows.map(e => ({ y: e.getBoundingClientRect().y, h: e.getBoundingClientRect().height }));
        document.getElementById('details').open = true;
        const original = geometry();
        const node = document.getElementById('leg-blocker');
        const sendLeg = (reason, height = 15, mode = 1, rx = null, position = null) => {
          window.sample();
          window.peer.onmessage({ data: JSON.stringify({
            type: 'wheel_diagnostics', boot_id: 'boot-test', leg_ms: window.sequence * 50,
            leg_mode: mode, leg_roll_confirmed: false, leg_roll_sign: 1, leg_rx: rx, leg_position: position,
            leg_age_ms: 0, leg_state: window.legState(reason, mode, rx),
            legs: [height, 0, 0, -48.9, 16, 14, 2034, 2055, 40, 30, reason, 3,
              reason === 1 || reason === 17 ? 0 : 1, [1, 8, 17].includes(reason) ? 0 : 1,
              0, reason === 1 ? 0 : 1, 4, 0, 27]
          }) });
        };
        const states = [];
        for (const [reason, height, mode] of [[1, 15, 1], [3, 16, 1], [2, 16, 1],
          [8, 16, 1], [1, 16, 1], [17, 16, 0], [1, 16, 1]]) {
          sendLeg(reason, height, mode);
          states.push({ reason, height, mode, geometry: geometry(),
            blocker: node.textContent, operation: document.getElementById('leg-operation').textContent,
            failure: document.getElementById('leg-failure').textContent,
            sync: document.getElementById('leg-sync').textContent,
            available: document.getElementById('leg-available').textContent });
        }
        sendLeg(9, 16, 1, [53587, 8, 2, 56, 2, 8, 26, 8, 171, 0, 0, 0,
          'FFFF010420F407DD', 'FFFF010420F407DD', 221, 1]);
        const rxGeometry = geometry();
        const rx = ['target', 'id', 'length', 'status', 'checksum', 'raw'].map(key =>
          document.getElementById(`leg-rx-${key}`).textContent);
        const rxBlocker = node.textContent;
        sendLeg(7, 16, 1, [53587, 8, 2, 56, 2, 8, 26, 8, 171, 0, 0, 0,
          'FFFF010420F407DD', 'FFFF010420F407DD', 221, 1],
          [1000, 0, 3, 3, [1002, 2009, 217, 7, 2009, 30.76923, 2010, 2061, false, 2061, 1892],
            [1052, 2053, 5, 8, 2053, 14.835165, 2026, 2081, true, 2026, 2208]]);
        const positionText = ['right', 'left', 'limits'].map(key => document.getElementById(`leg-position-${key}`).textContent);
        const positionGeometry = geometry();
        // An unchanged status update must not rewrite text, attributes or elements.
        let mutations = 0;
        const observer = new MutationObserver(records => { mutations += records.length; });
        observer.observe(section, { childList: true, subtree: true, characterData: true, attributes: true });
        window.sample();
        await Promise.resolve(); observer.disconnect();
        window.peer.close();
        const stale = document.getElementById('leg-freshness').textContent;
        const sameNode = document.getElementById('leg-blocker') === node;
        const closed = [...section.querySelectorAll('details')].every(e => !e.open);
        const overflow = rows.some(e => e.querySelector('dd').getBoundingClientRect().height > e.clientHeight);
        document.getElementById('details').open = false;
        return { original, states, mutations, stale, sameNode, closed, overflow, rx, rxBlocker, rxGeometry, positionText, positionGeometry };
      });
      // Short states retain minimum row heights. Long reasons may expand rows;
      // preserving readability takes precedence over identical y coordinates.
      for (const state of legResult.states) assert(state.geometry.every(row => row.h >= 0));
      assert(legResult.states[1].operation.includes('伸長'));
      assert(legResult.states[2].operation.includes('位置確認'));
      assert(legResult.states[3].blocker.includes('応答なし'));
      assert.equal(legResult.states[4].blocker, 'なし');
      assert(legResult.states[4].failure.includes('直近操作の記録'));
      assert(legResult.states[5].blocker.includes('Share'));
      assert.equal(legResult.states[6].blocker, 'なし');
      assert.equal(legResult.states[6].available, '上下操作可能');
      assert.equal(legResult.states[6].sync, '実位置確認前');
      assert(legResult.stale.includes('最終受信時'));
      assert(legResult.sameNode && legResult.closed && !legResult.overflow);
      assert.equal(legResult.mutations, 0);
      assert(legResult.rxGeometry.every(row => row.h >= 0));
      assert(legResult.positionGeometry.every(row => row.h >= 0));
      assert(legResult.positionText[0].includes('2009 tick') && legResult.positionText[0].includes('D9 07'));
      assert(legResult.positionText[0].includes('30.769230%') && legResult.positionText[0].includes('2010～2061'));
      assert(legResult.positionText[0].includes('通常操作範囲外（初回受付は別判定）'));
      assert(legResult.positionText[1].includes('操作範囲内'));
      assert(legResult.positionText[2].includes('初回受付・保持範囲と通常駆動'));
      assert(legResult.positionText[2].includes('実測端と余裕を別に設定'));
      assert(legResult.rxBlocker.includes('左：ID・状態・チェックサム'));
      assert(legResult.rx[1].includes('期待 0x02 ／ 取得 0x01'));
      assert(legResult.rx[3].includes('期待 0x00 ／ 取得 0x20'));
      assert(legResult.rx[4].includes('0xAB ／ 取得 0xDD'));
      assert(legResult.rx[5].includes('FF FF 01'));
      await page.click('#reconnect');
      await page.waitForFunction(() => window.peer.readyState === 1);
      await page.evaluate(() => window.sample());


      await page.evaluate(() => {
        window.peer.onmessage({ data: JSON.stringify({ type: 'wheel_diagnostics', boot_id: 'boot-test',
          leg_ms: window.sequence * 50, leg_age_ms: 0, leg_state: window.legState(1),
          leg_roll_confirmed: false, leg_roll_sign: 1,
          leg_accept: [0, 1, 1908, 1997, 2081, 2173, 0, 0],
          leg_geometry: [[2007, 1675, 10, 10, 1685, 1997, 1908, 1997], [2071, 2412, 10, 10, 2081, 2402, 2081, 2173], [0, 30]],
          leg_config: [1000, 1151, 0, 2, 0, 4, 15, 15, 0,
            [1001, 1051, 0, 4095, 0], [1101, 1151, 0, 4095, 1]],
          legs: [0, 0, 0, 0, null, null, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 1, 0, 0]
        }) });
        document.getElementById('details').open = true;
        document.getElementById('leg-config').open = true;
      });
      await page.evaluate(() => window.peer.onmessage({ data: JSON.stringify({ type: 'wheel_diagnostics', boot_id: 'boot-test',
        leg_ms: window.sequence * 50, leg_age_ms: 0, leg_state: window.legState(1),
        leg_stick: [100, -128, -72, 0, 0, .5, 1, .5, 0, 0, 0, 97],
        leg_accept: [0, 1, 1908, 1997, 2081, 2173, 0, 0],
        leg_geometry: [[2007, 1675, 10, 10, 1685, 1997, 1908, 1997], [2071, 2412, 10, 10, 2081, 2402, 2081, 2173], [0, 30]],
        leg_config: [1000, 1151, 0, 2, 0, 4, 15, 15, 0, [1001, 1051, 0, 4095, 0], [1101, 1151, 0, 4095, 1]],
        legs: [0, 0, 0, 0, null, null, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 1, 0, 0]
      }) }));
      await page.waitForFunction(() => document.getElementById('leg-stick-0').textContent.includes('LX=-128 LY=-72 RX=0 RY=0'));
      assert((await page.locator('#leg-stick-2').textContent()).includes('上下 0.500 ／ 左右 0.000'));
      await page.click('#leg-read-config');
      await page.waitForFunction(() => document.getElementById('leg-config-request').textContent.includes('読取要求を送信'));
      assert.deepEqual(legReadRequests, ['POST']);
      const configText = await page.locator('#leg-config-values').textContent();
      assert(configText.includes('初回受付：通常範囲のみ') && configText.includes('0～4095'));
      assert(configText.includes('0（位置モード）') && configText.includes('1（車輪モード）'));
      await page.evaluate(() => {
        document.getElementById('leg-config').open = false;
        document.getElementById('details').open = false;
      });

      await page.evaluate(() => {
        document.getElementById('details').open = true;
        document.getElementById('leg-measure').open = true;
      });
      await page.click('#leg-measure-start');
      assert.equal(legMeasureRequests.length, 0);
      assert((await page.locator('#leg-measure-action').textContent()).includes('開始しません'));
      await page.check('#leg-measure-supported');await page.click('#leg-measure-start');
      await page.waitForFunction(() => document.getElementById('leg-measure-action').textContent.includes('開始要求を送信'));
      await page.evaluate(() => window.peer.onmessage({ data: JSON.stringify({ type: 'wheel_diagnostics', boot_id: 'boot-test',
        leg_ms: window.sequence * 50, leg_age_ms: 0, leg_state: window.legState(1),
        leg_measure: [100, 0, 0, 3, 1, 3, 3, 0, 0, 0,
          [200, 2, 1900, 1890, 1977, 32.2289, 35.24096, 9.03614], [250, 2, 2200, 2082, 2200, 37.8299, 3.2258, 37.8299]],
        legs: [0, 0, 0, 0, null, null, 0, 0, 0, 0, 1, 2, 0, 0, 0, 0, 1, 0, 0]
      }) }));
      await page.waitForFunction(() => document.getElementById('leg-measure-values').textContent.includes('最小 1890・最大 1977'));
      assert((await page.locator('#leg-measure-values').textContent()).includes('換算：現在 32.23%'));
      await page.click('#leg-measure-stop');
      await page.waitForFunction(() => document.getElementById('leg-measure-action').textContent.includes('終了要求を送信'));
      assert.deepEqual(legMeasureRequests, [['legs/measure/start', 'POST'], ['legs/measure/stop', 'POST']]);
      await page.evaluate(() => {
        document.getElementById('leg-measure').open = false;
        document.getElementById('details').open = false;
      });

      await page.evaluate(() => window.sample({ gyro_window: [0, (window.sequence + 1) * 50, 200000, 101, 0, 1800, 2200, 2.5, 8.5] }));
      assert((await page.locator('#gyro-window').textContent()).includes('CONFIG=0x00'));
      await page.evaluate(() => window.sample({ gyro_window: [3, (window.sequence + 1) * 50, 200000, 100, 1, 1800, 2200, null, null] }));
      assert((await page.locator('#gyro-window').textContent()).includes('無効'));
      await page.evaluate(() => window.sample({ gyro_window: [3, (window.sequence + 1) * 50, 200000, 101, 0, 1800, 2200, 1, 2], imu_valid: false }));
      assert((await page.locator('#gyro-window').textContent()).includes('更新停止'));
      await page.evaluate(() => window.sample({ state: 'STARTUP_FAILED', imu_valid: false,
        imu_reason: 'NOT_INITIALIZED', startup_reason: 'IMU_CONFIG_FAILED', disabled: true,
        diagnosis: [100, 'IMU_CONFIG_FAILED', 3, 2, 2, 4, 4, 5, 0, 0, 4, 26, 3, -1, 0, 0, -1, 2] }));
      assert((await page.locator('#health-diagnosis').textContent()).includes('右FOC 未実行：IMU設定確認失敗のため'));
      assert((await page.locator('#health-diagnosis').textContent()).includes('受信長不足'));
      await page.evaluate(() => window.sample({ state: 'STARTUP_FAILED', imu_valid: false,
        imu_reason: 'LOOP_STALE', startup_reason: 'IMU_CONFIG_FAILED', disabled: true,
        diagnosis: [100, 'IMU_CONFIG_FAILED', 3, 2, 2, 4, 4, 5, 0, 0, 4, 26, 3, -1, 0, 0, -1, 2] }));
      assert((await page.locator('#health-diagnosis p').nth(1).textContent()).includes('IMU設定確認失敗 @ 100 ms'));
      const initialized = [0, '', 2, 2, 2, 2, 2, 0, 0, 0, 1, 28, 0, 0, 0, 1, 0, 0];
      await page.evaluate(diagnosis => window.sample({ state: 'FALL_RECOVERY_WAIT',
        recovery: 'UPRIGHT_REQUIRED', recovery_checks: 14, disabled: true,
        stop_reason: 'TILT_STOP', imu_valid: true, imu_reason: 'OK', diagnosis,
        values: { angle_deg: 15, reference_deg: 0, left_command_v: 0, right_command_v: 0 } }), initialized);
      for (const key of ['imu', 'wheels', 'foc']) {
        assert.equal(await page.locator(`#condition-${key}`).getAttribute('data-ok'), 'true');
      }
      assert.equal(await page.locator('#left-reading').textContent(), '遮断');
      await page.evaluate(diagnosis => window.sample({ state: 'BALANCING', recovery: 'RECOVERED',
        recovery_checks: 15, disabled: false, imu_valid: true, imu_reason: 'OK', diagnosis,
        values: { angle_deg: 1.9, reference_deg: 0, left_command_v: 0.1, right_command_v: -0.1 } }), initialized);
      assert.equal(await page.locator('#health').textContent(), '倒立中');
      const wheelFault = [...initialized]; wheelFault[0] = 200; wheelFault[1] = 'RIGHT_SENSOR'; wheelFault[3] = 3;
      await page.evaluate(diagnosis => window.sample({ state: 'FAULT', recovery_checks: 11,
        disabled: true, faults: 1, imu_valid: true, imu_reason: 'OK', right_error: 'I2C_ADDRESS/NACK', diagnosis }), wheelFault);
      assert.equal(await page.locator('#condition-wheels').getAttribute('data-ok'), 'false');
      for (const key of ['imu', 'foc']) assert.equal(await page.locator(`#condition-${key}`).getAttribute('data-ok'), 'true');
      assert((await page.locator('#health-diagnosis').textContent()).includes('右車輪センサー 実行して失敗：I2C_ADDRESS/NACK'));
      await page.evaluate(() => window.peer.onmessage({ data: JSON.stringify({
        type: 'wheel_diagnostics', boot_id: 'boot-test',
        wheel0: [53587, 5100, 53000, 0, 0, 0, -1, -1, 12, 2, 69, 3,
          100, 100, 53000, 0, 2, 2, 1, 2, 12, 2, 4, 1, 1, 1]
      }) }));
      await page.evaluate(() => window.sample({ sample_ms: 120, imu_ms: 120, command_ms: 120 }));
      assert((await page.locator('#health-diagnosis').textContent()).includes('現在の読取り正常'), await page.locator('#health-diagnosis').textContent());
      assert((await page.locator('#health-diagnosis').textContent()).includes('53587 ms'));
      await page.evaluate(() => window.sample({ sample_ms: 140, imu_ms: 140, command_ms: 140, faults: 1 }));
      assert((await page.locator('#health-diagnosis').textContent()).includes('現在の読取り失敗／次周期再試行'));
      await page.evaluate(() => window.sample({ sample_ms: 160, imu_ms: 160, command_ms: 160, faults: 0 }));
      assert((await page.locator('#health-diagnosis').textContent()).includes('現在の読取り正常'));
      assert((await page.locator('#health-diagnosis').textContent()).includes('失敗回数1'));
      await page.evaluate(() => { for (let i = 0; i < 1200; i++) window.sample(); });
      await page.click('#pause'); await page.click('#pause');
      assert(await page.evaluate(() => window.axisLabels.some(x => x.id === 'pose' && x.text === '30.0')));
      assert(await page.evaluate(() => window.axisLabels.some(x => x.id === 'pose' && x.text === '-30.0')));
      await page.evaluate(() => window.sample({ values: { angle_deg: 42.25, reference_deg: -0.8,
        right_command_v: 0.35, left_command_v: -0.2 } }));
      assert((await page.locator('#pose-value').textContent()).includes('▲ 上限超過 42.3'));
      assert.equal(await page.locator('#left-reading').textContent(), '+0.20 V');
      assert.equal(await page.locator('#right-reading').textContent(), '+0.35 V');
      const arcs = await page.evaluate(() => window.arcCalls);
      assert(arcs.some(x => x.id === 'left-meter' && x.args[5] === false));
      assert(arcs.some(x => x.id === 'right-meter' && x.args[5] === false));
      for (const angle of [-0.01, -48.94]) {
        await page.evaluate(angle => window.sample({ values: { angle_deg: angle,
          reference_deg: 0, left_command_v: -0.2, right_command_v: 0.2 } }), angle);
        assert((await page.locator('#pose-value').textContent()).includes(angle < -30 ? '▼ 下限超過 -48.9°' : '姿勢角 0.0°'));
        assert(!(await page.locator('#condition-upright').textContent()).includes('-0.0'));
        assert((await page.locator('#measurement').textContent()).includes(String(angle)));
      }
      for (const v of [0, 1, -1, 1.5, -1.5]) {
        await page.evaluate(v => window.sample({ values: { angle_deg: -42, reference_deg: -0.8,
          left_command_v: v, right_command_v: v } }), v);
        assert((await page.locator('#pose-value').textContent()).includes('▼ 下限超過'));
        assert.equal(await page.locator('#left-reading').getAttribute('data-state'),
          v === 0 ? 'zero' : v > 0 ? 'negative' : 'positive');
        assert.equal(await page.locator('#right-reading').getAttribute('data-state'),
          v === 0 ? 'zero' : v > 0 ? 'positive' : 'negative');
        if (Math.abs(v) > 1) assert((await page.locator('#left-limit').textContent()).includes('超過'));
      }
      await page.evaluate(() => window.sample({ disabled: true }));
      assert.equal(await page.locator('#left-reading').textContent(), '遮断');
      await page.evaluate(() => window.sample({ values: { angle_deg: null, reference_deg: -0.8,
        left_command_v: null, right_command_v: null }, imu_valid: false, imu_reason: 'RIGHT_WHEEL_FAULT',
        state: 'FAULT', faults: 1, right_error: 'I2C_NACK' }));
      assert.equal(await page.locator('#right-reading').textContent(), '更新停止／不明');
      assert((await page.locator('#measurement').textContent()).includes('右車輪異常'));
      assert((await page.locator('#recovery-details').textContent()).includes('I2C_NACK'));
      await page.evaluate(() => window.sample({ imu_valid: false, imu_reason: 'CALIBRATION_FAILED', state: 'FAULT' }));
      assert((await page.locator('#measurement').textContent()).includes('校正'));
      await page.evaluate(() => window.sample({ imu_valid: false, imu_reason: 'IMU_I2C_FAILED', imu_tx: 2, imu_rx: 0, state: 'FAULT' }));
      assert((await page.locator('#measurement').textContent()).includes('tx=2 rx=0/14'));
      await page.evaluate(() => window.sample({ state: 'AUTO_WAIT', disabled: true, recovery_checks: 14,
        reference_valid: true, values: { angle_deg: 15, reference_deg: 0, left_command_v: 0, right_command_v: 0 } }));
      assert((await page.locator('#health').textContent()).includes('起立待ち'));
      assert((await page.locator('#pose-value').textContent()).includes('15.0°'));
      assert.equal(await page.locator('#right-reading').textContent(), '遮断');
      assert.equal(await page.locator('#condition-upright').getAttribute('data-ok'), 'false');
      // Fixed attitude scale survives auxiliary autoscale.
      await page.check('#autoscale');
      await page.click('#pause'); await page.click('#pause');
      await page.uncheck('#autoscale');
      for (const preset of ['stationary', 'vibration', 'control', 'position', 'drive']) {
        const count = await page.evaluate(() => window.sent.length);
        await page.locator('#selection summary').click(); await page.selectOption('#preset', preset);
        await page.locator('#selection summary').click();
        await page.waitForFunction(n => window.sent.length > n, count);
        await page.evaluate(() => window.sample());
        const layout = await page.evaluate(() => ({ w: document.documentElement.scrollWidth,
          h: document.documentElement.scrollHeight,
          boxes: ['pose', 'left-meter', 'right-meter', 'plot', 'legend', 'start', 'details'].map(id => {
            const r = document.getElementById(id).getBoundingClientRect();
            return { id, x: r.x, y: r.y, right: r.right, bottom: r.bottom };
          }) }));
        assert(layout.w <= size.width && layout.h <= size.height, JSON.stringify(layout));
        for (const box of layout.boxes) assert(box.x >= 0 && box.y >= 0 && box.right <= size.width && box.bottom <= size.height,
          JSON.stringify(box));
        console.log('LAYOUT', size.width, size.height, preset, JSON.stringify(layout.boxes));
      }
      assert((await page.locator('#left-meter').boundingBox()).x < (await page.locator('#right-meter').boundingBox()).x);
      const peers = await page.evaluate(() => window.peers);
      await page.locator('#legend button').first().click();
      assert.equal(await page.locator('#legend button').first().getAttribute('aria-pressed'), 'false');
      assert.equal(await page.evaluate(() => window.peers), peers);
      await page.click('#pause');
      const frozen = await page.locator('#pose').evaluate(c => c.toDataURL());
      await page.evaluate(() => window.sample({ state: 'FALL_STOP', stop_reason: 'TILT_STOP',
        values: { angle_deg: 12, reference_deg: -0.8, left_command_v: 0.3, right_command_v: -0.4 } }));
      assert.equal(await page.locator('#pose').evaluate(c => c.toDataURL()), frozen);
      assert((await page.locator('#health').textContent()).includes('転倒停止'));
      assert.equal(await page.locator('#left-reading').textContent(), '−0.30 V');
      await page.click('#pause');
      // All optional fields off leaves permanent attitude/output streaming.
      const beforeClear = await page.evaluate(() => {
        for (const input of document.querySelectorAll('#fields input')) input.checked = false;
        document.getElementById('fields').dispatchEvent(new Event('change', { bubbles: true }));
        return window.sent.length;
      });
      await page.waitForFunction(n => window.sent.length > n, beforeClear);
      assert.equal(await page.evaluate(() => window.sent.at(-1).fields.length), 4);
      assert.equal(await page.evaluate(() => window.sent.at(-1).type), 'subscribe');
      await page.click('#stop');
      assert.equal(await page.locator('#left-reading').textContent(), '更新停止／不明');
      const stopped = await page.evaluate(() => window.sent.length);
      await page.locator('#selection summary').click(); await page.selectOption('#preset', 'drive');
      await page.locator('#selection summary').click(); await page.waitForTimeout(250);
      assert.equal(await page.evaluate(() => window.sent.length), stopped);
      await page.click('#start'); await page.evaluate(() => window.sample());
      await page.evaluate(() => window.peer.close());
      assert((await page.locator('#health').textContent()).includes('通信停止・最終受信時の情報'));
      assert((await page.locator('#health-diagnosis').textContent()).includes('現在状態は不明'));
      assert.equal(await page.locator('#right-reading').textContent(), '更新停止／不明');
      await page.click('#reconnect'); await page.waitForFunction(() => window.peer.readyState === 1);
      await page.evaluate(() => window.sample());
      await page.waitForFunction(() => document.getElementById('right-reading').textContent === '更新停止／不明');
      await page.locator('#aux-toggle').click();
      assert(await page.evaluate(() => document.documentElement.scrollHeight <= innerHeight));
      await page.locator('#aux-toggle').click();
      // All optional signals must still fit without hiding instruments or controls.
      const beforeAll = await page.evaluate(() => {
        for (const input of document.querySelectorAll('#fields input')) input.checked = true;
        document.getElementById('fields').dispatchEvent(new Event('change', { bubbles: true }));
        return window.sent.length;
      });
      await page.waitForFunction(n => window.sent.length > n, beforeAll);
      await page.evaluate(() => window.sample());
      assert(await page.evaluate(() => document.documentElement.scrollHeight <= innerHeight && document.documentElement.scrollWidth <= innerWidth));
      // Numbers/history are in the closed details, current recovery/calibration stay on top.
      await page.evaluate(() => window.sample({ state: 'CALIBRATION_REQUIRED', calibration: 'MISSING_INVALID_CONFIG' }));
      assert((await page.locator('#health').textContent()).includes('要校正'));
      assert((await page.locator('#calibration').textContent()).includes('設定不一致'));
      await page.evaluate(() => window.sample({ state: 'FALL_RECOVERY_WAIT', recovery: 'UPRIGHT_REQUIRED', calibration: 'RESTORED', stop_reason: 'TILT_STOP' }));
      assert((await page.locator('#health').textContent()).includes('転倒復帰待ち'));
      assert((await page.locator('#recovery-details').textContent()).includes('進入待ち'));
      await page.evaluate(() => window.sample({ state: 'BALANCING', running: true, recovery: 'RECOVERED', calibration: 'RESTORED', stop_reason: 'TILT_STOP' }));
      assert((await page.locator('#recovery-details').textContent()).includes('復帰完了'));
      assert(!(await page.locator('#health').textContent()).includes('転倒停止'));
      const labels = ['直立範囲', 'IMU', '車輪センサー', 'FOC'];
      assert.deepEqual(await page.locator('#recovery-checks strong').allTextContents(), labels);
      await page.evaluate(() => window.sample({ state: 'FALL_RECOVERY_WAIT', recovery_checks: 14,
        values: { angle_deg: 2.01, reference_deg: 0 } }));
      assert.equal(await page.locator('#condition-upright').getAttribute('data-ok'), 'false');
      assert((await page.locator('#condition-upright').textContent()).includes('× 2.0° / ±2°'));
      for (const key of ['imu', 'wheels', 'foc']) {
        assert.equal(await page.locator(`#condition-${key}`).getAttribute('data-ok'), 'true');
        assert((await page.locator(`#condition-${key}`).textContent()).includes('○'));
      }
      await page.evaluate(() => window.sample({ state: 'BALANCING', recovery_checks: 15,
        values: { angle_deg: -1.9, reference_deg: 0 } }));
      assert.equal(await page.locator('#condition-upright').getAttribute('data-ok'), 'true');
      await page.evaluate(() => window.sample({ state: 'EXPLICIT_STOP', recovery_checks: 15 }));
      assert.equal(await page.locator('#health').textContent(), '明示停止');
      await page.evaluate(() => window.sample({ state: 'FAULT', imu_valid: false,
        imu_reason: 'IMU_I2C_FAILED', recovery_checks: 4 }));
      assert.equal(await page.locator('#condition-imu').getAttribute('data-ok'), 'false');
      assert.equal(await page.locator('#condition-foc').getAttribute('data-ok'), 'false');
      assert.deepEqual(await page.locator('#recovery-checks strong').allTextContents(), labels);
      await page.waitForFunction(() => document.getElementById('condition-imu').dataset.ok === 'unknown');
      assert((await page.locator('#condition-imu').textContent()).includes('？'));
      await page.evaluate(() => window.sample());
      assert(await page.locator('#connection-details').evaluate(n => n.closest('details').open === false));
      assert(await page.locator('#measurement').evaluate(n => !!n.closest('details')));
      await page.locator('#details > summary').click();
      page.once('dialog', dialog => dialog.accept());
      await page.click('#calibrate');
      await page.waitForFunction(() => document.getElementById('calibration-action').textContent.includes('queued'));
      await page.click('#calibrate-abort');
      await page.waitForTimeout(50);
      assert.deepEqual(calibrationRequests, [['calibration/start', 'POST'], ['calibration/abort', 'POST']]);
      await page.locator('#details > summary').click();
      assert(await page.evaluate(() => document.documentElement.scrollHeight <= innerHeight));
      fs.mkdirSync('work/instruments-browser', { recursive: true });
      await page.screenshot({ path: `work/instruments-browser/${size.width}.png` });
      // Real canvas pixels: an out-of-range posture is visible on the edge,
      // including its first isolated sample and the full stroke below the frame.
      // Four-point UI through the real WebSocket dispatch and POST routes.
      await page.evaluate(() => { document.getElementById('leg-range-calibration').open = true; });
      await page.click('#leg-cal-button-start');
      assert.equal(legCalRequests.length, 0); // Support confirmation is explicit.
      await page.locator('#leg-range-calibration input').check();
      for (const action of ['start', 'candidate', 'right-zero', 'right-full', 'left-zero', 'left-full', 'save', 'abort']) {
        await page.click(`#leg-cal-button-${action}`);
        await page.waitForFunction(() => !document.getElementById('leg-cal-button-start').disabled);
      }
      assert.deepEqual(legCalRequests.map(x => x[0]), ['start', 'candidate', 'right-zero', 'right-full', 'left-zero', 'left-full', 'save', 'abort']);
      assert(legCalRequests.every(x => x[1] === 'POST'));
      await page.evaluate(() => {
        window.calPacket = { type: 'leg_calibration', boot_id: 'boot-test', phase: 1, mask: 0, point: 255,
          source: 0, generation: 0, slots: 0, result: 'OK', nvs_result: 'NOT_SAVED', save: 'FOUR_POINTS_REQUIRED',
          event_ms: 100, active: false, values: [0, 0, 0, 0], candidate: [0, 0, 0, 0],
          acquired_ms: [0, 0, 0, 0], torque: 2 };
        window.calReceive = () => window.peer.onmessage({ data: JSON.stringify(window.calPacket) });
        window.calReceive();
      });
      assert((await page.locator('#leg-cal-active').textContent()).includes('有効値なし'));
      assert((await page.locator('#leg-cal-right-zero').textContent()).includes('未取得（0ではありません）'));
      assert((await page.locator('#leg-cal-save').textContent()).includes('4点の取得が必要'));
      await page.evaluate(() => {
        Object.assign(window.calPacket, { active: true, source: 1, values: [2007, 1675, 2071, 2412],
          mask: 15, imported: 15, candidate: [2044, 1697, 2084, 2408], acquired_ms: [101, 102, 103, 104], save: 'OK' });
        window.calReceive();
      });
      assert((await page.locator('#leg-cal-active').textContent()).includes('2007 / 1675 / 2071 / 2412'));
      assert((await page.locator('#leg-cal-candidate').textContent()).includes('1940～2034'));
      assert((await page.locator('#leg-cal-candidate').textContent()).includes('2094～2181'));
      assert((await page.locator('#leg-cal-right-zero').textContent()).includes('確定候補読込'));
      await page.evaluate(() => { window.calPacket.result = 'NVS_WRITE_FAILED';window.calReceive(); });
      assert((await page.locator('#leg-cal-result').textContent()).includes('書込み失敗'));
      assert((await page.locator('#leg-cal-active').textContent()).includes('2007 / 1675'));
      const productionCal = process.env.LEG_CALIBRATION_JSON_OUTPUT
        ? JSON.parse(fs.readFileSync(process.env.LEG_CALIBRATION_JSON_OUTPUT, 'utf8'))
        : { phase: 4, source: 1, generation: 1, slots: 1, result: 'OK', nvs_result: 'OK', save: 'NOT_EDITING',
          active: true, values: [2044, 1697, 2084, 2408] };
      await page.evaluate(packet => {
        Object.assign(window.calPacket, packet, { boot_id: 'boot-test' });window.calReceive();
      }, productionCal);
      assert((await page.locator('#leg-cal-state').textContent()).includes('保存完了・停止／再同期待ち'));
      assert((await page.locator('#leg-cal-active').textContent()).includes('2044 / 1697 / 2084 / 2408'));
      const calMutations = await page.evaluate(async () => {
        const target = document.getElementById('leg-cal-result');let mutations = 0;
        const observer = new MutationObserver(events => { mutations += events.length; });
        observer.observe(target, { childList: true, characterData: true, subtree: true });
        for (let i = 0; i < 100; i++) window.calReceive();
        await Promise.resolve();observer.disconnect();return mutations;
      });
      assert.equal(calMutations, 0);
      await page.evaluate(() => window.peer.close());
      await page.waitForFunction(() => document.getElementById('leg-cal-fresh').textContent.includes('通信停止・最終受信時の情報'));
      await page.evaluate(() => { document.getElementById('leg-range-calibration').open = false; });
      const edgePixels = await page.evaluate(() => {
        const canvas = document.createElement('canvas');
        canvas.style.cssText = 'position:fixed;left:0;top:0;width:800px;height:300px';
        document.body.appendChild(canvas);
        const history = new DiagnosticHistory();
        const plot = new OverlayPlot(canvas, document.createElement('div'), history);
        plot.fixedAttitude = true;
        plot.select(['angle_deg', 'reference_deg']);
        const color = COLORS[0].slice(1).match(/../g).map(x => parseInt(x, 16));
        const pixel = (x, y) => {
          const rgba = canvas.getContext('2d').getImageData(x, y, 1, 1).data;
          return color.every((v, i) => Math.abs(v - rgba[i]) < 3) && rgba[3] > 100;
        };
        let seq = 0;
        const push = value => history.push({ boot_id: 'pixel-test', seq: ++seq,
          sample_ms: seq * 50, subscription_id: 1, dropped: 0,
          values: { angle_deg: value, reference_deg: 0 } });
        plot.draw();
        const empty = !pixel(788, 155) && !pixel(788, 270);
        push(-48.9);plot.draw();
        const first = pixel(788, 270) && pixel(788, 271) && !pixel(788, 155);
        for (let i = 0; i < 20; ++i) push(-48.9);
        plot.draw();
        const lower = pixel(783, 270);
        const raw = history.rows.at(-1).values.angle_deg === -48.9 && history.csv().includes('-48.9');
        push(-30);push(-20);plot.draw();
        const reentry = [786, 787, 788, 789].some(x =>
          [232, 233, 234, 235, 236, 237, 238, 239, 240].some(y => pixel(x, y)));
        history.clear();push(48.9);plot.draw();
        const upper = pixel(788, 40) && pixel(788, 39);
        history.clear();push(null);plot.draw();
        const invalid = !pixel(788, 155) && !pixel(788, 270);
        history.clear();history.breakNext = true;push(-48.9);plot.draw();
        const cleared = pixel(788, 270);
        canvas.remove();
        return { empty, first, lower, raw, reentry, upper, invalid, cleared };
      });
      for (const [check, passed] of Object.entries(edgePixels)) assert(passed, check);
      assert.deepEqual(errors, []);
      await page.close();
    }
    for (const failure of assets.slice(1)) {
      const page = await browser.newPage(); const calls = [];
      await page.route('http://127.0.0.1:8765/**', async route => {
        const file = new URL(route.request().url()).pathname.slice(1) || 'index.html'; calls.push(file);
        await route.fulfill({ status: file === failure ? 503 : 200,
          body: file === failure ? 'failure' : fs.readFileSync(`data/${file}`),
          contentType: file.endsWith('.html') ? 'text/html' : 'text/plain' });
      });
      await page.goto('http://127.0.0.1:8765/');
      await page.waitForFunction(() => document.getElementById('connection').textContent.includes('取得・準備失敗'));
      assert.equal(calls.at(-1), failure); await page.close();
    }
    console.log('PASS: natural-height leg rows/status transitions, unchanged DOM/no mutations, separated past failure, collapsed guide/details; real Chromium 100% 1366x768/1920x1080 instruments, fixed ±30/overflow, signed arcs/zero/limits/disable/stale, freshness/reasons, aux/legend/pause, permanent/optional/stopped selection, reconnect and sequential assets');
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exitCode = 1; });
