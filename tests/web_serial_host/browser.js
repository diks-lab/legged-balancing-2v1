// Real Chromium with production files and a deterministic WebSocket peer.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const assets = ['index.html', 'style.css', 'panels.css', 'signals.js', 'history.js',
  'plot.js', 'subscribe.js', 'diagnosis.js', 'status.js', 'dashboard.js', 'app.js'];
(async () => {
  const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH || '/usr/bin/chromium',
    args: ['--no-sandbox'] });
  try {
    for (const size of [{ width: 1366, height: 768 }, { width: 1920, height: 1080 }]) {
      const page = await browser.newPage({ viewport: size, deviceScaleFactor: 1 });
      const requests = [], errors = [], calibrationRequests = [];
      page.on('pageerror', e => errors.push(e.message));
      await page.route('http://127.0.0.1:8765/**', async route => {
        const file = new URL(route.request().url()).pathname.slice(1) || 'index.html';
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
      await page.evaluate(() => window.sample({ gyro_window: [0, 100, 200000, 101, 0, 1800, 2200, 2.5, 8.5] }));
      assert((await page.locator('#gyro-window').textContent()).includes('CONFIG=0x00'));
      await page.evaluate(() => window.sample({ gyro_window: [3, 150, 200000, 100, 1, 1800, 2200, null, null] }));
      assert((await page.locator('#gyro-window').textContent()).includes('無効'));
      await page.evaluate(() => window.sample({ gyro_window: [3, 200, 200000, 101, 0, 1800, 2200, 1, 2], imu_valid: false }));
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
          100, 100, 53000, 0, 2, 2, 1, 2, 12, 2, 4, 1, 1]
      }) }));
      await page.evaluate(() => window.sample({ sample_ms: 120, imu_ms: 120, command_ms: 120 }));
      assert((await page.locator('#health-diagnosis').textContent()).includes('通信回復を確認'), await page.locator('#health-diagnosis').textContent());
      assert((await page.locator('#health-diagnosis').textContent()).includes('53587 ms'));
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
      await page.locator('#details summary').click();
      page.once('dialog', dialog => dialog.accept());
      await page.click('#calibrate');
      await page.waitForFunction(() => document.getElementById('calibration-action').textContent.includes('queued'));
      await page.click('#calibrate-abort');
      await page.waitForTimeout(50);
      assert.deepEqual(calibrationRequests, [['calibration/start', 'POST'], ['calibration/abort', 'POST']]);
      await page.locator('#details summary').click();
      assert(await page.evaluate(() => document.documentElement.scrollHeight <= innerHeight));
      fs.mkdirSync('work/instruments-browser', { recursive: true });
      await page.screenshot({ path: `work/instruments-browser/${size.width}.png` });
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
    console.log('PASS: real Chromium 100% 1366x768/1920x1080 instruments, fixed ±30/overflow, signed arcs/zero/limits/disable/stale, freshness/reasons, aux/legend/pause, permanent/optional/stopped selection, reconnect and sequential assets');
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exitCode = 1; });
