// Real Chromium with production files and a deterministic WebSocket peer.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const assets = ['index.html', 'style.css', 'panels.css', 'signals.js', 'history.js',
  'plot.js', 'subscribe.js', 'dashboard.js', 'app.js'];
(async () => {
  const browser = await chromium.launch({ executablePath: process.env.CHROMIUM_PATH || '/usr/bin/chromium',
    args: ['--no-sandbox'] });
  try {
    for (const size of [{ width: 1366, height: 768 }, { width: 1920, height: 1080 }]) {
      const page = await browser.newPage({ viewport: size, deviceScaleFactor: 1 });
      const requests = [], errors = [];
      page.on('pageerror', e => errors.push(e.message));
      await page.route('http://127.0.0.1:8765/**', async route => {
        const file = new URL(route.request().url()).pathname.slice(1) || 'index.html';
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
            age_ms: 0, dropped: 0, state: 'READY_WAIT', startup_reason: 'READY',
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
      await page.evaluate(() => { for (let i = 0; i < 1200; i++) window.sample(); });
      await page.click('#pause'); await page.click('#pause');
      assert(await page.evaluate(() => window.axisLabels.some(x => x.id === 'pose' && x.text === '30.0')));
      assert(await page.evaluate(() => window.axisLabels.some(x => x.id === 'pose' && x.text === '-30.0')));
      await page.evaluate(() => window.sample({ values: { angle_deg: 42.25, reference_deg: -0.8,
        right_command_v: 0.35, left_command_v: -0.2 } }));
      assert((await page.locator('#pose-value').textContent()).includes('▲ 上限超過 42.25'));
      assert.equal(await page.locator('#left-reading').textContent(), '−0.20 V');
      assert.equal(await page.locator('#right-reading').textContent(), '+0.35 V');
      const arcs = await page.evaluate(() => window.arcCalls);
      assert(arcs.some(x => x.id === 'left-meter' && x.args[5] === true));
      assert(arcs.some(x => x.id === 'right-meter' && x.args[5] === false));
      for (const v of [0, 1, -1, 1.5, -1.5]) {
        await page.evaluate(v => window.sample({ values: { angle_deg: -42, reference_deg: -0.8,
          left_command_v: v, right_command_v: v } }), v);
        assert((await page.locator('#pose-value').textContent()).includes('▼ 下限超過'));
        if (Math.abs(v) > 1) assert((await page.locator('#left-limit').textContent()).includes('超過'));
      }
      await page.evaluate(() => window.sample({ disabled: true }));
      assert.equal(await page.locator('#left-reading').textContent(), '遮断');
      await page.evaluate(() => window.sample({ values: { angle_deg: null, reference_deg: -0.8,
        left_command_v: null, right_command_v: null }, imu_valid: false, imu_reason: 'RIGHT_WHEEL_FAULT',
        state: 'FAULT', faults: 1, right_error: 'I2C_NACK' }));
      assert.equal(await page.locator('#right-reading').textContent(), '更新停止／不明');
      assert((await page.locator('#measurement').textContent()).includes('右車輪異常'));
      assert((await page.locator('#health').textContent()).includes('I2C_NACK'));
      // Fixed attitude scale survives auxiliary autoscale.
      await page.check('#autoscale');
      await page.click('#pause'); await page.click('#pause');
      await page.uncheck('#autoscale');
      for (const preset of ['stationary', 'control', 'position', 'drive']) {
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
      assert.equal(await page.locator('#left-reading').textContent(), '+0.30 V');
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
