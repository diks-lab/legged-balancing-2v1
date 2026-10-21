// Real Chromium, production files; WebSocket peer is deterministic (no ESP32).
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const assets = ['index.html', 'style.css', 'signals.js', 'history.js', 'plot.js', 'subscribe.js', 'app.js'];
(async () => {
  const browser = await chromium.launch({
    executablePath: process.env.CHROMIUM_PATH || '/usr/bin/chromium',
    args: ['--no-sandbox']
  });
  try {
    for (const size of [{ width: 1366, height: 768 }, { width: 1920, height: 1080 }]) {
      const page = await browser.newPage({ viewport: size, deviceScaleFactor: 1 });
      const errors = [], requests = [];
      page.on('pageerror', e => errors.push(e.message));
      await page.route('http://127.0.0.1:8765/**', async route => {
        const name = new URL(route.request().url()).pathname.slice(1) || 'index.html';
        requests.push(name);
        await route.fulfill({ body: fs.readFileSync(`data/${name}`),
          contentType: name.endsWith('.html') ? 'text/html' : 'text/plain' });
      });
      await page.addInitScript(() => {
        window.sequence = 0;
        window.sampleNext = (extra = {}, gap = 1) => {
          window.sequence += gap;
          window.sample(window.sequence, extra);
        };
        window.sent = [];
        window.axisLabels = [];
        const fillText = CanvasRenderingContext2D.prototype.fillText;
        CanvasRenderingContext2D.prototype.fillText = function (text, ...args) {
          window.axisLabels.push(text);
          return fillText.call(this, text, ...args);
        };
        window.activeFetches = 0;
        window.maxFetches = 0;
        const fetch = window.fetch;
        window.fetch = async (...args) => {
          window.activeFetches++;
          window.maxFetches = Math.max(window.maxFetches, window.activeFetches);
          const response = await fetch(...args);
          const text = response.text.bind(response);
          response.text = async () => {
            try { return await text(); }
            finally { window.activeFetches--; }
          };
          return response;
        };
        window.autoAck = true;
        window.createdSockets = 0;
        window.reply = (packet, overrides = {}) => {
          window.peer.onmessage({ data: JSON.stringify({ type: 'subscribed',
            boot_id: 'test', request_id: packet.request_id,
            subscription_id: packet.request_id, period_ms: 50,
            fields: packet.fields || [], streaming: packet.type !== 'stop', ...overrides }) });
        };
        window.WebSocket = class {
          constructor() {
            if (window.activeFetches) throw Error('WS before scripts complete');
            window.peer = this;
            window.createdSockets++;
            this.readyState = 0;
            setTimeout(() => { this.readyState = 1; this.onopen(); }, 0);
          }
          send(text) {
            const request = JSON.parse(text);
            window.sent.push(request);
            if (window.autoAck) window.reply(request);
          }
          close() { this.readyState = 3; this.onclose({ code: 1006 }); }
        };
        window.sample = (seq, extra = {}) => {
          const request = window.sent.at(-1);
          const values = Object.fromEntries((request.fields || []).map((key, i) =>
            [key, key === 'angle_deg' ? 8 * Math.sin(seq / 40)
              : key === 'reference_deg' ? -0.8
              : key === 'gyro_deg_s' ? 4 * Math.cos(seq / 40)
              : key.includes('velocity') ? 0.0001 * Math.sin(seq / 20 + i)
              : ['control_angle_error_deg', 'position_error_rad', 'target_speed_rad_s',
                'target_tilt_deg'].includes(key) ? null : 0]));
          window.peer.onmessage({ data: JSON.stringify({
            type: 'sample', boot_id: 'test', subscription_id: window.sent.length,
            seq, source_seq: seq, sample_ms: seq * 50, age_ms: 0, dropped: 0,
            stop_reason: 'NONE', stop_ms: 0, faults: 0, running: false,
            disabled: true, ps4: false, foc_ready: true, values, ...extra
          }) });
        };
      });
      await page.goto('http://127.0.0.1:8765/');
      await page.waitForFunction(() => window.sent?.length === 1);
      assert.deepEqual(requests, assets);
      assert.equal(await page.evaluate(() => window.maxFetches), 1);
      assert.equal(await page.locator('canvas').count(), 1);
      assert.equal(await page.locator('#fields input:checked').count(), 5);
      assert.deepEqual(await page.evaluate(() => window.sent[0].fields),
        ['angle_deg', 'reference_deg', 'gyro_deg_s', 'right_command_v', 'left_command_v']);
      await page.evaluate(() => {
        window.sampleNext({ values: { angle_deg: 12.5, reference_deg: -0.8,
          gyro_deg_s: -3.2, right_command_v: 0, left_command_v: 0 } });
        window.sampleNext({ values: { angle_deg: -8.25, reference_deg: -0.8,
          gyro_deg_s: 1.6, right_command_v: 0, left_command_v: 0 } });
      });
      await page.click('#pause'); await page.click('#pause');
      assert((await page.locator('#health').textContent()).includes('停止中'));
      assert((await page.locator('#loss').textContent()).includes('seq: 2'));
      const normal = await page.locator('#plot').evaluate(c => c.toDataURL());
      await page.check('#autoscale');
      assert.notEqual(await page.locator('#plot').evaluate(c => c.toDataURL()), normal);
      await page.uncheck('#autoscale');
      assert.equal(await page.locator('#plot').evaluate(c => c.toDataURL()), normal);
      await page.evaluate(() => {
        window.axisLabels = [];
        window.sampleNext({ values: { angle_deg: 40, reference_deg: -0.8,
          gyro_deg_s: -3.2, right_command_v: 1.5, left_command_v: -1.5 } });
      });
      await page.click('#pause'); await page.click('#pause');
      assert(await page.evaluate(() => window.axisLabels.some(t => Number(t) > 40)));
      await page.click('#clear');
      for (const [preset, count] of [['stationary', 5], ['control', 7], ['position', 8], ['drive', 10]]) {
        await page.locator('#selection summary').click();
        await page.selectOption('#preset', preset);
        const before = await page.evaluate(() => window.sent.length);
        assert.equal(await page.locator('#fields input:checked').count(), count);
        assert.equal(await page.evaluate(() => window.sent.length), before);
        await page.locator('#selection summary').click();
        await page.waitForFunction(n => window.sent.length > n, before);
        await page.evaluate(() => { for (let i = 1; i <= 1200; i++) window.sampleNext(); });
        await page.click('#pause');
        await page.click('#pause');
        const layout = await page.evaluate(() => {
          const visible = ['plot', 'legend', 'start', 'stop', 'pause', 'clear', 'reconnect', 'selection', 'details'];
          return { width: innerWidth, height: innerHeight,
            scrollWidth: document.documentElement.scrollWidth,
            scrollHeight: document.documentElement.scrollHeight,
            boxes: visible.map(id => {
              const r = document.getElementById(id).getBoundingClientRect();
              return { id, left: r.left, top: r.top, right: r.right, bottom: r.bottom };
            }) };
        });
        assert(layout.scrollWidth <= size.width && layout.scrollHeight <= size.height,
          JSON.stringify(layout));
        for (const box of layout.boxes) {
          assert(box.left >= 0 && box.top >= 0 && box.right <= size.width && box.bottom <= size.height,
            JSON.stringify(box));
        }
        console.log('LAYOUT', size.width, size.height, preset, JSON.stringify(layout.boxes.find(x => x.id === 'plot')));
        const units = await page.evaluate(() => window.axisLabels);
        for (const unit of ['deg 主軸', 'deg/s', 'V']) assert(units.includes(unit));
        if (preset !== 'stationary') assert(units.includes('rad/s'));
        if (preset === 'position') assert(units.includes('rad'));
        fs.mkdirSync('work/attitude-browser', { recursive: true });
        await page.screenshot({ path: `work/attitude-browser/${size.width}-${preset}.png` });
      }
      const beforeChanges = await page.evaluate(() => {
        window.autoAck = false;
        const inputs = [...document.querySelectorAll('#fields input')];
        for (const key of ['angle_deg', 'reference_deg']) {
          const input = inputs.find(n => n.value === key);
          input.checked = true;
          input.dispatchEvent(new Event('change', { bubbles: true }));
        }
        return window.sent.length;
      });
      assert((await page.locator('#subscription').textContent()).includes('変更待ち'));
      await page.waitForFunction(n => window.sent.length === n + 1, beforeChanges);
      assert((await page.locator('#subscription').textContent()).includes('応答待ち'));
      const waiting = await page.evaluate(() => window.sent.at(-1));
      await page.locator('#selection summary').click();
      await page.check('#fields input[value="right_foc_q_v"]');
      await page.locator('#selection summary').click();
      // The second selection is newer than the pending request; its old ACK is ignored.
      await page.waitForTimeout(220);
      const legendBefore = await page.locator('#legend').textContent();
      await page.evaluate(packet => window.reply(packet), waiting);
      assert.equal(await page.locator('#legend').textContent(), legendBefore);
      await page.waitForFunction(n => window.sent.length === n + 2, beforeChanges);
      await page.evaluate(packet => window.reply(packet), waiting);
      assert.equal(await page.locator('#legend').textContent(), legendBefore);
      await page.evaluate(() => { window.reply(window.sent.at(-1)); window.autoAck = true; });
      assert((await page.locator('#legend').textContent()).includes('右FOC q電圧'));
      assert.equal(await page.evaluate(() => window.createdSockets), 1);
      await page.locator('#legend button').first().click();
      assert.equal(await page.locator('#legend button').first().getAttribute('aria-pressed'), 'false');
      await page.locator('#legend button').first().click();
      assert.equal(await page.locator('#legend button').first().getAttribute('aria-pressed'), 'true');
      await page.click('#pause');
      const frozen = await page.locator('#plot').evaluate(c => c.toDataURL());
      await page.evaluate(() => window.sampleNext({ faults: 2, stop_reason: 'TEST_FAULT', dropped: 4 }, 5));
      assert((await page.locator('#health').textContent()).includes('TEST_FAULT'));
      assert((await page.locator('#loss').textContent()).includes('欠落推定: 4'));
      assert.equal(await page.locator('#plot').evaluate(c => c.toDataURL()), frozen);
      await page.click('#pause');
      assert.notEqual(await page.locator('#plot').evaluate(c => c.toDataURL()), frozen);
      // CSV remains auxiliary; paused samples were still retained.
      await page.evaluate(() => {
        URL.createObjectURL = blob => { window.savedCsv = blob.text(); return 'blob:test'; };
        HTMLAnchorElement.prototype.click = function () {};
        document.getElementById('save').click();
      });
      const csv = await page.evaluate(() => window.savedCsv);
      assert(csv.split('\n').length <= 1201 && csv.split('\n').length > 1100);
      assert(csv.includes('TEST_FAULT'));
      await page.click('#stop');
      assert.equal(await page.evaluate(() => window.sent.at(-1).type), 'stop');
      const stoppedCount = await page.evaluate(() => window.sent.length);
      await page.locator('#selection summary').click();
      await page.uncheck('#fields input[value="right_foc_q_v"]');
      await page.locator('#selection summary').click();
      await page.waitForTimeout(250);
      assert.equal(await page.evaluate(() => window.sent.length), stoppedCount);
      assert((await page.locator('#subscription').textContent()).includes('配信停止中'));
      await page.evaluate(() => window.peer.close());
      assert((await page.locator('#connection').textContent()).includes('1006'));
      await page.click('#reconnect');
      await page.waitForFunction(() => window.peer.readyState === 1);
      await page.evaluate(() => window.sampleNext());
      assert((await page.locator('#loss').textContent()).includes(`seq: ${await page.evaluate(() => window.sequence)}`));
      // Even maximum selection fits the normal collapsed layout.
      await page.locator('#selection summary').click();
      await page.locator('#fields input').evaluateAll(nodes => nodes.forEach(n => n.checked = true));
      await page.locator('#selection summary').click();
      await page.click('#start');
      await page.evaluate(() => window.sampleNext());
      assert.equal(await page.locator('#legend button').count(), 32);
      assert(await page.evaluate(() => document.documentElement.scrollHeight <= innerHeight &&
        document.documentElement.scrollWidth <= innerWidth));
      const beforeEmpty = await page.evaluate(() => {
        for (const input of document.querySelectorAll('#fields input')) input.checked = false;
        document.getElementById('fields').dispatchEvent(new Event('change', { bubbles: true }));
        return window.sent.length;
      });
      await page.waitForFunction(n => window.sent.length > n, beforeEmpty);
      assert.equal(await page.evaluate(() => window.sent.at(-1).type), 'stop');
      assert.equal(await page.locator('#legend button').count(), 0);
      await page.locator('#selection summary').click();
      await page.selectOption('#preset', 'stationary');
      await page.locator('#selection summary').click();
      const afterEmpty = await page.evaluate(() => window.sent.length);
      await page.waitForTimeout(250);
      assert.equal(await page.evaluate(() => window.sent.length), afterEmpty);
      await page.click('#start');
      await page.evaluate(() => window.sample(1, { boot_id: 'restarted' }));
      await page.evaluate(() => document.getElementById('save').click());
      const restarted = await page.evaluate(() => window.savedCsv);
      assert.equal(restarted.split('\n').length, 2);
      assert(restarted.includes('restarted') && !restarted.includes('TEST_FAULT'));
      await page.click('#clear');
      assert.deepEqual(errors, []);
      fs.mkdirSync('work/attitude-browser', { recursive: true });
      await page.screenshot({ path: `work/attitude-browser/${size.width}.png` });
      await page.close();
    }
    // Each bootstrap failure is displayed once; later assets/WS never start.
    for (const fail of ['style.css', 'signals.js', 'history.js', 'plot.js', 'subscribe.js', 'app.js']) {
      const page = await browser.newPage();
      const requests = [];
      await page.route('http://127.0.0.1:8765/**', async route => {
        const file = new URL(route.request().url()).pathname.slice(1) || 'index.html';
        requests.push(file);
        await route.fulfill({ status: file === fail ? 503 : 200,
          body: file === fail ? 'failed' : fs.readFileSync(path.join('data', file)),
          contentType: file.endsWith('.html') ? 'text/html' : 'text/plain' });
      });
      await page.goto('http://127.0.0.1:8765/');
      await page.waitForFunction(() => document.getElementById('connection').textContent.includes('取得・準備失敗'));
      assert((await page.locator('#connection').textContent()).includes(fail));
      assert.equal(requests.at(-1), fail);
      assert.equal(requests.filter(f => f === fail).length, 1);
      await page.close();
    }
    console.log('PASS: real Chromium 100% 1366x768/1920x1080, all presets fit, one overlay, primary axis, normal/auto scale, legend toggles, debounced/old ACK selections, pause/health, loss, stop/reconnect, sequential assets and failures');
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
