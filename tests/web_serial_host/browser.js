// Real Chromium, production files; WebSocket peer is deterministic (no ESP32).
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const assets = ['index.html', 'style.css', 'signals.js', 'history.js', 'plot.js', 'app.js'];
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
        window.WebSocket = class {
          constructor() {
            if (window.activeFetches) throw Error('WS before scripts complete');
            window.peer = this;
            this.readyState = 0;
            setTimeout(() => { this.readyState = 1; this.onopen(); }, 0);
          }
          send(text) {
            const request = JSON.parse(text);
            window.sent.push(request);
            const subscription_id = window.sent.length;
            this.onmessage({ data: JSON.stringify({ type: 'subscribed',
              boot_id: 'test', request_id: request.request_id, subscription_id,
              period_ms: 50, fields: request.fields || [], streaming: request.type !== 'stop' }) });
          }
          close() { this.readyState = 3; this.onclose({ code: 1006 }); }
        };
        window.sample = (seq, extra = {}) => {
          const request = window.sent.at(-1);
          const values = Object.fromEntries((request.fields || []).map((key, i) =>
            [key, Math.sin(seq / 20 + i) * (i + 1)]));
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
      assert.equal(await page.locator('#fields input:checked').count(), 7);
      for (const [preset, count] of [['stationary', 7], ['position', 8], ['drive', 10]]) {
        await page.locator('#selection summary').click();
        await page.selectOption('#preset', preset);
        const before = await page.evaluate(() => window.sent.length);
        assert.equal(await page.locator('#fields input:checked').count(), count);
        assert.equal(await page.evaluate(() => window.sent.length), before);
        await page.locator('#selection summary').click();
        await page.click('#start');
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
        for (const unit of ['deg', 'deg/s', 'rad/s', 'V']) assert(units.includes(unit));
        if (preset === 'position') assert(units.includes('rad'));
        fs.mkdirSync('work/graph-browser', { recursive: true });
        await page.screenshot({ path: `work/graph-browser/${size.width}-${preset}.png` });
      }
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
      await page.evaluate(() => window.sample(1, { boot_id: 'restarted' }));
      await page.evaluate(() => document.getElementById('save').click());
      const restarted = await page.evaluate(() => window.savedCsv);
      assert.equal(restarted.split('\n').length, 2);
      assert(restarted.includes('restarted') && !restarted.includes('TEST_FAULT'));
      await page.click('#clear');
      assert.deepEqual(errors, []);
      fs.mkdirSync('work/graph-browser', { recursive: true });
      await page.screenshot({ path: `work/graph-browser/${size.width}.png` });
      await page.close();
    }
    // Each bootstrap failure is displayed once; later assets/WS never start.
    for (const fail of ['style.css', 'signals.js', 'history.js', 'plot.js', 'app.js']) {
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
    console.log('PASS: real Chromium 100% 1366x768/1920x1080, all presets fit, one overlay, legend toggles, deferred selections, pause/health, loss, stop/reconnect, sequential assets and failures');
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
