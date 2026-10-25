// Real production UI, with layout-equivalent desktop 100% / 125% zoom.
// At 125%, physical pixels stay constant and CSS viewport / DPR change together.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
(async () => {
  const browser = await chromium.launch({ executablePath: '/usr/bin/chromium', args: ['--no-sandbox'] });
  try {
    for (const physical of [{ width: 1366, height: 768 }, { width: 1920, height: 1080 }]) {
      for (const zoom of [1, 1.25]) {
        const viewport = { width: Math.floor(physical.width / zoom), height: Math.floor(physical.height / zoom) };
        const page = await browser.newPage({ viewport, deviceScaleFactor: zoom });
        const errors = [];page.on('pageerror', e => errors.push(e.message));
        await page.route('http://127.0.0.1:8765/**', async route => {
          const file = new URL(route.request().url()).pathname.slice(1) || 'index.html';
          await route.fulfill({ body: fs.readFileSync(`data/${file}`), contentType: file.endsWith('.html') ? 'text/html' : 'text/plain' });
        });
        await page.addInitScript(() => {
          window.WebSocket = class {
            constructor() { window.peer = this;this.readyState = 0;setTimeout(() => { this.readyState = 1;this.onopen(); }, 0); }
            send(data) {
              const request = JSON.parse(data);
              this.onmessage({ data: JSON.stringify({ type: 'subscribed', request_id: request.request_id,
                subscription_id: 1, fields: request.fields, period_ms: 50, streaming: true }) });
              const values = Object.fromEntries(request.fields.map(name => [name, 0]));
              this.onmessage({ data: JSON.stringify({ type: 'sample', boot_id: 'layout', subscription_id: 1,
                seq: 1, source_seq: 1, sample_ms: 53588, imu_ms: 53588, imu_seq: 1, age_ms: 0,
                imu_valid: true, imu_reason: 'OK', reference_valid: true, command_valid: false,
                command_ms: 53588, state: 'EXPLICIT_STOP', startup_reason: 'READY', stop_reason: 'DISARM',
                stop_ms: 53587, faults: 0, running: false, disabled: true, foc_ready: true,
                recovery_checks: 15, calibration: 'RESTORED', recovery: 'EXPLICIT_STOP', ps4: true,
                dropped: 0, values }) });
            }
            close() { this.readyState = 3;this.onclose({ code: 1006 }); }
          };
        });
        await page.goto('http://127.0.0.1:8765/');
        await page.waitForSelector('#leg-diagnostics', { state: 'attached' });
        await page.evaluate(() => {
          const events = [1, 2, 3, 4].map(i => [4294967200 + i, 4294967200 + i,
            i, 17, 9, i, i + 1, 15, 15, 63, 1023, 2]);
          window.packet = {
            type: 'wheel_diagnostics', boot_id: 'layout', leg_ms: 53588, leg_age_ms: 0,
            leg_mode: 1, leg_roll_confirmed: false, leg_roll_sign: 1,
            leg_state: [4294967295, 4294967295, 1, 0, 7, 9, 15, 15, 63, 4, 1023, 9, 53587, 3, 4294967295, 4, events],
            leg_accept: [1, 1, 1697, 2044, 2084, 2408, 1, 2],
            leg_geometry: [[2044, 1697, 10, 10, 1707, 2034, 1940, 2034], [2084, 2408, 10, 10, 2094, 2398, 2094, 2181], [0, 30]],
            leg_position: [53587, 0, 3, 3,
              [53587, 1697, 161, 6, 1697, 100, 1940, 2034, false, 2044, 1697],
              [53587, 2408, 104, 9, 2408, 100, 2094, 2181, false, 2084, 2408]],
            leg_rx: [53587, 10, 2, 56, 2, 0, 63, 8, 171, 4294967295, 4294967295, 1,
              'FFFF010420F407DD'.repeat(4), 'FFFF010420F407DD', 221, 1],
            leg_stick: [4294967295, -128, 127, 127, -128, 1, 1, 0, 0, 0.05, 0.1, 127],
            leg_config: [4294967200, 4294967295, 4294967295, 3, 9, 3, 15, 0, 25,
              [4294967295, 4294967295, 0, 4095, 0], [4294967295, 4294967295, 0, 4095, 0]],
            legs: [100, 3, 0, -48.9, 100, 100, 1697, 2408, 53588, 53587, 9, 2, 0, 0, 1, 1, 4294967295, 4294967295, 4294967295]
          };
          window.peer.onmessage({ data: JSON.stringify(window.packet) });
          window.peer.onmessage({ data: JSON.stringify({ type: 'leg_calibration', boot_id: 'layout',
            phase: 1, mask: 15, imported: 15, point: 255, active: true, source: 1, generation: 4294967295,
            slots: 3, result: 'NVS_VERIFY_FAILED', nvs_result: 'NVS_VERIFY_FAILED', save: 'FOUR_POINTS_REQUIRED',
            event_ms: 4294967295, torque: 2, values: [2044, 1697, 2084, 2408], candidate: [2044, 1697, 2084, 2408],
            acquired_ms: [4294967295, 4294967295, 4294967295, 4294967295] }) });
          document.getElementById('details').open = true;
          document.querySelectorAll('#leg-diagnostics details').forEach(details => { details.open = true; });
          document.getElementById('leg-range-calibration').open = true;
        });
        const check = () => page.evaluate(() => {
          const failures = [], roots = ['leg-diagnostics', 'leg-range-calibration'];
          const intersect = (a, b) => Math.min(a.right, b.right) - Math.max(a.left, b.left) > 0.5 &&
            Math.min(a.bottom, b.bottom) - Math.max(a.top, b.top) > 0.5;
          let lines = 0, wrapped = 0;
          for (const id of roots) {
            const root = document.getElementById(id);
            const nodes = [...root.querySelectorAll('p, pre, dt, dd, li, summary, label, button')];
            const textRects = nodes.map(node => {
              const range = document.createRange();range.selectNodeContents(node);
              const rects = [...range.getClientRects()].filter(rect => rect.width > 0 && rect.height > 0);
              lines += rects.length;if(rects.length > 1) wrapped++;
              for (let i = 0; i < rects.length; i++) for (let j = i + 1; j < rects.length; j++) {
                if (Math.abs(rects[i].top - rects[j].top) > 1 && intersect(rects[i], rects[j])) failures.push(`line overlap ${node.id || node.tagName}`);
              }
              const box = node.getBoundingClientRect();
              for (const rect of rects) {
                if (rect.left < box.left - 1 || rect.right > box.right + 1 || rect.top < box.top - 1 || rect.bottom > box.bottom + 1)
                  failures.push(`text outside ${node.id || node.tagName}`);
              }
              return { node, rects };
            });
            for (let i = 0; i < textRects.length; i++) for (let j = i + 1; j < textRects.length; j++) {
              const a = textRects[i], b = textRects[j];
              if (a.node.contains(b.node) || b.node.contains(a.node)) continue;
              if (a.rects.some(x => b.rects.some(y => intersect(x, y)))) failures.push(`overlap ${a.node.id || a.node.tagName} / ${b.node.id || b.node.tagName}`);
            }
            if (root.scrollWidth > root.clientWidth + 1) failures.push(`${id} horizontal overflow`);
          }
          const rows = [...document.querySelectorAll('.leg-field')].map(node => node.getBoundingClientRect());
          for (let i = 0; i < rows.length; i++) for (let j = i + 1; j < rows.length; j++) if (intersect(rows[i], rows[j])) failures.push('row intersection');
          return { failures: [...new Set(failures)], lines, wrapped,
            width: innerWidth, dpr: devicePixelRatio,
            columns: getComputedStyle(document.querySelector('.leg-fields')).gridTemplateColumns };
        });
        let result = await check();
        assert.deepEqual(result.failures, [], JSON.stringify({ physical, zoom, ...result }));
        assert(result.wrapped > 8 && result.lines > 40, 'exercise real wrapped text');
        assert.equal(await page.locator('#leg-state-trace').textContent().then(s => s.split('\n').length), 4);
        assert((await page.locator('#leg-blocker').textContent()).includes('左：'));
        // All actual long gate reasons remain readable; no artificial display delay.
        for (const reason of [7, 17, 21, 22, 23, 25, 27, 28, 29]) {
          await page.evaluate(reason => { window.packet.leg_state[5] = reason;window.peer.onmessage({ data: JSON.stringify(window.packet) }); }, reason);
          result = await check();assert.deepEqual(result.failures, [], `reason ${reason} zoom ${zoom}`);
        }
        assert.deepEqual(errors, []);
        console.log('PASS readability', physical, 'zoom', zoom, result);
        await page.close();
      }
    }
  } finally { await browser.close(); }
})().catch(error => { console.error(error);process.exitCode = 1; });
