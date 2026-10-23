'use strict';
window.startLogger = function () {
  const el = id => document.getElementById(id);
  async function calibrationAction(path) {
    const message = el('calibration-action');
    try {
      const response = await fetch(path, { method: 'POST', signal: AbortSignal.timeout(3000) });
      const result = await response.text();
      message.textContent = `${response.ok ? '受付' : '拒否'}: ${result}（状態欄で実行結果を確認）`;
    } catch (error) { message.textContent = `要求失敗: ${error.message}。自動再試行なし`; }
  }
  el('calibrate').onclick = () => {
    if (window.confirm('停止中だけ実行できます。機体を保持してください。両輪が校正回転します。完了後は自動倒立しません。')) {
      calibrationAction('/calibration/start');
    }
  };
  el('calibrate-abort').onclick = () => calibrationAction('/calibration/abort');
  const history = new DiagnosticHistory();
  const dashboard = new Dashboard(el, history);
  const plot = dashboard.aux;
  let socket = null, generation = null;
  let boot = null, lastSeq = null, lastDrop = 0, lost = 0;
  let paused = false, lastMessage = 0, lastDraw = 0, first = null;
  let accepted = [];
  const auxiliary = () => Array.from(el('fields').querySelectorAll('input:checked'), n => n.value);
  const selected = () => [...new Set([...PERMANENT, ...auxiliary()])];

  for (const [key, label, unit] of SIGNALS.filter(s => !['angle_deg', 'reference_deg'].includes(s[0]))) {
    const node = document.createElement('label');
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.value = key;
    input.checked = DEFAULTS.includes(key);
    node.append(input, document.createTextNode(`${label} [${unit}]`));
    el('fields').appendChild(node);
  }
  const subscription = new SubscriptionControl(
    text => socket.send(text), selected,
    message => { el('subscription').textContent = message; },
    (response, empty) => {
      generation = response.subscription_id;
      dashboard.live = response.streaming;
      dashboard.status();
      if (response.streaming || empty) {
        accepted = response.fields;
        plot.select(accepted.filter(key => auxiliary().includes(key)));
        dashboard.draw();
      }
    });
  el('fields').onchange = () => subscription.changed();
  el('preset').onchange = () => {
    const keys = PRESETS[el('preset').value];
    for (const input of el('fields').querySelectorAll('input')) {
      input.checked = keys.includes(input.value);
    }
    subscription.changed();
  };
  plot.select(DEFAULTS);
  new ResizeObserver(() => dashboard.draw()).observe(el('plot'));
  el('aux-toggle').onclick = () => {
    const collapsed = el('auxiliary').classList.toggle('collapsed');
    el('workspace').classList.toggle('no-aux', collapsed);
    el('aux-toggle').setAttribute('aria-expanded', String(!collapsed));
    dashboard.draw();
  };
  el('autoscale').onchange = () => {
    plot.autoScale = el('autoscale').checked;
    plot.draw();
  };

  function receive(s) {
    lastMessage = Date.now();
    if (s.type === 'error' || s.type === 'subscribed') {
      subscription.receive(s);
      return;
    }
    if (s.type !== 'sample' && s.type !== 'status') return;
    if (boot !== s.boot_id) {
      if (boot !== null) {
        history.clear();
        dashboard.pause(paused);
      }
      boot = s.boot_id;
      lastSeq = null;
      lastDrop = 0;
      lost = 0;
    }
    if (lastSeq !== null) {
      const delta = (s.seq - lastSeq) >>> 0;
      if (delta > 0 && delta < 0x80000000) {
        lost += Math.max(delta - 1, (s.dropped - lastDrop) >>> 0);
      }
    } else {
      lost = s.dropped;
    }
    lastSeq = s.seq;
    lastDrop = s.dropped;
    el('connection').textContent = '接続';
    el('connection-details').textContent = `boot ${boot} / ` +
      `取得 ${s.sample_ms} ms / age ${s.age_ms} ms`;
    dashboard.update(s);
    el('loss').textContent = `欠落推定: ${lost} / ` +
      `機体の送信破棄累計: ${s.dropped} / seq: ${s.seq}`;
    if (s.subscription_id !== generation || !Object.keys(s.values).length) return;
    history.push(s);
    if (!paused && Date.now() - lastDraw >= 100) {
      dashboard.draw();
      lastDraw = Date.now();
    }
    if (first !== generation) {
      first = generation;
      console.info('WS_FIRST_SAMPLE', generation, s.seq, s.sample_ms);
    }
  }

  function connect() {
    if (socket && socket.readyState < 2) return;
    el('connection').textContent = 'WebSocket接続中…';
    generation = first = null;
    subscription.disconnected();
    dashboard.stop();
    // Keep old lines separate even if reconnect preserves the device generation.
    history.breakNext = true;
    const ws = new WebSocket(`ws://${location.hostname}:81/diagnostics`);
    socket = ws;
    ws.onopen = () => {
      if (socket !== ws) return;
      lastMessage = Date.now();
      el('connection').textContent = '接続：配信設定中';
      subscription.connected();
    };
    ws.onmessage = event => {
      if (socket !== ws) return;
      let s;
      try { s = JSON.parse(event.data); }
      catch {
        el('connection').textContent = '不正なJSONを受信';
        ws.close();
        return;
      }
      if (s.type === 'wheel_diagnostics') {
        if (s.boot_id === boot) {
          dashboard.wheelReads = { wheel0: s.wheel0, wheel1: s.wheel1 };
          dashboard.legPacket = {
            legs: s.legs,
            leg_ms: s.leg_ms,
            leg_mode: s.leg_mode,
            leg_roll_confirmed: s.leg_roll_confirmed,
            leg_roll_sign: s.leg_roll_sign
          };
          dashboard.status();
        }
        return;
      }
      receive(s);
    };
    ws.onerror = () => {
      if (socket === ws) {
        el('connection').textContent = 'WebSocket通信失敗（自動再接続なし）';
      }
    };
    ws.onclose = event => {
      if (socket === ws) {
        subscription.disconnected();
    dashboard.stop();
        el('connection').textContent = `切断 code=${event.code}（再接続は手動）`;
      }
    };
  }

  el('start').onclick = () => subscription.start();
  el('stop').onclick = () => { dashboard.stop(); subscription.start(false); };
  el('reconnect').onclick = connect;
  el('pause').onclick = () => {
    paused = !paused;
    dashboard.pause(paused);
    el('pause').textContent = paused ? '表示再開' : '表示一時停止';
    if (!paused) dashboard.draw();
  };
  el('clear').onclick = () => {
    history.clear();
    dashboard.pause(paused);
    plot.draw();
  };
  el('save').onclick = () => {
    const url = URL.createObjectURL(new Blob([history.csv()], {
      type: 'text/csv;charset=utf-8'
    }));
    const a = document.createElement('a');
    a.href = url;
    a.download = 'esp32-browser.csv';
    a.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  };
  setInterval(() => {
    if (socket && socket.readyState === 1 && Date.now() - lastMessage > 6000) {
      el('connection').textContent = '更新停止（6秒）。再接続してください';
      socket.close();
    }
  }, 1000);
  connect();
};
