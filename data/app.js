'use strict';
window.startLogger = function () {
  const el = id => document.getElementById(id);
  const history = new DiagnosticHistory();
  const plot = new OverlayPlot(el('plot'), el('legend'), history);
  let socket = null, request = 0, pending = 0, generation = null;
  let boot = null, lastSeq = null, lastDrop = 0, lost = 0;
  let paused = false, lastMessage = 0, lastDraw = 0, first = null;
  let accepted = [];
  const selected = () => Array.from(
    el('fields').querySelectorAll('input:checked'), n => n.value);

  for (const [key, label, unit] of SIGNALS) {
    const node = document.createElement('label');
    const input = document.createElement('input');
    input.type = 'checkbox';
    input.value = key;
    input.checked = DEFAULTS.includes(key);
    node.append(input, document.createTextNode(`${label} [${unit}]`));
    el('fields').appendChild(node);
  }
  el('preset').onchange = () => {
    const keys = PRESETS[el('preset').value];
    for (const input of el('fields').querySelectorAll('input')) {
      input.checked = keys.includes(input.value);
    }
  };
  plot.select(DEFAULTS);
  new ResizeObserver(() => { if (!paused) plot.draw(); }).observe(el('plot'));

  function subscribe(stop = false) {
    if (!socket || socket.readyState !== 1) {
      el('connection').textContent = '未接続：再接続してください';
      return;
    }
    const fields = selected();
    if (!stop && !fields.length) {
      el('subscription').textContent = '項目を1つ以上選択してください';
      return;
    }
    pending = ++request;
    el('subscription').textContent = '配信応答待ち…';
    socket.send(JSON.stringify(stop
      ? { type: 'stop', request_id: pending }
      : { type: 'subscribe', request_id: pending, period_ms: 50, fields }));
  }

  function receive(s) {
    lastMessage = Date.now();
    if (s.type === 'error') {
      el('subscription').textContent = `配信設定拒否: ${s.reason}`;
      return;
    }
    if (s.type === 'subscribed') {
      if (s.request_id !== pending) return;
      generation = s.subscription_id;
      if (s.streaming) accepted = s.fields;
      el('subscription').textContent = `世代 ${generation} / ${s.period_ms} ms / ` +
        `${s.streaming ? '配信中' : '配信停止'} / 選択${accepted.length}項目`;
      if (s.streaming) {
        plot.select(accepted);
        if (!paused) plot.draw();
      }
      return;
    }
    if (s.type !== 'sample' && s.type !== 'status') return;
    if (boot !== s.boot_id) {
      if (boot !== null) history.clear();
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
    el('connection').textContent = `接続 / boot ${boot} / ` +
      `取得 ${s.sample_ms} ms / age ${s.age_ms} ms`;
    el('health').textContent = `${s.running ? '倒立／試験中' : '停止中'} / ` +
      `停止理由: ${s.stop_reason} @ ${s.stop_ms} ms / 異常bit: ${s.faults} / ` +
      `出力遮断:${s.disabled} / FOC:${s.foc_ready} / PS4:${s.ps4}`;
    el('loss').textContent = `欠落推定: ${lost} / ` +
      `機体の送信破棄累計: ${s.dropped} / seq: ${s.seq}`;
    if (s.subscription_id !== generation || !Object.keys(s.values).length) return;
    history.push(s);
    if (!paused && Date.now() - lastDraw >= 100) {
      plot.draw();
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
    // Keep old lines separate even if reconnect preserves the device generation.
    history.breakNext = true;
    const ws = new WebSocket(`ws://${location.hostname}:81/diagnostics`);
    socket = ws;
    ws.onopen = () => {
      if (socket !== ws) return;
      lastMessage = Date.now();
      el('connection').textContent = '接続：配信設定中';
      subscribe();
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
      receive(s);
    };
    ws.onerror = () => {
      if (socket === ws) {
        el('connection').textContent = 'WebSocket通信失敗（自動再接続なし）';
      }
    };
    ws.onclose = event => {
      if (socket === ws) {
        el('connection').textContent = `切断 code=${event.code}（再接続は手動）`;
      }
    };
  }

  el('start').onclick = () => subscribe();
  el('stop').onclick = () => subscribe(true);
  el('reconnect').onclick = connect;
  el('pause').onclick = () => {
    paused = !paused;
    plot.paused = paused;
    el('pause').textContent = paused ? '表示再開' : '表示一時停止';
    if (!paused) plot.draw();
  };
  el('clear').onclick = () => { history.clear(); plot.draw(); };
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
