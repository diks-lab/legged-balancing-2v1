'use strict';
function legMeasurementLines(packet, fresh) {
  const m = packet?.leg_measure;
  if (!Array.isArray(m) || m.length !== 12) return [
    '位置測定：未実行／未受信', '右：未取得', '左：未取得',
    '×停止・脚トルクOFF送信後、機体を支持して明示開始してください。',
    '測定値を初回受付や操作範囲へ自動反映しません。'
  ];
  const [start, end, rejected, revision, status, acquired, valid, side, error, rejection, right, left] = m;
  const reason = n => n ? LEG_BLOCKERS[n] || `理由${n}` : 'なし';
  const state = ['未実行', '測定中・読取り専用', '測定終了', '通信失敗・測定終了'][status] || '不明';
  const lines = [`位置測定：${state} ／ 開始 ${start}・終了 ${end} ms ／ ${fresh ? '最新診断' : '通信停止・最終受信情報（現在状態は不明）'}`];
  for (const [i, data] of [[0, right], [1, left]]) {
    const name = i ? '左 ID2' : '右 ID1';
    if (!(acquired & (1 << i)) || !Array.isArray(data)) {
      lines.push(`${name}：未取得（0 tickではありません）`);
      continue;
    }
    const [ms, count, current, minimum, maximum, currentPct, minTickPct, maxTickPct] = data;
    const pct = v => Number.isFinite(v) ? `${v.toFixed(2)}%` : '換算未受信';
    const validity = valid & (1 << i) ? '正常応答の取得値' : '通信失敗・最後の正常取得値';
    lines.push(`${name}：現在（最終取得） ${current} tick ／ 最小 ${minimum}・最大 ${maximum} tick
換算：現在 ${pct(currentPct)} ／ 最小tick ${pct(minTickPct)} ／ 最大tick ${pct(maxTickPct)}
取得 ${ms} ms ／ ${count} 回 ／ ${validity}${status === 1 && fresh ? '' : '・測定更新停止'}`);
  }
  lines.push(`測定失敗：${reason(error)} ／ 最近の開始拒否：${rejected ? `${reason(rejection)} @ ${rejected} ms` : 'なし'}`);
  lines.push('正常応答の最小・最大値です。機械的限界や安全範囲の保証ではなく、許可範囲へ自動反映しません。');
  return lines;
}
function createLegMeasurementPanel(section) {
  const details = document.createElement('details');
  details.id = 'leg-measure';
  const summary = document.createElement('summary');
  summary.textContent = '読取り専用の位置測定（現在・最小・最大tick）';
  const label = document.createElement('label');
  const supported = document.createElement('input');
  supported.id = 'leg-measure-supported';supported.type = 'checkbox';
  label.append(supported, document.createTextNode('×で全停止し、機体を支持しています'));
  const start = document.createElement('button');
  start.id = 'leg-measure-start';start.type = 'button';start.textContent = '位置測定開始（動かさない）';
  const stop = document.createElement('button');
  stop.id = 'leg-measure-stop';stop.type = 'button';stop.textContent = '位置測定終了';
  const action = document.createElement('p');action.id = 'leg-measure-action';
  action.textContent = 'Shareは不要。無理なく手で上下させて測定。位置指令・トルクON・設定書込みは送りません。';
  const values = document.createElement('div');values.id = 'leg-measure-values';
  details.append(summary, label, start, stop, action, values);section.appendChild(details);
  async function request(begin) {
    if (begin && !supported.checked) {
      action.textContent = '開始しません：全停止・支持を確認してチェックしてください。';
      return;
    }
    start.disabled = stop.disabled = true;
    try {
      const response = await fetch(`/legs/measure/${begin ? 'start' : 'stop'}`, { method: 'POST', cache: 'no-store' });
      if (!response.ok) throw Error(`HTTP ${response.status}`);
      action.textContent = `${begin ? '開始' : '終了'}要求を送信。受理・拒否と値は診断配信の結果を確認してください。`;
    } catch (error) { action.textContent = `要求失敗：${error.message}（自動再試行なし）`; }
    finally { start.disabled = stop.disabled = false; }
  }
  start.onclick = () => request(true);stop.onclick = () => request(false);
  const nodes = [];
  for (let i = 0; i < 5; i++) {
    const node = document.createElement('p');values.appendChild(node);nodes.push(node);
  }
  return nodes;
}
function renderLegMeasurement(nodes, packet, fresh) {
  const lines = legMeasurementLines(packet, fresh);
  nodes.forEach((node, i) => {
    const text = lines[i] || '';
    if (node.textContent !== text) node.textContent = text;
  });
}
