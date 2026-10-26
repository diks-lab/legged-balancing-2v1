'use strict';
function legAcceptanceLines(packet) {
  const a = packet?.leg_accept, g = packet?.leg_geometry;
  if (!Array.isArray(a) || a.length !== 8) return ['受付設定：未受信'];
  const known = Array.isArray(g) && g.length === 3;
  const phases = ['通常操作', '通常駆動範囲外の位置保持・左中央待ち', '手動で通常範囲へ戻す'];
  return [
    `初回受付：${a[0] ? '確認済設定で有効' : '通常範囲のみ'} ／ 設定整合：${a[1] ? '○' : '×'}`,
    `初回受付：右 ${a[2]}～${a[3]} tick ／ 左 ${a[4]}～${a[5]} tick`,
    known ? `通常駆動：右 ${g[0][6]}～${g[0][7]} ／ 左 ${g[1][6]}～${g[1][7]} tick（${g[2][0]}～${g[2][1]}%と端余裕の共通範囲）` : '通常駆動範囲：未受信',
    known ? `実測0%／100%：右 ${g[0][0]}／${g[0][1]}、左 ${g[1][0]}／${g[1][1]} tick。%はサーボ位置の可動量で、脚長・機体高さとの比例は未確認。` : '実測端：未受信',
    known ? `端余裕（最縮／最伸）：右 ${g[0][2]}／${g[0][3]}、左 ${g[1][2]}／${g[1][3]} tick・暫定未検証。余裕内側：右 ${g[0][4]}～${g[0][5]}、左 ${g[1][4]}～${g[1][5]} tick。` : '端余裕：未受信',
    `状態：${phases[a[6]] || '不明'} ／ 左中央後の戻し方向：↑${a[7] & 1 ? '○' : '×'}・↓${a[7] & 2 ? '○' : '×'}`,
    '初回は取得tickをそのまま保持。最縮側は↑、伸長側は↓から戻す。左右の要求方向が矛盾すれば移動せず、片側が進行方向の端なら両脚を停止。自動移動・姿勢補正なし。'
  ];
}
function legConfigurationLines(packet) {
  const c = packet?.leg_config;
  if (!Array.isArray(c) || c.length !== 11) return ['サーボ設定：未受信'];
  const [start, finish, rejected, status, error, step, acquired, current, rejectReason, right, left] = c;
  const errors = { 5: '停止操作で中断', 8: '応答／全体期限切れ',
    9: 'ID・長さ・状態・チェックサム検証失敗', 10: 'UART送信失敗', 12: '送信領域不足（全体期限）' };
  const result = [`読取り：${['未実行', '実行中', '正常完了', '失敗／中断'][status] || '不明'} ／ 段階 ${step}/4 ／ 開始 ${start}・完了 ${finish} ms`,
    `読取り失敗理由：${error ? errors[error] || LEG_BLOCKERS[error] || error : 'なし'} ／ 最近の要求拒否 ${rejected || 'なし'} ms（${rejectReason ? LEG_BLOCKERS[rejectReason] || rejectReason : 'なし'}）`];
  for (const [side, data] of [[0, right], [1, left]]) {
    if (!Array.isArray(data) || data.length !== 5) continue;
    const limits = side * 2, mode = limits + 1, name = side ? '左 ID2' : '右 ID1';
    const label = bit => current & (1 << bit) ? '今回取得' : '前回取得・今回未取得';
    result.push(`${name} 角度制限（9～12番・符号なしLE）：` +
      (acquired & (1 << limits) ? `${data[2]}～${data[3]} tick・${label(limits)} @ ${data[0]} ms` : '未取得'));
    const meaning = data[4] === 0 ? '位置モード' : data[4] === 1 ? '車輪モード' : '名称未確認';
    result.push(`${name} 動作モード（33番）：` +
      (acquired & (1 << mode) ? `${data[4]}（${meaning}）・${label(mode)} @ ${data[1]} ms` : '未取得'));
  }
  result.push('読取り専用。サーボの角度制限は機構の安全範囲を保証せず、初回受付へ自動適用しません。');
  return result;
}
function createLegConfigPanel(section) {
  const details = document.createElement('details');
  details.id = 'leg-config';
  const summary = document.createElement('summary');
  summary.textContent = '初回受付範囲・サーボ設定（読取り専用）';
  const button = document.createElement('button');
  button.id = 'leg-read-config';button.type = 'button';
  button.textContent = 'サーボ設定を読む（動かさない）';
  const request = document.createElement('p');request.id = 'leg-config-request';
  request.textContent = '車輪・脚を停止して使用。Share操作は不要です。';
  const content = document.createElement('div');content.id = 'leg-config-values';
  details.append(summary, button, request, content);section.appendChild(details);
  button.onclick = async () => {
    button.disabled = true;
    try {
      const response = await fetch('/legs/read-config', { method: 'POST', cache: 'no-store' });
      if (!response.ok) throw Error(`HTTP ${response.status}`);
      request.textContent = '読取要求を送信。配信中の診断で受理・拒否・取得結果を確認してください。';
    } catch (error) { request.textContent = `読取要求失敗：${error.message}（自動再試行なし）`; }
    finally { button.disabled = false; }
  };
  const nodes = [];
  for (let i = 0; i < 15; i++) {
    const p = document.createElement('p');content.appendChild(p);nodes.push(p);
  }
  return nodes;
}
function renderLegConfig(nodes, packet, fresh) {
  const lines = [...legAcceptanceLines(packet), ...legConfigurationLines(packet)];
  lines.push(fresh ? '最新の脚診断情報' : '通信停止／配信停止・最終受信情報。現在状態は不明');
  nodes.forEach((node, i) => {
    const text = lines[i] || '';
    if (node.textContent !== text) node.textContent = text;
  });
}

// Configuration display only; no control value or received sample is changed.
function legMotionProfile(packet) {
  const rate = Number.isFinite(packet?.leg_rate) ? packet.leg_rate : '未取得';
  const g = packet?.leg_geometry;
  let range = '通常範囲未取得';
  if (Array.isArray(g) && g.length === 3) {
    const side = (values, right) => {
      const percent = tick => 100 * (tick - values[0]) / (values[1] - values[0]);
      const low = percent(values[right ? 7 : 6]), high = percent(values[right ? 6 : 7]);
      return `${low.toFixed(2)}～${high.toFixed(2)}%（${values[6]}～${values[7]} tick）`;
    };
    range = `通常範囲：右 ${side(g[0], true)} ／ 左 ${side(g[1], false)}`;
  }
  return `上下最大 ${rate} ポイント/秒 ／ ${range}。サーボ可動量の割合。`;
}

// READ feedback is diagnostic only. UART write success is not arrival feedback.
function legFollowing(packet, fresh, elapsed) {
  const f = packet?.leg_follow, l = packet?.legs;
  if (!Array.isArray(f) || f.length !== 6 || !Array.isArray(l)) return '追従位置：未取得';
  const now = ((packet.leg_ms || 0) + elapsed) >>> 0;
  return [0, 1].map(side => {
    const p = f[4 + side], sent = l[15] ? l[6 + side] : null;
    const name = side ? '左' : '右';
    if (!Array.isArray(p) || p.length !== 7) return `${name}：追従診断不正・現在値不明`;
    let status = ['未取得', '読取り中', '正常取得', '読取り失敗', '停止・切断後の最終値', '送信空き不足', 'READ送信失敗'][p[0]] || '不明';
    const valid = fresh && p[0] === 2 && ((now - p[2]) >>> 0) <= 1500;
    if (p[0] === 2 && !valid) status = '古い取得値・現在値不明';
    const reasons = LEG_RX_ISSUES.filter(([bit]) => p[6] & bit).map(([, name]) => name).join('・') || 'なし';
    const calc = f[0] ? f[1 + side] : '未計算';
    const error = valid && Number.isFinite(sent) ? p[1] - sent : '不明';
    return `${name}：計算 ${calc} ／ UART指令 ${sent ?? '未送信'} ／ 実位置 ${valid ? p[1] : '現在値不明'} tick` +
      `\n${status}${!fresh ? '・通信停止／最終受信情報' : ''} ／ 取得 ${p[2]} ms ／ READ ${p[3]} ms ／ ${reasons}（${p[6]}）` +
      `\n最終取得 ${p[2] ? p[1] : '未取得'} tick ／ 差（実位置−最新UART指令）${error} tick ／ READ時指令 ${p[4]}（${p[5]} ms）。到達・受理保証なし。`;
  }).join('\n');
}
