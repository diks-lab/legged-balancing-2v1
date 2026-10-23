'use strict';
const LEG_BLOCKERS = {
  0: '十字キーを解放してから操作',
  4: '走行・減速中。停止後に解放・再操作',
  5: '停止中。支持試験は全解放→Share',
  6: 'IMU左右姿勢が無効',
  7: '実位置が0～30%相当外。設定・機構を確認',
  8: '実位置の応答なし。接続確認後に解放・再操作',
  9: '位置応答不正。ID・通信を確認',
  10: 'UART送信失敗。解放して再操作',
  11: '更新間隔超過。解放して再操作',
  12: 'UART送信領域待ち',
  13: '車輪通信の再試行中。脚指令は保持',
  14: '左右は方向未確認。上下のみ操作可能',
  15: '必須初期化未完了。機体の異常詳細を確認',
  16: '起立待ち。支持試験は×→全解放→Share',
  17: '明示停止中。支持・全解放後にShareで有効化',
  18: 'FOC校正中',
  19: '車輪極性試験中'
};
const LEG_LABELS = {
  mode: '試験モード', available: '操作可否', correction: '左右姿勢補正',
  operation: '現在の操作', blocker: '動作を妨げる理由', sync: '位置同期',
  height: '高さ指令', roll: '左右傾斜', failure: '直近の操作失敗'
};
function legPresentation(packet, live, sampleMs, cache) {
  const l = packet?.legs;
  const valid = Array.isArray(l) && l.length === 19;
  const fresh = valid && live && ((sampleMs - packet.leg_ms) >>> 0) <= 1500;
  const n = (v, unit) => Number.isFinite(v) ? `${v.toFixed(1)} ${unit}` : '不明';
  const reason = valid ? l[10] : -1;
  if (fresh && cache.ms !== packet.leg_ms) {
    const previous = cache.height;
    cache.operation = reason === 2 ? '位置確認中' : !l[12] ? '待機' :
      Number.isFinite(previous) && l[0] > previous ? '伸長（高さ指令）' :
      Number.isFinite(previous) && l[0] < previous ? '短縮（高さ指令）' : '待機・高さ保持';
    if ([7, 8, 9, 10, 11, 14].includes(reason) && cache.reason !== reason) {
      cache.failure = `${LEG_BLOCKERS[reason]}（発生 ${packet.leg_ms} ms）`;
    }
    cache.height = l[0];
    cache.reason = reason;
    cache.ms = packet.leg_ms;
  }
  const confirmed = packet?.leg_roll_confirmed === true;
  const blocked = !fresh || packet.leg_mode === 0 || ![1, 2, 3, 14].includes(reason);
  const raw = valid && Math.abs(packet.leg_roll_sign) === 1 ? l[3] / packet.leg_roll_sign : null;
  const values = {
    mode: valid ? ['操作不可', '支持試験（倒立OFF）', '静止倒立'][packet.leg_mode] || '未取得' : '未取得',
    available: fresh ? blocked ? '操作不可／条件待ち' : confirmed ? '上下・左右操作可能' : '上下操作可能' : '現在不明',
    correction: confirmed ? '有効・確認済設定' : '無効・方向未確認',
    operation: fresh ? reason === 2 ? '位置確認中' : blocked ? '停止・条件待ち' : cache.operation || '待機' : '現在不明',
    blocker: fresh ? LEG_BLOCKERS[reason] || 'なし' : valid ? '通信停止。現在状態は不明' : '診断未受信',
    sync: !valid ? '未確認' : reason === 2 ? '確認中' : [7, 8, 9, 10].includes(reason) ? '失敗' : l[13] ? '確認済' : '未確認',
    height: valid && l[15] ? `左 ${n(l[5], '%')} ／ 右 ${n(l[4], '%')}\n最終送信指令・実位置ではありません` : '指令未送信',
    roll: valid ? `AngleX ${Number.isFinite(l[3]) ? n(raw, '°') : '不明'}\n目標 ${n(l[1], '°')} ／ 適用 ${n(l[2], '°')}` : '未取得',
    failure: cache.failure ? `過去の記録：${cache.failure}` : '記録なし（この画面で受信した範囲）'
  };
  const detail = valid ? {
    position: `最終送信位置指令：左 ${l[15] ? l[7] : '未送信'} ／ 右 ${l[15] ? l[6] : '未送信'}（実位置ではありません）`,
    timing: `指令 ${l[8]} ms ／ 同期 ${l[9]} ms ／ 診断 ${packet.leg_ms} ms`,
    counters: `送信 ${l[16]} 回 ／ 送信余裕不足 ${l[17]} 回 ／ 処理最大 ${l[18]} us`,
    torque: `トルク：${['未確認', 'OFF送信待ち', 'OFF指令送信済', 'ON指令送信済'][l[11]] || '不明'}（応答未確認）`,
    limits: `可動制限：${l[14] ? 'あり' : 'なし'} ／ 共通高さ目標 ${n(l[0], '%')}`
  } : { position: '未取得', timing: '未取得', counters: '未取得', torque: '未取得', limits: '未取得' };
  return { values, detail, fresh, note: fresh ? '最新受信情報（操作表示は1 Hzの指令変化から判定）' :
    valid ? '通信停止・最終受信時の情報。現在の状態ではありません' : '診断未受信' };
}
function createLegPanel(view) {
  const make = (tag, text, parent, id) => {
    const node = document.createElement(tag);
    if (text) node.textContent = text;
    if (id) node.id = id;
    parent.appendChild(node);
    return node;
  };
  const section = make('section', '', view.el('details').querySelector('.explanation'), 'leg-diagnostics');
  make('h2', '脚部操作・状態', section);
  const note = make('p', '', section, 'leg-freshness');
  const fields = make('dl', '', section);
  fields.className = 'leg-fields';
  const nodes = {};
  for (const [key, label] of Object.entries(LEG_LABELS)) {
    const row = make('div', '', fields);
    row.className = `leg-field leg-${key}`;
    make('dt', label, row);
    nodes[key] = make('dd', '', row, `leg-${key}`);
  }
  const details = make('details', '', section);
  make('summary', '脚部詳細（指令・時刻・通信）', details);
  const detail = {};
  for (const key of ['position', 'timing', 'counters', 'torque', 'limits']) {
    detail[key] = make('p', '', details);
  }
  const guide = make('details', '', section);
  make('summary', '操作説明・支持試験の手順', guide);
  const list = make('ul', '', guide);
  for (const text of [
    '上下：↑で伸長、↓で短縮。解放すると高さ保持。対向同時押しはその軸を保持。',
    '左右：方向確認設定が有効な場合のみ←左傾斜・→右傾斜。解放で目標0°へ戻す。',
    '停止：×は車輪全停止・脚トルクOFF。走行と脚操作の同時使用は対象外。',
    '支持試験：外部電源のまま機体を支持。×→全解放→Share→Share解放。倒立は開始しません。',
    '初回：実位置を非同期確認。範囲外・応答失敗では動作拒否。HOMEへ自動移動しません。',
    '方向確認：初期設定は上下のみ。AngleXの符号と伸縮方向を実測してから左右補正を有効化。',
    '再接続・転倒後：解放・再操作から位置を再確認。未確認の古い目標へ戻しません。'
  ]) make('li', text, list);
  return { nodes, detail, note, cache: {} };
}
function renderLegDiagnostics(view, current) {
  const ui = view.legUI || (view.legUI = createLegPanel(view));
  if (ui.boot !== view.sample.boot_id) {
    ui.cache = {};
    ui.boot = view.sample.boot_id;
  }
  const state = legPresentation(view.legPacket, current, view.sample.sample_ms, ui.cache);
  const update = (node, text) => { if (node.textContent !== text) node.textContent = text; };
  update(ui.note, state.note);
  for (const [key, text] of Object.entries(state.values)) update(ui.nodes[key], text);
  for (const [key, text] of Object.entries(state.detail)) update(ui.detail[key], text);
  const blocked = String(state.fresh && state.values.blocker !== 'なし');
  if (ui.nodes.blocker.dataset.blocked !== blocked) ui.nodes.blocker.dataset.blocked = blocked;
}
