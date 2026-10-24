'use strict';
const LEG_SOURCES = ['未確定', 'PS4入力', '脚制御', 'UART処理', '安全判定', '停止処理', 'シリアル'];
const LEG_PHASES = ['待機', '右要求', '右応答待ち', '左要求', '左応答待ち', '位置プリロード', 'トルクON待ち', 'OFF処理', '動作中', '設定要求', '設定応答待ち'];
const LEG_OPERATIONS = ['待機・高さ保持', '伸長指令', '短縮指令', '左傾斜指令', '右傾斜指令',
  '位置確認・同期中', '停止・条件待ち', '操作失敗・解放して再操作', '十字キー解放待ち'];
function legDecision(packet, live, elapsed) {
  const s = packet?.leg_state;
  const valid = Array.isArray(s) && s.length === 17;
  const age = Math.max(0, elapsed) + (packet?.leg_age_ms || 0);
  const fresh = valid && live && age <= 1500;
  const recovery = packet?.leg_accept?.[6] || 0;
  const reason = n => n === 255 ? 'なし' : LEG_BLOCKERS[n] || `理由${n}`;
  const rx = legReplyPresentation(packet?.leg_rx);
  const blocker = valid && s[5] === 9 && rx && rx.ms === s[12] ? rx.summary : valid ? reason(s[5]) : '未受信';
  return {
    fresh,
    mode: valid ? ['操作不可', '支持試験（倒立OFF）', '静止倒立'][s[2]] || '未確定' : '未取得',
    available: !fresh ? '現在不明' : s[4] === 5 ? '同期処理中' : s[3] ?
      recovery ? '範囲へ戻す方向のみ操作可' : packet.leg_roll_confirmed ? '上下・左右操作可能' : '上下操作可能' : '操作不可／条件待ち',
    operation: fresh ? LEG_OPERATIONS[s[4]] || '未確定' : '現在不明',
    blocker: fresh ? blocker : valid ? '通信停止。現在状態は不明' : '確定状態未受信',
    sync: !valid ? '未確認' : s[4] === 5 ? '確認中' : s[4] === 7 ? '失敗' :
      s[8] & 8 ? '確認済' : '実位置確認前',
    failure: valid && s[11] !== 255 ?
      `直近操作の記録（現在状態とは別）：${reason(s[11])}（${s[12]} ms・${LEG_SOURCES[s[13]] || '未確定'}）` : '記録なし',
    note: fresh ? '脚診断自身の受信鮮度と、同じ確定状態から表示' :
      valid ? '通信停止・最終受信時の情報。現在の状態ではありません' : '脚の確定状態を未受信'
  };
}
function createLegStatePanel(section) {
  const details = document.createElement('details');
  details.id = 'leg-state-detail';
  const summary = document.createElement('summary');
  summary.textContent = '脚入力・許可判定・同期遷移（最大4件）';
  const current = document.createElement('p');
  const trace = document.createElement('pre');
  current.id = 'leg-state-current'; trace.id = 'leg-state-trace';
  trace.style.whiteSpace = 'pre-wrap';
  details.append(summary, current, trace);section.appendChild(details);
  return { current, trace };
}
function renderLegState(nodes, packet) {
  const s = packet?.leg_state;
  const gateNames = ['必須初期化', '倒立', '支持試験', '明示停止', '走行', 'IMU正常', '車輪再試行', '校正', '極性試験', 'PS4接続'];
  const keys = mask => ['↑', '↓', '←', '→'].filter((_, i) => mask & (1 << i)).join('') || '解放';
  let current = '確定状態未受信', trace = '遷移未受信';
  if (Array.isArray(s) && s.length === 17) {
    current = `確定 #${s[0]} @ ${s[1]} ms ／ 受信 ${keys(s[6])}・適用 ${keys(s[7])} ／ ` +
      `入力ゲート${s[8] & 1 ? '○' : '×'}・解放確認${s[8] & 2 ? '○' : '×'} ／ ` +
      `許可${s[3] ? '○' : '×'}・同期 ${LEG_PHASES[s[9]] || s[9]} ／ 条件bit=${s[10]} ／ ` +
      `送信待ち${s[8] & 32 ? '○' : '×'} ／ 更新元 ${LEG_SOURCES[s[15]] || s[15]}・古い遷移の上書き ${s[14]}件 ／ ` +
      gateNames.map((name, i) => `${name}${s[10] & (1 << i) ? '○' : '×'}`).join(' ');
    trace = (s[16] || []).map(e =>
      `#${e[0]} ${e[1]} ms ${LEG_SOURCES[e[2]] || e[2]}：理由 ${e[3]}→${e[4]} ／ ` +
      `同期 ${LEG_PHASES[e[5]] || e[5]}→${LEG_PHASES[e[6]] || e[6]} ／ ` +
      `受信 ${keys(e[7])}・適用 ${keys(e[8])}・入力flags=${e[9]}・条件=${e[10]}・モード=${e[11]}`
    ).join('\n') || '遷移なし';
  }
  if (nodes.current.textContent !== current) nodes.current.textContent = current;
  if (nodes.trace.textContent !== trace) nodes.trace.textContent = trace;
}
