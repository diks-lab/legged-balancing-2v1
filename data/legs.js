'use strict';
const LEG_REASONS = [
  '十字キー解放待ち', '操作待ち', '左右実位置を非同期確認中', '停止／静止倒立で操作を受理',
  '走行・減速・走行復帰中は脚操作を停止', '停止・再操作前に位置同期',
  'IMU左右姿勢が無効', '実位置が0～30%相当外・動作拒否',
  'サーボ位置応答タイムアウト・動作拒否', '応答ID／長さ／状態／チェックサム不正',
  'UART送信失敗・動作拒否', '更新間隔超過・解放して再操作', 'UART送信余裕待ち',
  '車輪通信の再試行中・脚指令を保持',
  '左右操作不可：方向未確認。上下のみ操作可能',
  '操作不可：必須初期化未完了／起動異常（詳細理由を参照）',
  '操作不可：起立／開始待ち。支持試験は×→全解放→Share',
  '操作不可：明示停止。支持・全解放を確認してShareでゼロ指令有効化',
  '操作不可：FOC校正中', '操作不可：車輪極性試験中'
];
function legLines(packet, live, sampleMs) {
  const l = packet?.legs;
  if (!Array.isArray(l) || l.length !== 19) return ['脚部：診断未受信'];
  const fresh = live && ((sampleMs - packet.leg_ms) >>> 0) <= 1500;
  const n = (x, unit) => Number.isFinite(x) ? `${x.toFixed(1)} ${unit}` : '不明';
  const torque = ['未確認', 'OFF送信待ち', 'OFF指令送信済（応答未確認）',
    'ON指令送信済（応答未確認）'][l[11]] || '不明';
  const mode = ['操作不可', '支持した停止試験（倒立OFF）', '静止倒立中'][packet.leg_mode] || '状態未確認';
  const confirmed = packet.leg_roll_confirmed === true;
  const rawAngleX = Number.isFinite(l[3]) && Math.abs(packet.leg_roll_sign) === 1 ?
    l[3] / packet.leg_roll_sign : null;
  return [
    `脚試験状態：${fresh ? mode : '現在不明'} / 左右姿勢補正：${confirmed ? '確認済設定で有効' : '無効（方向未確認・上下のみ）'}`,
    fresh ? `脚操作：${LEG_REASONS[l[10]] || '不明'}` : '脚部：通信停止／最終受信時の情報・現在状態は不明',
    `動作用同期：${l[13] ? '確認済' : '未同期／再確認待ち'} @ ${l[9]} ms / トルク：${torque}`,
    `高さ指令：${l[13] ? n(l[0], '%') : '未同期'} / 目標左右傾斜（左＋）：要求 ${n(l[1], '°')}・適用 ${n(l[2], '°')}`,
    `AngleX：${n(rawAngleX, '°')}（既存IMU値） / 左＋換算：${confirmed ? n(l[3], '°') : '未確認・駆動に使用しません'}`,
    `自動脚制御の最終送信指令（実位置ではありません）：左 ${l[15] ? n(l[5], '%') : '自動指令なし'}・位置 ${l[15] ? l[7] : '不明'} / ` +
      `右 ${l[15] ? n(l[4], '%') : '自動指令なし'}・位置 ${l[15] ? l[6] : '不明'} @ ${l[8]} ms`,
    `可動制限：${l[14] ? 'あり（高さを変えず左右差を制限）' : 'なし'} / ` +
      `サーボ送信 ${l[16]} 回・送信余裕不足 ${l[17]} 回・処理最大 ${l[18]} us`
  ];
}
function renderLegDiagnostics(view, current) {
  let node = view.el('leg-diagnostics');
  if (!node) {
    const section = document.createElement('section');
    const title = document.createElement('h2');
    title.textContent = '脚部操作・状態（停止／静止倒立用）';
    const guide = document.createElement('p');
    guide.textContent = '十字キー ↑伸長・↓短縮、離すと高さ保持。方向確認設定が有効な場合は←左傾斜・→右傾斜、離すと目標0°へ。' +
      '対向キー同時押しはその指令を保持。接続後は全解放確認。' +
      '左スティック前後進・右スティック旋回・×全停止は従来どおり。' +
      '外部電源での支持試験は×停止後に全解放、Shareで車輪ゼロ指令有効化（倒立開始ではありません）。' +
      '初期設定では上下のみ。左右操作・水平補正は符号と伸縮方向を確認して設定を有効にするまで拒否します。' +
      '脚は初回／転倒後に実位置同期し、0～30%相当外なら動作を拒否。' +
      '走行中は脚指令を保持し、停止後は十字キー解放・再操作が必要です。' +
      'PS4切断では高さ保持、動作中の傾斜目標はゆっくり0°へ。HOMEには移動しません。';
    node = document.createElement('div');
    node.id = 'leg-diagnostics';
    section.append(title, guide, node);
    view.el('details').querySelector('.explanation').appendChild(section);
  }
  const lines = legLines(view.legPacket, current, view.sample.sample_ms);
  while (node.children.length > lines.length) node.lastChild.remove();
  while (node.children.length < lines.length) node.appendChild(document.createElement('p'));
  lines.forEach((line, i) => { node.children[i].textContent = line; });
}
