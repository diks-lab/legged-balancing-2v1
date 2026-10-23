'use strict';
const LEG_RX_ISSUES = [
  [1, 'ヘッダー'], [2, 'ID'], [4, '長さ'],
  [8, '状態'], [16, 'チェックサム'], [32, '受信タイムアウト']
];
function legReplyPresentation(record) {
  if (!Array.isArray(record) || record.length !== 16 || !record[6]) return null;
  const [ms, elapsed, target, reg, requested, received, issues, size, expected,
    echoes, skipped, truncated, rawHex, frameHex] = record;
  const bytes = text => typeof text === 'string' ? (text.match(/[0-9A-F]{2}/g) || []) : [];
  const frame = bytes(frameHex);
  const hex = x => Number.isInteger(x) ? `0x${x.toString(16).toUpperCase().padStart(2, '0')}` : '未受信';
  const actual = i => i < frame.length ? hex(parseInt(frame[i], 16)) : '未受信';
  const failures = LEG_RX_ISSUES.filter(([bit]) => issues & bit).map(([, name]) => name).join('・');
  const side = target === 1 ? '右' : target === 2 ? '左' : '要求先不明';
  const header = issues & 1 ? bytes(rawHex).slice(0, 2) : frame.slice(0, 2);
  const complete = record[15] === 1;
  return {
    ms, summary: `${side}：${failures}`,
    fields: {
      record: `失敗記録 ${ms} ms（現在状態とは別の履歴）`,
      stage: failures,
      target: `${side} ID=${target} ／ レジスター=${hex(reg)} ／ 要求=${requested} B`,
      header: `期待 FF FF ／ ${issues & 1 ? '受信先頭' : '候補'} ${header.join(' ') || '未受信'}`,
      id: `期待 ${hex(target)} ／ 取得 ${actual(2)}`,
      length: `期待 0x04（全8 B） ／ 取得 ${actual(3)}`,
      status: `期待 0x00 ／ 取得 ${actual(4)}${complete ? '' : '（候補未確定）'}`,
      checksum: `計算期待 ${complete ? hex(expected) : '未算出'} ／ 取得 ${complete ? hex(record[14]) : '未受信／位置未確定'}`,
      counts: `候補 ${size}/8 B ／ 総受信 ${received} B（エコー・先行データを含む）`,
      timing: `要求から ${elapsed} ms ／ 全体期限10 msを維持`,
      echo: `完全一致エコー ${echoes}${echoes === 255 ? '以上' : ''} 回 ／ 境界前の破棄 ${skipped} B`,
      raw: `受信順の生データ：${bytes(rawHex).join(' ') || '未受信'}${truncated ? '（32 Bで記録打切り）' : '（取得済み範囲・打切りなし）'}`,
      frame: `失敗候補の先頭8 B：${frame.join(' ') || '未受信'}`
    }
  };
}
function createLegReplyPanel(section) {
  const details = document.createElement('details');
  details.id = 'leg-reply';
  const summary = document.createElement('summary');
  summary.textContent = '位置応答の失敗詳細（直近1件）';
  const node = document.createElement('div');
  details.append(summary, node);
  section.appendChild(details);
  const nodes = {};
  for (const key of ['record', 'stage', 'target', 'header', 'id', 'length', 'status',
    'checksum', 'counts', 'timing', 'echo', 'raw', 'frame']) {
    const p = document.createElement('p');
    p.id = `leg-rx-${key}`;
    node.appendChild(p);
    nodes[key] = p;
  }
  return nodes;
}
function renderLegReply(nodes, record, fresh) {
  const view = legReplyPresentation(record);
  for (const [key, node] of Object.entries(nodes)) {
    let text = view ? view.fields[key] : key === 'record' ? '位置応答の失敗記録なし' : '未取得';
    if (key === 'record' && !fresh) text += ' ／ 通信停止・最終受信時の情報';
    if (node.textContent !== text) node.textContent = text;
  }
}
