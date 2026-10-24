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
      length: `期待 ${hex(requested + 2)}（全${requested + 6} B） ／ 取得 ${actual(3)}`,
      status: `期待 0x00 ／ 取得 ${actual(4)}${complete ? '' : '（候補未確定）'}`,
      checksum: `計算期待 ${complete ? hex(expected) : '未算出'} ／ 取得 ${complete ? hex(record[14]) : '未受信／位置未確定'}`,
      counts: `候補 ${size}/${requested + 6} B ／ 総受信 ${received} B（エコー・先行データを含む）`,
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
  const positionDetails = document.createElement('details');
  positionDetails.id = 'leg-position';
  const positionTitle = document.createElement('summary');
  positionTitle.textContent = '実位置確認（取得値・変換・操作範囲）';
  positionDetails.appendChild(positionTitle);
  section.appendChild(positionDetails);
  const position = {};
  for (const key of ['attempt', 'right', 'left', 'limits']) {
    const text = document.createElement('p');
    text.id = `leg-position-${key}`;
    positionDetails.appendChild(text);
    position[key] = text;
  }
  return { rx: nodes, position };
}
function legPositionPresentation(record) {
  if (!Array.isArray(record) || record.length !== 6) return {
    attempt: '実位置確認の診断未受信', right: '右：未取得', left: '左：未取得',
    limits: '機械的可動限界：未設定。0～30%は操作制限です。'
  };
  const [attempt, readOnly, acquired, current, right, left] = record;
  const hex = x => Number.isInteger(x) ? x.toString(16).toUpperCase().padStart(2, '0') : '不明';
  const side = (row, index, name) => {
    if (!Array.isArray(row) || row.length !== 11) return `${name}：未取得`;
    const [ms, word, low, high, decoded, percent, min, max, inside, home, extended] = row;
    const range = `操作判定の整数上下限 ${min}～${max} tick（0～30%）`;
    if (!(acquired & (1 << index))) return `${name}：未取得 ／ ${range}`;
    const pct = Number.isFinite(percent) ? percent.toFixed(6) : '不明';
    return `${name}：${current & (1 << index) ? '今回取得' : '前回取得・今回未取得'} @ ${ms} ms
` +
      `実位置 ${decoded} tick ／ 生データL,H=${hex(low)} ${hex(high)} ／ 結合word=${word}
` +
      `換算 ${pct}%（制限前） ／ ${range} ／ ${inside ? '操作範囲内' : '操作範囲外・駆動拒否'}
` +
      `換算係数 HOME=${home}、100%=${extended}（機械的限界ではありません）`;
  };
  return {
    attempt: `確認開始 ${attempt} ms ／ ${readOnly ? '読取り専用診断' : '操作開始時の実位置同期'}（指令値ではありません）`,
    right: side(right, 0, '右'), left: side(left, 1, '左'),
    limits: '機械的可動限界：未設定・未確認。初回同期にも操作範囲を適用する現行の拒否条件を維持。' +
      '整数への既存換算で境界の%値に端数が出ます。表示値を0～30%へ丸めません。'
  };
}
function renderLegReply(groups, record, fresh, positions) {
  const nodes = groups.rx;
  const view = legReplyPresentation(record);
  for (const [key, node] of Object.entries(nodes)) {
    let text = view ? view.fields[key] : key === 'record' ? '位置応答の失敗記録なし' : '未取得';
    if (key === 'record' && !fresh) text += ' ／ 通信停止・最終受信時の情報';
    if (node.textContent !== text) node.textContent = text;
  }
  const position = legPositionPresentation(positions);
  if (!fresh) position.attempt += ' ／ 通信停止・最終受信時の情報';
  for (const [key, node] of Object.entries(groups.position)) {
    if (node.textContent !== position[key]) node.textContent = position[key];
  }
}
