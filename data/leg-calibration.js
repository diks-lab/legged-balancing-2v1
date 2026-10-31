'use strict';
const LEG_CAL_REASONS = {
  OK: '正常', SUPPORTED_STOP_TORQUE_OFF_REQUIRED: '機体支持・×停止・脚トルクOFFが必要',
  BUSY: '処理中。完了を確認してください', NOT_EDITING: '先に校正開始が必要',
  FOUR_POINTS_REQUIRED: '4点の取得が必要', TICK_OUT_OF_RANGE: 'tickが0～4095外',
  DIRECTION_INVALID: '右は減少・左は増加が必要。同値・逆順は保存不可',
  STROKE_OR_NORMAL_RANGE_INVALID: '可動量不足・通常駆動範囲が成立しない',
  READ_FAILED: '位置取得失敗。0として採用せず、再記録してください',
  NVS_OPEN_FAILED: 'NVS開始失敗', NVS_WRITE_FAILED: 'NVS書込み失敗・有効値維持',
  NVS_VERIFY_FAILED: 'NVS読戻し不一致・有効値維持', NOT_SAVED: '未保存・脚は要校正',
  SAVE_WORKER_UNAVAILABLE: '保存処理タスク未準備。起動結果を確認',
  NO_VALID_SLOT: '有効な保存スロットなし・脚は要校正', ABORTED: '中断・有効値維持'
};
function legCalReason(reason) { return LEG_CAL_REASONS[reason] || reason || '未取得'; }
function legCalGeometry(values) {
  const [r0, r1, l0, l1] = values;
  if (![r0, r1, l0, l1].every(Number.isInteger) ||
      Math.min(r0, r1, l0, l1) < 0 || Math.max(r0, r1, l0, l1) > 4095 ||
      r0 <= r1 || l0 >= l1) return '方向・範囲未成立';
  const rn = r0 + Math.trunc((r1 - r0) * 30 / 100);
  const ln = l0 + Math.trunc((l1 - l0) * 30 / 100);
  return `右：減少で伸長・${r0 - r1} tick、受付 ${r1}～${r0}、通常 ${Math.max(r1 + 10, rn)}～${r0 - 10} tick ／ ` +
    `左：増加で伸長・${l1 - l0} tick、受付 ${l0}～${l1}、通常 ${l0 + 10}～${Math.min(l1 - 10, ln)} tick`;
}
class LegCalibrationPanel {
  constructor() {
    const details = document.createElement('details');
    details.id = 'leg-range-calibration';
    const summary = document.createElement('summary');
    summary.textContent = '脚可動範囲の校正（動かさない・4点記録／NVS保存）';
    details.appendChild(summary);
    const note = document.createElement('p');
    note.textContent = '機体を支持して×停止。脚トルクOFFで実施。Share不要。位置指令・トルクON・自動探索なし。' +
      '保存後も停止を維持。通常操作0～30%と各端10 tickの暫定余裕は不変。%はサーボ可動量で、実長には比例しません。';
    details.appendChild(note);
    const label = document.createElement('label');
    this.support = document.createElement('input');this.support.type = 'checkbox';
    label.append(this.support, '支持・車輪停止・脚トルクOFFを確認');
    details.appendChild(label);
    this.nodes = {};
    for (const name of ['fresh', 'state', 'active', 'candidate', 'right-zero', 'right-full', 'left-zero', 'left-full', 'result', 'save']) {
      const p = document.createElement('p');p.id = `leg-cal-${name}`;
      p.style.minHeight = '1.5em';p.textContent = '未取得';
      this.nodes[name] = p;details.appendChild(p);
    }
    const reference = document.createElement('p');
    reference.textContent = '今回確定値（未保存の候補）：右0%=2044／100%=1697、左0%=2084／100%=2408 tick。端の余裕10 tickの十分性は実機未検証。';
    details.appendChild(reference);
    this.buttons = [];
    for (const [action, text] of [
      ['start', '校正開始'], ['candidate', '今回の確定4点を候補へ読込'],
      ['right-zero', '右最縮を記録'], ['right-full', '右最伸を記録'],
      ['left-zero', '左最縮を記録'], ['left-full', '左最伸を記録'],
      ['save', '4点を明示保存'], ['abort', '校正中断']
    ]) {
      const button = document.createElement('button');
      button.id = `leg-cal-button-${action}`;button.textContent = text;button.type = 'button';
      button.onclick = () => this.request(action);
      details.appendChild(button);this.buttons.push(button);
    }
    this.action = document.createElement('p');this.action.id = 'leg-cal-action';
    this.action.textContent = '×停止後、校正開始→候補読込（または各点記録）→新旧値確認→明示保存。';
    details.appendChild(this.action);
    document.getElementById('details').before(details);
  }
  async request(action) {
    if (!this.support.checked && action !== 'abort') {
      this.action.textContent = '要求しません：支持・停止・OFFを確認してください。';return;
    }
    this.buttons.forEach(button => { button.disabled = true; });
    try {
      const response = await fetch(`/legs/calibration/${action}`, { method: 'POST', cache: 'no-store' });
      if (!response.ok) throw Error(`HTTP ${response.status}`);
      this.action.textContent = `${action}要求送信済み。以下の機体側結果を確認（HTTP受理だけでは保存成功ではありません）。`;
    } catch (error) { this.action.textContent = `要求失敗：${error.message}。自動再試行なし。`; }
    finally { this.buttons.forEach(button => { button.disabled = false; }); }
  }
  receive(packet, boot) {
    if (packet.boot_id !== boot) return;
    this.packet = packet;this.at = Date.now();this.render(packet, true);
  }
  update(live, boot) {
    this.render(this.packet, live && this.packet?.boot_id === boot && Date.now() - this.at < 2500);
  }
  render(packet, fresh) {
    const set = (key, text) => {
      if (this.nodes[key].textContent !== text) this.nodes[key].textContent = text;
    };
    set('fresh', fresh ? '診断配信中' : '通信停止・最終受信時の情報（現在状態ではありません）');
    if (!packet) return;
    const phases = ['停止中', '校正中・4点編集', '校正中・位置読取り', 'NVS保存中（中断不可）', '保存完了・停止／再同期待ち', '中断・停止／再同期待ち'];
    set('state', `${phases[packet.phase] || '不明'} ／ 脚トルク指令 ${packet.torque === 2 ? 'OFF' : 'OFF未確認'} ／ 世代 ${packet.generation}・有効スロット ${packet.slots}`);
    set('active', packet.active ? `有効値（NVS）：${packet.values.join(' / ')} tick。${legCalGeometry(packet.values)}` : '有効値なし：脚駆動のみ禁止。車輪制御と読取り診断は独立。');
    set('candidate', packet.mask === 15 ? `候補（まだ有効値ではありません）：${packet.candidate.join(' / ')} tick。${legCalGeometry(packet.candidate)}` : '候補：4点の記録待ち');
    ['right-zero', 'right-full', 'left-zero', 'left-full'].forEach((key, i) => {
      const name = ['右最縮0%', '右最伸100%', '左最縮0%', '左最伸100%'][i];
      set(key, `${name}：${packet.mask & (1 << i) ? `取得済 ${packet.candidate[i]} tick ／ ${packet.imported & (1 << i) ? '確定候補読込' : '実読取'} ${packet.acquired_ms[i]} ms` : '未取得（0ではありません）'}`);
    });
    set('result', `直近結果：${legCalReason(packet.result)} ／ ${packet.event_ms} ms。NVS保存／採用結果：${legCalReason(packet.nvs_result)}`);
    set('save', `保存可否：${packet.save === 'OK' ? '保存可能（新旧を確認して明示保存）' : legCalReason(packet.save)}`);
  }
}
if (typeof module !== 'undefined') module.exports = { legCalGeometry, legCalReason };
