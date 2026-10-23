'use strict';
const DIAG_BARRIERS = [
  'なし', '校正監視タイマー初期化失敗', '車輪センサー初期化失敗',
  '右AS5600応答なし', 'MPU6050応答なし', 'IMU設定確認失敗', 'IMU校正失敗',
  '保存FOC校正が必要', '右ドライバー初期化失敗', '右モーター初期化失敗',
  '右FOC初期化失敗', '左AS5600応答なし', '左ドライバー初期化失敗',
  '左モーター初期化失敗', '左FOC初期化失敗', 'FOC後の車輪センサー確認失敗',
  'Web等の必須初期化未完了', '校正中断', '校正値保存失敗'
];
const DIAG_FIRST = {
  RIGHT_FOC_NOT_RUN: '右FOC前提不足（未実行）', LEFT_FOC_NOT_RUN: '左FOC前提不足（未実行）',
  FOC_ALIGNMENT_TIMEOUT: 'FOC監視タイムアウト（左右はFOC詳細参照）',
  WIFI_TIMEOUT: 'Wi-Fi接続タイムアウト', HTTP_LISTEN: 'HTTP待受開始失敗',
  I2C0_RESOURCES_FAILED: '右I2C資源確保失敗', I2C1_RESOURCES_FAILED: '左I2C資源確保失敗',
  IMU_CONFIG_FAILED: 'IMU設定確認失敗', MPU6050: 'MPU6050応答なし',
  IMU_CALIBRATION_FAILED: 'IMU校正失敗', IMU_I2C_FAILED: 'IMU測定I2C失敗',
  IMU_NONFINITE: 'IMU測定値が非有限', IMU_LOOP_STALE: 'IMU測定更新停止',
  RIGHT_SENSOR: '右車輪センサー異常', LEFT_SENSOR: '左車輪センサー異常'
};
const MPU_FAILURES = ['未実行', '正常', '設定書込み失敗', 'レジスター指定失敗',
  '受信長不足', 'read失敗', '設定不一致'];
const FOC_FAILURES = ['詳細なし', 'センサー前提未成立', '保存校正なし',
  '監視タイマー未準備', '監視タイマー開始失敗', 'タイムアウト',
  '車輪センサー異常', '校正中断', 'initFOC戻り値0', 'PP未合格'];
function diagnosisName(reason) {
  const codes = ['NONE', 'WHEEL_ALIGN_TIMER', 'WHEEL_SENSORS', 'RIGHT_AS5600',
    'MPU6050', 'IMU_CONFIG_FAILED', 'IMU_CALIBRATION_FAILED', 'CALIBRATION_REQUIRED',
    'RIGHT_DRIVER', 'RIGHT_MOTOR', 'RIGHT_FOC', 'LEFT_AS5600', 'LEFT_DRIVER',
    'LEFT_MOTOR', 'LEFT_FOC', 'POST_FOC_SENSORS', 'WEB_PREREQUISITE',
    'CALIBRATION_ABORTED', 'CALIBRATION_SAVE_FAILED'];
  return DIAG_FIRST[reason] || DIAG_BARRIERS[codes.indexOf(reason)] || reason;
}
function healthLines(sample) {
  const d = sample.diagnosis;
  if (!Array.isArray(d) || d.length !== 18 ||
      !d.every((value, i) => i === 1 ? typeof value === 'string' : Number.isInteger(value))) return ['詳細診断：未受信'];
  const first = d[1] ? `${diagnosisName(d[1])} @ ${d[0]} ms` : '未記録';
  const lines = [`最初の異常（起動内で保持）：${first}`];
  const names = ['IMU', '右車輪センサー', '左車輪センサー', '右FOC', '左FOC'];
  for (let i = 0; i < 5; i++) {
    const state = d[i + 2];
    let text = ['未実行', '実行中', '初期化成功', '実行して失敗', '未実行'][state] || '不明';
    if (state === 4) {
      const cause = i >= 3 && d[i + 5] ? FOC_FAILURES[d[i + 5]] : DIAG_BARRIERS[d[7]];
      text += `：${cause || '前提未成立'}のため`;
    }
    if (state === 3 && i === 0) text += `：${['原因未取得', 'MPU6050応答なし', 'IMU設定確認失敗', 'IMU校正失敗', 'IMU測定I2C失敗', '非有限測定値', '測定更新停止'][d[17]] || '原因不明'}`;
    if (state === 3 && (i === 1 || i === 2)) text += `：${i === 1 ? sample.right_error : sample.left_error}`;
    if (i >= 3 && state !== 4 && d[i + 5]) text += `（${FOC_FAILURES[d[i + 5]] || '原因不明'}）`;
    lines.push(`${names[i]} ${text}`);
  }
  const hex = n => n < 0 ? '未取得' : `0x${n.toString(16).padStart(2, '0')}`;
  const regs = { 26: 'CONFIG', 25: 'SMPLRT_DIV', 27: 'GYRO_CONFIG', 28: 'ACCEL_CONFIG' };
  lines.push(`IMU設定記録（最初の失敗を保持）：${MPU_FAILURES[d[10]] || '不明'} / ` +
    `${regs[d[11]] || '未指定'}(${hex(d[11])}) 期待=${hex(d[12])} 取得=${hex(d[13])} / ` +
    `tx=${d[14]} rx=${d[15]} read=${d[16]}（−1は未実行／未取得）`);
  return lines;
}
function renderHealthDiagnostics(view, sample, current) {
  let node = view.el('health-diagnosis');
  if (!node) {
    node = document.createElement('div');
    node.id = 'health-diagnosis';
    view.el('recovery-details').after(node);
  }
  const prefix = current ? '診断情報' :
    `通信停止・最終受信時の情報（現在状態は不明／最終制御状態：${MACHINE_NAMES[sample.state] || sample.state}）`;
  const lines = [prefix, ...healthLines(sample),
    `後続状態：${sample.disabled ? '出力遮断' : '遮断フラグなし'} / ` +
    `測定：${IMU_REASONS[sample.imu_reason] || sample.imu_reason} / 最新停止履歴：${sample.stop_reason}`];
  // Fixed rows keep the first cause apart from secondary stale/disabled state.
  while (node.children.length > lines.length) node.lastElementChild.remove();
  while (node.children.length < lines.length) node.appendChild(document.createElement('p'));
  lines.forEach((line, i) => { node.children[i].textContent = line; });
}
