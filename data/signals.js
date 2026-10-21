'use strict';
const SIGNALS = [
  ['angle_deg', '姿勢角', 'deg'],
  ['reference_deg', '基準角', 'deg'],
  ['angle_error_deg', '基準角との差', 'deg'],
  ['gyro_deg_s', '機体角速度', 'deg/s'],
  ['right_position_rad', '右位置', 'rad'],
  ['left_position_rad', '左位置', 'rad'],
  ['right_velocity_rad_s', '右車輪速度', 'rad/s'],
  ['left_velocity_rad_s', '左車輪速度', 'rad/s'],
  ['mean_velocity_rad_s', '制御平均速度', 'rad/s'],
  ['angle_term_v', '姿勢項', 'V'],
  ['gyro_term_v', '角速度項', 'V'],
  ['position_term_v', '位置項', 'V'],
  ['speed_term_v', '速度項（補正済み）', 'V'],
  ['total_output_v', '合計出力', 'V'],
  ['right_command_v', '右指令電圧', 'V'],
  ['left_command_v', '左指令電圧', 'V'],
  ['right_foc_q_v', '右FOC q電圧', 'V'],
  ['left_foc_q_v', '左FOC q電圧', 'V'],
  ['control_dt_ms', '制御周期', 'ms'],
  ['loop_us', 'ループ時間', 'us'],
  ['right_foc_us', '右FOC時間', 'us'],
  ['left_foc_us', '左FOC時間', 'us'],
  ['imu_us', 'IMU時間', 'us'],
  ['move_us', 'move時間', 'us'],
  ['internal_free_b', '内部空きRAM', 'B'],
  ['internal_largest_b', '内部最大連続領域', 'B'],
  ['control_angle_error_deg', '制御角度誤差', 'deg'],
  ['position_error_rad', '基準位置誤差', 'rad'],
  ['target_speed_rad_s', '目標速度', 'rad/s'],
  ['target_tilt_deg', '目標傾斜角', 'deg'],
  ['yaw_command_v', '旋回指令（制限後）', 'V'],
  ['speed_raw_rad_s', '速度LPF前（比較用）', 'rad/s']
];

const PERMANENT = ['angle_deg', 'reference_deg', 'right_command_v', 'left_command_v'];
const DEFAULTS = ['gyro_deg_s'];
const CONTROL_FIELDS = [
  'control_angle_error_deg', 'gyro_deg_s', 'mean_velocity_rad_s',
  'right_velocity_rad_s', 'left_velocity_rad_s',
  'right_command_v', 'left_command_v'
];
const PRESETS = {
  stationary: DEFAULTS,
  control: CONTROL_FIELDS,
  position: [...CONTROL_FIELDS, 'position_error_rad'],
  drive: [
    'control_angle_error_deg', 'gyro_deg_s', 'target_speed_rad_s',
    'mean_velocity_rad_s', 'target_tilt_deg', 'yaw_command_v',
    'right_velocity_rad_s', 'left_velocity_rad_s',
    'right_command_v', 'left_command_v'
  ]
};
// Stable by field, independent of selection order; left/right never share a color.
const COLORS = [
  '#4263eb', '#9c36b5', '#e8590c', '#087f5b', '#6741d9', '#a61e4d',
  '#1864ab', '#c2255c', '#5c940d', '#d9480f', '#1098ad', '#862e9c',
  '#a15c00', '#343a40', '#f08c00', '#0b7285', '#364fc7', '#c92a2a',
  '#2b8a3e', '#5f3dc4', '#c05621', '#099268', '#b08900', '#495057',
  '#0c8599', '#ae3ec9', '#7048e8', '#845ef7', '#37b24d', '#d6336c',
  '#15aabf', '#868e96'
];
if (typeof module !== 'undefined') {
  module.exports = { SIGNALS, DEFAULTS, PRESETS, COLORS, PERMANENT };
}
