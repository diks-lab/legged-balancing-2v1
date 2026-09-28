#include <Arduino.h>
#include <SimpleFOC.h>
#include <MPU6050_tockn.h>
#include <PS4Controller.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#include "Servo_STS3032.h"
#include "ssid.h"

namespace {
//==================================================
// Hardware / Control Objects
//==================================================
constexpr uint8_t motor1EnablePin = 27;

constexpr uint8_t motor2EnablePin = 32;
constexpr uint8_t ledPin = 22;
constexpr uint8_t kRightLegServoId = 1;
constexpr uint8_t kLeftLegServoId = 2;
constexpr int16_t kRightLegHomePosition = 2061;
constexpr int16_t kRightLegFullyExtendedPosition = 1892;
constexpr int16_t kRightLegTestExtend10PercentPosition = 2044;
constexpr uint16_t kRightLegTestSpeed = 150;
constexpr uint8_t kRightLegTestAcceleration = 15;
constexpr int16_t kLeftLegHomePosition = 2026;
constexpr int16_t kLeftLegFullyExtendedPosition = 2208;
constexpr int16_t kLeftLegTestExtend10PercentPosition = 2044;
constexpr uint16_t kLeftLegTestSpeed = 150;
constexpr uint8_t kLeftLegTestAcceleration = 15;
constexpr uint16_t kBothLegsTestSpeed = 150;
constexpr uint8_t kBothLegsTestAcceleration = 15;

constexpr uint8_t kAs5600Address = 0x36;
constexpr uint8_t kMpu6050Address = 0x68;
constexpr float kWheelDiameterM = 0.058f;
constexpr float kWheelRadiusM = kWheelDiameterM / 2.0f;
constexpr float kTestVoltage = 0.5f;
constexpr float kLqrTestVoltageLimit = 1.0f;
constexpr float kLqrDriveTestSpeed = 0.30f;
constexpr float kLqrDriveStartAngleLimitDeg = 0.30f;
constexpr float kLqrDriveStartGyroLimitDegPerSec = 2.0f;
constexpr float kLqrDriveStartSpeedLimit = 0.15f;
constexpr size_t kLqrDriveStartSpeedAverageSamples = 32;
constexpr uint32_t kLqrDriveStartAngleReferenceTimeMs = 500;
constexpr float kLqrDriveStartAngleReferenceLimitDeg = 1.00f;
constexpr uint32_t kLqrDriveStartStableTimeMs = 200;
constexpr uint32_t kLqrDriveStartTimeoutMs = 5000;
constexpr uint32_t kLqrDriveWaitDiagIntervalMs = 200;
constexpr uint32_t kLqrSpeedSourcesDiagIntervalMs = 200;
constexpr float kLqrSpeedJumpDiagThreshold = 0.50f;
constexpr bool kDisableDirectDriveSpeedControl = true;
constexpr bool kReverseDirectDriveSpeedControl = true;

// Retain the existing direct-speed configuration for non-PS4 drive behavior
// and diagnostics.  Active PS4 driving bypasses the direct motor term below
// and uses the windowed speed feedback to command a bounded target tilt.
// Safety recovery profile: PS4 drive only asks the established balance loop
// for a very small lean.  Do not combine that lean with direct motor speed
// drive until direction has been verified on the real machine.
constexpr bool kEnablePs4LimitedDirectSpeedControl = false;
constexpr float kPs4DirectSpeedControlLimit = 0.10f;
constexpr uint32_t kPs4DriveControlDiagnosticIntervalMs = 200;
// An AS5600 count is 2*pi/4096 rad.  Over 10 ms that is about 0.153 rad/s
// per wheel (about 0.077 in the combined speed after each 0.5 direction
// factor), which explains why the diagnostic alternates between zero and
// discrete nonzero values at low speed.  A 150 ms angle window contains about
// 9.8 counts at the maximum PS4 speed of 0.10 while limiting command-feedback
// delay to 150 ms.  In comparison, 50/100/200 ms contain about
// 3.3/6.5/13.0 counts: 50 ms remains coarse, while 200 ms adds avoidable lag.
// Do not add another moving average, deadband, or LPF to this PS4-only path.
constexpr uint32_t kPs4SpeedWindowMs = 150;
constexpr size_t kPs4SpeedWindowHistorySize = 20;

// Non-PS4 automatic drive retains its established angle bias.
// If the 10 ms wheel-speed diagnostic clearly shows wrong-way motion,
// allow a small reversed speed term to help recover before the
// wrong-way emergency stop becomes necessary.
constexpr bool kEnableWrongWaySpeedAssist = false;
constexpr float kLqrDriveWrongWaySpeedAssistThreshold = 0.05f;
constexpr float kLqrDriveWrongWaySpeedAssistLimit = 0.15f;

constexpr float kLqrDriveSpeedControlGain = 1.25f;
constexpr bool kDisableDriveAngleBias = false;

constexpr float kLqrDriveAngleBiasGain = 0.125f;
constexpr float kLqrDriveAngleBiasLimitDeg = 0.10f;
// Fixed recovery-profile authority: no adaptive ceiling and no integral lean.
constexpr float kPs4DriveTargetTiltInitialLimitDeg = 0.03f;
constexpr float kPs4DriveTargetTiltSlewDegPerSec = 0.05f;
constexpr float kLqrDriveAutoDistance = 0.05f;
constexpr float kLqrDriveDecelStartDistance = 0.03f;
constexpr float kLqrDriveWrongWayLimit = 0.015f;
constexpr uint32_t kLqrDriveWrongWayGraceTimeMs = 150;
constexpr float kLqrDriveWrongWayHardLimit = 0.03f;
constexpr uint32_t kLqrDriveWrongWayNormalPersistMs = 120;
constexpr uint32_t kLqrDriveWrongWayHardPersistMs = 100;

// Drive-only recovery thresholds are intentionally well inside the actual
// fall threshold.  Crossing one of these limits removes the drive request but
// leaves the wheel drivers, balance controller, and leg holding torque alive.
constexpr float kDriveSafetyAngleErrorDeg = 2.5f;
constexpr float kDriveSafetyAngleJumpDeg = 1.0f;
constexpr float kDriveSafetyGyroDegPerSec = 25.0f;
constexpr float kDriveSafetyWheelSpeed = 6.0f;
constexpr uint32_t kDriveSafetySaturationPersistMs = 60;
constexpr uint32_t kDriveSafetySaturationReversalWindowMs = 250;
constexpr float kActualFallAngleDeg = 20.0f;
constexpr float kDriveRecoveryStableAngleDeg = 1.0f;
constexpr float kDriveRecoveryStableGyroDegPerSec = 4.0f;
constexpr float kDriveRecoveryStableSpeed = 0.08f;
constexpr uint32_t kDriveRecoveryStableTimeMs = 500;
constexpr uint32_t kDriveRecoveryPositionRampMs = 1000;
constexpr uint32_t kDriveRecoveryDiagnosticIntervalMs = 20;
constexpr uint32_t kFallConfirmTimeMs = 80;
constexpr uint32_t kFallHardConfirmTimeMs = 30;
constexpr float kFallHardAngleDeg = 35.0f;
constexpr uint32_t kFallMinimumSamples = 3;
constexpr float kFallAngleDiscontinuityDeg = 8.0f;
constexpr float kFallReferenceDiscontinuityDeg = 0.5f;
constexpr uint32_t kFallDiagnosticIntervalMs = 20;

constexpr size_t kLqrStopTraceMaxSamples = 50;
constexpr uint32_t kLqrStopTraceSampleIntervalMs = 20;
constexpr uint32_t kLqrStopTraceDurationMs = 1000;
constexpr float kLqrDriveTravelLimit = 0.70f;
constexpr float kLqrSoftStopDecelPerSec = 1.0f;
constexpr float kLqrSoftStopSpeedLimit = 0.08f;
constexpr uint32_t kLqrSoftStopStableTimeMs = 250;
constexpr uint32_t kLqrDriveSignTraceIntervalMs = 10;
constexpr size_t kLqrDriveSignTraceCapacity = 60;
constexpr float kBalanceBrakeVoltageLimit = 0.20f;
constexpr float kPolarityTestVoltage = 0.15f;
constexpr float kPolarityTestVoltageStep = 0.01f;
constexpr float kPolarityTestVoltageMin = 0.10f;
constexpr float kPolarityTestVoltageMax = 0.30f;
constexpr float kLqrTestTiltLimitDeg = 10.0f;
constexpr float kBalanceWaitAngleLimitDeg = 2.0f;
constexpr float kBalanceWaitGyroLimitDegPerSec = 2.0f;
constexpr float kBalanceWaitWheelSpeedLimit = 0.8f;
constexpr float kBalanceHoldGyroLimitDegPerSec = 4.0f;
constexpr float kBalanceResetAngleLimitDeg = 5.0f;
constexpr float kBalanceResetGyroLimitDegPerSec = 8.0f;
constexpr uint32_t kBalancePrepareTimeMs = 2000;
constexpr uint32_t kBalanceStableTimeMs = 1500;
constexpr uint32_t kStatusLedWaitBlinkMs = 500;
constexpr uint32_t kStatusLedStabilizingBlinkMs = 100;
constexpr uint32_t kBalanceWaitDiagnosticMs = 500;
constexpr uint32_t kLqrStartupDiagnosticIntervalMs = 50;
constexpr uint32_t kLqrStartupDiagnosticDurationMs = 1000;
constexpr uint32_t kLqrRunDiagnosticIntervalMs = 100;
constexpr bool kEnableLqrStartupDiagnostic = false;
constexpr bool kEnableLqrRunDiagnostic = false;
constexpr uint32_t kPolarityPulseTimeMs = 500;
constexpr uint32_t kPolarityDiagnosticIntervalMs = 500;
constexpr uint32_t kWheelVelocityDiagnosticIntervalUs = 10000;
constexpr uint32_t k10msTimingLogIntervalMs = 500;
constexpr bool kEnableWifiLogServer = true; // 机上テスト時はWifi機能を停止する。
constexpr uint32_t kWifiConnectTimeoutMs = 5000;
constexpr uint32_t kWifiStatusCheckIntervalMs = 1000;
constexpr uint32_t kPs4DiagnosticIntervalMs = 50;
constexpr uint32_t kPs4DriveTrackDiagnosticIntervalMs = 150;
constexpr uint32_t kPs4DriveStopDiagnosticIntervalMs = 200;
constexpr float kPs4DriveTargetSlewPerUpdate = 0.02f;
// Keep the first direction-verification command deliberately small.
constexpr float kPs4DriveMaxSpeed = 0.05f;
constexpr float kPs4DriveTestTravelLimit = 0.15f / kWheelRadiusM;
constexpr int kPs4DriveStickDeadzone = 10;
constexpr float kPs4LegExtensionStepPercent = 10.0f;
constexpr float kPs4LegExtensionMinPercent = 0.0f;
constexpr float kPs4LegExtensionMaxPercent = 30.0f;
// Pair the controller to this host address before the first connection.
// Keep this value stable so the controller can reconnect after a restart.
constexpr char kPs4HostMac[] = "01:02:03:04:05:06";

struct Ps4InputState {
  int8_t left_stick_x;
  int8_t left_stick_y;
  int8_t right_stick_x;
  int8_t right_stick_y;
  uint8_t l2;
  uint8_t r2;
  bool cross;
  bool circle;
  bool square;
  bool triangle;
  bool dpad_up;
  bool dpad_down;
  bool dpad_left;
  bool dpad_right;
  bool l1;
  bool r1;
  bool options;
  bool share;
  bool ps_button;
};

bool ps4_was_connected = false;
bool ps4_input_initialized = false;
uint32_t ps4_last_diagnostic_ms = 0;
uint32_t ps4_drive_target_update_last_ms = 0;
uint32_t ps4_drive_track_diagnostic_last_ms = 0;
uint32_t ps4_drive_control_diagnostic_last_ms = 0;
uint32_t ps4_drive_stop_diagnostic_last_ms = 0;
// PS4 soft-stop diagnostics only; these values never affect control.
uint32_t ps4_drive_stop_start_ms = 0;
uint32_t soft_stop_low_speed_enter_count = 0;
uint32_t soft_stop_stable_reset_count = 0;
bool soft_stop_speed_was_low = false;
float stop_speed_min = 0.0f;
float stop_speed_max = 0.0f;
float stop_speed_abs_max = 0.0f;
Ps4InputState previous_ps4_input = {};
float ps4_leg_extension_percent = kPs4LegExtensionMinPercent;
float ps4_drive_speed_request = 0.0f;
int ps4_drive_stick_y = 0;
enum class Ps4DriveState {
  Neutral,
  OutsideDeadzone,
  DriveRequested,
  Stopping,
};
Ps4DriveState ps4_drive_state = Ps4DriveState::Neutral;

WebServer wifi_log_server(80);
bool wifi_log_server_started = false;
bool mdns_started = false;
bool mdns_attempted = false;
uint32_t wifi_status_last_check_ms = 0;

BLDCMotor motor1(7);
BLDCDriver3PWM driver1(13, 12, 14, motor1EnablePin);
TwoWire I2Cone(0);
MagneticSensorI2C sensor1(AS5600_I2C);
MPU6050 mpu6050(I2Cone);

BLDCMotor motor2(7);
BLDCDriver3PWM driver2(26, 25, 33, motor2EnablePin);
TwoWire I2Ctwo(1);
MagneticSensorI2C sensor2(AS5600_I2C);

//==================================================
// Control Variables
//==================================================
float right_target_voltage = 0.0f;
float left_target_voltage = 0.0f;
bool test_system_ready = false;
bool emergency_stop_active = false;
bool driver1_initialized = false;
bool driver2_initialized = false;

PIDController pid_angle{1, 0, 0, 100000, 8};
PIDController pid_gyro{0.08, 0, 0, 100000, 8};
PIDController pid_distance{0.4, 0, 0, 100000, 8};
PIDController pid_speed{0.5, 0, 0, 100000, 8};

float m1_direction = 0.0f;
float m2_direction = 0.0f;
float angle_zeropoint = 0.0f;
float distance_zeropoint = 0.0f;
float LQR_angle = 0.0f;
float LQR_gyro = 0.0f;
float LQR_distance = 0.0f;
float LQR_speed = 0.0f;
float diag_right_velocity_10ms = 0.0f;
float diag_left_velocity_10ms = 0.0f;
float diag_LQR_speed_10ms = 0.0f;
float lqr_speed_raw_for_control = 0.0f;
float lqr_speed_for_control = 0.0f;
bool lqr_control_speed_filter_initialized = false;
float lqr_ps4_speed_for_control = 0.0f;
float lqr_ps4_window_speed = 0.0f;
uint32_t lqr_ps4_window_elapsed_ms = 0;
float lqr_ps4_window_angle_delta_right = 0.0f;
float lqr_ps4_window_angle_delta_left = 0.0f;
float lqr_ps4_target_tilt_limit_deg =
    kPs4DriveTargetTiltInitialLimitDeg;
float lqr_ps4_drive_angle_reference = 0.0f;
float lqr_ps4_target_tilt_integral_deg = 0.0f;
float lqr_ps4_target_tilt_deg = 0.0f;
struct Ps4WheelAngleSample {
  uint32_t timestamp_ms;
  float right_angle;
  float left_angle;
};
Ps4WheelAngleSample lqr_ps4_speed_history[kPs4SpeedWindowHistorySize] = {};
size_t lqr_ps4_speed_history_next = 0;
size_t lqr_ps4_speed_history_count = 0;
float lqr_target_speed = 0.0f;
float lqr_drive_angle_bias = 0.0f;
bool lqr_drive_start_pending = false;
char lqr_drive_pending_command = 0;
float lqr_drive_pending_speed = 0.0f;
uint32_t lqr_drive_start_request_ms = 0;
uint32_t lqr_drive_stable_start_ms = 0;
uint32_t lqr_drive_wait_diag_last_ms = 0;
uint32_t lqr_drive_max_stable_ms = 0;
uint32_t lqr_drive_angle_fail_count = 0;
uint32_t lqr_drive_gyro_fail_count = 0;
uint32_t lqr_drive_speed_fail_count = 0;
uint32_t lqr_drive_speed_current_fail_count = 0;
uint32_t lqr_drive_speed_control_fail_count = 0;
uint32_t lqr_drive_speed_average_fail_count = 0;
float lqr_drive_start_angle_min = 0.0f;
float lqr_drive_start_angle_max = 0.0f;
float lqr_drive_start_angle_sum = 0.0f;
uint32_t lqr_drive_start_angle_sample_count = 0;
float lqr_drive_start_angle_reference_sum = 0.0f;
uint32_t lqr_drive_start_angle_reference_count = 0;
float lqr_drive_start_angle_reference = 0.0f;
bool lqr_drive_start_angle_reference_ready = false;
float lqr_drive_start_speed_samples[
    kLqrDriveStartSpeedAverageSamples] = {};
size_t lqr_drive_start_speed_sample_index = 0;
size_t lqr_drive_start_speed_sample_count = 0;
float lqr_drive_start_speed_sum = 0.0f;
float lqr_drive_start_speed_average = 0.0f;

// Drive-start speed diagnostic.
// These values are diagnostic only and do not affect control.
float lqr_drive_start_speed_abs_sum = 0.0f;
float lqr_drive_start_speed_abs_average = 0.0f;
float lqr_drive_start_speed_abs_max = 0.0f;
uint32_t lqr_drive_start_speed_diag_count = 0;

// Drive-start speed-source diagnostic.
// Diagnostic only; these values never affect control.
constexpr uint32_t kLqrDriveSpeedSourceDiagIntervalMs = 20;
uint32_t lqr_drive_speed_source_diag_last_ms = 0;
uint32_t lqr_drive_speed_source_diag_count = 0;

float lqr_drive_sf_m1_abs_sum = 0.0f;
float lqr_drive_sf_m1_abs_max = 0.0f;
float lqr_drive_sf_m2_abs_sum = 0.0f;
float lqr_drive_sf_m2_abs_max = 0.0f;

float lqr_drive_diag_right_abs_sum = 0.0f;
float lqr_drive_diag_right_abs_max = 0.0f;
float lqr_drive_diag_left_abs_sum = 0.0f;
float lqr_drive_diag_left_abs_max = 0.0f;

float lqr_drive_control_speed_abs_sum = 0.0f;
float lqr_drive_control_speed_abs_max = 0.0f;
float lqr_drive_gate_speed_abs_sum = 0.0f;
float lqr_drive_gate_speed_abs_max = 0.0f;

float lqr_drive_speed_source_diff_abs_sum = 0.0f;
float lqr_drive_speed_source_diff_abs_max = 0.0f;

// Speed-path timestamps and snapshots are diagnostics only. They are never
// consulted by a gate, controller, safety check, or state transition.
uint32_t lqr_simplefoc_speed_read_ms = 0;
uint32_t lqr_speed_10ms_update_ms = 0;
uint32_t lqr_control_speed_update_ms = 0;
uint32_t lqr_gate_speed_read_ms = 0;
uint32_t lqr_speed_sources_diag_last_ms = 0;
float lqr_soft_stop_diag_previous_speed = 0.0f;
bool lqr_soft_stop_diag_previous_valid = false;

bool lqr_soft_stop_active = false;
uint32_t lqr_soft_stop_stable_start_ms = 0;
float lqr_soft_stop_entry_speed = 0.0f;
bool lqr_auto_drive_active = false;
float lqr_drive_start_distance = 0.0f;
float lqr_drive_origin_distance = 0.0f;
uint32_t lqr_drive_motion_start_ms = 0;
float lqr_drive_wrong_way_peak = 0.0f;
uint32_t lqr_drive_wrong_way_normal_since_ms = 0;
uint32_t lqr_drive_wrong_way_hard_since_ms = 0;
float drive_safety_previous_angle = 0.0f;
bool drive_safety_angle_initialized = false;
uint32_t drive_safety_saturation_since_ms = 0;
int8_t drive_safety_last_saturation_sign = 0;
uint32_t drive_safety_last_saturation_ms = 0;

enum class DriveRecoveryState {
  Inactive,
  AttitudeOnly,
  PositionRamp,
};
DriveRecoveryState drive_recovery_state = DriveRecoveryState::Inactive;
uint32_t drive_recovery_start_ms = 0;
uint32_t drive_recovery_stable_since_ms = 0;
uint32_t drive_recovery_position_ramp_start_ms = 0;
uint32_t drive_recovery_diagnostic_last_ms = 0;
uint32_t fall_exceeded_since_ms = 0;
uint32_t fall_exceeded_samples = 0;
uint32_t fall_diagnostic_last_ms = 0;
float fall_previous_angle_error = 0.0f;
float fall_previous_reference = 0.0f;
bool fall_previous_sample_valid = false;

struct LqrStopTraceSample {
  uint32_t elapsed_ms;
  float angle_error;
  float gyro;
  float angle_control;
  float gyro_control;
  float drive_angle_bias;
  float target_speed;
  float actual_speed;
  float speed_error;
  float speed_control;
  float speed_direct_term;
  float distance_control;
  float lqr_u;
  float right_output;
  float left_output;
  float distance_delta;
};

LqrStopTraceSample
    lqr_stop_trace[kLqrStopTraceMaxSamples];
size_t lqr_stop_trace_count = 0;
bool lqr_stop_trace_active = false;
uint32_t lqr_stop_trace_start_ms = 0;
uint32_t lqr_stop_trace_last_sample_ms = 0;
float lqr_stop_trace_origin_distance = 0.0f;

struct LqrDriveSignTraceSample {
  uint32_t elapsed_ms;
  float angle_error;
  float gyro;
  float angle_control;
  float gyro_control;
  float drive_angle_bias;
  float target_speed;
  float raw_speed;
  float actual_speed;
  float speed_error;
  float speed_control;
  float speed_direct_term;
  float lqr_u;
  float right_output;
  float left_output;
  float motor1_velocity;
  float motor2_velocity;
  float expected_motion_sign;
  float actual_motion_sign;
  bool motion_sign_ok;
  float distance_delta;
};

LqrDriveSignTraceSample
    lqr_drive_sign_trace[kLqrDriveSignTraceCapacity];
size_t lqr_drive_sign_trace_count = 0;
uint32_t lqr_drive_sign_trace_start_ms = 0;
uint32_t lqr_drive_sign_trace_last_ms = 0;
bool lqr_drive_sign_trace_active = false;

float previous_right_angle = 0.0f;
float previous_left_angle = 0.0f;
uint32_t previous_velocity_diagnostic_us = 0;
constexpr size_t kVelocityTraceSize = 128;
constexpr size_t kLqrControlTraceSize = 128;
constexpr uint32_t kLqrControlTraceIntervalUs = 10000;

struct VelocityTraceSample {
  uint32_t elapsed_ms;
  uint32_t elapsed_us;

  float current_right_angle;
  float previous_right_angle;
  float right_angle_delta;

  float current_left_angle;
  float previous_left_angle;
  float left_angle_delta;

  float diag_right_velocity_10ms;
  float diag_left_velocity_10ms;
  float diag_LQR_speed_10ms;

  float simplefoc_right_velocity;
  float simplefoc_left_velocity;
  float LQR_speed;

  bool diag_speed_spike;
};

VelocityTraceSample velocity_trace[kVelocityTraceSize];

constexpr size_t kLoopTimingTraceSize = 64;

struct LoopTimingSample {
  uint32_t elapsed_ms;
  uint32_t whole_loop_us;
  uint32_t motor1_loopfoc_us;
  uint32_t motor2_loopfoc_us;
  uint32_t mpu_update_us;
  uint32_t motor_move_us;
};

struct LqrControlTraceSample {
  uint32_t elapsed_ms;
  float angle;
  float angle_zeropoint;
  float angle_error;
  float gyro;
  float lqr_speed;
  float speed_for_control;

  float angle_control;
  float gyro_control;
  float distance_control;
  float speed_control;

  float lqr_u;
  float right_output;
  float left_output;
};

LoopTimingSample loop_timing_trace[kLoopTimingTraceSize];
size_t loop_timing_write_index = 0;
size_t loop_timing_count = 0;

LqrControlTraceSample lqr_control_trace[kLqrControlTraceSize];
size_t lqr_control_trace_write_index = 0;
size_t lqr_control_trace_count = 0;
uint32_t lqr_control_trace_last_us = 0;

size_t velocity_trace_write_index = 0;
size_t velocity_trace_count = 0;
uint32_t last_10ms_timing_log_ms = 0;
float angle_control = 0.0f;
float gyro_control = 0.0f;
float distance_control = 0.0f;
float speed_control = 0.0f;
float LQR_u = 0.0f;
float test_LQR_u = 0.0f;
float balance_brake_right = 0.0f;
float balance_brake_left = 0.0f;
bool balance_brake_enabled = true;
float control_dt_ms = 0.0f;
uint32_t last_control_update_us = 0;
bool lqr_zero_set = false;
bool lqr_diagnostic_armed = false;
bool lqr_direction_test_armed = false;
uint32_t lqr_direction_test_start_ms = 0;
enum class LqrTestMode {
  Full,
  AttitudeSpeed,
  AttitudeDistance,
  AttitudeOnly,
};
LqrTestMode lqr_test_mode = LqrTestMode::Full;

enum class LqrSpeedSource {
  SimpleFoc,
  Diagnostic10ms,
};

LqrSpeedSource lqr_speed_source = LqrSpeedSource::SimpleFoc;

// Low-pass filter applied only to the speed used by the LQR control loop.
// Keep LQR_speed itself raw so diagnostics can compare raw vs filtered.
constexpr float kLqrControlSpeedFilterAlpha = 0.15f;

bool balance_wait_active = false;
bool balance_prepare_active = false;
bool balance_stable_message_shown = false;
bool status_led_on = false;
float balance_candidate_angle = 0.0f;
float balance_angle_sum = 0.0f;
float balance_distance_sum = 0.0f;
uint32_t balance_sample_count = 0;
uint32_t balance_prepare_start_ms = 0;
uint32_t balance_stable_start_ms = 0;
uint32_t balance_stable_elapsed_ms = 0;
uint32_t status_led_last_toggle_ms = 0;
uint32_t balance_wait_last_diagnostic_ms = 0;
bool balance_timer_reset = false;
bool lqr_startup_diagnostic_active = false;
uint32_t lqr_startup_diagnostic_start_ms = 0;
uint32_t lqr_startup_diagnostic_last_ms = 0;
uint32_t lqr_run_diagnostic_last_ms = 0;
bool polarity_test_active = false;
bool polarity_pulse_active = false;
bool polarity_drive_right = false;
bool polarity_drive_left = false;
float polarity_reference_angle = 0.0f;
float polarity_test_voltage = kPolarityTestVoltage;
float polarity_pulse_voltage = 0.0f;
float polarity_right_velocity_sum = 0.0f;
float polarity_left_velocity_sum = 0.0f;
float polarity_right_velocity_peak = 0.0f;
float polarity_left_velocity_peak = 0.0f;
uint32_t polarity_velocity_sample_count = 0;
char polarity_pulse_command = '?';
float polarity_right_start_angle = 0.0f;
float polarity_left_start_angle = 0.0f;
uint32_t polarity_pulse_start_ms = 0;
uint32_t polarity_diagnostic_last_ms = 0;

//==================================================
// Function Prototypes
//==================================================
// Motor / sensor control
void setBothTargetsToZero();
void stopBothMotorsAtZero();
void disableBothDrivers();
bool i2cDeviceIsPresent(TwoWire& bus, uint8_t address);
void calibrateMpu6050();

// LQR / balance control
void updateLqrState();
void updateWheelVelocityDiagnostic();
const char* lqrSpeedSourceName();
void updateTestLqrOutput(float angle_term, float gyro_term,
                         float distance_term, float speed_term);
void updateLqrDiagnostic();
void printLqrSoftStopSpeedDiagnostic(float right_output, float left_output);
void printAndResetLqrRunTime();
bool requestLqrDrive(float target_speed, char command);
void requestLqrSoftStop();
void stopLqrDriveOnly(const char* reason, bool require_recenter);
void terminateLqrDriveSession(const char* reason, bool require_recenter);
const char* detectDriveSafetyFault(float right_output, float left_output);
bool confirmActualFall();
const char* ps4DriveStateName(Ps4DriveState state);
void setPs4DriveState(Ps4DriveState next_state);
void beginPs4DriveStopping(uint32_t now_ms);

void resetLqrDriveSignTrace();
void printLqrDriveSignTrace(const char* reason);
void resetLqrStopTrace();
void printLqrStopTrace(const char* reason);
void resetLqrTestState();
void clearVelocityTrace();
void printVelocityTrace();
void clearLoopTimingTrace();
void printLoopTimingTrace();
void updateLqrControlTrace(float right_output, float left_output,
                           bool force_sample = false);
void clearLqrControlTrace();
void printLqrControlTrace();
void updateLqrRunDiagnostic(float right_output, float left_output);

// Wi-Fi trace logger
void setupWifiLogServer();
void updateWifiLogServer();
void handleWifiStatusPage();
void handleVelocityTraceDownload();
void handleLoopTimingTraceDownload();
void handleLqrControlTraceDownload();

// Safety / state
void setStatusLed(bool on);
void cancelBalanceWait();
void updateBalanceWait();
bool motionCommandAllowed();

// Test / monitor
void setupPs4Controller();
void updatePs4Controller();
float ps4DriveSpeedRequestFromLeftStickY(int stick_y);
void printPs4InputDiagnostic(const Ps4InputState& input);
const char* ps4DriveStateName(Ps4DriveState state);
void setPs4DriveState(Ps4DriveState next_state);
void handleSerialCommand();
void printHelp();
void printLqrStatus();
void printDistanceDiagnostic();
void printSpeedDiagnostic();
void pingLegServos();
void ensureLegServoTorqueEnabled();
void disableLegServoTorqueForTestEnd(const char* reason);
void moveRightLegForTest(int16_t position);
void moveLeftLegForTest(int16_t position, const char* action);
int16_t rightLegPositionFromPercent(float percent);
int16_t leftLegPositionFromPercent(float percent);
void moveBothLegsToPercent(float percent);
void printLqrStartupDiagnostic(uint32_t elapsed_ms, float right_output,
                               float left_output);
}  // namespace

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, 16, 17);
  sms_sts.pSerial = &Serial2;
  delay(1000);
  Serial.println("\n--- SimpleFOC two-wheel + MPU6050 integration test ---");
  setupPs4Controller();

  pinMode(motor1EnablePin, OUTPUT);
  pinMode(motor2EnablePin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(motor1EnablePin, LOW);
  digitalWrite(motor2EnablePin, LOW);
  setStatusLed(false);

  I2Cone.begin(19, 18, 400000);
  if (!i2cDeviceIsPresent(I2Cone, kAs5600Address)) {
    Serial.println("ERROR: right AS5600 not found at 0x36.");
    disableBothDrivers();
    return;
  }
  sensor1.init(&I2Cone);
  motor1.linkSensor(&sensor1);

  if (!i2cDeviceIsPresent(I2Cone, kMpu6050Address)) {
    Serial.println("ERROR: MPU6050 not found at 0x68.");
    disableBothDrivers();
    return;
  }
  mpu6050.begin();
  calibrateMpu6050();

  driver1.voltage_power_supply = 8.0f;
  driver1.voltage_limit = 2.0f;
  if (!driver1.init()) {
    Serial.println("ERROR: Right driver initialization failed.");
    disableBothDrivers();
    return;
  }
  driver1_initialized = true;
  driver1.disable();

  motor1.linkDriver(&driver1);
  motor1.voltage_sensor_align = 2.0f;
  motor1.voltage_limit = 2.0f;
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor1.useMonitoring(Serial);
  if (!motor1.init()) {
    Serial.println("ERROR: Right motor initialization failed.");
    disableBothDrivers();
    return;
  }
  const int motor1FocStatus = motor1.initFOC();
  Serial.printf("Right FOC initialization result: %d\n", motor1FocStatus);
  if (motor1FocStatus == 0) {
    Serial.println("ERROR: Right FOC initialization failed.");
    disableBothDrivers();
    return;
  }
  driver1.disable();

  I2Ctwo.begin(23, 5, 400000);
  if (!i2cDeviceIsPresent(I2Ctwo, kAs5600Address)) {
    Serial.println("ERROR: left AS5600 not found at 0x36.");
    disableBothDrivers();
    return;
  }
  sensor2.init(&I2Ctwo);
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = 8.0f;
  driver2.voltage_limit = 2.0f;
  if (!driver2.init()) {
    Serial.println("ERROR: Left driver initialization failed.");
    disableBothDrivers();
    return;
  }
  driver2_initialized = true;
  driver2.disable();

  motor2.linkDriver(&driver2);
  motor2.voltage_sensor_align = 2.0f;
  motor2.voltage_limit = 2.0f;
  motor2.torque_controller = TorqueControlType::voltage;
  motor2.controller = MotionControlType::torque;
  motor2.useMonitoring(Serial);
  if (!motor2.init()) {
    Serial.println("ERROR: Left motor initialization failed.");
    disableBothDrivers();
    return;
  }
  const int motor2FocStatus = motor2.initFOC();
  Serial.printf("Left FOC initialization result: %d\n", motor2FocStatus);
  if (motor2FocStatus == 0) {
    Serial.println("ERROR: Left FOC initialization failed.");
    disableBothDrivers();
    return;
  }
  driver2.disable();

  m1_direction = motor1.sensor_direction == CW ? -0.5f : 0.5f;
  m2_direction = motor2.sensor_direction == CCW ? -0.5f : 0.5f;
  Serial.printf("Right sensor_direction=%s m1_direction=%.2f\n",
                motor1.sensor_direction == CW ? "CW" : "CCW", m1_direction);
  Serial.printf("Left sensor_direction=%s m2_direction=%.2f\n",
                motor2.sensor_direction == CW ? "CW" : "CCW", m2_direction);

  setBothTargetsToZero();
  driver1.enable();
  driver2.enable();
  test_system_ready = true;
  lqr_diagnostic_armed = false;
  lqr_direction_test_armed = false;
  balance_wait_active = false;
  Serial.println("OK: both motors initialized and stopped at 0.00 V.");
  setupWifiLogServer();
  printHelp();
}

void loop() {
  updatePs4Controller();

  if (!test_system_ready) {
    handleSerialCommand();
    updateWifiLogServer();
    delay(10);
    return;
  }

  const uint32_t loop_start_us = micros();

  uint32_t section_start_us = micros();
  motor1.loopFOC();
  const uint32_t motor1_loopfoc_us =
      micros() - section_start_us;

  section_start_us = micros();
  motor2.loopFOC();
  const uint32_t motor2_loopfoc_us =
      micros() - section_start_us;

  section_start_us = micros();
  mpu6050.update();
  const uint32_t mpu_update_us =
      micros() - section_start_us;

  updateLqrState();

  updateWheelVelocityDiagnostic();
  updateLqrDiagnostic();
  updateBalanceWait();

  float right_output = right_target_voltage;
  float left_output = left_target_voltage;
  if (lqr_direction_test_armed) {
    const float motor1_output_sign =
      m1_direction < 0.0f ? -1.0f : 1.0f;
    const float motor2_output_sign =
      m2_direction < 0.0f ? -1.0f : 1.0f;

    right_output = constrain(-test_LQR_u * motor1_output_sign,
                            -kLqrTestVoltageLimit,
                            kLqrTestVoltageLimit);
    left_output = constrain(-test_LQR_u * motor2_output_sign,
                            -kLqrTestVoltageLimit,
                            kLqrTestVoltageLimit);
  }

  if (polarity_test_active) {
    const uint32_t now_ms = millis();
    const float polarity_m1_sign =
        m1_direction < 0.0f ? -1.0f : 1.0f;
    const float polarity_m2_sign =
        m2_direction < 0.0f ? -1.0f : 1.0f;
    const float polarity_angle_delta =
        LQR_angle - polarity_reference_angle;
    if (polarity_pulse_active) {
      const float right_velocity = motor1.shaft_velocity;
      const float left_velocity = motor2.shaft_velocity;

      polarity_right_velocity_sum += right_velocity;
      polarity_left_velocity_sum += left_velocity;
      polarity_velocity_sample_count++;

      const float right_velocity_abs = abs(right_velocity);
      const float left_velocity_abs = abs(left_velocity);

      if (right_velocity_abs > polarity_right_velocity_peak) {
        polarity_right_velocity_peak = right_velocity_abs;
      }
      if (left_velocity_abs > polarity_left_velocity_peak) {
        polarity_left_velocity_peak = left_velocity_abs;
      }
    }
    if (abs(polarity_angle_delta) > kLqrTestTiltLimitDeg) {
      if (polarity_pulse_active) {
        Serial.println("POLARITY PULSE STOP: tilt exceeded 10 degrees.");
      }
      polarity_pulse_active = false;
      polarity_pulse_voltage = 0.0f;
      setBothTargetsToZero();
    } else if (polarity_pulse_active &&
               now_ms - polarity_pulse_start_ms >= kPolarityPulseTimeMs) {
      polarity_pulse_active = false;
      polarity_pulse_voltage = 0.0f;
      setBothTargetsToZero();
const float right_velocity_avg =
    polarity_velocity_sample_count > 0
        ? polarity_right_velocity_sum / polarity_velocity_sample_count
        : 0.0f;

const float left_velocity_avg =
    polarity_velocity_sample_count > 0
        ? polarity_left_velocity_sum / polarity_velocity_sample_count
        : 0.0f;
const float right_angle_delta =
    motor1.shaft_angle - polarity_right_start_angle;
const float left_angle_delta =
    motor2.shaft_angle - polarity_left_start_angle;

    Serial.println("POLARITY RESULT:");
    Serial.printf("command=%c\n", polarity_pulse_command);
    Serial.printf("right_enabled=%s\n",
                  polarity_drive_right ? "yes" : "no");
    Serial.printf("left_enabled=%s\n",
                  polarity_drive_left ? "yes" : "no");
    Serial.printf("AngleY=%.6f\n", LQR_angle);
    Serial.printf("GyroY=%.6f\n", LQR_gyro);
    Serial.printf("right_angle_delta=%.6f\n", right_angle_delta);
    Serial.printf("left_angle_delta=%.6f\n", left_angle_delta);
    Serial.printf("right_velocity_avg=%.6f\n", right_velocity_avg);
    Serial.printf("left_velocity_avg=%.6f\n", left_velocity_avg);
    Serial.printf("right_velocity_peak=%.6f\n",
                  polarity_right_velocity_peak);
    Serial.printf("left_velocity_peak=%.6f\n",
                  polarity_left_velocity_peak);
      Serial.printf("sample_count=%lu\n",
                    static_cast<unsigned long>(
                        polarity_velocity_sample_count));
    }

    right_output =
        polarity_pulse_active && polarity_drive_right
            ? polarity_pulse_voltage * polarity_m1_sign
            : 0.0f;

    left_output =
        polarity_pulse_active && polarity_drive_left
            ? polarity_pulse_voltage * polarity_m2_sign
            : 0.0f;
    if (now_ms - polarity_diagnostic_last_ms >=
        kPolarityDiagnosticIntervalMs) {
      polarity_diagnostic_last_ms = now_ms;
      Serial.println("LQR POLARITY:");
      Serial.printf("AngleY=%.6f\n", LQR_angle);
      Serial.printf("GyroY=%.6f\n", LQR_gyro);
      Serial.printf("angle_delta=%.6f\n", polarity_angle_delta);
    }
  }

  if (emergency_stop_active) {
    right_output = 0.0f;
    left_output = 0.0f;
  }

  printLqrSoftStopSpeedDiagnostic(right_output, left_output);

  if (drive_recovery_state != DriveRecoveryState::Inactive) {
    const uint32_t now_ms = millis();
    const bool attitude_stable =
        abs(LQR_angle - angle_zeropoint) <= kDriveRecoveryStableAngleDeg &&
        abs(LQR_gyro) <= kDriveRecoveryStableGyroDegPerSec &&
        abs(lqr_speed_for_control) <= kDriveRecoveryStableSpeed;

    if (drive_recovery_state == DriveRecoveryState::AttitudeOnly) {
      // Follow the wheels without exerting position torque until the body has
      // genuinely settled for a continuous interval.
      distance_zeropoint = LQR_distance;
      if (attitude_stable) {
        if (drive_recovery_stable_since_ms == 0) {
          drive_recovery_stable_since_ms = now_ms;
        } else if (now_ms - drive_recovery_stable_since_ms >=
                   kDriveRecoveryStableTimeMs) {
          drive_recovery_state = DriveRecoveryState::PositionRamp;
          drive_recovery_position_ramp_start_ms = now_ms;
          distance_zeropoint = LQR_distance;
          pid_distance.reset();
          Serial.println("DRIVE RECOVERY: state=POSITION_RAMP");
        }
      } else {
        drive_recovery_stable_since_ms = 0;
      }
    } else if (!attitude_stable) {
      drive_recovery_state = DriveRecoveryState::AttitudeOnly;
      drive_recovery_stable_since_ms = 0;
      distance_zeropoint = LQR_distance;
      pid_distance.reset();
      Serial.println("DRIVE RECOVERY: state=ATTITUDE_ONLY reason=UNSTABLE");
    } else if (now_ms - drive_recovery_position_ramp_start_ms >=
               kDriveRecoveryPositionRampMs) {
      drive_recovery_state = DriveRecoveryState::Inactive;
      Serial.println("DRIVE RECOVERY: state=COMPLETE position_hold=ENABLED");
    }
  }

  const bool was_recovering =
      drive_recovery_state != DriveRecoveryState::Inactive;
  if (const char* drive_fault =
          detectDriveSafetyFault(right_output, left_output)) {
    if (was_recovering) {
      drive_recovery_state = DriveRecoveryState::AttitudeOnly;
      drive_recovery_stable_since_ms = 0;
      distance_zeropoint = LQR_distance;
      pid_distance.reset();
    } else {
      Serial.printf(
          "LQR DRIVE ENVELOPE STOP: reason=%s angle=%+.3f gyro=%+.3f "
          "speed=%+.3f right_output=%+.3f left_output=%+.3f\n",
          drive_fault, LQR_angle - angle_zeropoint, LQR_gyro,
          LQR_speed, right_output, left_output);
      stopLqrDriveOnly(drive_fault, true);
      // Discard the drive-cycle output once; subsequent recovery cycles keep
      // balance torque continuously active.
      right_output = 0.0f;
      left_output = 0.0f;
    }
  }

  if (drive_recovery_state != DriveRecoveryState::Inactive &&
      millis() - drive_recovery_diagnostic_last_ms >=
          kDriveRecoveryDiagnosticIntervalMs) {
    drive_recovery_diagnostic_last_ms = millis();
    const char* state = drive_recovery_state == DriveRecoveryState::AttitudeOnly
                            ? "ATTITUDE_ONLY"
                            : "POSITION_RAMP";
    const uint32_t saturation_ms = drive_safety_saturation_since_ms == 0
        ? 0 : millis() - drive_safety_saturation_since_ms;
    Serial.printf(
        "DRIVE RECOVERY: state=%s elapsed_ms=%lu angle=%+.6f gyro=%+.6f "
        "position_error=%+.6f speed_feedback=%+.6f angle_control=%+.6f "
        "gyro_control=%+.6f speed_control=%+.6f position_control=%+.6f "
        "right_output=%+.6f left_output=%+.6f saturation_ms=%lu\n",
        state, static_cast<unsigned long>(millis() - drive_recovery_start_ms),
        LQR_angle - angle_zeropoint, LQR_gyro,
        distance_zeropoint - LQR_distance, lqr_speed_for_control,
        angle_control, gyro_control, speed_control, distance_control,
        right_output, left_output, static_cast<unsigned long>(saturation_ms));
  }

  if (lqr_startup_diagnostic_active) {
    const uint32_t now_ms = millis();
    const uint32_t elapsed_ms = now_ms - lqr_startup_diagnostic_start_ms;
    if (elapsed_ms > kLqrStartupDiagnosticDurationMs) {
      lqr_startup_diagnostic_active = false;
    } else if (now_ms - lqr_startup_diagnostic_last_ms >=
               kLqrStartupDiagnosticIntervalMs) {
      lqr_startup_diagnostic_last_ms = now_ms;
      if (kEnableLqrStartupDiagnostic &&
        lqr_speed_source == LqrSpeedSource::SimpleFoc) {
        printLqrStartupDiagnostic(elapsed_ms, right_output, left_output);
      }
    }
  }

  if (lqr_direction_test_armed && confirmActualFall()) {
    if (lqr_startup_diagnostic_active) {
      const uint32_t elapsed_ms =
          millis() - lqr_startup_diagnostic_start_ms;
      Serial.println("LQR STARTUP STOP:");
      Serial.printf("elapsed_ms=%lu\n",
                    static_cast<unsigned long>(elapsed_ms));
      Serial.printf("angle_error=%.6f\n",
                    LQR_angle - angle_zeropoint);
      Serial.printf("gyro=%.6f\n", LQR_gyro);
      Serial.printf("test_LQR_u=%.6f\n", test_LQR_u);
      Serial.printf("right_output=%.6f\n", right_output);
      Serial.printf("left_output=%.6f\n", left_output);
    }
    lqr_startup_diagnostic_active = false;
    lqr_direction_test_armed = false;
    setStatusLed(false);
    setBothTargetsToZero();
    right_output = 0.0f;
    left_output = 0.0f;
    motor1.move(0.0f);
    motor2.move(0.0f);

    updateLqrControlTrace(right_output, left_output, true);

    Serial.println("LQR direction test stopped: actual fall angle exceeded.");

    lqr_stop_trace_active = false;

    if (lqr_drive_sign_trace_count > 0) {
      printLqrDriveSignTrace("TILT_STOP");
      resetLqrDriveSignTrace();
    }

    if (lqr_stop_trace_count > 0) {
      printLqrStopTrace("TILT_STOP");
      resetLqrStopTrace();
    }

    printAndResetLqrRunTime();

    printVelocityTrace();
    printLoopTimingTrace();
    printLqrControlTrace();
    clearVelocityTrace();
    clearLoopTimingTrace();
    clearLqrControlTrace();

    terminateLqrDriveSession("TILT_STOP", true);
    emergency_stop_active = true;
    disableBothDrivers();
    Serial.println(
        "SAFETY STOP: mode=FULL_STOP reason=TILT_STOP drivers=DISABLED balance=OFF");
    disableLegServoTorqueForTestEnd("TILT_STOP");
  }

  updateLqrControlTrace(right_output, left_output);
  updateLqrRunDiagnostic(right_output, left_output);

  section_start_us = micros();

  motor1.move(right_output);
  motor2.move(left_output);

  const uint32_t motor_move_us =
      micros() - section_start_us;

  handleSerialCommand();

  const uint32_t whole_loop_us =
      micros() - loop_start_us;

  if (lqr_direction_test_armed &&
      (whole_loop_us >= 15000 ||
      motor1_loopfoc_us >= 5000 ||
      motor2_loopfoc_us >= 5000 ||
      mpu_update_us >= 5000 ||
      motor_move_us >= 5000)) {

    LoopTimingSample& sample =
        loop_timing_trace[loop_timing_write_index];

    sample.elapsed_ms =
        millis() - lqr_direction_test_start_ms;
    sample.whole_loop_us = whole_loop_us;
    sample.motor1_loopfoc_us = motor1_loopfoc_us;
    sample.motor2_loopfoc_us = motor2_loopfoc_us;
    sample.mpu_update_us = mpu_update_us;
    sample.motor_move_us = motor_move_us;

    loop_timing_write_index =
        (loop_timing_write_index + 1) %
        kLoopTimingTraceSize;

    if (loop_timing_count < kLoopTimingTraceSize) {
      ++loop_timing_count;
    }
  }

  updateWifiLogServer();
}

namespace {
//==================================================
// Motor / Sensor Control
//==================================================
void setBothTargetsToZero() {
  right_target_voltage = 0.0f;
  left_target_voltage = 0.0f;
  motor1.target = 0.0f;
  motor2.target = 0.0f;
}

void stopBothMotorsAtZero() {
  setBothTargetsToZero();
  if (test_system_ready && !emergency_stop_active) {
    motor1.move(0.0f);
    motor2.move(0.0f);
  }
}

void disableBothDrivers() {
  setBothTargetsToZero();
  if (driver1_initialized) {
    driver1.disable();
  }
  if (driver2_initialized) {
    driver2.disable();
  }
  digitalWrite(motor1EnablePin, LOW);
  digitalWrite(motor2EnablePin, LOW);
}

bool i2cDeviceIsPresent(TwoWire& bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void calibrateMpu6050() {
  stopBothMotorsAtZero();
  Serial.println("MPU6050 calibration starting. Keep the robot completely still.");
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 calibration complete; both motor targets remain 0 V.");
}

//==================================================
// LQR / Balance Control
//==================================================
void updateLqrState() {
  const uint32_t now_us = micros();
  if (last_control_update_us != 0) {
    control_dt_ms = (now_us - last_control_update_us) / 1000.0f;
  }
  last_control_update_us = now_us;

  LQR_angle = mpu6050.getAngleY();
  LQR_gyro = mpu6050.getGyroY();
  LQR_distance = motor1.shaft_angle * m1_direction +
                 motor2.shaft_angle * m2_direction;
  LQR_speed = motor1.shaft_velocity * m1_direction +
              motor2.shaft_velocity * m2_direction;
  lqr_simplefoc_speed_read_ms = millis();
}

void updateWheelVelocityDiagnostic() {
  const uint32_t current_timestamp = micros();
  const float current_right_angle = motor1.shaft_angle;
  const float current_left_angle = motor2.shaft_angle;

  if (previous_velocity_diagnostic_us == 0) {
    previous_right_angle = current_right_angle;
    previous_left_angle = current_left_angle;
    previous_velocity_diagnostic_us = current_timestamp;
    return;
  }

  const uint32_t elapsed_us =
      current_timestamp - previous_velocity_diagnostic_us;

  if (elapsed_us < kWheelVelocityDiagnosticIntervalUs) {
    return;
  }

  const float dt = elapsed_us / 1000000.0f;

  const float right_angle_delta =
      current_right_angle - previous_right_angle;

  const float left_angle_delta =
      current_left_angle - previous_left_angle;

  diag_right_velocity_10ms =
      right_angle_delta / dt;

  diag_left_velocity_10ms =
      left_angle_delta / dt;

  diag_LQR_speed_10ms =
      diag_right_velocity_10ms * m1_direction +
      diag_left_velocity_10ms * m2_direction;
  lqr_speed_10ms_update_ms = millis();

  const uint32_t current_ms = millis();
  lqr_ps4_speed_history[lqr_ps4_speed_history_next] =
      {current_ms, current_right_angle, current_left_angle};
  lqr_ps4_speed_history_next =
      (lqr_ps4_speed_history_next + 1) % kPs4SpeedWindowHistorySize;
  if (lqr_ps4_speed_history_count < kPs4SpeedWindowHistorySize) {
    ++lqr_ps4_speed_history_count;
  }

  // Walk back to the newest sample which is at least the requested window
  // old.  Using its real timestamp avoids assuming that every diagnostic
  // update arrived at exactly 10 ms.
  bool ps4_window_ready = false;
  Ps4WheelAngleSample ps4_window_start = {};
  for (size_t age = 1; age < lqr_ps4_speed_history_count; ++age) {
    const size_t index =
        (lqr_ps4_speed_history_next + kPs4SpeedWindowHistorySize - 1 - age) %
        kPs4SpeedWindowHistorySize;
    const Ps4WheelAngleSample& candidate = lqr_ps4_speed_history[index];
    if (current_ms - candidate.timestamp_ms >= kPs4SpeedWindowMs) {
      ps4_window_start = candidate;
      ps4_window_ready = true;
      break;
    }
  }

  if (ps4_window_ready) {
    lqr_ps4_window_elapsed_ms = current_ms - ps4_window_start.timestamp_ms;
    lqr_ps4_window_angle_delta_right =
        current_right_angle - ps4_window_start.right_angle;
    lqr_ps4_window_angle_delta_left =
        current_left_angle - ps4_window_start.left_angle;
    const float window_dt = lqr_ps4_window_elapsed_ms / 1000.0f;
    const float right_window_speed =
        lqr_ps4_window_angle_delta_right / window_dt;
    const float left_window_speed =
        lqr_ps4_window_angle_delta_left / window_dt;
    lqr_ps4_window_speed = right_window_speed * m1_direction +
                           left_window_speed * m2_direction;
    lqr_ps4_speed_for_control = lqr_ps4_window_speed;
  } else {
    lqr_ps4_window_elapsed_ms = 0;
    lqr_ps4_window_angle_delta_right = 0.0f;
    lqr_ps4_window_angle_delta_left = 0.0f;
    lqr_ps4_window_speed = 0.0f;
    lqr_ps4_speed_for_control = 0.0f;
  }

if (lqr_direction_test_armed) {
  VelocityTraceSample& sample =
      velocity_trace[velocity_trace_write_index];

  sample.elapsed_ms =
      millis() - lqr_direction_test_start_ms;
  sample.elapsed_us = elapsed_us;

  sample.current_right_angle = current_right_angle;
  sample.previous_right_angle = previous_right_angle;
  sample.right_angle_delta = right_angle_delta;

  sample.current_left_angle = current_left_angle;
  sample.previous_left_angle = previous_left_angle;
  sample.left_angle_delta = left_angle_delta;

  sample.diag_right_velocity_10ms =
      diag_right_velocity_10ms;
  sample.diag_left_velocity_10ms =
      diag_left_velocity_10ms;
  sample.diag_LQR_speed_10ms =
      diag_LQR_speed_10ms;

  sample.simplefoc_right_velocity =
      motor1.shaft_velocity;
  sample.simplefoc_left_velocity =
      motor2.shaft_velocity;
  sample.LQR_speed = LQR_speed;

  sample.diag_speed_spike =
      abs(diag_LQR_speed_10ms) >= 2.0f;

  velocity_trace_write_index =
      (velocity_trace_write_index + 1) %
      kVelocityTraceSize;

  if (velocity_trace_count < kVelocityTraceSize) {
    ++velocity_trace_count;
  }
}
  previous_right_angle = current_right_angle;
  previous_left_angle = current_left_angle;
  previous_velocity_diagnostic_us = current_timestamp;
}

const char* lqrSpeedSourceName() {
  return lqr_speed_source == LqrSpeedSource::SimpleFoc ? "SIMPLEFOC"
                                                       : "10MS";
}
void updateLqrControlTrace(float right_output, float left_output,
                           bool force_sample) {
  if (!force_sample && !lqr_direction_test_armed) {
    return;
  }

  const uint32_t now_us = micros();

  if (!force_sample && lqr_control_trace_last_us != 0 &&
      now_us - lqr_control_trace_last_us <
          kLqrControlTraceIntervalUs) {
    return;
  }

  lqr_control_trace_last_us = now_us;

  LqrControlTraceSample& sample =
      lqr_control_trace[lqr_control_trace_write_index];

  sample.elapsed_ms =
      millis() - lqr_direction_test_start_ms;
  sample.angle = LQR_angle;
  sample.angle_zeropoint = angle_zeropoint;
  sample.angle_error = LQR_angle - angle_zeropoint;
  sample.gyro = LQR_gyro;
  sample.lqr_speed = LQR_speed;
  sample.speed_for_control = lqr_speed_for_control;

  sample.angle_control = angle_control;
  sample.gyro_control = gyro_control;
  sample.distance_control = distance_control;
  sample.speed_control = speed_control;

  sample.lqr_u = test_LQR_u;
  sample.right_output = right_output;
  sample.left_output = left_output;

  lqr_control_trace_write_index =
      (lqr_control_trace_write_index + 1) %
      kLqrControlTraceSize;

  if (lqr_control_trace_count < kLqrControlTraceSize) {
    ++lqr_control_trace_count;
  }
}

void clearLqrControlTrace() {
  lqr_control_trace_write_index = 0;
  lqr_control_trace_count = 0;
  lqr_control_trace_last_us = 0;
}

void printLqrControlTrace() {
  if (lqr_control_trace_count == 0) {
    return;
  }

  Serial.println("LQR CONTROL TRACE BEGIN");
  Serial.printf(
      "count=%u\n",
      static_cast<unsigned>(lqr_control_trace_count));

  Serial.println(
      "elapsed_ms,angle,angle_zeropoint,angle_error,gyro,"
      "lqr_speed,speed_for_control,"
      "angle_control,gyro_control,distance_control,speed_control,"
      "lqr_u,right_output,left_output");

  const size_t first_index =
      (lqr_control_trace_write_index +
       kLqrControlTraceSize -
       lqr_control_trace_count) %
      kLqrControlTraceSize;

  for (size_t i = 0; i < lqr_control_trace_count; ++i) {
    const LqrControlTraceSample& sample =
        lqr_control_trace[
            (first_index + i) % kLqrControlTraceSize];

    Serial.printf(
        "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f\n",
        static_cast<unsigned long>(sample.elapsed_ms),
          sample.angle,
          sample.angle_zeropoint,
          sample.angle_error,
          sample.gyro,
          sample.lqr_speed,
          sample.speed_for_control,
          sample.angle_control,
          sample.gyro_control,
          sample.distance_control,
          sample.speed_control,
          sample.lqr_u,
          sample.right_output,
          sample.left_output);
  }

  Serial.println("LQR CONTROL TRACE END");
}

void updateTestLqrOutput(float angle_term, float gyro_term,
                         float distance_term, float speed_term) {
  switch (lqr_test_mode) {
    case LqrTestMode::Full:
      test_LQR_u =
          angle_term + gyro_term + distance_term + speed_term;
      break;

    case LqrTestMode::AttitudeSpeed:
      test_LQR_u = angle_term + gyro_term + speed_term;
      break;
    case LqrTestMode::AttitudeDistance:
      test_LQR_u = angle_term + gyro_term + distance_term;
      break;
    case LqrTestMode::AttitudeOnly:
      test_LQR_u = angle_term + gyro_term;
      break;
  }
}

const char* lqrTestModeLogName() {
  switch (lqr_test_mode) {
    case LqrTestMode::Full:
      return "FULL";
    case LqrTestMode::AttitudeSpeed:
      return "ATTITUDE_SPEED";
    case LqrTestMode::AttitudeDistance:
      return "ATTITUDE_DISTANCE";
    case LqrTestMode::AttitudeOnly:
      return "ATTITUDE_ONLY";
  }

  return "UNKNOWN";
}

const char* lqrTestModeDisplayName() {
  switch (lqr_test_mode) {
    case LqrTestMode::Full:
      return "FULL";
    case LqrTestMode::AttitudeSpeed:
      return "ATTITUDE + SPEED";
    case LqrTestMode::AttitudeDistance:
      return "ATTITUDE + DISTANCE";
    case LqrTestMode::AttitudeOnly:
      return "ATTITUDE ONLY";
  }

  return "UNKNOWN";
}

void resetLqrDriveSignTrace() {
  lqr_drive_sign_trace_count = 0;
  lqr_drive_sign_trace_start_ms = millis();
  lqr_drive_sign_trace_last_ms = 0;
  lqr_drive_sign_trace_active = true;
}

void captureLqrDriveSignTrace(float speed_direct_term) {
  if (!lqr_drive_sign_trace_active ||
      lqr_drive_sign_trace_count >=
          kLqrDriveSignTraceCapacity) {
    return;
  }

  const uint32_t now_ms = millis();

  if (lqr_drive_sign_trace_count != 0 &&
      now_ms - lqr_drive_sign_trace_last_ms <
          kLqrDriveSignTraceIntervalMs) {
    return;
  }

  lqr_drive_sign_trace_last_ms = now_ms;

  LqrDriveSignTraceSample& sample =
      lqr_drive_sign_trace[lqr_drive_sign_trace_count++];

  sample.elapsed_ms =
      now_ms - lqr_drive_sign_trace_start_ms;
  sample.angle_error =
      LQR_angle - angle_zeropoint;
  sample.gyro = LQR_gyro;
  sample.angle_control = angle_control;
  sample.gyro_control = gyro_control;
  sample.drive_angle_bias = lqr_drive_angle_bias;
  sample.target_speed = lqr_target_speed;
  sample.raw_speed = lqr_speed_raw_for_control;
  sample.actual_speed = lqr_speed_for_control;
  sample.speed_error =
      lqr_target_speed - lqr_speed_for_control;
  sample.speed_control = speed_control;
  sample.speed_direct_term = speed_direct_term;
  sample.lqr_u = LQR_u;

  const float trace_motor1_output_sign =
      m1_direction < 0.0f ? -1.0f : 1.0f;
  const float trace_motor2_output_sign =
      m2_direction < 0.0f ? -1.0f : 1.0f;

  sample.right_output =
      constrain(-test_LQR_u * trace_motor1_output_sign,
                -kLqrTestVoltageLimit,
                kLqrTestVoltageLimit);

  sample.left_output =
      constrain(-test_LQR_u * trace_motor2_output_sign,
                -kLqrTestVoltageLimit,
                kLqrTestVoltageLimit);

  sample.motor1_velocity = motor1.shaft_velocity;
  sample.motor2_velocity = motor2.shaft_velocity;

  sample.expected_motion_sign =
      lqr_target_speed > 0.0f
          ? 1.0f
          : (lqr_target_speed < 0.0f ? -1.0f : 0.0f);

  const float actual_motion_velocity =
      motor1.shaft_velocity * m1_direction +
      motor2.shaft_velocity * m2_direction;

  sample.actual_motion_sign =
      actual_motion_velocity > 0.0f
          ? 1.0f
          : (actual_motion_velocity < 0.0f ? -1.0f : 0.0f);

  sample.motion_sign_ok =
      sample.expected_motion_sign == 0.0f ||
      sample.actual_motion_sign == 0.0f ||
      sample.expected_motion_sign == sample.actual_motion_sign;

  sample.distance_delta =
      LQR_distance - lqr_drive_start_distance;
}

void printLqrDriveSignTrace(const char* reason) {
  lqr_drive_sign_trace_active = false;

  Serial.printf(
      "LQR DRIVE SIGN TRACE [%s] BEGIN\n",
      reason);
  Serial.printf(
      "count=%u\n",
      static_cast<unsigned>(
          lqr_drive_sign_trace_count));
  Serial.println(
      "elapsed_ms,angle_error,gyro,"
      "angle_control,gyro_control,drive_angle_bias,"
      "target_speed,raw_speed,filtered_speed,"
      "speed_error,speed_control,speed_direct_term,lqr_u,"
      "right_output,left_output,"
      "motor1_velocity,motor2_velocity,"
      "expected_motion_sign,actual_motion_sign,"
      "motion_sign_ok,distance_delta");

  for (size_t i = 0;
       i < lqr_drive_sign_trace_count;
       ++i) {
    const LqrDriveSignTraceSample& sample =
        lqr_drive_sign_trace[i];

    Serial.printf(
        "%lu,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,"
        "%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,"
        "%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,"
        "%+.1f,%+.1f,%s,%+.6f\n",
        static_cast<unsigned long>(sample.elapsed_ms),
        sample.angle_error,
        sample.gyro,
        sample.angle_control,
        sample.gyro_control,
        sample.drive_angle_bias,
        sample.target_speed,
        sample.raw_speed,
        sample.actual_speed,
        sample.speed_error,
        sample.speed_control,
        sample.speed_direct_term,
        sample.lqr_u,
        sample.right_output,
        sample.left_output,
        sample.motor1_velocity,
        sample.motor2_velocity,
        sample.expected_motion_sign,
        sample.actual_motion_sign,
        sample.motion_sign_ok ? "yes" : "no",
        sample.distance_delta);
  }

  Serial.printf(
      "LQR DRIVE SIGN TRACE [%s] END\n",
      reason);
}

void printLqrDriveSnapshot(const char* phase);

void resetLqrStopTrace() {
  lqr_stop_trace_count = 0;
  lqr_stop_trace_active = false;
  lqr_stop_trace_start_ms = 0;
  lqr_stop_trace_last_sample_ms = 0;
  lqr_stop_trace_origin_distance = 0.0f;
}

void startLqrStopTrace() {
  lqr_stop_trace_count = 0;
  lqr_stop_trace_active = true;
  lqr_stop_trace_start_ms = millis();
  lqr_stop_trace_last_sample_ms = 0;
  lqr_stop_trace_origin_distance = LQR_distance;
}

void captureLqrStopTrace(float speed_direct_term) {
  if (!lqr_stop_trace_active) {
    return;
  }

  const uint32_t now_ms = millis();
  const uint32_t elapsed_ms =
      now_ms - lqr_stop_trace_start_ms;

  if (elapsed_ms > kLqrStopTraceDurationMs ||
      lqr_stop_trace_count >= kLqrStopTraceMaxSamples) {
    lqr_stop_trace_active = false;
    return;
  }

  if (lqr_stop_trace_count > 0 &&
      now_ms - lqr_stop_trace_last_sample_ms <
          kLqrStopTraceSampleIntervalMs) {
    return;
  }

  lqr_stop_trace_last_sample_ms = now_ms;

  LqrStopTraceSample& sample =
      lqr_stop_trace[lqr_stop_trace_count++];

  sample.elapsed_ms = elapsed_ms;
  sample.angle_error =
      LQR_angle - angle_zeropoint;
  sample.gyro = LQR_gyro;
  sample.angle_control = angle_control;
  sample.gyro_control = gyro_control;
  sample.drive_angle_bias = lqr_drive_angle_bias;
  sample.target_speed = lqr_target_speed;
  sample.actual_speed = lqr_speed_for_control;
  sample.speed_error =
      lqr_target_speed - lqr_speed_for_control;
  sample.speed_control = speed_control;
  sample.speed_direct_term = speed_direct_term;
  sample.distance_control = distance_control;
  sample.lqr_u = LQR_u;

  const float stop_trace_motor1_output_sign =
      m1_direction < 0.0f ? -1.0f : 1.0f;
  const float stop_trace_motor2_output_sign =
      m2_direction < 0.0f ? -1.0f : 1.0f;

  sample.right_output =
      constrain(-test_LQR_u * stop_trace_motor1_output_sign,
                -kLqrTestVoltageLimit,
                kLqrTestVoltageLimit);

  sample.left_output =
      constrain(-test_LQR_u * stop_trace_motor2_output_sign,
                -kLqrTestVoltageLimit,
                kLqrTestVoltageLimit);

  sample.distance_delta =
      LQR_distance - lqr_stop_trace_origin_distance;
}

void printLqrStopTrace(const char* reason) {
  Serial.printf(
      "LQR STOP TRACE [%s] BEGIN\n",
      reason);
  Serial.printf(
      "count=%lu\n",
      static_cast<unsigned long>(
          lqr_stop_trace_count));

  Serial.println(
      "elapsed_ms,angle_error,gyro,"
      "angle_control,gyro_control,drive_angle_bias,"
      "target_speed,actual_speed,speed_error,"
      "speed_control,speed_direct_term,distance_control,lqr_u,"
      "right_output,left_output,distance_delta");

  for (size_t i = 0;
       i < lqr_stop_trace_count;
       ++i) {
    const LqrStopTraceSample& sample =
        lqr_stop_trace[i];

    Serial.printf(
        "%lu,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,"
        "%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,"
        "%+.6f,%+.6f,%+.6f,%+.6f\n",
        static_cast<unsigned long>(sample.elapsed_ms),
        sample.angle_error,
        sample.gyro,
        sample.angle_control,
        sample.gyro_control,
        sample.drive_angle_bias,
        sample.target_speed,
        sample.actual_speed,
        sample.speed_error,
        sample.speed_control,
        sample.speed_direct_term,
        sample.distance_control,
        sample.lqr_u,
        sample.right_output,
        sample.left_output,
        sample.distance_delta);
  }

  Serial.printf(
      "LQR STOP TRACE [%s] END\n",
      reason);
}

bool requestLqrDrive(float target_speed, char command) {
  if (!test_system_ready || emergency_stop_active ||
      !lqr_direction_test_armed || lqr_auto_drive_active ||
      lqr_soft_stop_active || lqr_drive_start_pending) {
    return false;
  }

  lqr_drive_start_pending = true;
  lqr_drive_pending_command = command;
  lqr_drive_pending_speed =
      constrain(target_speed, -kLqrDriveTestSpeed, kLqrDriveTestSpeed);
  lqr_drive_start_request_ms = millis();
  lqr_drive_stable_start_ms = 0;
  lqr_drive_wait_diag_last_ms = 0;
  lqr_drive_max_stable_ms = 0;
  lqr_drive_angle_fail_count = 0;
  lqr_drive_gyro_fail_count = 0;
  lqr_drive_speed_fail_count = 0;
  lqr_drive_speed_current_fail_count = 0;
  lqr_drive_speed_control_fail_count = 0;
  lqr_drive_speed_average_fail_count = 0;
  lqr_drive_start_angle_min = 0.0f;
  lqr_drive_start_angle_max = 0.0f;
  lqr_drive_start_angle_sum = 0.0f;
  lqr_drive_start_angle_sample_count = 0;
  lqr_drive_start_angle_reference_sum = 0.0f;
  lqr_drive_start_angle_reference_count = 0;
  lqr_drive_start_angle_reference = 0.0f;
  lqr_drive_start_angle_reference_ready = false;

  for (size_t i = 0; i < kLqrDriveStartSpeedAverageSamples; ++i) {
    lqr_drive_start_speed_samples[i] = 0.0f;
  }

  lqr_drive_start_speed_sample_index = 0;
  lqr_drive_start_speed_sample_count = 0;
  lqr_drive_start_speed_sum = 0.0f;
  lqr_drive_start_speed_average = 0.0f;
  lqr_drive_start_speed_abs_sum = 0.0f;
  lqr_drive_start_speed_abs_average = 0.0f;
  lqr_drive_start_speed_abs_max = 0.0f;
  lqr_drive_start_speed_diag_count = 0;
  lqr_drive_speed_source_diag_last_ms = 0;
  lqr_drive_speed_source_diag_count = 0;
  lqr_drive_sf_m1_abs_sum = 0.0f;
  lqr_drive_sf_m1_abs_max = 0.0f;
  lqr_drive_sf_m2_abs_sum = 0.0f;
  lqr_drive_sf_m2_abs_max = 0.0f;
  lqr_drive_diag_right_abs_sum = 0.0f;
  lqr_drive_diag_right_abs_max = 0.0f;
  lqr_drive_diag_left_abs_sum = 0.0f;
  lqr_drive_diag_left_abs_max = 0.0f;
  lqr_drive_control_speed_abs_sum = 0.0f;
  lqr_drive_control_speed_abs_max = 0.0f;
  lqr_drive_gate_speed_abs_sum = 0.0f;
  lqr_drive_gate_speed_abs_max = 0.0f;
  lqr_drive_speed_source_diff_abs_sum = 0.0f;
  lqr_drive_speed_source_diff_abs_max = 0.0f;

  Serial.printf("LQR DRIVE REQUEST ARMED: command=%c "
                "stable_time=%lu ms timeout=%lu ms\n",
                command, static_cast<unsigned long>(kLqrDriveStartStableTimeMs),
                static_cast<unsigned long>(kLqrDriveStartTimeoutMs));
  return true;
}

void requestLqrSoftStop() {
  lqr_drive_wrong_way_normal_since_ms = 0;
  lqr_drive_wrong_way_hard_since_ms = 0;
  // Releasing the stick must release accumulated propulsion immediately;
  // the existing speed feedback then performs the controlled stop.
  lqr_ps4_target_tilt_integral_deg = 0.0f;
  lqr_ps4_target_tilt_deg = 0.0f;

  if (lqr_drive_start_pending) {
    lqr_drive_start_pending = false;
    lqr_drive_pending_command = 0;
    lqr_drive_pending_speed = 0.0f;
    lqr_drive_start_request_ms = 0;
    lqr_drive_stable_start_ms = 0;
    Serial.println("LQR DRIVE REQUEST CANCELLED.");
  } else if (lqr_target_speed == 0.0f &&
             abs(lqr_speed_for_control) <= kLqrSoftStopSpeedLimit) {
    lqr_auto_drive_active = false;
    distance_zeropoint = LQR_distance;
    pid_distance.reset();
    pid_speed.reset();
    Serial.println("LQR DRIVE: already stopped; holding current position.");
  } else {
    const float stop_entry_target_speed = lqr_target_speed;
    lqr_auto_drive_active = false;
    lqr_soft_stop_entry_speed = lqr_speed_for_control;
    lqr_soft_stop_active = true;
    lqr_soft_stop_stable_start_ms = 0;
    lqr_target_speed = 0.0f;
    Serial.printf("LQR DRIVE: ramp stop started from target_speed=%+.2f\n",
                  stop_entry_target_speed);
  }
}

void startPendingLqrDrive() {
  lqr_drive_start_pending = false;
  lqr_drive_pending_command = 0;
  lqr_drive_start_request_ms = 0;
  lqr_drive_stable_start_ms = 0;

  lqr_soft_stop_active = false;
  lqr_soft_stop_stable_start_ms = 0;

  distance_zeropoint = LQR_distance;
  pid_distance.reset();
  pid_speed.reset();

  lqr_drive_start_distance = LQR_distance;
  lqr_drive_origin_distance = LQR_distance;
  lqr_drive_motion_start_ms = millis();
  lqr_drive_wrong_way_peak = 0.0f;
  lqr_drive_wrong_way_normal_since_ms = 0;
  lqr_drive_wrong_way_hard_since_ms = 0;
  lqr_auto_drive_active = true;

  // Drive uses the same reference that was proven while balancing.  Do not
  // substitute a separately sampled drive posture.
  lqr_ps4_drive_angle_reference = angle_zeropoint;
  lqr_ps4_target_tilt_integral_deg = 0.0f;
  lqr_ps4_target_tilt_deg = 0.0f;

  lqr_target_speed = lqr_drive_pending_speed;
  lqr_drive_pending_speed = 0.0f;

  resetLqrDriveSignTrace();

  Serial.println("LQR DRIVE STABLE: starting.");
  Serial.printf(
      "LQR DRIVE: target_speed=%+.2f start=%.3f "
      "auto_distance=%.3f safety_limit=%.3f\n",
      lqr_target_speed,
      lqr_drive_start_distance,
      kLqrDriveAutoDistance,
      kLqrDriveTravelLimit);
  Serial.printf(
      "LQR DRIVE WRONG-WAY GUARD: grace=%lu ms "
      "normal_limit=%.3f normal_persist=%lu ms "
      "hard_limit=%.3f hard_persist=%lu ms\n",
      static_cast<unsigned long>(
          kLqrDriveWrongWayGraceTimeMs),
      kLqrDriveWrongWayLimit,
      static_cast<unsigned long>(kLqrDriveWrongWayNormalPersistMs),
      kLqrDriveWrongWayHardLimit,
      static_cast<unsigned long>(kLqrDriveWrongWayHardPersistMs));
  Serial.printf(
      "LQR DRIVE DIRECT SPEED CONTROL: %s sign=%s\n",
      kDisableDirectDriveSpeedControl
          ? (ps4_drive_state == Ps4DriveState::DriveRequested &&
                     kEnablePs4LimitedDirectSpeedControl
                 ? "PS4 LIMITED"
                 : "DISABLED")
          : "ENABLED",
      kReverseDirectDriveSpeedControl
          ? "REVERSED"
          : "NORMAL");
  if (ps4_drive_state == Ps4DriveState::DriveRequested &&
      kEnablePs4LimitedDirectSpeedControl) {
    Serial.printf("LQR PS4 DIRECT SPEED LIMIT: %.3f\n",
                  kPs4DirectSpeedControlLimit);
  }
  Serial.printf(
      "LQR DRIVE ANGLE BIAS: %s gain=%.3f limit=%.3f deg\n",
      kDisableDriveAngleBias
          ? "DISABLED"
          : "ENABLED",
      kLqrDriveAngleBiasGain,
      kLqrDriveAngleBiasLimitDeg);
  printLqrDriveSnapshot("START");
}

void printLqrDriveSnapshot(const char* phase) {
  const float snapshot_angle_error =
      LQR_angle - angle_zeropoint;
  const float snapshot_distance_error =
      distance_zeropoint - LQR_distance;

  Serial.printf(
      "LQR DRIVE SNAPSHOT [%s]: "
      "angle=%.6f angle_zero=%.6f angle_error=%+.6f "
      "distance=%.6f distance_zero=%.6f distance_error=%+.6f "
      "speed=%+.6f target_speed=%+.6f\n",
      phase,
      LQR_angle,
      angle_zeropoint,
      snapshot_angle_error,
      LQR_distance,
      distance_zeropoint,
      snapshot_distance_error,
      lqr_speed_for_control,
      lqr_target_speed);
}

void updateLqrDiagnostic() {
  lqr_speed_raw_for_control =
      lqr_speed_source == LqrSpeedSource::SimpleFoc
          ? LQR_speed
          : diag_LQR_speed_10ms;

  if (!lqr_control_speed_filter_initialized) {
    lqr_speed_for_control = lqr_speed_raw_for_control;
    lqr_control_speed_filter_initialized = true;
  } else {
    lqr_speed_for_control +=
        kLqrControlSpeedFilterAlpha *
        (lqr_speed_raw_for_control - lqr_speed_for_control);
  }
  lqr_control_speed_update_ms = millis();
  if (!lqr_diagnostic_armed) {
    angle_control = 0.0f;
    gyro_control = 0.0f;
    distance_control = 0.0f;
    speed_control = 0.0f;
    LQR_u = 0.0f;
    updateTestLqrOutput(angle_control, gyro_control, distance_control,
                        speed_control);
    return;
  }

  if (lqr_direction_test_armed) {
    const float drive_start_gate_speed =
        diag_LQR_speed_10ms;

    if (lqr_drive_start_pending) {
      const uint32_t now_ms = millis();
      lqr_gate_speed_read_ms = now_ms;

      if (lqr_drive_speed_source_diag_count == 0 ||
          now_ms - lqr_drive_speed_source_diag_last_ms >=
              kLqrDriveSpeedSourceDiagIntervalMs) {
        lqr_drive_speed_source_diag_last_ms = now_ms;
        ++lqr_drive_speed_source_diag_count;

        const float sf_m1_abs =
            abs(motor1.shaft_velocity);
        const float sf_m2_abs =
            abs(motor2.shaft_velocity);
        const float diag_right_abs =
            abs(diag_right_velocity_10ms);
        const float diag_left_abs =
            abs(diag_left_velocity_10ms);
        const float control_speed_abs =
            abs(lqr_speed_for_control);
        const float gate_speed_abs =
            abs(drive_start_gate_speed);
        const float speed_source_diff_abs =
            abs(lqr_speed_for_control -
                drive_start_gate_speed);

        lqr_drive_sf_m1_abs_sum += sf_m1_abs;
        lqr_drive_sf_m2_abs_sum += sf_m2_abs;
        lqr_drive_diag_right_abs_sum += diag_right_abs;
        lqr_drive_diag_left_abs_sum += diag_left_abs;
        lqr_drive_control_speed_abs_sum +=
            control_speed_abs;
        lqr_drive_gate_speed_abs_sum += gate_speed_abs;
        lqr_drive_speed_source_diff_abs_sum +=
            speed_source_diff_abs;

        if (sf_m1_abs > lqr_drive_sf_m1_abs_max) {
          lqr_drive_sf_m1_abs_max = sf_m1_abs;
        }

        if (sf_m2_abs > lqr_drive_sf_m2_abs_max) {
          lqr_drive_sf_m2_abs_max = sf_m2_abs;
        }

        if (diag_right_abs >
            lqr_drive_diag_right_abs_max) {
          lqr_drive_diag_right_abs_max =
              diag_right_abs;
        }

        if (diag_left_abs >
            lqr_drive_diag_left_abs_max) {
          lqr_drive_diag_left_abs_max =
              diag_left_abs;
        }

        if (control_speed_abs >
            lqr_drive_control_speed_abs_max) {
          lqr_drive_control_speed_abs_max =
              control_speed_abs;
        }

        if (gate_speed_abs >
            lqr_drive_gate_speed_abs_max) {
          lqr_drive_gate_speed_abs_max =
              gate_speed_abs;
        }

        if (speed_source_diff_abs >
            lqr_drive_speed_source_diff_abs_max) {
          lqr_drive_speed_source_diff_abs_max =
              speed_source_diff_abs;
        }
      }

      if (lqr_drive_start_speed_sample_count <
          kLqrDriveStartSpeedAverageSamples) {
        lqr_drive_start_speed_samples[
            lqr_drive_start_speed_sample_index] =
            drive_start_gate_speed;

        lqr_drive_start_speed_sum +=
            drive_start_gate_speed;

        ++lqr_drive_start_speed_sample_count;

        lqr_drive_start_speed_sample_index =
            (lqr_drive_start_speed_sample_index + 1) %
            kLqrDriveStartSpeedAverageSamples;
      } else {
        lqr_drive_start_speed_sum -=
            lqr_drive_start_speed_samples[
                lqr_drive_start_speed_sample_index];

        lqr_drive_start_speed_samples[
            lqr_drive_start_speed_sample_index] =
            drive_start_gate_speed;

        lqr_drive_start_speed_sum +=
            drive_start_gate_speed;

        lqr_drive_start_speed_sample_index =
            (lqr_drive_start_speed_sample_index + 1) %
            kLqrDriveStartSpeedAverageSamples;
      }

      const float drive_start_speed_abs =
          abs(drive_start_gate_speed);

      lqr_drive_start_speed_abs_sum +=
          drive_start_speed_abs;
      ++lqr_drive_start_speed_diag_count;

      lqr_drive_start_speed_abs_average =
          lqr_drive_start_speed_abs_sum /
          static_cast<float>(
              lqr_drive_start_speed_diag_count);

      if (drive_start_speed_abs >
          lqr_drive_start_speed_abs_max) {
        lqr_drive_start_speed_abs_max =
            drive_start_speed_abs;
      }

      if (lqr_drive_start_speed_sample_count > 0) {
        lqr_drive_start_speed_average =
            lqr_drive_start_speed_sum /
            static_cast<float>(
                lqr_drive_start_speed_sample_count);
      } else {
        lqr_drive_start_speed_average = 0.0f;
      }

      const float drive_start_angle_error =
          LQR_angle - angle_zeropoint;

      if (lqr_drive_start_angle_sample_count == 0) {
        lqr_drive_start_angle_min =
            drive_start_angle_error;
        lqr_drive_start_angle_max =
            drive_start_angle_error;
      } else {
        if (drive_start_angle_error <
            lqr_drive_start_angle_min) {
          lqr_drive_start_angle_min =
              drive_start_angle_error;
        }

        if (drive_start_angle_error >
            lqr_drive_start_angle_max) {
          lqr_drive_start_angle_max =
              drive_start_angle_error;
        }
      }

      lqr_drive_start_angle_sum +=
          drive_start_angle_error;
      ++lqr_drive_start_angle_sample_count;

      if (!lqr_drive_start_angle_reference_ready) {
        const uint32_t reference_elapsed_ms =
            now_ms - lqr_drive_start_request_ms;

        if (reference_elapsed_ms <
            kLqrDriveStartAngleReferenceTimeMs) {
          lqr_drive_start_angle_reference_sum +=
              drive_start_angle_error;
          ++lqr_drive_start_angle_reference_count;
        } else {
          if (lqr_drive_start_angle_reference_count > 0) {
            lqr_drive_start_angle_reference =
                lqr_drive_start_angle_reference_sum /
                static_cast<float>(
                    lqr_drive_start_angle_reference_count);
          } else {
            lqr_drive_start_angle_reference =
                drive_start_angle_error;
          }

          lqr_drive_start_angle_reference_ready = true;

          Serial.printf(
              "LQR DRIVE START ANGLE REFERENCE: "
              "samples=%lu reference=%+.3f deg "
              "limit=%.2f deg\n",
              static_cast<unsigned long>(
                  lqr_drive_start_angle_reference_count),
              lqr_drive_start_angle_reference,
              kLqrDriveStartAngleReferenceLimitDeg);

        }
      }

      const float drive_start_angle_deviation =
          drive_start_angle_error -
          lqr_drive_start_angle_reference;

      const bool drive_start_reference_ok =
          lqr_drive_start_angle_reference_ready &&
          abs(lqr_drive_start_angle_reference) <=
              kLqrDriveStartAngleReferenceLimitDeg;

      const bool drive_start_angle_ok =
          drive_start_reference_ok &&
          abs(drive_start_angle_deviation) <=
              kLqrDriveStartAngleLimitDeg;

      const bool drive_start_gyro_ok =
          abs(LQR_gyro) <=
              kLqrDriveStartGyroLimitDegPerSec;

      const bool drive_start_speed_average_ok =
          abs(lqr_drive_start_speed_average) <=
              kLqrDriveStartSpeedLimit;

      const bool drive_start_speed_current_ok =
          abs(drive_start_gate_speed) <=
              kLqrDriveStartSpeedLimit;

      const bool drive_start_speed_control_ok =
          abs(lqr_speed_for_control) <=
              kLqrDriveStartSpeedLimit;

      // Use the 10 ms wheel-angle-difference speed for the drive-start
      // gate. SimpleFOC shaft_velocity is intentionally excluded from
      // this gate because its near-zero noise is much larger.
      // drive_start_speed_control_ok remains diagnostic only.
      const bool drive_start_speed_ok =
          drive_start_speed_average_ok &&
          drive_start_speed_current_ok;

      const bool drive_start_stable =
          lqr_drive_start_angle_reference_ready &&
          drive_start_angle_ok &&
          drive_start_gyro_ok &&
          drive_start_speed_ok;

      if (lqr_speed_sources_diag_last_ms == 0 ||
          now_ms - lqr_speed_sources_diag_last_ms >=
              kLqrSpeedSourcesDiagIntervalMs) {
        lqr_speed_sources_diag_last_ms = now_ms;
        const float simplefoc_right =
            motor1.shaft_velocity * m1_direction;
        const float simplefoc_left =
            motor2.shaft_velocity * m2_direction;
        const float speed_10ms_right =
            diag_right_velocity_10ms * m1_direction;
        const float speed_10ms_left =
            diag_left_velocity_10ms * m2_direction;

        Serial.println("LQR SPEED SOURCES:");
        Serial.printf("simplefoc_right_raw=%+.6f\n", motor1.shaft_velocity);
        Serial.printf("simplefoc_left_raw=%+.6f\n", motor2.shaft_velocity);
        Serial.printf(
            "right_correction_factor=%+.1f left_correction_factor=%+.1f\n",
            m1_direction, m2_direction);
        Serial.println("reversed_speed_sign_applied=no");
        Serial.printf("direct_drive_speed_control_sign=%s\n",
                      kReverseDirectDriveSpeedControl ? "REVERSED" : "NORMAL");
        Serial.printf("simplefoc_right_corrected=%+.6f\n", simplefoc_right);
        Serial.printf("simplefoc_left_corrected=%+.6f\n", simplefoc_left);
        Serial.printf("simplefoc_combined=%+.6f\n", LQR_speed);
        Serial.printf("simplefoc_corrected_difference=%+.6f\n",
                      simplefoc_right - simplefoc_left);
        Serial.printf("speed_10ms_right_raw=%+.6f\n",
                      diag_right_velocity_10ms);
        Serial.printf("speed_10ms_left_raw=%+.6f\n",
                      diag_left_velocity_10ms);
        Serial.printf("speed_10ms_right_corrected=%+.6f\n", speed_10ms_right);
        Serial.printf("speed_10ms_left_corrected=%+.6f\n", speed_10ms_left);
        Serial.printf("speed_10ms_combined=%+.6f\n", diag_LQR_speed_10ms);
        Serial.printf("speed_10ms_corrected_difference=%+.6f\n",
                      speed_10ms_right - speed_10ms_left);
        Serial.printf("selected_source=%s\n",
                      lqr_speed_source == LqrSpeedSource::SimpleFoc
                          ? "SIMPLEFOC" : "10MS");
        Serial.printf("gate_speed=%+.6f\n", drive_start_gate_speed);
        Serial.printf("control_speed_raw=%+.6f\n", lqr_speed_raw_for_control);
        Serial.printf("control_speed_filtered=%+.6f\n", lqr_speed_for_control);
        Serial.printf("average_gate_speed=%+.6f\n",
                      lqr_drive_start_speed_average);
        Serial.printf("target_speed=%+.6f\n", lqr_target_speed);
        Serial.printf("gate_age_ms=%lu control_age_ms=%lu "
                      "speed10ms_age_ms=%lu simplefoc_age_ms=%lu\n",
                      static_cast<unsigned long>(
                          now_ms - lqr_gate_speed_read_ms),
                      static_cast<unsigned long>(
                          now_ms - lqr_control_speed_update_ms),
                      static_cast<unsigned long>(
                          now_ms - lqr_speed_10ms_update_ms),
                      static_cast<unsigned long>(
                          now_ms - lqr_simplefoc_speed_read_ms));
      }

      const char* drive_start_wait_reason = "READY";

      if (!lqr_drive_start_angle_reference_ready) {
        drive_start_wait_reason = "ANGLE_REFERENCE_PENDING";
      } else if (!drive_start_reference_ok) {
        drive_start_wait_reason = "START_ANGLE_OUT_OF_RANGE";
      } else if (!drive_start_angle_ok) {
        drive_start_wait_reason = "ANGLE_NOT_STABLE";
      } else if (!drive_start_gyro_ok) {
        drive_start_wait_reason = "GYRO_NOT_STABLE";
      } else if (!drive_start_speed_average_ok) {
        drive_start_wait_reason = "AVERAGE_SPEED_NOT_STABLE";
      } else if (!drive_start_speed_current_ok) {
        drive_start_wait_reason = "CURRENT_SPEED_NOT_STABLE";
      }

      if (!lqr_drive_start_angle_reference_ready) {
        lqr_drive_stable_start_ms = 0;
      } else if (drive_start_stable) {
        if (lqr_drive_stable_start_ms == 0) {
          lqr_drive_stable_start_ms = now_ms;
        }

        const uint32_t stable_ms =
            now_ms - lqr_drive_stable_start_ms;

        if (stable_ms > lqr_drive_max_stable_ms) {
          lqr_drive_max_stable_ms = stable_ms;
        }

        if (stable_ms >= kLqrDriveStartStableTimeMs) {
          Serial.println("LQR DRIVE WAIT: READY");
          // The unchanged start decision is:
          // reference_ready AND angle_ok AND gyro_ok AND
          // (gate_ok AND average_ok). control_ok is diagnostic only.
          Serial.println("LQR DRIVE START DECISION:");
          Serial.printf("reference_ok=%s angle_ok=%s gyro_ok=%s\n",
                        drive_start_reference_ok ? "yes" : "no",
                        drive_start_angle_ok ? "yes" : "no",
                        drive_start_gyro_ok ? "yes" : "no");
          Serial.printf("gate_ok=%s control_ok=%s average_ok=%s\n",
                        drive_start_speed_current_ok ? "yes" : "no",
                        drive_start_speed_control_ok ? "yes" : "no",
                        drive_start_speed_average_ok ? "yes" : "no");
          Serial.printf("final_start_allowed=%s\n",
                        drive_start_stable ? "yes" : "no");
          Serial.printf(
            "LQR DRIVE START CHECK: "
            "gate_speed=%+.6f "
            "control_speed=%+.6f "
            "average_speed=%+.6f "
            "gate_ok=%s "
            "control_ok=%s "
            "average_ok=%s "
            "samples=%lu "
            "gyro=%+.6f "
            "angle_deviation=%+.6f\n",
            drive_start_gate_speed,
            lqr_speed_for_control,
            lqr_drive_start_speed_average,
            drive_start_speed_current_ok ? "yes" : "no",
            drive_start_speed_control_ok ? "yes" : "no",
            drive_start_speed_average_ok ? "yes" : "no",
            static_cast<unsigned long>(
                lqr_drive_start_speed_sample_count),
            LQR_gyro,
            drive_start_angle_deviation);
          startPendingLqrDrive();
        }
      } else {
        if (!drive_start_angle_ok) {
          ++lqr_drive_angle_fail_count;
        }

        if (!drive_start_gyro_ok) {
          ++lqr_drive_gyro_fail_count;
        }

        if (!drive_start_speed_ok) {
          ++lqr_drive_speed_fail_count;

          if (!drive_start_speed_current_ok) {
            ++lqr_drive_speed_current_fail_count;
          }

          if (!drive_start_speed_control_ok) {
            ++lqr_drive_speed_control_fail_count;
          }

          if (!drive_start_speed_average_ok) {
            ++lqr_drive_speed_average_fail_count;
          }
        }

        lqr_drive_stable_start_ms = 0;
      }

      if (lqr_drive_start_pending &&
          (lqr_drive_wait_diag_last_ms == 0 ||
           now_ms - lqr_drive_wait_diag_last_ms >=
               kLqrDriveWaitDiagIntervalMs)) {
        lqr_drive_wait_diag_last_ms = now_ms;

        const uint32_t stable_elapsed_ms =
            lqr_drive_stable_start_ms == 0
                ? 0
                : now_ms - lqr_drive_stable_start_ms;
        const uint32_t request_elapsed_ms =
            now_ms - lqr_drive_start_request_ms;

        if (drive_start_stable &&
            stable_elapsed_ms < kLqrDriveStartStableTimeMs) {
          drive_start_wait_reason =
              "STABLE_TIME_NOT_REACHED";
        }

        Serial.printf(
            "LQR DRIVE WAIT: command=%c requested_speed=%+.2f "
            "angle_error=%+.3f angle_reference=%+.3f "
            "angle_deviation=%+.3f gyro=%+.3f "
            "gate_speed=%+.3f average_speed=%+.3f "
            "reference_ok=%s angle_ok=%s gyro_ok=%s "
            "speed_current_ok=%s speed_average_ok=%s "
            "stable_elapsed_ms=%lu required_stable_ms=%lu "
            "request_elapsed_ms=%lu timeout_ms=%lu "
            "start_angle_limit=%.2f reason=%s\n",
            lqr_drive_pending_command,
            lqr_drive_pending_speed,
            drive_start_angle_error,
            lqr_drive_start_angle_reference,
            drive_start_angle_deviation,
            LQR_gyro,
            drive_start_gate_speed,
            lqr_drive_start_speed_average,
            drive_start_reference_ok ? "yes" : "no",
            drive_start_angle_ok ? "yes" : "no",
            drive_start_gyro_ok ? "yes" : "no",
            drive_start_speed_current_ok ? "yes" : "no",
            drive_start_speed_average_ok ? "yes" : "no",
            static_cast<unsigned long>(stable_elapsed_ms),
            static_cast<unsigned long>(
                kLqrDriveStartStableTimeMs),
            static_cast<unsigned long>(request_elapsed_ms),
            static_cast<unsigned long>(
                kLqrDriveStartTimeoutMs),
            kLqrDriveStartAngleReferenceLimitDeg,
            drive_start_wait_reason);
      }

      if (lqr_drive_start_pending &&
          now_ms - lqr_drive_start_request_ms >=
              kLqrDriveStartTimeoutMs) {
        Serial.printf(
            "LQR DRIVE START TIMEOUT: "
            "angle_error=%+.3f angle_ref=%+.3f "
            "angle_dev=%+.3f gyro=%+.3f "
            "speed_raw=%+.3f speed_avg=%+.3f "
            "speed_samples=%lu\n",
            drive_start_angle_error,
            lqr_drive_start_angle_reference,
            drive_start_angle_deviation,
            LQR_gyro,
            lqr_speed_for_control,
            lqr_drive_start_speed_average,
            static_cast<unsigned long>(
                lqr_drive_start_speed_sample_count));

        Serial.printf(
            "LQR DRIVE START SPEED DIAG: "
            "current=%+.6f "
            "signed_avg=%+.6f "
            "abs_avg=%+.6f "
            "abs_max=%+.6f "
            "diag_samples=%lu\n",
            lqr_speed_for_control,
            lqr_drive_start_speed_average,
            lqr_drive_start_speed_abs_average,
            lqr_drive_start_speed_abs_max,
            static_cast<unsigned long>(
                lqr_drive_start_speed_diag_count));

        if (lqr_drive_speed_source_diag_count > 0) {
          const float speed_source_diag_divisor =
              static_cast<float>(
                  lqr_drive_speed_source_diag_count);

          Serial.printf(
              "LQR DRIVE START WHEEL SPEED DIAG: "
              "samples=%lu "
              "sf_m1_abs_avg=%.6f sf_m1_abs_max=%.6f "
              "sf_m2_abs_avg=%.6f sf_m2_abs_max=%.6f\n",
              static_cast<unsigned long>(
                  lqr_drive_speed_source_diag_count),
              lqr_drive_sf_m1_abs_sum /
                  speed_source_diag_divisor,
              lqr_drive_sf_m1_abs_max,
              lqr_drive_sf_m2_abs_sum /
                  speed_source_diag_divisor,
              lqr_drive_sf_m2_abs_max);

          Serial.printf(
              "LQR DRIVE START DIAG WHEEL SPEED: "
              "right_abs_avg=%.6f right_abs_max=%.6f "
              "left_abs_avg=%.6f left_abs_max=%.6f\n",
              lqr_drive_diag_right_abs_sum /
                  speed_source_diag_divisor,
              lqr_drive_diag_right_abs_max,
              lqr_drive_diag_left_abs_sum /
                  speed_source_diag_divisor,
              lqr_drive_diag_left_abs_max);

          Serial.printf(
              "LQR DRIVE START SPEED SOURCE DIAG: "
              "control_abs_avg=%.6f control_abs_max=%.6f "
              "gate_abs_avg=%.6f gate_abs_max=%.6f "
              "diff_abs_avg=%.6f diff_abs_max=%.6f\n",
              lqr_drive_control_speed_abs_sum /
                  speed_source_diag_divisor,
              lqr_drive_control_speed_abs_max,
              lqr_drive_gate_speed_abs_sum /
                  speed_source_diag_divisor,
              lqr_drive_gate_speed_abs_max,
              lqr_drive_speed_source_diff_abs_sum /
                  speed_source_diag_divisor,
              lqr_drive_speed_source_diff_abs_max);
        }

        Serial.printf(
            "LQR DRIVE START DIAG: "
            "max_stable=%lu ms "
            "angle_fail=%lu gyro_fail=%lu "
            "speed_fail=%lu gate_speed_fail=%lu "
            "control_speed_fail=%lu average_speed_fail=%lu\n",
            static_cast<unsigned long>(
                lqr_drive_max_stable_ms),
            static_cast<unsigned long>(
                lqr_drive_angle_fail_count),
            static_cast<unsigned long>(
                lqr_drive_gyro_fail_count),
            static_cast<unsigned long>(
                lqr_drive_speed_fail_count),
            static_cast<unsigned long>(
                lqr_drive_speed_current_fail_count),
            static_cast<unsigned long>(
                lqr_drive_speed_control_fail_count),
            static_cast<unsigned long>(
                lqr_drive_speed_average_fail_count));

        const float angle_avg =
            lqr_drive_start_angle_sample_count > 0
                ? lqr_drive_start_angle_sum /
                      static_cast<float>(
                          lqr_drive_start_angle_sample_count)
                : 0.0f;

        Serial.printf(
            "LQR DRIVE START ANGLE DIAG: "
            "samples=%lu min=%+.3f max=%+.3f avg=%+.3f "
            "ref=%+.3f ref_ok=%s\n",
            static_cast<unsigned long>(
                lqr_drive_start_angle_sample_count),
            lqr_drive_start_angle_min,
            lqr_drive_start_angle_max,
            angle_avg,
            lqr_drive_start_angle_reference,
            abs(lqr_drive_start_angle_reference) <=
                    kLqrDriveStartAngleReferenceLimitDeg
                ? "yes"
                : "no");

        stopLqrDriveOnly("DRIVE_CONDITIONS_TIMEOUT", true);
      }
    }

    if (lqr_soft_stop_active) {
      lqr_drive_wrong_way_normal_since_ms = 0;
      lqr_drive_wrong_way_hard_since_ms = 0;

      // Braking mode:
      // target speed is already zero. Keep the distance target at the
      // current physical position until the wheels have actually
      // stopped, so position control cannot fight the braking motion.
      lqr_target_speed = 0.0f;
      distance_zeropoint = LQR_distance;

      const uint32_t now_ms = millis();

      const bool speed_is_low =
          abs(lqr_speed_for_control) <= kLqrSoftStopSpeedLimit;

      const bool speed_crossed_zero =
          (lqr_soft_stop_entry_speed > 0.0f &&
           lqr_speed_for_control <= 0.0f) ||
          (lqr_soft_stop_entry_speed < 0.0f &&
           lqr_speed_for_control >= 0.0f);

      // Observe the existing soft-stop inputs while a PS4 stop is pending.
      // These counters and extrema are diagnostic only.
      if (ps4_drive_state == Ps4DriveState::Stopping) {
        stop_speed_min = min(stop_speed_min, lqr_speed_for_control);
        stop_speed_max = max(stop_speed_max, lqr_speed_for_control);
        stop_speed_abs_max = max(stop_speed_abs_max,
                                 abs(lqr_speed_for_control));
        if (speed_is_low && !soft_stop_speed_was_low) {
          ++soft_stop_low_speed_enter_count;
        }
        if (!speed_is_low && lqr_soft_stop_stable_start_ms != 0) {
          ++soft_stop_stable_reset_count;
        }
        soft_stop_speed_was_low = speed_is_low;
      }

      if (speed_is_low) {
        if (lqr_soft_stop_stable_start_ms == 0) {
          lqr_soft_stop_stable_start_ms = now_ms;
        }
      } else {
        lqr_soft_stop_stable_start_ms = 0;
      }

      const bool speed_stable =
          lqr_soft_stop_stable_start_ms != 0 &&
          now_ms - lqr_soft_stop_stable_start_ms >=
              kLqrSoftStopStableTimeMs;

      if (speed_crossed_zero || speed_stable) {
        lqr_soft_stop_active = false;
        lqr_soft_stop_stable_start_ms = 0;
        lqr_soft_stop_entry_speed = 0.0f;

        distance_zeropoint = LQR_distance;
        pid_distance.reset();
        pid_speed.reset();

        printLqrDriveSnapshot("POSITION_HOLD");
        Serial.printf(
            "LQR DRIVE STOP: position hold engaged, "
            "speed=%.3f reason=%s\n",
            lqr_speed_for_control,
            speed_crossed_zero
                ? "ZERO_CROSS"
                : "LOW_SPEED_STABLE");
      }
    } else {
      if (lqr_auto_drive_active) {
        // Only during an active drive command should the distance
        // target follow the physical wheel position. While waiting
        // after AUTO START, keep the captured position target so the
        // robot retains normal position hold.
        distance_zeropoint = LQR_distance;

        const float distance_delta =
            LQR_distance - lqr_drive_start_distance;

        const bool drive_positive =
            lqr_target_speed > 0.0f;

        const float traveled =
            drive_positive ? distance_delta : -distance_delta;

        const float wrong_way =
            drive_positive ? -distance_delta : distance_delta;

        if (wrong_way > lqr_drive_wrong_way_peak) {
          lqr_drive_wrong_way_peak = wrong_way;
        }

        const uint32_t drive_elapsed_ms =
            millis() - lqr_drive_motion_start_ms;

        const bool wrong_way_grace_active =
            drive_elapsed_ms < kLqrDriveWrongWayGraceTimeMs;

        const float actual_motion_velocity =
            motor1.shaft_velocity * m1_direction +
            motor2.shaft_velocity * m2_direction;

        // Distance thresholds reject small measurement noise, while the
        // velocity sign makes persistence consecutive: normal balancing
        // motion back in the requested direction immediately clears both
        // timers even if the accumulated distance has not yet recovered.
        const bool moving_wrong_way =
            (drive_positive && actual_motion_velocity < 0.0f) ||
            (!drive_positive && actual_motion_velocity > 0.0f);

        const uint32_t now_ms = millis();
        if (!wrong_way_grace_active && moving_wrong_way &&
            wrong_way >= kLqrDriveWrongWayLimit) {
          if (lqr_drive_wrong_way_normal_since_ms == 0) {
            lqr_drive_wrong_way_normal_since_ms = now_ms;
          }
        } else {
          lqr_drive_wrong_way_normal_since_ms = 0;
        }

        // Unlike the normal guard, the hard guard is never delayed by the
        // startup grace period.  It applies from the first drive sample.
        if (wrong_way >= kLqrDriveWrongWayHardLimit) {
          if (lqr_drive_wrong_way_hard_since_ms == 0) {
            lqr_drive_wrong_way_hard_since_ms = now_ms;
          }
        } else {
          lqr_drive_wrong_way_hard_since_ms = 0;
        }

        const uint32_t wrong_way_normal_duration_ms =
            lqr_drive_wrong_way_normal_since_ms == 0
                ? 0
                : now_ms - lqr_drive_wrong_way_normal_since_ms;
        const uint32_t wrong_way_hard_duration_ms =
            lqr_drive_wrong_way_hard_since_ms == 0
                ? 0
                : now_ms - lqr_drive_wrong_way_hard_since_ms;

        const bool wrong_way_hard_fault =
            wrong_way_hard_duration_ms >=
            kLqrDriveWrongWayHardPersistMs;

        const bool wrong_way_normal_fault =
            wrong_way_normal_duration_ms >=
            kLqrDriveWrongWayNormalPersistMs;

        if (traveled >= kLqrDriveDecelStartDistance &&
            traveled < kLqrDriveAutoDistance) {
          const float decel_range =
              kLqrDriveAutoDistance -
              kLqrDriveDecelStartDistance;

          const float remaining =
              kLqrDriveAutoDistance - traveled;

          float speed_scale =
              remaining / decel_range;

          if (speed_scale < 0.0f) {
            speed_scale = 0.0f;
          } else if (speed_scale > 1.0f) {
            speed_scale = 1.0f;
          }

          const float commanded_speed =
              kLqrDriveTestSpeed * speed_scale;

          lqr_target_speed =
              drive_positive
                  ? commanded_speed
                  : -commanded_speed;
        }

        if (wrong_way_hard_fault ||
            wrong_way_normal_fault) {
          lqr_auto_drive_active = false;
          lqr_target_speed = 0.0f;
          lqr_drive_angle_bias = 0.0f;
          lqr_soft_stop_active = false;
          lqr_soft_stop_stable_start_ms = 0;

          pid_distance.reset();
          pid_speed.reset();

          Serial.printf(
              "LQR DRIVE WRONG-WAY STOP: "
              "wrong_way=%.3f peak=%.3f elapsed=%lu ms "
              "wrong_way_duration_ms=%lu normal_limit=%.3f "
              "hard_limit=%.3f reason=%s\n",
              wrong_way,
              lqr_drive_wrong_way_peak,
              static_cast<unsigned long>(drive_elapsed_ms),
              static_cast<unsigned long>(
                  wrong_way_hard_fault
                      ? wrong_way_hard_duration_ms
                      : wrong_way_normal_duration_ms),
              kLqrDriveWrongWayLimit,
              kLqrDriveWrongWayHardLimit,
              wrong_way_hard_fault ? "HARD_LIMIT"
                                   : "POST_GRACE_LIMIT");
          printLqrDriveSnapshot("WRONG_WAY_DRIVE_STOP");

          if (lqr_drive_sign_trace_count > 0) {
            printLqrDriveSignTrace("WRONG_WAY_DRIVE_STOP");
            resetLqrDriveSignTrace();
          }

          stopLqrDriveOnly("WRONG_WAY", true);
        } else if (traveled >=
                   (ps4_drive_state == Ps4DriveState::DriveRequested
                        ? kPs4DriveTestTravelLimit
                        : kLqrDriveTravelLimit)) {
          if (ps4_drive_state == Ps4DriveState::DriveRequested) {
            requestLqrSoftStop();
            beginPs4DriveStopping(millis());

            Serial.println("PS4 DRIVE SAFETY STOP:");
            Serial.printf("travel=%.3f\n", traveled);
            Serial.printf("entry_speed=%+.3f\n",
                          lqr_soft_stop_entry_speed);
            Serial.printf("target_speed=%+.3f\n", lqr_target_speed);
            Serial.printf("ps4_drive_state=%s\n",
                          ps4DriveStateName(ps4_drive_state));
            printLqrDriveSnapshot("PS4_SAFETY_SOFT_STOP");
            startLqrStopTrace();
          } else {
            lqr_auto_drive_active = false;
            lqr_target_speed = 0.0f;
            lqr_soft_stop_active = false;
            lqr_soft_stop_stable_start_ms = 0;
            lqr_drive_wrong_way_normal_since_ms = 0;
            lqr_drive_wrong_way_hard_since_ms = 0;

            distance_zeropoint = LQR_distance;
            pid_distance.reset();
            pid_speed.reset();

            // Restore distance hold from the current physical position.
            Serial.printf(
                "LQR DRIVE SAFETY STOP: traveled=%.3f\n",
                traveled);
            printLqrDriveSnapshot("SAFETY_HOLD");
          }
        } else if (ps4_drive_state != Ps4DriveState::DriveRequested &&
                   traveled >= kLqrDriveAutoDistance) {
          lqr_auto_drive_active = false;

          // Stop the commanded speed immediately, but do not engage
          // position hold while the wheels are still moving.
          // During braking, distance_zeropoint follows LQR_distance.
          lqr_target_speed = 0.0f;
          lqr_soft_stop_active = true;
          lqr_soft_stop_stable_start_ms = 0;
          lqr_soft_stop_entry_speed = lqr_speed_for_control;
          lqr_drive_wrong_way_normal_since_ms = 0;
          lqr_drive_wrong_way_hard_since_ms = 0;

          distance_zeropoint = LQR_distance;
          pid_distance.reset();
          pid_speed.reset();

          Serial.printf(
              "LQR DRIVE AUTO STOP: traveled=%.3f, "
              "zero-speed braking entry_speed=%+.3f\n",
              traveled,
              lqr_soft_stop_entry_speed);
          printLqrDriveSnapshot("AUTO_STOP_BRAKE");
          startLqrStopTrace();
        }
      }
    }
  }

  const bool ps4_speed_outer_loop_active =
      lqr_auto_drive_active &&
      ps4_drive_state == Ps4DriveState::DriveRequested;

  // PS4 driving always closes its outer speed loop with the wheel-angle
  // change measured over kPs4SpeedWindowMs.  In particular, this selection
  // must not depend on whether the optional direct motor term is enabled:
  // SimpleFOC's instantaneous speed and the existing balance-loop speed are
  // intentionally excluded from the PS4 speed error.
  const float drive_speed_for_control =
      ps4_speed_outer_loop_active ? lqr_ps4_window_speed
                                  : lqr_speed_for_control;
  const float drive_speed_error =
      lqr_target_speed - drive_speed_for_control;

  speed_control = pid_speed(drive_speed_error);

  if (lqr_auto_drive_active || lqr_soft_stop_active ||
      lqr_target_speed != 0.0f) {
    speed_control *= kLqrDriveSpeedControlGain;
  }

  const bool ps4_target_tilt_active = ps4_speed_outer_loop_active;

  // Recovery profile: use one fixed, tiny lean proportional to the requested
  // speed.  There is no independently captured drive posture, adaptive
  // authority, or integral offset in this path.
  if (ps4_target_tilt_active) {
    const float tilt_limit_dt =
        constrain(control_dt_ms / 1000.0f, 0.0f, 0.05f);
    lqr_ps4_target_tilt_limit_deg = kPs4DriveTargetTiltInitialLimitDeg;
    lqr_ps4_target_tilt_integral_deg = 0.0f;
    const float requested_target_tilt = constrain(
        lqr_target_speed / kPs4DriveMaxSpeed,
        -1.0f, 1.0f) * kPs4DriveTargetTiltInitialLimitDeg;
    const float max_tilt_step =
        kPs4DriveTargetTiltSlewDegPerSec * tilt_limit_dt;
    lqr_ps4_target_tilt_deg += constrain(
        requested_target_tilt - lqr_ps4_target_tilt_deg,
        -max_tilt_step, max_tilt_step);
  } else {
    lqr_ps4_target_tilt_limit_deg =
        kPs4DriveTargetTiltInitialLimitDeg;
    lqr_ps4_target_tilt_integral_deg = 0.0f;
    lqr_ps4_target_tilt_deg = 0.0f;
  }

  const float lqr_ps4_target_tilt =
      ps4_target_tilt_active ? lqr_ps4_target_tilt_deg : 0.0f;

  if (lqr_auto_drive_active && !ps4_target_tilt_active) {
    if (kDisableDriveAngleBias) {
      lqr_drive_angle_bias = 0.0f;
    } else {
      lqr_drive_angle_bias =
          drive_speed_error * kLqrDriveAngleBiasGain;

      if (lqr_drive_angle_bias >
          kLqrDriveAngleBiasLimitDeg) {
        lqr_drive_angle_bias =
            kLqrDriveAngleBiasLimitDeg;
      } else if (lqr_drive_angle_bias <
                 -kLqrDriveAngleBiasLimitDeg) {
        lqr_drive_angle_bias =
            -kLqrDriveAngleBiasLimitDeg;
      }
    }
  } else {
    lqr_drive_angle_bias = 0.0f;
  }

  // PS4 speed is an outer loop: its output is a requested body tilt, while
  // angle and gyro remain the balancing inner loop.  The existing inner-loop
  // motor polarity accelerates the measured wheel speed in the same direction
  // as target_tilt, so negate the speed-controller output here.  This keeps
  // the established balancing gains and motor direction settings unchanged
  // while making positive/negative target_speed mean forward/backward in the
  // measured-speed convention used by the wrong-way guard.
  const float lqr_angle_error =
      LQR_angle - (angle_zeropoint + lqr_ps4_target_tilt) +
      lqr_drive_angle_bias;
  angle_control = pid_angle(lqr_angle_error);
  gyro_control = pid_gyro(LQR_gyro);

  if (lqr_auto_drive_active ||
      drive_recovery_state == DriveRecoveryState::AttitudeOnly) {
    // During an active drive command, do not let position control
    // fight the requested wheel motion.
    distance_control = 0.0f;
  } else {
    distance_control = pid_distance(distance_zeropoint - LQR_distance);
    if (drive_recovery_state == DriveRecoveryState::PositionRamp) {
      const float ramp = constrain(
          static_cast<float>(millis() - drive_recovery_position_ramp_start_ms) /
              static_cast<float>(kDriveRecoveryPositionRampMs),
          0.0f, 1.0f);
      distance_control *= ramp;
    }
  }

  float lqr_speed_direct_term = speed_control;

  if (lqr_auto_drive_active) {
    if (ps4_target_tilt_active) {
      // Do not apply the same PS4 speed error to both the outer tilt loop and
      // the motor command.  Direction Test and soft-stop retain their existing
      // direct-speed paths.
      lqr_speed_direct_term = 0.0f;
    } else if (kDisableDirectDriveSpeedControl) {
      lqr_speed_direct_term = 0.0f;

      // PS4 driving is a speed command, so retain a bounded path from its
      // target into the existing speed loop.  Use the already-established
      // reversed drive sign, but cap only this active-drive contribution;
      // soft-stop continues to use its existing uncapped braking path below.
      if (ps4_drive_state == Ps4DriveState::DriveRequested &&
          kEnablePs4LimitedDirectSpeedControl) {
        const float signed_speed_control =
            kReverseDirectDriveSpeedControl ? -speed_control
                                            : speed_control;
        lqr_speed_direct_term =
            constrain(signed_speed_control,
                      -kPs4DirectSpeedControlLimit,
                      kPs4DirectSpeedControlLimit);
      }

      const bool wrong_way_speed_confirmed =
          kEnableWrongWaySpeedAssist &&
          abs(diag_LQR_speed_10ms) >=
              kLqrDriveWrongWaySpeedAssistThreshold &&
          lqr_target_speed * diag_LQR_speed_10ms < 0.0f;

      if (wrong_way_speed_confirmed &&
          lqr_speed_direct_term == 0.0f) {
        lqr_speed_direct_term =
            constrain(
                -speed_control,
                -kLqrDriveWrongWaySpeedAssistLimit,
                kLqrDriveWrongWaySpeedAssistLimit);
      }
    } else if (kReverseDirectDriveSpeedControl) {
      lqr_speed_direct_term = -speed_control;
    }
  } else if (lqr_soft_stop_active) {
    // During soft stop, use speed feedback directly as braking.
    // Active drive may disable direct speed control, but braking
    // still needs the zero-speed correction to reach LQR_u.
    lqr_speed_direct_term = speed_control;
  }

  // A DRIVE_ONLY recovery deliberately gives the inner attitude loop sole
  // authority.  Stale wheel-speed feedback must not become a second braking
  // command while the body is still settling.
  if (drive_recovery_state != DriveRecoveryState::Inactive) {
    lqr_speed_direct_term = 0.0f;
  }

  LQR_u =
      angle_control +
      gyro_control +
      distance_control +
      lqr_speed_direct_term;

  // Update the actual test-control output before capturing traces
  // so LQR_u, test_LQR_u and reconstructed motor outputs belong
  // to the same control cycle.
  updateTestLqrOutput(
      angle_control,
      gyro_control,
      distance_control,
      lqr_speed_direct_term);

  if (lqr_auto_drive_active &&
      ps4_drive_state == Ps4DriveState::DriveRequested) {
    const uint32_t now_ms = millis();
    if (now_ms - ps4_drive_control_diagnostic_last_ms >=
        kPs4DriveControlDiagnosticIntervalMs) {
      ps4_drive_control_diagnostic_last_ms = now_ms;
      const float motor1_output_sign =
          m1_direction < 0.0f ? -1.0f : 1.0f;
      const float motor2_output_sign =
          m2_direction < 0.0f ? -1.0f : 1.0f;
      const float diagnostic_right_output =
          constrain(-test_LQR_u * motor1_output_sign,
                    -kLqrTestVoltageLimit, kLqrTestVoltageLimit);
      const float diagnostic_left_output =
          constrain(-test_LQR_u * motor2_output_sign,
                    -kLqrTestVoltageLimit, kLqrTestVoltageLimit);

      Serial.printf(
          "PS4 DRIVE CONTROL: drive_reference=%+.6f current_angle=%+.6f "
          "angle_from_reference=%+.6f target_speed=%+.6f "
          "actual_window_speed=%+.6f speed_error=%+.6f "
          "target_tilt=%+.6f target_tilt_limit=%+.6f "
          "tilt_integral=%+.6f wheel_angle_delta_right=%+.6f "
          "wheel_angle_delta_left=%+.6f right_output=%+.6f "
          "left_output=%+.6f\n",
          lqr_ps4_drive_angle_reference, LQR_angle,
          LQR_angle - lqr_ps4_drive_angle_reference, lqr_target_speed,
          drive_speed_for_control, drive_speed_error,
          lqr_ps4_target_tilt, lqr_ps4_target_tilt_limit_deg,
          lqr_ps4_target_tilt_integral_deg,
          lqr_ps4_window_angle_delta_right,
          lqr_ps4_window_angle_delta_left, diagnostic_right_output,
          diagnostic_left_output);
      Serial.printf("stick_ly=%d\n", ps4_drive_stick_y);
      Serial.printf(
          "simplefoc_speed=%+.6f speed_10ms=%+.6f\n",
          LQR_speed, diag_LQR_speed_10ms);
      Serial.printf(
          "existing_lqr_speed_for_control=%+.6f "
          "ps4_window_speed=%+.6f ps4_speed_for_control=%+.6f\n",
          lqr_speed_for_control, lqr_ps4_window_speed,
          lqr_ps4_speed_for_control);
      Serial.printf(
          "window_elapsed_ms=%lu wheel_angle_delta_right=%+.6f "
          "wheel_angle_delta_left=%+.6f speed_error=%+.6f\n",
          static_cast<unsigned long>(lqr_ps4_window_elapsed_ms),
          lqr_ps4_window_angle_delta_right,
          lqr_ps4_window_angle_delta_left, drive_speed_error);
      Serial.printf(
          "speed_control=%+.6f speed_direct_term=%+.6f "
          "target_tilt=%+.6f target_tilt_limit=%+.6f "
          "angle_error=%+.6f\n",
          speed_control, lqr_speed_direct_term,
          lqr_ps4_target_tilt, lqr_ps4_target_tilt_limit_deg,
          lqr_angle_error);
      Serial.printf(
          "angle_control=%+.6f gyro_control=%+.6f\n",
          angle_control, gyro_control);
      Serial.printf(
          "test_LQR_u=%+.6f right_output=%+.6f left_output=%+.6f\n",
          test_LQR_u, diagnostic_right_output,
          diagnostic_left_output);
    }
  }

  if (lqr_auto_drive_active) {
    captureLqrDriveSignTrace(lqr_speed_direct_term);
  }

  if (lqr_stop_trace_active) {
    captureLqrStopTrace(lqr_speed_direct_term);

    const uint32_t stop_trace_elapsed_ms =
        millis() - lqr_stop_trace_start_ms;

    if (lqr_stop_trace_count >=
            kLqrStopTraceMaxSamples ||
        stop_trace_elapsed_ms >=
            kLqrStopTraceDurationMs) {
      lqr_stop_trace_active = false;
    }
  }

}

void printLqrSoftStopSpeedDiagnostic(float right_output,
                                     float left_output) {
  if (!lqr_soft_stop_active) {
    lqr_soft_stop_diag_previous_valid = false;
    return;
  }

  const uint32_t now_ms = millis();
  if (lqr_speed_sources_diag_last_ms != 0 &&
      now_ms - lqr_speed_sources_diag_last_ms <
          kLqrSpeedSourcesDiagIntervalMs) {
    return;
  }
  lqr_speed_sources_diag_last_ms = now_ms;

  const float simplefoc_right = motor1.shaft_velocity * m1_direction;
  const float simplefoc_left = motor2.shaft_velocity * m2_direction;
  const float speed_10ms_right = diag_right_velocity_10ms * m1_direction;
  const float speed_10ms_left = diag_left_velocity_10ms * m2_direction;

  Serial.println("LQR SOFT STOP SPEED:");
  Serial.printf("ps4_drive_speed_request=%+.6f\n", ps4_drive_speed_request);
  Serial.printf("target_speed=%+.6f\n", lqr_target_speed);
  Serial.printf("simplefoc_right_raw=%+.6f\n", motor1.shaft_velocity);
  Serial.printf("simplefoc_left_raw=%+.6f\n", motor2.shaft_velocity);
  Serial.printf(
      "right_correction_factor=%+.1f left_correction_factor=%+.1f\n",
      m1_direction, m2_direction);
  Serial.println("reversed_speed_sign_applied=no");
  Serial.printf("direct_drive_speed_control_sign=%s\n",
                kReverseDirectDriveSpeedControl ? "REVERSED" : "NORMAL");
  Serial.printf("simplefoc_right_corrected=%+.6f\n", simplefoc_right);
  Serial.printf("simplefoc_left_corrected=%+.6f\n", simplefoc_left);
  Serial.printf("simplefoc_combined=%+.6f\n", LQR_speed);
  Serial.printf("simplefoc_corrected_difference=%+.6f\n",
                simplefoc_right - simplefoc_left);
  Serial.printf("speed_10ms_right_raw=%+.6f\n", diag_right_velocity_10ms);
  Serial.printf("speed_10ms_left_raw=%+.6f\n", diag_left_velocity_10ms);
  Serial.printf("speed_10ms_right_corrected=%+.6f\n", speed_10ms_right);
  Serial.printf("speed_10ms_left_corrected=%+.6f\n", speed_10ms_left);
  Serial.printf("speed_10ms_combined=%+.6f\n", diag_LQR_speed_10ms);
  Serial.printf("speed_10ms_corrected_difference=%+.6f\n",
                speed_10ms_right - speed_10ms_left);
  Serial.printf("selected_source=%s\n",
                lqr_speed_source == LqrSpeedSource::SimpleFoc
                    ? "SIMPLEFOC" : "10MS");
  Serial.printf("selected_speed_raw=%+.6f\n", lqr_speed_raw_for_control);
  Serial.printf("lqr_speed_for_control=%+.6f\n", lqr_speed_for_control);
  Serial.printf("simplefoc_age_ms=%lu speed10ms_age_ms=%lu "
                "control_age_ms=%lu\n",
                static_cast<unsigned long>(now_ms - lqr_simplefoc_speed_read_ms),
                static_cast<unsigned long>(now_ms - lqr_speed_10ms_update_ms),
                static_cast<unsigned long>(now_ms - lqr_control_speed_update_ms));
  Serial.printf("angle_error=%+.6f gyro=%+.6f distance_error=%+.6f\n",
                LQR_angle - angle_zeropoint, LQR_gyro,
                distance_zeropoint - LQR_distance);
  Serial.printf("speed_control=%+.6f output_right=%+.6f output_left=%+.6f\n",
                speed_control, right_output, left_output);

  if (lqr_soft_stop_diag_previous_valid) {
    const float speed_delta =
        lqr_speed_for_control - lqr_soft_stop_diag_previous_speed;
    if (abs(speed_delta) >= kLqrSpeedJumpDiagThreshold) {
      Serial.printf("LQR SPEED JUMP: previous=%+.6f current=%+.6f "
                    "delta=%+.6f\n",
                    lqr_soft_stop_diag_previous_speed,
                    lqr_speed_for_control, speed_delta);
    }
  }
  lqr_soft_stop_diag_previous_speed = lqr_speed_for_control;
  lqr_soft_stop_diag_previous_valid = true;
}

void printAndResetLqrRunTime() {
  if (lqr_direction_test_start_ms == 0) {
    return;
  }

  Serial.printf("lqr_run_time_ms=%lu\n",
                static_cast<unsigned long>(
                    millis() - lqr_direction_test_start_ms));
  lqr_direction_test_start_ms = 0;
}

void resetLqrTestState() {
  drive_recovery_state = DriveRecoveryState::Inactive;
  drive_recovery_start_ms = 0;
  drive_recovery_stable_since_ms = 0;
  drive_recovery_position_ramp_start_ms = 0;
  drive_recovery_diagnostic_last_ms = 0;
  fall_exceeded_since_ms = 0;
  fall_exceeded_samples = 0;
  fall_diagnostic_last_ms = 0;
  fall_previous_angle_error = 0.0f;
  fall_previous_reference = 0.0f;
  fall_previous_sample_valid = false;
  pid_angle.reset();
  pid_gyro.reset();
  pid_distance.reset();
  pid_speed.reset();

  angle_control = 0.0f;
  gyro_control = 0.0f;
  distance_control = 0.0f;
  speed_control = 0.0f;
  LQR_u = 0.0f;
  test_LQR_u = 0.0f;

  diag_right_velocity_10ms = 0.0f;
  diag_left_velocity_10ms = 0.0f;
  diag_LQR_speed_10ms = 0.0f;
  lqr_speed_raw_for_control = 0.0f;
  lqr_speed_for_control = 0.0f;
  lqr_control_speed_filter_initialized = false;
  lqr_ps4_speed_for_control = 0.0f;
  lqr_ps4_window_speed = 0.0f;
  lqr_ps4_window_elapsed_ms = 0;
  lqr_ps4_window_angle_delta_right = 0.0f;
  lqr_ps4_window_angle_delta_left = 0.0f;
  lqr_ps4_target_tilt_limit_deg =
      kPs4DriveTargetTiltInitialLimitDeg;
  lqr_ps4_drive_angle_reference = 0.0f;
  lqr_ps4_target_tilt_integral_deg = 0.0f;
  lqr_ps4_target_tilt_deg = 0.0f;
  for (size_t i = 0; i < kPs4SpeedWindowHistorySize; ++i) {
    lqr_ps4_speed_history[i] = {};
  }
  lqr_ps4_speed_history_next = 0;
  lqr_ps4_speed_history_count = 0;
  lqr_target_speed = 0.0f;
  lqr_drive_angle_bias = 0.0f;
  lqr_drive_start_pending = false;
  lqr_drive_pending_command = 0;
  lqr_drive_pending_speed = 0.0f;
  lqr_drive_start_request_ms = 0;
  lqr_drive_stable_start_ms = 0;
  lqr_drive_max_stable_ms = 0;
  lqr_drive_angle_fail_count = 0;
  lqr_drive_gyro_fail_count = 0;
  lqr_drive_speed_fail_count = 0;
  lqr_drive_start_angle_min = 0.0f;
  lqr_drive_start_angle_max = 0.0f;
  lqr_drive_start_angle_sum = 0.0f;
  lqr_drive_start_angle_sample_count = 0;
  lqr_drive_start_angle_reference_sum = 0.0f;
  lqr_drive_start_angle_reference_count = 0;
  lqr_drive_start_angle_reference = 0.0f;
  lqr_drive_start_angle_reference_ready = false;
  for (size_t i = 0;
       i < kLqrDriveStartSpeedAverageSamples;
       ++i) {
    lqr_drive_start_speed_samples[i] = 0.0f;
  }
  lqr_drive_start_speed_sample_index = 0;
  lqr_drive_start_speed_sample_count = 0;
  lqr_drive_start_speed_sum = 0.0f;
  lqr_drive_start_speed_average = 0.0f;
  lqr_drive_start_speed_abs_sum = 0.0f;
  lqr_drive_start_speed_abs_average = 0.0f;
  lqr_drive_start_speed_abs_max = 0.0f;
  lqr_drive_start_speed_diag_count = 0;
  lqr_drive_speed_source_diag_last_ms = 0;
  lqr_drive_speed_source_diag_count = 0;
  lqr_drive_sf_m1_abs_sum = 0.0f;
  lqr_drive_sf_m1_abs_max = 0.0f;
  lqr_drive_sf_m2_abs_sum = 0.0f;
  lqr_drive_sf_m2_abs_max = 0.0f;
  lqr_drive_diag_right_abs_sum = 0.0f;
  lqr_drive_diag_right_abs_max = 0.0f;
  lqr_drive_diag_left_abs_sum = 0.0f;
  lqr_drive_diag_left_abs_max = 0.0f;
  lqr_drive_control_speed_abs_sum = 0.0f;
  lqr_drive_control_speed_abs_max = 0.0f;
  lqr_drive_gate_speed_abs_sum = 0.0f;
  lqr_drive_gate_speed_abs_max = 0.0f;
  lqr_drive_speed_source_diff_abs_sum = 0.0f;
  lqr_drive_speed_source_diff_abs_max = 0.0f;
  lqr_soft_stop_active = false;
  lqr_soft_stop_stable_start_ms = 0;
  lqr_auto_drive_active = false;
  lqr_drive_wrong_way_normal_since_ms = 0;
  lqr_drive_wrong_way_hard_since_ms = 0;
  lqr_drive_start_distance = 0.0f;
  lqr_drive_origin_distance = 0.0f;

  lqr_drive_sign_trace_count = 0;
  lqr_drive_sign_trace_start_ms = 0;
  lqr_drive_sign_trace_last_ms = 0;
  lqr_drive_sign_trace_active = false;

  previous_right_angle = motor1.shaft_angle;
  previous_left_angle = motor2.shaft_angle;
  previous_velocity_diagnostic_us = micros();

  last_control_update_us = micros();
  control_dt_ms = 0.0f;

  Serial.println("LQR test state reset.");
}

//==================================================
// Safety / Balance Wait
//==================================================
void setStatusLed(bool on) {
  status_led_on = on;
  digitalWrite(ledPin, on ? HIGH : LOW);
}

void cancelBalanceWait() {
  balance_wait_active = false;
  lqr_startup_diagnostic_active = false;
  balance_prepare_active = false;
  balance_prepare_start_ms = 0;
  balance_stable_start_ms = 0;
  balance_stable_elapsed_ms = 0;
  balance_candidate_angle = 0.0f;
  balance_angle_sum = 0.0f;
  balance_distance_sum = 0.0f;
  balance_sample_count = 0;
  balance_stable_message_shown = false;
  balance_wait_last_diagnostic_ms = 0;
  balance_timer_reset = false;
  balance_brake_right = 0.0f;
  balance_brake_left = 0.0f;
  setStatusLed(false);
}

void updateBalanceWait() {
  if (!balance_wait_active) {
    return;
  }

  if (balance_brake_enabled) {
    balance_brake_right = constrain(speed_control * m1_direction,
                                    -kBalanceBrakeVoltageLimit,
                                    kBalanceBrakeVoltageLimit);
    balance_brake_left = constrain(speed_control * m2_direction,
                                   -kBalanceBrakeVoltageLimit,
                                   kBalanceBrakeVoltageLimit);
  } else {
    balance_brake_right = 0.0f;
    balance_brake_left = 0.0f;
  }
  right_target_voltage = balance_brake_right;
  left_target_voltage = balance_brake_left;
  motor1.target = balance_brake_right;
  motor2.target = balance_brake_left;
  const uint32_t now_ms = millis();
  const bool gyro_ok = abs(LQR_gyro) <= kBalanceWaitGyroLimitDegPerSec;
  const bool wheel_speed_ok =
      abs(LQR_speed) <= kBalanceWaitWheelSpeedLimit;
  bool stabilizing = balance_stable_start_ms != 0;
  bool counting = false;
  if (balance_prepare_active &&
      now_ms - balance_prepare_start_ms >= kBalancePrepareTimeMs) {
    balance_prepare_active = false;
    if (!balance_stable_message_shown) {
      balance_stable_message_shown = true;
      Serial.println("BALANCE MEASURE:");
      Serial.println("Hold the robot at its neutral position.");
    }
  }

  if (!balance_prepare_active) {
    if (!stabilizing && gyro_ok && wheel_speed_ok) {
      balance_candidate_angle = LQR_angle;
      balance_stable_start_ms = now_ms;
      balance_timer_reset = false;
      stabilizing = true;
      counting = true;
      balance_angle_sum = LQR_angle;
      balance_distance_sum = LQR_distance;
      balance_sample_count = 1;
    } else if (stabilizing) {
      const float current_angle_delta = LQR_angle - balance_candidate_angle;
      const bool current_hold_ok =
          abs(current_angle_delta) <= kBalanceWaitAngleLimitDeg &&
          abs(LQR_gyro) <= kBalanceHoldGyroLimitDegPerSec &&
          wheel_speed_ok;

      const bool current_reset_ok =
          abs(current_angle_delta) <= kBalanceResetAngleLimitDeg &&
          abs(LQR_gyro) <= kBalanceResetGyroLimitDegPerSec &&
          wheel_speed_ok;
      if (!current_reset_ok) {
        balance_stable_start_ms = 0;
        balance_stable_elapsed_ms = 0;
        balance_candidate_angle = 0.0f;
        balance_angle_sum = 0.0f;
        balance_distance_sum = 0.0f;
        balance_sample_count = 0;
        balance_timer_reset = true;
        stabilizing = false;
      } else {
        if (current_hold_ok) {
            balance_stable_elapsed_ms += now_ms - balance_stable_start_ms;
            balance_angle_sum += LQR_angle;
            balance_distance_sum += LQR_distance;
            balance_sample_count++;
            counting = true;
        } else {
            // Stability must be continuous. Do not mix samples collected
            // before and after a hold-condition break.
            balance_stable_elapsed_ms = 0;
            balance_angle_sum = 0.0f;
            balance_distance_sum = 0.0f;
            balance_sample_count = 0;
        }
        balance_stable_start_ms = now_ms;
      }
    }
  }

  const float angle_delta =
      stabilizing ? LQR_angle - balance_candidate_angle : 0.0f;
  const bool angle_ok =
      stabilizing && abs(angle_delta) <= kBalanceWaitAngleLimitDeg;
  const bool hold_ok =
      angle_ok &&
      abs(LQR_gyro) <= kBalanceHoldGyroLimitDegPerSec &&
      wheel_speed_ok;
  const uint32_t stable_timer_ms = balance_stable_elapsed_ms;
  const uint32_t led_blink_ms = !balance_prepare_active && stabilizing
                                    ? kStatusLedStabilizingBlinkMs
                                    : kStatusLedWaitBlinkMs;
  if (now_ms - status_led_last_toggle_ms >= led_blink_ms) {
    status_led_last_toggle_ms = now_ms;
    setStatusLed(!status_led_on);
  }

  if (now_ms - balance_wait_last_diagnostic_ms >=
      kBalanceWaitDiagnosticMs) {
    balance_wait_last_diagnostic_ms = now_ms;
    Serial.println("BALANCE WAIT:");
    Serial.printf("angle_error=%.2f deg\n", angle_delta);
    Serial.printf("gyro=%.2f deg/s\n", LQR_gyro);
    Serial.printf("motor1_angle=%.6f\n", motor1.shaft_angle);
    Serial.printf("motor1_velocity=%.6f\n", motor1.shaft_velocity);
    Serial.printf("motor2_angle=%.6f\n", motor2.shaft_angle);
    Serial.printf("motor2_velocity=%.6f\n", motor2.shaft_velocity);
    Serial.printf("LQR_speed=%.6f\n", LQR_speed);
    Serial.printf("speed_control=%.6f\n", speed_control);
    Serial.printf("balance_brake=%s\n", balance_brake_enabled ? "ON" : "OFF");
    Serial.printf("balance_brake_right=%.2f V\n", balance_brake_right);
    Serial.printf("balance_brake_left=%.2f V\n", balance_brake_left);
    Serial.printf("angle_ok=%s\n", angle_ok ? "yes" : "no");
    Serial.printf("gyro_ok=%s\n", gyro_ok ? "yes" : "no");
    Serial.printf("hold_ok=%s\n", hold_ok ? "yes" : "no");
    const char* balance_state = balance_prepare_active
                                    ? "PREPARE"
                                    : (stabilizing ? "STABILIZING" : "WAIT");
    Serial.printf("balance_state=%s\n", balance_state);
    const char* timer_state = balance_prepare_active
                                  ? "PREPARE"
                                  : (stabilizing
                                         ? (counting ? "COUNTING" : "PAUSED")
                                         : (balance_timer_reset ? "RESET" : "WAIT"));
    Serial.printf("balance_timer_state=%s\n", timer_state);
    Serial.printf("candidate_angle=%.6f\n", balance_candidate_angle);
    Serial.printf("angle_delta=%.6f\n", angle_delta);
    Serial.printf("zero_sample_count=%lu\n",
                  static_cast<unsigned long>(balance_sample_count));
    Serial.printf("stable_timer_ms=%lu\n",
                  static_cast<unsigned long>(stable_timer_ms));
  }

  if (stable_timer_ms >= kBalanceStableTimeMs) {
    if (balance_sample_count == 0) {
      balance_stable_start_ms = 0;
      balance_stable_elapsed_ms = 0;
      balance_candidate_angle = 0.0f;
      balance_angle_sum = 0.0f;
      balance_distance_sum = 0.0f;
      balance_timer_reset = true;
      return;
    }
    angle_zeropoint = balance_angle_sum / balance_sample_count;
    distance_zeropoint = balance_distance_sum / balance_sample_count;
    lqr_zero_set = true;
    Serial.println("LQR zero points AUTO captured:");
    Serial.printf("angle=%.6f\n", angle_zeropoint);
    Serial.printf("distance=%.6f\n", distance_zeropoint);
    balance_wait_active = false;
    balance_prepare_active = false;
    balance_prepare_start_ms = 0;
    balance_stable_start_ms = 0;
    balance_stable_elapsed_ms = 0;
    balance_candidate_angle = 0.0f;
    balance_angle_sum = 0.0f;
    balance_distance_sum = 0.0f;
    balance_sample_count = 0;
    balance_brake_right = 0.0f;
    balance_brake_left = 0.0f;
    setBothTargetsToZero();
    updateLqrDiagnostic();

    // BALANCE WAIT may have moved the wheels since resetLqrTestState().
    // Use the actual position at AUTO START as the travel-limit origin.
    lqr_drive_origin_distance = LQR_distance;
    lqr_drive_start_distance = LQR_distance;
    lqr_auto_drive_active = false;

    lqr_direction_test_start_ms = millis();
    clearVelocityTrace();
    clearLoopTimingTrace();
    clearLqrControlTrace();
    lqr_direction_test_armed = true;
    lqr_startup_diagnostic_active = true;
    lqr_startup_diagnostic_start_ms = millis();
    lqr_startup_diagnostic_last_ms =
        lqr_startup_diagnostic_start_ms -
        kLqrStartupDiagnosticIntervalMs;
    setStatusLed(true);
    Serial.println("Balance brake released:");
    Serial.printf("right=%.2f V\n", balance_brake_right);
    Serial.printf("left=%.2f V\n", balance_brake_left);
    Serial.printf("LQR drive origin=%.6f\n",
                  lqr_drive_origin_distance);
    Serial.println("LQR Direction Test AUTO START.");
  }
}

bool motionCommandAllowed() {
  if (!test_system_ready) {
    Serial.println("IGNORED: both motors are not initialized.");
    return false;
  }
  if (emergency_stop_active) {
    Serial.println("IGNORED: drivers disabled; send 'u' to re-enable at 0 V.");
    return false;
  }
  if (polarity_test_active) {
    Serial.println("IGNORED: use '+' or '-' in LQR polarity test mode.");
    return false;
  }
  if (lqr_diagnostic_armed) {
    Serial.println("IGNORED: manual drive is disabled while LQR diagnostics are armed.");
    return false;
  }
  return true;
}

//==================================================
// TEST / MONITOR FUNCTIONS
//==================================================
// Normally there is no need to edit below this line.
void clearVelocityTrace() {
  velocity_trace_write_index = 0;
  velocity_trace_count = 0;
}

void printVelocityTrace() {
  if (velocity_trace_count == 0) {
    return;
  }

  Serial.println("10MS TRACE BEGIN");
  Serial.printf("count=%u\n",
                static_cast<unsigned int>(velocity_trace_count));

  Serial.println(
      "elapsed_ms,elapsed_us,"
      "current_right_angle,previous_right_angle,right_angle_delta,"
      "current_left_angle,previous_left_angle,left_angle_delta,"
      "diag_right_velocity_10ms,diag_left_velocity_10ms,"
      "diag_LQR_speed_10ms,"
      "simplefoc_right_velocity,simplefoc_left_velocity,"
      "LQR_speed,diag_speed_spike");

  const size_t start_index =
      velocity_trace_count < kVelocityTraceSize
          ? 0
          : velocity_trace_write_index;

  for (size_t i = 0; i < velocity_trace_count; ++i) {
    const size_t index =
        (start_index + i) % kVelocityTraceSize;

    const VelocityTraceSample& sample =
        velocity_trace[index];

    Serial.printf(
        "%lu,%lu,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%s\n",
        static_cast<unsigned long>(sample.elapsed_ms),
        static_cast<unsigned long>(sample.elapsed_us),
        sample.current_right_angle,
        sample.previous_right_angle,
        sample.right_angle_delta,
        sample.current_left_angle,
        sample.previous_left_angle,
        sample.left_angle_delta,
        sample.diag_right_velocity_10ms,
        sample.diag_left_velocity_10ms,
        sample.diag_LQR_speed_10ms,
        sample.simplefoc_right_velocity,
        sample.simplefoc_left_velocity,
        sample.LQR_speed,
        sample.diag_speed_spike ? "yes" : "no");
  }

  Serial.println("10MS TRACE END");
}
void clearLoopTimingTrace() {
  loop_timing_write_index = 0;
  loop_timing_count = 0;
}

void printLoopTimingTrace() {
  if (loop_timing_count == 0) {
    return;
  }

  Serial.println("LOOP TIMING TRACE BEGIN");
  Serial.printf("count=%u\n",
                static_cast<unsigned>(loop_timing_count));

  Serial.println(
      "elapsed_ms,whole_loop_us,"
      "motor1_loopfoc_us,motor2_loopfoc_us,"
      "mpu_update_us,motor_move_us");

  const size_t first_index =
      (loop_timing_write_index + kLoopTimingTraceSize -
       loop_timing_count) %
      kLoopTimingTraceSize;

  for (size_t i = 0; i < loop_timing_count; ++i) {
    const LoopTimingSample& sample =
        loop_timing_trace[
            (first_index + i) % kLoopTimingTraceSize];

    Serial.printf(
        "%lu,%lu,%lu,%lu,%lu,%lu\n",
        static_cast<unsigned long>(sample.elapsed_ms),
        static_cast<unsigned long>(sample.whole_loop_us),
        static_cast<unsigned long>(sample.motor1_loopfoc_us),
        static_cast<unsigned long>(sample.motor2_loopfoc_us),
        static_cast<unsigned long>(sample.mpu_update_us),
        static_cast<unsigned long>(sample.motor_move_us));
  }

  Serial.println("LOOP TIMING TRACE END");
}

bool cycleLqrTestMode() {
  if (lqr_direction_test_armed || balance_wait_active ||
      polarity_test_active) {
    Serial.println(
        "IGNORED: stop active tests before changing LQR test mode.");
    return false;
  }

  setBothTargetsToZero();
  switch (lqr_test_mode) {
    case LqrTestMode::Full:
      lqr_test_mode = LqrTestMode::AttitudeSpeed;
      break;
    case LqrTestMode::AttitudeSpeed:
      lqr_test_mode = LqrTestMode::AttitudeDistance;
      break;
    case LqrTestMode::AttitudeDistance:
      lqr_test_mode = LqrTestMode::AttitudeOnly;
      break;
    case LqrTestMode::AttitudeOnly:
      lqr_test_mode = LqrTestMode::Full;
      break;
  }

  updateLqrDiagnostic();
  Serial.printf("LQR TEST MODE: %s\n", lqrTestModeDisplayName());
  return true;
}

bool cycleLqrSpeedSource() {
  if (lqr_direction_test_armed || balance_wait_active ||
      polarity_test_active) {
    Serial.println(
        "IGNORED: stop active tests before changing LQR speed source.");
    return false;
  }

  setBothTargetsToZero();
  lqr_speed_source =
      lqr_speed_source == LqrSpeedSource::SimpleFoc
          ? LqrSpeedSource::Diagnostic10ms
          : LqrSpeedSource::SimpleFoc;
  updateLqrDiagnostic();
  Serial.printf("LQR SPEED SOURCE: %s\n", lqrSpeedSourceName());
  return true;
}

bool startBalanceTest() {
  if (!test_system_ready) {
    Serial.println("IGNORED: system initialization is not complete.");
    return false;
  }
  if (emergency_stop_active) {
    Serial.println("IGNORED: drivers are disabled; send 'u' first.");
    return false;
  }
  if (polarity_test_active) {
    Serial.println("IGNORED: exit LQR polarity test mode with 'k' first.");
    return false;
  }
  if (lqr_direction_test_armed || balance_wait_active) {
    Serial.println("IGNORED: LQR test is already active.");
    return false;
  }

  Serial.printf("LQR TEST MODE: %s\n", lqrTestModeDisplayName());
  Serial.printf("LQR SPEED SOURCE: %s\n", lqrSpeedSourceName());
  stopBothMotorsAtZero();
  resetLqrTestState();
  lqr_diagnostic_armed = true;
  printAndResetLqrRunTime();
  lqr_direction_test_armed = false;
  lqr_startup_diagnostic_active = false;
  lqr_zero_set = false;
  balance_wait_active = true;
  balance_prepare_active = true;
  balance_prepare_start_ms = millis();
  balance_stable_start_ms = 0;
  balance_stable_elapsed_ms = 0;
  balance_candidate_angle = 0.0f;
  balance_angle_sum = 0.0f;
  balance_distance_sum = 0.0f;
  balance_sample_count = 0;
  balance_stable_message_shown = false;
  balance_timer_reset = false;
  balance_brake_enabled = true;
  balance_brake_right = 0.0f;
  balance_brake_left = 0.0f;
  status_led_last_toggle_ms = millis();
  balance_wait_last_diagnostic_ms = millis();
  setStatusLed(false);
  Serial.println("BALANCE PREPARE:");
  Serial.println("Place both hands on the robot.");
  Serial.println("Stability measurement starts in 2 seconds.");
  return true;
}

bool confirmActualFall() {
  const uint32_t now_ms = millis();
  const float angle = LQR_angle;
  const float reference = angle_zeropoint;
  const float angle_error = angle - reference;

  // A non-finite IMU sample must never be promoted into an actuator shutdown.
  // Clear the candidate and wait for a coherent sample on the next loop.
  if (!isfinite(angle) || !isfinite(reference) || !isfinite(angle_error)) {
    Serial.printf(
        "FALL CHECK: decision=REJECT_INVALID time_ms=%lu angle=%+.6f "
        "reference=%+.6f angle_error=%+.6f threshold=%+.3f "
        "continuous_ms=0 samples=0\n",
        static_cast<unsigned long>(now_ms), angle, reference, angle_error,
        kActualFallAngleDeg);
    fall_exceeded_since_ms = 0;
    fall_exceeded_samples = 0;
    fall_previous_sample_valid = false;
    return false;
  }

  const bool angle_discontinuous = fall_previous_sample_valid &&
      abs(angle_error - fall_previous_angle_error) >=
          kFallAngleDiscontinuityDeg;
  const bool reference_discontinuous = fall_previous_sample_valid &&
      abs(reference - fall_previous_reference) >=
          kFallReferenceDiscontinuityDeg;
  fall_previous_angle_error = angle_error;
  fall_previous_reference = reference;
  fall_previous_sample_valid = true;

  if (angle_discontinuous || reference_discontinuous) {
    Serial.printf(
        "FALL CHECK: decision=REJECT_DISCONTINUITY time_ms=%lu angle=%+.6f "
        "reference=%+.6f angle_error=%+.6f threshold=%+.3f "
        "continuous_ms=0 samples=0 angle_jump=%s reference_jump=%s\n",
        static_cast<unsigned long>(now_ms), angle, reference, angle_error,
        kActualFallAngleDeg, angle_discontinuous ? "yes" : "no",
        reference_discontinuous ? "yes" : "no");
    fall_exceeded_since_ms = 0;
    fall_exceeded_samples = 0;
    return false;
  }

  const float absolute_error = abs(angle_error);
  if (absolute_error <= kActualFallAngleDeg) {
    if (fall_exceeded_since_ms != 0) {
      Serial.printf(
          "FALL CHECK: decision=CLEARED time_ms=%lu angle=%+.6f "
          "reference=%+.6f angle_error=%+.6f threshold=%+.3f "
          "continuous_ms=%lu samples=%lu\n",
          static_cast<unsigned long>(now_ms), angle, reference, angle_error,
          kActualFallAngleDeg,
          static_cast<unsigned long>(now_ms - fall_exceeded_since_ms),
          static_cast<unsigned long>(fall_exceeded_samples));
    }
    fall_exceeded_since_ms = 0;
    fall_exceeded_samples = 0;
    return false;
  }

  if (fall_exceeded_since_ms == 0) {
    fall_exceeded_since_ms = now_ms;
    fall_exceeded_samples = 1;
  } else {
    ++fall_exceeded_samples;
  }
  const uint32_t continuous_ms = now_ms - fall_exceeded_since_ms;
  const uint32_t required_ms = absolute_error >= kFallHardAngleDeg
      ? kFallHardConfirmTimeMs : kFallConfirmTimeMs;
  const bool confirmed = continuous_ms >= required_ms &&
      fall_exceeded_samples >= kFallMinimumSamples;

  if (confirmed || now_ms - fall_diagnostic_last_ms >=
                       kFallDiagnosticIntervalMs) {
    fall_diagnostic_last_ms = now_ms;
    const char* recovery_state =
        drive_recovery_state == DriveRecoveryState::AttitudeOnly
            ? "ATTITUDE_ONLY"
            : drive_recovery_state == DriveRecoveryState::PositionRamp
                  ? "POSITION_RAMP" : "INACTIVE";
    Serial.printf(
        "FALL CHECK: decision=%s time_ms=%lu angle=%+.6f reference=%+.6f "
        "angle_error=%+.6f threshold=%+.3f hard_threshold=%+.3f "
        "required_ms=%lu continuous_ms=%lu samples=%lu recovery_state=%s\n",
        confirmed ? "FULL_STOP" : "PENDING",
        static_cast<unsigned long>(now_ms), angle, reference, angle_error,
        kActualFallAngleDeg, kFallHardAngleDeg,
        static_cast<unsigned long>(required_ms),
        static_cast<unsigned long>(continuous_ms),
        static_cast<unsigned long>(fall_exceeded_samples), recovery_state);
  }
  return confirmed;
}

const char* detectDriveSafetyFault(float right_output, float left_output) {
  const float angle_delta = drive_safety_angle_initialized
      ? abs(LQR_angle - drive_safety_previous_angle)
      : 0.0f;
  drive_safety_previous_angle = LQR_angle;
  drive_safety_angle_initialized = true;

  if (!lqr_auto_drive_active &&
      drive_recovery_state == DriveRecoveryState::Inactive) {
    drive_safety_saturation_since_ms = 0;
    drive_safety_last_saturation_sign = 0;
    drive_safety_last_saturation_ms = 0;
    return nullptr;
  }

  const bool both_saturated =
      abs(right_output) >= kLqrTestVoltageLimit - 0.001f &&
      abs(left_output) >= kLqrTestVoltageLimit - 0.001f;
  const uint32_t now_ms = millis();
  if (!both_saturated) {
    drive_safety_saturation_since_ms = 0;
  } else if (drive_safety_saturation_since_ms == 0) {
    drive_safety_saturation_since_ms = now_ms;
  }

  if (abs(LQR_angle - angle_zeropoint) >= kDriveSafetyAngleErrorDeg) {
    return "ANGLE_ENVELOPE";
  }
  if (angle_delta >= kDriveSafetyAngleJumpDeg) {
    return "ANGLE_JUMP";
  }
  if (abs(LQR_gyro) >= kDriveSafetyGyroDegPerSec) {
    return "GYRO_RATE";
  }
  if (abs(motor1.shaft_velocity) >= kDriveSafetyWheelSpeed ||
      abs(motor2.shaft_velocity) >= kDriveSafetyWheelSpeed) {
    return "WHEEL_OVERSPEED";
  }

  if (!both_saturated) {
    return nullptr;
  }

  const int8_t saturation_sign =
      right_output + left_output >= 0.0f ? 1 : -1;
  if (drive_safety_last_saturation_sign != 0 &&
      saturation_sign != drive_safety_last_saturation_sign &&
      now_ms - drive_safety_last_saturation_ms <=
          kDriveSafetySaturationReversalWindowMs) {
    return "SATURATION_REVERSAL";
  }
  drive_safety_last_saturation_sign = saturation_sign;
  drive_safety_last_saturation_ms = now_ms;
  if (now_ms - drive_safety_saturation_since_ms >=
      kDriveSafetySaturationPersistMs) {
    return "OUTPUT_SATURATION";
  }
  return nullptr;
}

void stopLqrDriveOnly(const char* reason, bool require_recenter) {
  lqr_target_speed = 0.0f;
  lqr_drive_angle_bias = 0.0f;
  lqr_ps4_target_tilt_limit_deg =
      kPs4DriveTargetTiltInitialLimitDeg;
  lqr_ps4_target_tilt_integral_deg = 0.0f;
  lqr_ps4_target_tilt_deg = 0.0f;
  lqr_drive_start_pending = false;
  lqr_drive_pending_command = 0;
  lqr_drive_pending_speed = 0.0f;
  lqr_drive_start_request_ms = 0;
  lqr_drive_stable_start_ms = 0;
  lqr_soft_stop_active = false;
  lqr_soft_stop_stable_start_ms = 0;
  lqr_auto_drive_active = false;
  lqr_drive_wrong_way_normal_since_ms = 0;
  lqr_drive_wrong_way_hard_since_ms = 0;
  lqr_drive_start_distance = 0.0f;
  lqr_drive_origin_distance = 0.0f;

  drive_recovery_state = DriveRecoveryState::AttitudeOnly;
  drive_recovery_start_ms = millis();
  drive_recovery_stable_since_ms = 0;
  drive_recovery_position_ramp_start_ms = 0;
  drive_recovery_diagnostic_last_ms = 0;
  drive_safety_saturation_since_ms = 0;
  drive_safety_last_saturation_sign = 0;
  drive_safety_last_saturation_ms = 0;

  // Forget every drive-era feedback/controller state. The next sample seeds
  // the normal filter; the PS4 window cannot reuse pre-stop wheel samples.
  lqr_control_speed_filter_initialized = false;
  lqr_ps4_speed_for_control = 0.0f;
  lqr_ps4_window_speed = 0.0f;
  lqr_ps4_window_elapsed_ms = 0;
  lqr_ps4_window_angle_delta_right = 0.0f;
  lqr_ps4_window_angle_delta_left = 0.0f;
  lqr_ps4_speed_history_next = 0;
  lqr_ps4_speed_history_count = 0;

  // Capture the current position for the existing balance controller.  A
  // drive-only fault must not disable either wheel driver or disarm LQR: doing
  // so removes the torque that is keeping an otherwise upright robot balanced.
  distance_zeropoint = LQR_distance;
  pid_angle.reset();
  pid_gyro.reset();
  pid_distance.reset();
  pid_speed.reset();

  if (require_recenter) {
    setPs4DriveState(Ps4DriveState::OutsideDeadzone);
  }

  if (reason != nullptr) {
    Serial.println("SAFETY STOP:");
    Serial.println("mode=DRIVE_ONLY");
    Serial.printf("reason=%s\n", reason);
    Serial.println("target_speed=0.000");
    Serial.println("wheel_drivers=ENABLED");
    Serial.println("leg_servo_torque=UNCHANGED");
    Serial.println("balance_control=CONTINUING");
    Serial.println("recovery_state=ATTITUDE_ONLY");
    Serial.println("position_hold=DISABLED");
    Serial.printf("recenter_required=%s\n",
                  require_recenter ? "yes" : "no");
    Serial.printf("ps4_drive_state=%s\n",
                  ps4DriveStateName(ps4_drive_state));
    Serial.printf("lqr_auto_drive_active=%s\n",
                  lqr_auto_drive_active ? "yes" : "no");
    Serial.printf("lqr_drive_start_pending=%s\n",
                  lqr_drive_start_pending ? "yes" : "no");
    Serial.printf("lqr_soft_stop_active=%s\n",
                  lqr_soft_stop_active ? "yes" : "no");
  }
}

void terminateLqrDriveSession(const char* reason, bool require_recenter) {
  stopLqrDriveOnly(nullptr, require_recenter);

  drive_recovery_state = DriveRecoveryState::Inactive;

  lqr_direction_test_armed = false;
  lqr_diagnostic_armed = false;
  lqr_startup_diagnostic_active = false;

  if (reason != nullptr) {
    Serial.println("LQR TEST TERMINATED:");
    Serial.println("mode=FULL_STOP");
    Serial.printf("reason=%s\n", reason);
    Serial.printf("ps4_drive_state=%s\n",
                  ps4DriveStateName(ps4_drive_state));
  }
}

bool stopBalanceTest() {
  lqr_stop_trace_active = false;

  polarity_test_active = false;
  polarity_pulse_active = false;
  polarity_pulse_voltage = 0.0f;
  terminateLqrDriveSession(nullptr, false);
  lqr_drive_start_request_ms = 0;
  lqr_drive_stable_start_ms = 0;
  lqr_drive_max_stable_ms = 0;
  lqr_drive_angle_fail_count = 0;
  lqr_drive_gyro_fail_count = 0;
  lqr_drive_speed_fail_count = 0;
  lqr_drive_start_angle_min = 0.0f;
  lqr_drive_start_angle_max = 0.0f;
  lqr_drive_start_angle_sum = 0.0f;
  lqr_drive_start_angle_sample_count = 0;
  lqr_drive_start_angle_reference_sum = 0.0f;
  lqr_drive_start_angle_reference_count = 0;
  lqr_drive_start_angle_reference = 0.0f;
  lqr_drive_start_angle_reference_ready = false;
  for (size_t i = 0;
       i < kLqrDriveStartSpeedAverageSamples;
       ++i) {
    lqr_drive_start_speed_samples[i] = 0.0f;
  }
  lqr_drive_start_speed_sample_index = 0;
  lqr_drive_start_speed_sample_count = 0;
  lqr_drive_start_speed_sum = 0.0f;
  lqr_drive_start_speed_average = 0.0f;
  lqr_drive_start_speed_abs_sum = 0.0f;
  lqr_drive_start_speed_abs_average = 0.0f;
  lqr_drive_start_speed_abs_max = 0.0f;
  lqr_drive_start_speed_diag_count = 0;
  lqr_drive_speed_source_diag_last_ms = 0;
  lqr_drive_speed_source_diag_count = 0;
  lqr_drive_sf_m1_abs_sum = 0.0f;
  lqr_drive_sf_m1_abs_max = 0.0f;
  lqr_drive_sf_m2_abs_sum = 0.0f;
  lqr_drive_sf_m2_abs_max = 0.0f;
  lqr_drive_diag_right_abs_sum = 0.0f;
  lqr_drive_diag_right_abs_max = 0.0f;
  lqr_drive_diag_left_abs_sum = 0.0f;
  lqr_drive_diag_left_abs_max = 0.0f;
  lqr_drive_control_speed_abs_sum = 0.0f;
  lqr_drive_control_speed_abs_max = 0.0f;
  lqr_drive_gate_speed_abs_sum = 0.0f;
  lqr_drive_gate_speed_abs_max = 0.0f;
  lqr_drive_speed_source_diff_abs_sum = 0.0f;
  lqr_drive_speed_source_diff_abs_max = 0.0f;
  cancelBalanceWait();
  stopBothMotorsAtZero();
  updateLqrDiagnostic();

  if (lqr_drive_sign_trace_count > 0) {
    printLqrDriveSignTrace("BALANCE_STOP");
    resetLqrDriveSignTrace();
  }

  if (lqr_stop_trace_count > 0) {
    printLqrStopTrace("BALANCE_STOP");
    resetLqrStopTrace();
  }

  printAndResetLqrRunTime();
  Serial.println("LQR diagnostics DISARMED; both motor targets are 0 V.");
  return true;
}

//==================================================
// Wi-Fi Trace Logger
//==================================================
bool traceDownloadBlocked() {
  return lqr_direction_test_armed || balance_wait_active;
}

void sendTraceBusyResponse() {
  wifi_log_server.send(
      409, "text/plain",
      "TRACE NOT AVAILABLE WHILE LQR TEST IS RUNNING\n");
}

void beginCsvResponse(const char* filename) {
  wifi_log_server.sendHeader(
      "Content-Disposition",
      String("attachment; filename=\"") + filename + "\"");
  wifi_log_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  wifi_log_server.send(200, "text/csv", "");
}

void handleVelocityTraceDownload() {
  if (traceDownloadBlocked()) {
    sendTraceBusyResponse();
    return;
  }

  beginCsvResponse("velocity-trace.csv");
  wifi_log_server.sendContent(
      "elapsed_ms,elapsed_us,"
      "current_right_angle,previous_right_angle,right_angle_delta,"
      "current_left_angle,previous_left_angle,left_angle_delta,"
      "diag_right_velocity_10ms,diag_left_velocity_10ms,"
      "diag_LQR_speed_10ms,"
      "simplefoc_right_velocity,simplefoc_left_velocity,"
      "LQR_speed,diag_speed_spike\n");

  const size_t first_index =
      (velocity_trace_write_index + kVelocityTraceSize -
       velocity_trace_count) %
      kVelocityTraceSize;
  char line[320];
  for (size_t i = 0; i < velocity_trace_count; ++i) {
    if (!wifi_log_server.client().connected()) {
      break;
    }
    const VelocityTraceSample& sample =
        velocity_trace[(first_index + i) % kVelocityTraceSize];
    snprintf(
        line, sizeof(line),
        "%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n",
        static_cast<unsigned long>(sample.elapsed_ms),
        static_cast<unsigned long>(sample.elapsed_us),
        sample.current_right_angle, sample.previous_right_angle,
        sample.right_angle_delta, sample.current_left_angle,
        sample.previous_left_angle, sample.left_angle_delta,
        sample.diag_right_velocity_10ms,
        sample.diag_left_velocity_10ms, sample.diag_LQR_speed_10ms,
        sample.simplefoc_right_velocity,
        sample.simplefoc_left_velocity, sample.LQR_speed,
        sample.diag_speed_spike ? "yes" : "no");
    wifi_log_server.sendContent(line);
  }
  wifi_log_server.sendContent("");
}

void handleLoopTimingTraceDownload() {
  if (traceDownloadBlocked()) {
    sendTraceBusyResponse();
    return;
  }

  beginCsvResponse("loop-timing-trace.csv");
  wifi_log_server.sendContent(
      "elapsed_ms,whole_loop_us,motor1_loopfoc_us,motor2_loopfoc_us,"
      "mpu_update_us,motor_move_us\n");

  const size_t first_index =
      (loop_timing_write_index + kLoopTimingTraceSize -
       loop_timing_count) %
      kLoopTimingTraceSize;
  char line[128];
  for (size_t i = 0; i < loop_timing_count; ++i) {
    if (!wifi_log_server.client().connected()) {
      break;
    }
    const LoopTimingSample& sample =
        loop_timing_trace[(first_index + i) % kLoopTimingTraceSize];
    snprintf(line, sizeof(line), "%lu,%lu,%lu,%lu,%lu,%lu\n",
             static_cast<unsigned long>(sample.elapsed_ms),
             static_cast<unsigned long>(sample.whole_loop_us),
             static_cast<unsigned long>(sample.motor1_loopfoc_us),
             static_cast<unsigned long>(sample.motor2_loopfoc_us),
             static_cast<unsigned long>(sample.mpu_update_us),
             static_cast<unsigned long>(sample.motor_move_us));
    wifi_log_server.sendContent(line);
  }
  wifi_log_server.sendContent("");
}

void handleLqrControlTraceDownload() {
  if (traceDownloadBlocked()) {
    sendTraceBusyResponse();
    return;
  }

  beginCsvResponse("lqr-control-trace.csv");
  wifi_log_server.sendContent(
      "elapsed_ms,angle,angle_zeropoint,angle_error,gyro,"
      "lqr_speed,speed_for_control,"
      "angle_control,gyro_control,distance_control,speed_control,"
      "lqr_u,right_output,left_output\n");

  const size_t first_index =
      (lqr_control_trace_write_index + kLqrControlTraceSize -
       lqr_control_trace_count) %
      kLqrControlTraceSize;
  char line[256];
  for (size_t i = 0; i < lqr_control_trace_count; ++i) {
    if (!wifi_log_server.client().connected()) {
      break;
    }
    const LqrControlTraceSample& sample =
        lqr_control_trace[(first_index + i) % kLqrControlTraceSize];
    snprintf(
        line, sizeof(line),
        "%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        static_cast<unsigned long>(sample.elapsed_ms), sample.angle,
        sample.angle_zeropoint, sample.angle_error, sample.gyro,
        sample.lqr_speed, sample.speed_for_control,
        sample.angle_control, sample.gyro_control,
        sample.distance_control, sample.speed_control, sample.lqr_u,
        sample.right_output, sample.left_output);
    wifi_log_server.sendContent(line);
  }
  wifi_log_server.sendContent("");
}

void sendControlResult(bool succeeded, const char* conflict_message) {
  if (!succeeded) {
    wifi_log_server.send(409, "text/plain", conflict_message);
    return;
  }
  wifi_log_server.sendHeader("Location", "/");
  wifi_log_server.send(303, "text/plain", "See Other\n");
}

void handleCycleLqrTestMode() {
  sendControlResult(
      cycleLqrTestMode(),
      "MODE CHANGE NOT ALLOWED WHILE TEST IS ACTIVE\n");
}

void handleCycleLqrSpeedSource() {
  sendControlResult(
      cycleLqrSpeedSource(),
      "SPEED SOURCE CHANGE NOT ALLOWED WHILE TEST IS ACTIVE\n");
}

void handleStartBalanceTest() {
  sendControlResult(startBalanceTest(), "START NOT ALLOWED\n");
}

void handleStopBalanceTest() {
  sendControlResult(stopBalanceTest(), "STOP NOT ALLOWED\n");
}

void handleWifiStatusPage() {
  char page[2600];
  const String ip_address = WiFi.localIP().toString();
  const bool test_active =
      lqr_direction_test_armed || balance_wait_active || polarity_test_active;
  const bool start_allowed =
      test_system_ready && !emergency_stop_active && !test_active;
  const char* state =
      polarity_test_active
          ? "POLARITY TEST"
          : (balance_wait_active
                 ? "BALANCE WAIT"
                 : (lqr_direction_test_armed ? "RUNNING" : "STOPPED"));
  snprintf(
      page, sizeof(page),
      "<!doctype html><html><head><meta charset=\"utf-8\">"
      "<title>ESP32 LQR Logger</title></head><body>"
      "<h1>ESP32 LQR Logger</h1>"
      "<p>WiFi: %s<br>IP: %s</p>"
      "<p>State: <strong>%s</strong></p>"
      "<p>LQR Test Mode: <strong>%s</strong></p>"
      "<form method=\"POST\" action=\"/control/mode\">"
      "<button type=\"submit\"%s>Change Mode</button></form>"
      "<p>LQR Speed Source: <strong>%s</strong></p>"
      "<form method=\"POST\" action=\"/control/speed-source\">"
      "<button type=\"submit\"%s>Change Speed Source</button></form>"
      "<h2>Control</h2>"
      "<form method=\"POST\" action=\"/control/start\">"
      "<button type=\"submit\"%s>START BALANCE TEST</button></form>"
      "<form method=\"POST\" action=\"/control/stop\">"
      "<button type=\"submit\">STOP TEST</button></form>"
      "<p>Trace samples:<br>Velocity: %u<br>Loop Timing: %u<br>"
      "LQR Control: %u</p>"
      "<p>Trace:<br>"
      "<a href=\"/trace/velocity.csv\">Velocity Trace</a><br>"
      "<a href=\"/trace/loop.csv\">Loop Timing Trace</a><br>"
      "<a href=\"/trace/lqr.csv\">LQR Control Trace</a></p>"
      "</body></html>",
      WiFi.status() == WL_CONNECTED ? "connected" : "disconnected",
      ip_address.c_str(), state, lqrTestModeDisplayName(),
      test_active ? " disabled" : "", lqrSpeedSourceName(),
      test_active ? " disabled" : "", start_allowed ? "" : " disabled",
      static_cast<unsigned>(velocity_trace_count),
      static_cast<unsigned>(loop_timing_count),
      static_cast<unsigned>(lqr_control_trace_count));
  wifi_log_server.send(200, "text/html; charset=utf-8", page);
}

void startWifiHttpServer() {
  if (wifi_log_server_started) {
    return;
  }

  wifi_log_server.on("/", HTTP_GET, handleWifiStatusPage);
  wifi_log_server.on("/trace/velocity.csv", HTTP_GET,
                     handleVelocityTraceDownload);
  wifi_log_server.on("/trace/loop.csv", HTTP_GET,
                     handleLoopTimingTraceDownload);
  wifi_log_server.on("/trace/lqr.csv", HTTP_GET,
                     handleLqrControlTraceDownload);
  wifi_log_server.on("/control/mode", HTTP_POST,
                     handleCycleLqrTestMode);
  wifi_log_server.on("/control/speed-source", HTTP_POST,
                     handleCycleLqrSpeedSource);
  wifi_log_server.on("/control/start", HTTP_POST,
                     handleStartBalanceTest);
  wifi_log_server.on("/control/stop", HTTP_POST,
                     handleStopBalanceTest);
  wifi_log_server.onNotFound([]() {
    wifi_log_server.send(404, "text/plain", "NOT FOUND\n");
  });
  wifi_log_server.begin();
  wifi_log_server_started = true;
  Serial.println("WiFi trace HTTP server started.");
}

void startMdns() {
  if (mdns_attempted) {
    return;
  }
  mdns_attempted = true;
  if (MDNS.begin("legged-balance")) {
    MDNS.addService("http", "tcp", 80);
    mdns_started = true;
    Serial.println("mDNS: http://legged-balance.local/");
  } else {
    Serial.println("mDNS unavailable; use the IP address URL.");
  }
}

void setupWifiLogServer() {
  if (!kEnableWifiLogServer) {
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(SSID_NAME, SSID_KEY);
  Serial.printf("WiFi: connecting to SSID %s", SSID_NAME);
  const uint32_t connect_start_ms = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - connect_start_ms < kWifiConnectTimeoutMs) {
    delay(100);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection timed out; robot control will continue.");
    return;
  }

  const String ip_address = WiFi.localIP().toString();
  Serial.printf("WiFi connected: SSID %s\n", SSID_NAME);
  Serial.printf("WiFi DHCP IP: %s\n", ip_address.c_str());
  Serial.printf("HTTP access URL: http://%s/\n", ip_address.c_str());
  startWifiHttpServer();
  startMdns();
}

void updateWifiLogServer() {
  if (!kEnableWifiLogServer) {
    return;
  }

  const uint32_t now_ms = millis();
  if (now_ms - wifi_status_last_check_ms >=
      kWifiStatusCheckIntervalMs) {
    wifi_status_last_check_ms = now_ms;
    if (WiFi.status() == WL_CONNECTED) {
      startWifiHttpServer();
      startMdns();
    }
  }

  if (wifi_log_server_started && WiFi.status() == WL_CONNECTED) {
    wifi_log_server.handleClient();
  }
}

void updateLqrRunDiagnostic(float right_output, float left_output) {
  if (!lqr_direction_test_armed) {
    lqr_run_diagnostic_last_ms = 0;
    return;
  }
  if (!kEnableLqrRunDiagnostic) {
    return;
  }
  if (lqr_speed_source != LqrSpeedSource::SimpleFoc) {
    return;
  }
  const uint32_t now_ms = millis();

  if (now_ms - lqr_run_diagnostic_last_ms <
      kLqrRunDiagnosticIntervalMs) {
    return;
  }

  lqr_run_diagnostic_last_ms = now_ms;

  const bool output_saturated =
      abs(right_output) >= kLqrTestVoltageLimit - 0.001f ||
      abs(left_output) >= kLqrTestVoltageLimit - 0.001f;

  Serial.println("LQR RUN:");
  Serial.printf("elapsed_ms=%lu\n",
                static_cast<unsigned long>(
                    now_ms - lqr_direction_test_start_ms));
  Serial.printf("angle_error=%.6f\n",
                LQR_angle - angle_zeropoint);
  Serial.printf("gyro=%.6f\n", LQR_gyro);
  Serial.printf("distance_error=%.6f\n",
                distance_zeropoint - LQR_distance);
  Serial.printf("LQR_distance=%.6f\n", LQR_distance);
  Serial.printf("LQR_speed=%.6f\n", LQR_speed);
  Serial.printf("lqr_speed_for_control=%.6f\n",
                lqr_speed_for_control);
  Serial.printf("angle_control=%.6f\n", angle_control);
  Serial.printf("gyro_control=%.6f\n", gyro_control);
  Serial.printf("distance_control=%.6f\n", distance_control);
  Serial.printf("speed_control=%.6f\n", speed_control);
  Serial.printf("lqr_test_mode=%s\n", lqrTestModeLogName());
  Serial.printf("lqr_speed_source=%s\n", lqrSpeedSourceName());
  Serial.printf("test_LQR_u=%.6f\n", test_LQR_u);
  Serial.printf("right_output=%.6f\n", right_output);
  Serial.printf("left_output=%.6f\n", left_output);
  Serial.printf("output_saturated=%s\n",
                output_saturated ? "yes" : "no");
  Serial.printf("right_shaft_velocity=%.6f\n",
                motor1.shaft_velocity);
  Serial.printf("left_shaft_velocity=%.6f\n",
                motor2.shaft_velocity);
}

void printLqrStartupDiagnostic(uint32_t elapsed_ms, float right_output,
                               float left_output) {
  const bool output_saturated =
      abs(right_output) >= kLqrTestVoltageLimit - 0.001f ||
      abs(left_output) >= kLqrTestVoltageLimit - 0.001f;
  Serial.println("LQR STARTUP:");
  Serial.printf("elapsed_ms=%lu\n", static_cast<unsigned long>(elapsed_ms));
  Serial.printf("angle_error=%.6f\n", LQR_angle - angle_zeropoint);
  Serial.printf("gyro=%.6f\n", LQR_gyro);
  Serial.printf("distance_error=%.6f\n",
                distance_zeropoint - LQR_distance);
  Serial.printf("LQR_distance=%.6f\n", LQR_distance);
  Serial.printf("LQR_speed=%.6f\n", LQR_speed);
  Serial.printf("lqr_speed_for_control=%.6f\n", lqr_speed_for_control);
  Serial.printf("lqr_speed_source=%s\n", lqrSpeedSourceName());
  Serial.printf("angle_control=%.6f\n", angle_control);
  Serial.printf("gyro_control=%.6f\n", gyro_control);
  Serial.printf("distance_control=%.6f\n", distance_control);
  Serial.printf("speed_control=%.6f\n", speed_control);
  Serial.printf("lqr_test_mode=%s\n", lqrTestModeLogName());
  Serial.printf("test_LQR_u=%.6f\n", test_LQR_u);
  Serial.printf("right_output=%.6f\n", right_output);
  Serial.printf("left_output=%.6f\n", left_output);
  Serial.printf("output_saturated=%s\n", output_saturated ? "yes" : "no");
  Serial.printf("right_shaft_velocity=%.6f\n", motor1.shaft_velocity);
  Serial.printf("left_shaft_velocity=%.6f\n", motor2.shaft_velocity);
}

void printHelp() {
  Serial.println();
  Serial.println("SimpleFOC two-wheel + MPU6050 integration test");
  Serial.println("  1 : right +0.50 V, left 0 V");
  Serial.println("  2 : right -0.50 V, left 0 V");
  Serial.println("  3 : right 0 V, left +0.50 V");
  Serial.println("  4 : right 0 V, left -0.50 V");
  Serial.println("  f : both +0.50 V");
  Serial.println("  r : both -0.50 V");
  Serial.println("  x : right +0.50 V, left -0.50 V");
  Serial.println("  s or 0 : both 0 V");
  Serial.println("  i : toggle LQR speed source SIMPLEFOC / 10MS");
  Serial.println("  e : stop drive only; keep balance and drivers enabled");
  Serial.println("  u : re-enable both drivers at 0 V");
  Serial.println("  p : print both motor sensors and MPU6050 values once");
  Serial.println("  q : print distance-control diagnostics without motor output");
  Serial.println("  v : print speed-control diagnostics without motor output");
  Serial.println("  l : ping right (ID 1), then left (ID 2) leg servo");
  Serial.println("  m : move right leg (ID 1) to 2044");
  Serial.println("  n : return right leg (ID 1) to HOME 2061");
  Serial.println("  o : move left leg (ID 2) to 2044 (10% extend)");
  Serial.println("  y : return left leg (ID 2) to HOME 2026");
  Serial.println("  w : move both legs to 10% extension");
  Serial.println("  ! : move both legs to 20% extension");
  Serial.println("  @ : move both legs to 30% extension");
  Serial.println("  9 : return both legs to 0% HOME");
  Serial.println("  c : recalibrate MPU6050 gyro offsets with both motors at 0 V");
  Serial.println("  z : capture current AngleY and wheel distance zero points");
  Serial.println("  a : arm LQR calculation diagnostics (no motor output)");
  Serial.println("  t : enter Balance Wait for the limited LQR direction test");
  Serial.println("  > : short LQR drive test in + direction");
  Serial.println("  . : LQR drive test ramp stop, then hold position");
  Serial.println("  < : short LQR drive test in - direction");
  Serial.println(
                 "  g : cycle LQR test mode FULL / ATTITUDE + SPEED / "
                 "ATTITUDE + DISTANCE / ATTITUDE ONLY");
  Serial.println("  b : toggle the speed brake during Balance Wait");
  Serial.println("  j : enter LQR polarity test mode");
  Serial.println("  + or - : both wheels, 500 ms polarity pulse");
  Serial.println("  5 : right only positive polarity voltage for 500 ms");
  Serial.println("  6 : right only negative polarity voltage for 500 ms");
  Serial.println("  7 : left only positive polarity voltage for 500 ms");
  Serial.println("  8 : left only negative polarity voltage for 500 ms");
  Serial.println("  [ : decrease polarity test voltage by 0.01 V");
  Serial.println("  ] : increase polarity test voltage by 0.01 V");
  Serial.println("  k : exit LQR polarity test mode");
  Serial.println("  d : disarm LQR calculation diagnostics");
  Serial.println("  h or ? : show this help");
  Serial.println();
}

void printLqrStatus() {
  const float snapshot_angle_y = mpu6050.getAngleY();
  const float snapshot_gyro_y = mpu6050.getGyroY();
  const float snapshot_right_angle = motor1.shaft_angle;
  const float snapshot_right_velocity = motor1.shaft_velocity;
  const float snapshot_left_angle = motor2.shaft_angle;
  const float snapshot_left_velocity = motor2.shaft_velocity;
  const float snapshot_lqr_distance =
      snapshot_right_angle * m1_direction + snapshot_left_angle * m2_direction;
  const float snapshot_lqr_speed = snapshot_right_velocity * m1_direction +
                                   snapshot_left_velocity * m2_direction;
  const float snapshot_lqr_speed_for_control =
      lqr_speed_source == LqrSpeedSource::SimpleFoc
          ? snapshot_lqr_speed
          : diag_LQR_speed_10ms;
  float snapshot_angle_control = 0.0f;
  float snapshot_gyro_control = 0.0f;
  float snapshot_distance_control = 0.0f;
  float snapshot_speed_control = 0.0f;
  float snapshot_lqr_u = 0.0f;
  if (lqr_diagnostic_armed) {
    snapshot_angle_control = pid_angle(snapshot_angle_y - angle_zeropoint);
    snapshot_gyro_control = pid_gyro(snapshot_gyro_y);
    snapshot_distance_control =
        pid_distance(distance_zeropoint - snapshot_lqr_distance);
    snapshot_speed_control =
        pid_speed(0.0f - snapshot_lqr_speed_for_control);
    snapshot_lqr_u = snapshot_angle_control + snapshot_gyro_control +
                     snapshot_distance_control + snapshot_speed_control;
  }
  updateTestLqrOutput(snapshot_angle_control, snapshot_gyro_control,
                      snapshot_distance_control, snapshot_speed_control);
  const float snapshot_motor1_output_sign =
    m1_direction < 0.0f ? -1.0f : 1.0f;
  const float snapshot_motor2_output_sign =
    m2_direction < 0.0f ? -1.0f : 1.0f;

  const float snapshot_right_test_output =
    constrain(-test_LQR_u * snapshot_motor1_output_sign,
              -kLqrTestVoltageLimit,
              kLqrTestVoltageLimit);
  const float snapshot_left_test_output =
    constrain(-test_LQR_u * snapshot_motor2_output_sign,
              -kLqrTestVoltageLimit,
              kLqrTestVoltageLimit);

  Serial.printf("AngleY=%.6f GyroY=%.6f\n", snapshot_angle_y,
                snapshot_gyro_y);
  Serial.printf("Right shaft_angle=%.6f shaft_velocity=%.6f\n",
                snapshot_right_angle, snapshot_right_velocity);
  Serial.printf("Left shaft_angle=%.6f shaft_velocity=%.6f\n",
                snapshot_left_angle, snapshot_left_velocity);
  Serial.printf("Right velocity_10ms=%.6f\n",
                diag_right_velocity_10ms);
  Serial.printf("Left velocity_10ms=%.6f\n",
                diag_left_velocity_10ms);
  Serial.printf("LQR_speed_10ms=%.6f\n",
                diag_LQR_speed_10ms);
  Serial.printf("m1_direction=%.2f m2_direction=%.2f\n", m1_direction,
                m2_direction);
  Serial.printf("angle_zeropoint=%.6f distance_zeropoint=%.6f zero_set=%s\n",
                angle_zeropoint, distance_zeropoint,
                lqr_zero_set ? "yes" : "no");
  Serial.printf("LQR_angle=%.6f LQR_gyro=%.6f\n", snapshot_angle_y,
                snapshot_gyro_y);
  Serial.printf("LQR_distance=%.6f LQR_speed=%.6f\n",
                snapshot_lqr_distance, snapshot_lqr_speed);
  Serial.printf("angle_control=%.6f gyro_control=%.6f\n",
                snapshot_angle_control, snapshot_gyro_control);
  Serial.printf("lqr_speed_for_control=%.6f\n",
                snapshot_lqr_speed_for_control);
  Serial.printf("lqr_speed_source=%s\n", lqrSpeedSourceName());
  Serial.printf("distance_control=%.6f speed_control=%.6f\n",
                snapshot_distance_control, snapshot_speed_control);
  Serial.printf("lqr_test_mode=%s\n", lqrTestModeLogName());
  Serial.printf("test_LQR_u=%.6f\n", test_LQR_u);
  Serial.printf("LQR_u=%.6f diagnostic=%s direction_test=%s dt_ms=%.3f\n",
                snapshot_lqr_u,
                lqr_diagnostic_armed ? "ARMED" : "DISARMED",
                lqr_direction_test_armed ? "ARMED" : "OFF", control_dt_ms);
  Serial.printf("right_output[V]=%.6f left_output[V]=%.6f\n",
                snapshot_right_test_output, snapshot_left_test_output);
}

void printDistanceDiagnostic() {
  const float snapshot_right_angle = motor1.shaft_angle;
  const float snapshot_left_angle = motor2.shaft_angle;
  const float snapshot_lqr_distance =
      snapshot_right_angle * m1_direction + snapshot_left_angle * m2_direction;
  const float snapshot_distance_error =
      distance_zeropoint - snapshot_lqr_distance;
  const float snapshot_distance_control =
      pid_distance(snapshot_distance_error);
  const float virtual_right = snapshot_distance_control * m1_direction;
  const float virtual_left = snapshot_distance_control * m2_direction;

  Serial.printf("Right shaft_angle=%.6f\n", snapshot_right_angle);
  Serial.printf("Left shaft_angle=%.6f\n", snapshot_left_angle);
  Serial.printf("m1_direction=%.2f m2_direction=%.2f\n", m1_direction,
                m2_direction);
  Serial.printf("distance_zeropoint=%.6f\n", distance_zeropoint);
  Serial.printf("LQR_distance=%.6f\n", snapshot_lqr_distance);
  Serial.printf("distance_error=%.6f\n", snapshot_distance_error);
  Serial.printf("distance_control=%.6f\n", snapshot_distance_control);
  Serial.printf("virtual_right=%.6f virtual_left=%.6f\n", virtual_right,
                virtual_left);
}

void printSpeedDiagnostic() {
  const float snapshot_right_velocity = motor1.shaft_velocity;
  const float snapshot_left_velocity = motor2.shaft_velocity;
  const float snapshot_lqr_speed = snapshot_right_velocity * m1_direction +
                                   snapshot_left_velocity * m2_direction;
  const float snapshot_speed_control = pid_speed(0.0f - snapshot_lqr_speed);
  const float virtual_speed_right = snapshot_speed_control * m1_direction;
  const float virtual_speed_left = snapshot_speed_control * m2_direction;

  Serial.printf("Right shaft_velocity=%.6f\n", snapshot_right_velocity);
  Serial.printf("Left shaft_velocity=%.6f\n", snapshot_left_velocity);
  Serial.printf("m1_direction=%.2f m2_direction=%.2f\n", m1_direction,
                m2_direction);
  Serial.printf("LQR_speed=%.6f\n", snapshot_lqr_speed);
  Serial.printf("speed_control=%.6f\n", snapshot_speed_control);
  Serial.printf("virtual_speed_right=%.6f virtual_speed_left=%.6f\n",
                virtual_speed_right, virtual_speed_left);
}

void setupPs4Controller() {
//   PS4.begin(kPs4HostMac);
//   Serial.printf("PS4 CONTROLLER: initialized (host MAC %s); waiting without blocking.\n",
//                 kPs4HostMac);
  PS4.begin();
  Serial.printf("PS4 CONTROLLER: initialized .\n");
}


bool ps4InputStatesEqual(const Ps4InputState& lhs,
                         const Ps4InputState& rhs) {
  return lhs.left_stick_x == rhs.left_stick_x &&
         lhs.left_stick_y == rhs.left_stick_y &&
         lhs.right_stick_x == rhs.right_stick_x &&
         lhs.right_stick_y == rhs.right_stick_y && lhs.l2 == rhs.l2 &&
         lhs.r2 == rhs.r2 && lhs.cross == rhs.cross &&
         lhs.circle == rhs.circle && lhs.square == rhs.square &&
         lhs.triangle == rhs.triangle && lhs.dpad_up == rhs.dpad_up &&
         lhs.dpad_down == rhs.dpad_down &&
         lhs.dpad_left == rhs.dpad_left &&
         lhs.dpad_right == rhs.dpad_right && lhs.l1 == rhs.l1 &&
         lhs.r1 == rhs.r1 && lhs.options == rhs.options &&
         lhs.share == rhs.share && lhs.ps_button == rhs.ps_button;
}

Ps4InputState readPs4Input() {
  return {
      PS4.LStickX(), PS4.LStickY(), PS4.RStickX(), PS4.RStickY(),
      PS4.L2Value(), PS4.R2Value(), PS4.Cross(), PS4.Circle(),
      PS4.Square(), PS4.Triangle(), PS4.Up(), PS4.Down(), PS4.Left(),
      PS4.Right(), PS4.L1(), PS4.R1(), PS4.Options(), PS4.Share(),
      PS4.PSButton(),
  };
}

float ps4DriveSpeedRequestFromLeftStickY(int stick_y) {
  const int limited_stick_y = constrain(stick_y, -127, 127);
  if (abs(limited_stick_y) <= kPs4DriveStickDeadzone) {
    return 0.0f;
  }

  // The PS4 controller reports a negative LY when the stick is pushed forward;
  // keep the diagnostic convention explicit: positive speed = forward,
  // negative speed = backward.
  const float speed_request =
      -static_cast<float>(limited_stick_y) / 127.0f * kPs4DriveMaxSpeed;
  return constrain(speed_request, -kPs4DriveMaxSpeed,
                   kPs4DriveMaxSpeed);
}

void printPs4DriveRequestDiagnostic(const Ps4InputState& input) {
  ps4_drive_speed_request =
      ps4DriveSpeedRequestFromLeftStickY(input.left_stick_y);
  Serial.println("PS4 DRIVE REQUEST:");
  Serial.printf("LY=%d\n", input.left_stick_y);
  Serial.printf("speed=%+.2f\n", ps4_drive_speed_request);
}

void printPs4InputDiagnostic(const Ps4InputState& input) {
  Serial.printf(
      "PS4 INPUT: LX=%d LY=%d RX=%d RY=%d L2=%u R2=%u "
      "Cross=%u Circle=%u Square=%u Triangle=%u "
      "Up=%u Down=%u Left=%u Right=%u L1=%u R1=%u "
      "Options=%u Share=%u PS=%u\n",
      input.left_stick_x, input.left_stick_y, input.right_stick_x,
      input.right_stick_y, input.l2, input.r2, input.cross, input.circle,
      input.square, input.triangle, input.dpad_up, input.dpad_down,
      input.dpad_left, input.dpad_right, input.l1, input.r1, input.options,
      input.share, input.ps_button);
}

const char* ps4DriveStateName(Ps4DriveState state) {
  switch (state) {
    case Ps4DriveState::Neutral:
      return "NEUTRAL";
    case Ps4DriveState::OutsideDeadzone:
      return "WAIT_FOR_RECENTER";
    case Ps4DriveState::DriveRequested:
      return "DRIVE_REQUESTED";
    case Ps4DriveState::Stopping:
      return "STOPPING";
  }

  return "UNKNOWN";
}

void printPs4DriveStopDiagnostic(const Ps4InputState& input,
                                 bool stick_in_deadzone,
                                 bool stop_complete,
                                 uint32_t now_ms) {
  if (now_ms - ps4_drive_stop_diagnostic_last_ms <
      kPs4DriveStopDiagnosticIntervalMs) {
    return;
  }
  ps4_drive_stop_diagnostic_last_ms = now_ms;

  const bool soft_stop_speed_is_low =
      abs(lqr_speed_for_control) <= kLqrSoftStopSpeedLimit;
  const bool soft_stop_speed_crossed_zero =
      (lqr_soft_stop_entry_speed > 0.0f &&
       lqr_speed_for_control <= 0.0f) ||
      (lqr_soft_stop_entry_speed < 0.0f &&
       lqr_speed_for_control >= 0.0f);
  const uint32_t soft_stop_stable_elapsed_ms =
      lqr_soft_stop_stable_start_ms == 0
          ? 0
          : now_ms - lqr_soft_stop_stable_start_ms;
  const bool soft_stop_speed_stable =
      lqr_soft_stop_stable_start_ms != 0 &&
      soft_stop_stable_elapsed_ms >= kLqrSoftStopStableTimeMs;

  const char* reason = "READY";
  if (lqr_auto_drive_active) {
    reason = "AUTO_DRIVE_ACTIVE";
  } else if (lqr_drive_start_pending) {
    reason = "DRIVE_START_PENDING";
  } else if (lqr_soft_stop_active) {
    reason = "SOFT_STOP_ACTIVE";
  }

  Serial.println("PS4 DRIVE STOP WAIT:");
  Serial.printf("stop_elapsed_ms=%lu\n",
                static_cast<unsigned long>(now_ms - ps4_drive_stop_start_ms));
  Serial.printf("ps4_drive_state=%s\n", ps4DriveStateName(ps4_drive_state));
  Serial.printf("lqr_auto_drive_active=%s\n",
                lqr_auto_drive_active ? "yes" : "no");
  Serial.printf("lqr_drive_start_pending=%s\n",
                lqr_drive_start_pending ? "yes" : "no");
  Serial.printf("lqr_soft_stop_active=%s\n",
                lqr_soft_stop_active ? "yes" : "no");
  Serial.printf("lqr_target_speed=%+.3f\n", lqr_target_speed);
  Serial.printf("ps4_drive_speed_request=%+.3f\n",
                ps4_drive_speed_request);
  Serial.printf("LY=%d\n", input.left_stick_y);
  Serial.printf("stick_in_deadzone=%s\n",
                stick_in_deadzone ? "yes" : "no");
  Serial.printf("stop_complete=%s\n", stop_complete ? "yes" : "no");
  Serial.printf("lqr_speed_for_control=%+.3f\n", lqr_speed_for_control);
  Serial.printf("lqr_soft_stop_entry_speed=%+.3f\n",
                lqr_soft_stop_entry_speed);
  Serial.printf("soft_stop_speed_is_low=%s\n",
                soft_stop_speed_is_low ? "yes" : "no");
  Serial.printf("soft_stop_speed_crossed_zero=%s\n",
                soft_stop_speed_crossed_zero ? "yes" : "no");
  Serial.printf("lqr_soft_stop_stable_start_ms=%lu\n",
                static_cast<unsigned long>(lqr_soft_stop_stable_start_ms));
  Serial.printf("soft_stop_stable_elapsed_ms=%lu\n",
                static_cast<unsigned long>(soft_stop_stable_elapsed_ms));
  Serial.printf("soft_stop_speed_stable=%s\n",
                soft_stop_speed_stable ? "yes" : "no");
  Serial.printf("soft_stop_low_speed_enter_count=%lu\n",
                static_cast<unsigned long>(soft_stop_low_speed_enter_count));
  Serial.printf("soft_stop_stable_reset_count=%lu\n",
                static_cast<unsigned long>(soft_stop_stable_reset_count));
  Serial.printf("stop_speed_min=%+.3f\n", stop_speed_min);
  Serial.printf("stop_speed_max=%+.3f\n", stop_speed_max);
  Serial.printf("stop_speed_abs_max=%.3f\n", stop_speed_abs_max);
  Serial.printf("reason=%s\n", reason);
}

void printPs4DriveStopSummary(uint32_t now_ms) {
  Serial.println("PS4 DRIVE STOP SUMMARY:");
  Serial.printf("elapsed_ms=%lu\n",
                static_cast<unsigned long>(now_ms - ps4_drive_stop_start_ms));
  Serial.printf("low_speed_enter_count=%lu\n",
                static_cast<unsigned long>(soft_stop_low_speed_enter_count));
  Serial.printf("stable_reset_count=%lu\n",
                static_cast<unsigned long>(soft_stop_stable_reset_count));
  Serial.printf("speed_min=%+.3f\n", stop_speed_min);
  Serial.printf("speed_max=%+.3f\n", stop_speed_max);
  Serial.printf("speed_abs_max=%.3f\n", stop_speed_abs_max);
  Serial.printf("final_speed=%+.3f\n", lqr_speed_for_control);
}

void setPs4DriveState(Ps4DriveState next_state) {
  if (ps4_drive_state == next_state) {
    return;
  }

  Serial.println("PS4 DRIVE STATE:");
  Serial.printf("%s -> %s\n", ps4DriveStateName(ps4_drive_state),
                ps4DriveStateName(next_state));
  ps4_drive_state = next_state;
}

void beginPs4DriveStopping(uint32_t now_ms) {
  setPs4DriveState(Ps4DriveState::Stopping);
  ps4_drive_stop_start_ms = now_ms;
  ps4_drive_stop_diagnostic_last_ms = 0;
  soft_stop_low_speed_enter_count = 0;
  soft_stop_stable_reset_count = 0;
  soft_stop_speed_was_low = false;
  stop_speed_min = lqr_speed_for_control;
  stop_speed_max = lqr_speed_for_control;
  stop_speed_abs_max = abs(lqr_speed_for_control);
}

void updatePs4Controller() {
  const bool connected = PS4.isConnected();
  if (connected != ps4_was_connected) {
    ps4_was_connected = connected;
    ps4_input_initialized = false;
    Serial.println(connected ? "PS4 CONTROLLER CONNECTED"
                             : "PS4 CONTROLLER DISCONNECTED");
    if (!connected) {
      ps4_drive_speed_request = 0.0f;
      ps4_drive_stick_y = 0;
      if (ps4_drive_state == Ps4DriveState::DriveRequested) {
        Serial.println("PS4 DRIVE:");
        Serial.println("DISCONNECTED -> STOP REQUEST");
        requestLqrSoftStop();
      }
      ps4_drive_state = Ps4DriveState::Neutral;
    }
  }

  if (!connected) {
    return;
  }

  const uint32_t now_ms = millis();
  if (ps4_input_initialized &&
      now_ms - ps4_last_diagnostic_ms < kPs4DiagnosticIntervalMs) {
    return;
  }

  const Ps4InputState input = readPs4Input();
  ps4_drive_stick_y = input.left_stick_y;
  ps4_drive_speed_request =
      ps4DriveSpeedRequestFromLeftStickY(input.left_stick_y);
  if (ps4_input_initialized) {
    const bool up_pressed = input.dpad_up && !previous_ps4_input.dpad_up;
    const bool down_pressed = input.dpad_down && !previous_ps4_input.dpad_down;

    if (up_pressed || down_pressed) {
      if (ps4_drive_state == Ps4DriveState::DriveRequested) {
        Serial.println("PS4 LEG CONTROL:");
        Serial.println("ignored while driving");
      } else {
        const float previous_percent = ps4_leg_extension_percent;
        const float step = up_pressed ? kPs4LegExtensionStepPercent
                                      : -kPs4LegExtensionStepPercent;
        ps4_leg_extension_percent =
            constrain(previous_percent + step, kPs4LegExtensionMinPercent,
                      kPs4LegExtensionMaxPercent);

        Serial.println("PS4 LEG CONTROL:");
        Serial.printf("direction=%s\n", up_pressed ? "UP" : "DOWN");
        Serial.printf("percent=%.1f\n", ps4_leg_extension_percent);
        if (ps4_leg_extension_percent == previous_percent) {
          Serial.printf("limit=%s\n", up_pressed ? "MAX" : "MIN");
        } else {
          moveBothLegsToPercent(ps4_leg_extension_percent);
        }
      }
    }
  }

  const bool stick_in_deadzone = ps4_drive_speed_request == 0.0f;
  if (!ps4_input_initialized && !stick_in_deadzone) {
    setPs4DriveState(Ps4DriveState::OutsideDeadzone);
    Serial.println(
        "PS4 DRIVE: connected outside deadzone; recenter stick to arm.");
  } else if (stick_in_deadzone) {
    if (ps4_drive_state == Ps4DriveState::DriveRequested) {
      Serial.println("PS4 DRIVE:");
      Serial.println("STOP REQUESTED");
      requestLqrSoftStop();
      beginPs4DriveStopping(now_ms);
    } else if (ps4_drive_state == Ps4DriveState::Stopping) {
      const bool stop_complete = !lqr_auto_drive_active &&
                                 !lqr_drive_start_pending &&
                                 !lqr_soft_stop_active;
      printPs4DriveStopDiagnostic(input, stick_in_deadzone, stop_complete,
                                  now_ms);
      if (stop_complete) {
        printPs4DriveStopSummary(now_ms);
        setPs4DriveState(Ps4DriveState::Neutral);
        Serial.println("PS4 DRIVE:");
        Serial.println("READY FOR NEXT REQUEST");
      }
    } else if (ps4_drive_state == Ps4DriveState::OutsideDeadzone) {
      setPs4DriveState(Ps4DriveState::Neutral);
      Serial.println("PS4 DRIVE:");
      Serial.println("READY FOR NEXT REQUEST");
    }
  } else if (ps4_drive_state == Ps4DriveState::Stopping) {
    const bool stop_complete = !lqr_auto_drive_active &&
                               !lqr_drive_start_pending &&
                               !lqr_soft_stop_active;
    printPs4DriveStopDiagnostic(input, stick_in_deadzone, stop_complete,
                                now_ms);
    if (stop_complete) {
      printPs4DriveStopSummary(now_ms);
      setPs4DriveState(Ps4DriveState::OutsideDeadzone);
    }
  } else if (ps4_drive_state == Ps4DriveState::Neutral) {
    Serial.println("PS4 DRIVE:");
    Serial.printf("REQUEST speed=%+.2f\n", ps4_drive_speed_request);
    if (requestLqrDrive(ps4_drive_speed_request,
                        ps4_drive_speed_request > 0.0f ? '>' : '<')) {
      setPs4DriveState(Ps4DriveState::DriveRequested);
      Serial.println("PS4 DRIVE:");
      Serial.printf("START REQUESTED speed=%+.2f\n",
                    ps4_drive_speed_request);
    } else {
      setPs4DriveState(Ps4DriveState::OutsideDeadzone);
      Serial.println(
          "PS4 DRIVE: request rejected by LQR safety gate; recenter stick.");
    }
  }

  if (ps4_drive_state == Ps4DriveState::DriveRequested &&
      lqr_auto_drive_active && !lqr_soft_stop_active &&
      now_ms - ps4_drive_target_update_last_ms >=
          kPs4DiagnosticIntervalMs) {
    const float target_delta =
        ps4_drive_speed_request - lqr_target_speed;
    lqr_target_speed +=
        constrain(target_delta, -kPs4DriveTargetSlewPerUpdate,
                  kPs4DriveTargetSlewPerUpdate);
    lqr_target_speed = constrain(lqr_target_speed, -kPs4DriveMaxSpeed,
                                 kPs4DriveMaxSpeed);
    ps4_drive_target_update_last_ms = now_ms;

    if (now_ms - ps4_drive_track_diagnostic_last_ms >=
        kPs4DriveTrackDiagnosticIntervalMs) {
      Serial.println("PS4 DRIVE TRACK:");
      Serial.printf("LY=%d\n", input.left_stick_y);
      Serial.printf("requested=%+.2f\n", ps4_drive_speed_request);
      Serial.printf("target=%+.2f\n", lqr_target_speed);
      ps4_drive_track_diagnostic_last_ms = now_ms;
    }
  }

  if (!ps4_input_initialized ||
      !ps4InputStatesEqual(input, previous_ps4_input)) {
    printPs4InputDiagnostic(input);
    if (!ps4_input_initialized ||
        input.left_stick_y != previous_ps4_input.left_stick_y) {
      printPs4DriveRequestDiagnostic(input);
    }
    previous_ps4_input = input;
    ps4_input_initialized = true;
    ps4_last_diagnostic_ms = now_ms;
  }
}

void pingLegServos() {
  Serial.println("LEG SERVO PING: starting (no motion command is sent)");

  const int right_result = sms_sts.Ping(kRightLegServoId);
  if (right_result == kRightLegServoId) {
    Serial.println("  Right leg ID 1: OK");
  } else {
    Serial.printf("  Right leg ID 1: FAILED (result=%d)\n", right_result);
  }

  const int left_result = sms_sts.Ping(kLeftLegServoId);
  if (left_result == kLeftLegServoId) {
    Serial.println("  Left leg ID 2: OK");
  } else {
    Serial.printf("  Left leg ID 2: FAILED (result=%d)\n", left_result);
  }

  Serial.println("LEG SERVO PING: complete");
}

void ensureLegServoTorqueEnabled() {
  if (!sms_sts.servo_off) {
    return;
  }

  sms_sts.on_all_servo();
  Serial.println("LEG SERVOS:");
  Serial.println("torque=ON");
  Serial.println("reason=LEG_COMMAND");
}

void disableLegServoTorqueForTestEnd(const char* reason) {
  if (sms_sts.servo_off) {
    return;
  }

  // Current test-phase behavior: release leg holding torque when a test ends.
  sms_sts.off_all_servo();
  Serial.println("LEG SERVOS:");
  Serial.println("torque=OFF");
  Serial.printf("reason=%s\n", reason);
}

void moveRightLegForTest(int16_t position) {
  uint8_t ids[] = {kRightLegServoId};
  int16_t positions[] = {position};
  uint16_t speeds[] = {kRightLegTestSpeed};
  uint8_t accelerations[] = {kRightLegTestAcceleration};

  ensureLegServoTorqueEnabled();
  sms_sts.SyncWritePosEx(ids, 1, positions, speeds, accelerations);
  Serial.printf("RIGHT LEG TEST: ID 1 moving to %d (speed=150, ACC=15)\n",
                position);
}

void moveLeftLegForTest(int16_t position, const char* action) {
  uint8_t ids[1] = {kLeftLegServoId};
  int16_t positions[1] = {position};
  uint16_t speeds[1] = {kLeftLegTestSpeed};
  uint8_t accelerations[1] = {kLeftLegTestAcceleration};

  Serial.println("LEFT LEG TEST:");
  Serial.println("ID=2");
  Serial.printf("target=%d\n", position);
  Serial.printf("action=%s\n", action);
  Serial.println("speed=150");
  Serial.println("acc=15");
  ensureLegServoTorqueEnabled();
  sms_sts.SyncWritePosEx(ids, 1, positions, speeds, accelerations);
}

int16_t rightLegPositionFromPercent(float percent) {
  const float constrained_percent = constrain(percent, 0.0f, 100.0f);
  const float position =
      kRightLegHomePosition +
      (kRightLegFullyExtendedPosition - kRightLegHomePosition) *
          constrained_percent / 100.0f;
  return static_cast<int16_t>(roundf(position));
}

int16_t leftLegPositionFromPercent(float percent) {
  const float constrained_percent = constrain(percent, 0.0f, 100.0f);
  const float position =
      kLeftLegHomePosition +
      (kLeftLegFullyExtendedPosition - kLeftLegHomePosition) *
          constrained_percent / 100.0f;
  return static_cast<int16_t>(roundf(position));
}

void moveBothLegsToPercent(float percent) {
  const float constrained_percent = constrain(percent, 0.0f, 100.0f);
  uint8_t ids[] = {kRightLegServoId, kLeftLegServoId};
  int16_t positions[] = {
      rightLegPositionFromPercent(constrained_percent),
      leftLegPositionFromPercent(constrained_percent),
  };
  uint16_t speeds[] = {kBothLegsTestSpeed, kBothLegsTestSpeed};
  uint8_t accelerations[] = {kBothLegsTestAcceleration,
                             kBothLegsTestAcceleration};

  Serial.println("BOTH LEGS TEST:");
  Serial.printf("percent=%.1f\n", constrained_percent);
  Serial.printf("right_id=%u\n", ids[0]);
  Serial.printf("right_target=%d\n", positions[0]);
  Serial.printf("left_id=%u\n", ids[1]);
  Serial.printf("left_target=%d\n", positions[1]);
  Serial.printf("speed=%u\n", kBothLegsTestSpeed);
  Serial.printf("acc=%u\n", kBothLegsTestAcceleration);

  ensureLegServoTorqueEnabled();
  sms_sts.SyncWritePosEx(ids, 2, positions, speeds, accelerations);
}

void handleSerialCommand() {
  if (Serial.available() <= 0) {
    return;
  }

  const char command = Serial.read();

  if (command == '\r' || command == '\n' ||
      command == ' ' || command == '\t') {
    return;
  }

  Serial.printf("COMMAND: %c\n", command);

  switch (command) {
    case '1':
      if (motionCommandAllowed()) {
        right_target_voltage = kTestVoltage;
        left_target_voltage = 0.0f;
      }
      break;
    case '2':
      if (motionCommandAllowed()) {
        right_target_voltage = -kTestVoltage;
        left_target_voltage = 0.0f;
      }
      break;
    case '3':
      if (motionCommandAllowed()) {
        right_target_voltage = 0.0f;
        left_target_voltage = kTestVoltage;
      }
      break;
    case '4':
      if (motionCommandAllowed()) {
        right_target_voltage = 0.0f;
        left_target_voltage = -kTestVoltage;
      }
      break;
    case 'f':
    case 'F':
      if (motionCommandAllowed()) {
        right_target_voltage = kTestVoltage;
        left_target_voltage = kTestVoltage;
      }
      break;
    case 'r':
    case 'R':
      if (motionCommandAllowed()) {
        right_target_voltage = -kTestVoltage;
        left_target_voltage = -kTestVoltage;
      }
      break;
    case 'x':
    case 'X':
      if (motionCommandAllowed()) {
        right_target_voltage = kTestVoltage;
        left_target_voltage = -kTestVoltage;
      }
      break;
    case 's':
    case 'S':
    case '0':
      polarity_pulse_active = false;
      polarity_pulse_voltage = 0.0f;
      setBothTargetsToZero();
      Serial.println("STOP: both targets = 0.00 V");
      break;
    case 'e':
    case 'E':
      polarity_test_active = false;
      polarity_pulse_active = false;
      polarity_pulse_voltage = 0.0f;
      stopLqrDriveOnly("MANUAL_DRIVE_STOP", true);
      break;
    case 'u':
    case 'U':
      if (!test_system_ready) {
        Serial.println("IGNORED: both motors are not initialized.");
        break;
      }
      setBothTargetsToZero();
      driver1.enable();
      driver2.enable();
      emergency_stop_active = false;
      Serial.println("DRIVERS ENABLED: both targets remain 0.00 V.");
      break;
    case 'p':
    case 'P':
      if (test_system_ready) {
        printLqrStatus();
      }
      break;
    case 'q':
    case 'Q':
      if (test_system_ready) {
        stopBothMotorsAtZero();
        printDistanceDiagnostic();
      }
      break;
    case 'v':
    case 'V':
      if (test_system_ready) {
        stopBothMotorsAtZero();
        printSpeedDiagnostic();
      }
      break;
    case 'l':
    case 'L':
      pingLegServos();
      break;
    case 'm':
    case 'M':
      moveRightLegForTest(kRightLegTestExtend10PercentPosition);
      break;
    case 'n':
    case 'N':
      moveRightLegForTest(kRightLegHomePosition);
      break;
    case 'o':
    case 'O':
      moveLeftLegForTest(kLeftLegTestExtend10PercentPosition,
                         "EXTEND_10_PERCENT");
      break;
    case 'y':
    case 'Y':
      moveLeftLegForTest(kLeftLegHomePosition, "HOME");
      break;
    case 'w':
    case 'W':
      moveBothLegsToPercent(10.0f);
      break;
    case '!':
      moveBothLegsToPercent(20.0f);
      break;
    case '@':
      moveBothLegsToPercent(30.0f);
      break;
    case '9':
      moveBothLegsToPercent(0.0f);
      break;
    case 'c':
    case 'C':
      if (test_system_ready) {
        calibrateMpu6050();
      }
      break;
    case 'z':
    case 'Z':
      if (balance_wait_active || lqr_direction_test_armed) {
        Serial.println("IGNORED: zero points cannot change during Balance Wait or Direction Test.");
      } else if (test_system_ready) {
        stopBothMotorsAtZero();
        angle_zeropoint = LQR_angle;
        distance_zeropoint = LQR_distance;
        lqr_zero_set = true;
        Serial.printf("LQR zero points captured: angle=%.4f distance=%.4f\n",
                      angle_zeropoint, distance_zeropoint);
      }
      break;
    case 'a':
    case 'A':
      if (!test_system_ready) {
        Serial.println("IGNORED: system initialization is not complete.");
      } else if (!lqr_zero_set) {
        Serial.println("IGNORED: capture zero points with 'z' before arming.");
      } else {
        stopBothMotorsAtZero();
        cancelBalanceWait();
        printAndResetLqrRunTime();
        lqr_direction_test_armed = false;

        printVelocityTrace();
        printLoopTimingTrace();
        printLqrControlTrace();

        lqr_diagnostic_armed = true;
        Serial.println("LQR diagnostics ARMED; LQR_u is not connected to motors.");
      }
      break;
    case 'g':
    case 'G':
      cycleLqrTestMode();
      break;
    case 'i':
    case 'I':
      cycleLqrSpeedSource();
      break;
    case 't':
    case 'T':
      startBalanceTest();
      break;
    case '>':
    case '<':
    case '.':
      if (!lqr_direction_test_armed) {
        Serial.println(
            "IGNORED: LQR drive commands are available only after AUTO START.");
      } else if (command == '.') {
        requestLqrSoftStop();
      } else if (!requestLqrDrive(
                     command == '>' ? kLqrDriveTestSpeed
                                    : -kLqrDriveTestSpeed,
                     command)) {
        Serial.println("IGNORED: LQR drive request is already active.");
      }
      break;
    case 'j':
    case 'J':
      if (!test_system_ready) {
        Serial.println("IGNORED: system initialization is not complete.");
      } else if (emergency_stop_active) {
        Serial.println("IGNORED: drivers are disabled; send 'u' first.");
      } else {
        printAndResetLqrRunTime();
        lqr_direction_test_armed = false;
        lqr_diagnostic_armed = false;
        lqr_startup_diagnostic_active = false;
        cancelBalanceWait();
        stopBothMotorsAtZero();
        printVelocityTrace();
        printLoopTimingTrace();
        printLqrControlTrace();
        updateLqrDiagnostic();
        polarity_test_active = true;
        polarity_pulse_active = false;
        polarity_drive_right = false;
        polarity_drive_left = false;
        polarity_pulse_voltage = 0.0f;
        polarity_right_velocity_sum = 0.0f;
        polarity_left_velocity_sum = 0.0f;
        polarity_right_velocity_peak = 0.0f;
        polarity_left_velocity_peak = 0.0f;
        polarity_velocity_sample_count = 0;
        polarity_pulse_command = '?';
        polarity_reference_angle = LQR_angle;
        polarity_diagnostic_last_ms = millis();
        Serial.println("LQR POLARITY TEST:");
        Serial.println("Hold the robot securely with both hands.");
        Serial.println("Tilt it slightly forward/backward.");
        Serial.println("Use + / - commands for short low-voltage wheel movement.");
        Serial.printf("polarity_test_voltage=%.2f V\n",
                      polarity_test_voltage);
      }
      break;
    case '+':
    case '-':
      if (!polarity_test_active) {
        Serial.println("IGNORED: enter LQR polarity test mode with 'j' first.");
      } else if (abs(LQR_angle - polarity_reference_angle) >
                 kLqrTestTiltLimitDeg) {
        polarity_pulse_active = false;
        polarity_pulse_voltage = 0.0f;
        setBothTargetsToZero();
        Serial.println("IGNORED: polarity pulse tilt exceeds 10 degrees.");
      } else {
        polarity_right_velocity_sum = 0.0f;
        polarity_left_velocity_sum = 0.0f;
        polarity_right_velocity_peak = 0.0f;
        polarity_left_velocity_peak = 0.0f;
        polarity_velocity_sample_count = 0;
        polarity_pulse_command = command;
        polarity_drive_right = true;
        polarity_drive_left = true;
        polarity_right_start_angle = motor1.shaft_angle;
        polarity_left_start_angle = motor2.shaft_angle;
        polarity_pulse_voltage =
            command == '+' ? polarity_test_voltage : -polarity_test_voltage;
        polarity_pulse_start_ms = millis();
        polarity_pulse_active = true;
        const float polarity_m1_sign =
            m1_direction < 0.0f ? -1.0f : 1.0f;
        const float polarity_m2_sign =
            m2_direction < 0.0f ? -1.0f : 1.0f;
        Serial.println("POLARITY PULSE:");
        Serial.printf("command=%c\n", command);
        Serial.printf("right_output=%.6f\n",
                      polarity_pulse_voltage * polarity_m1_sign);
        Serial.printf("left_output=%.6f\n",
                      polarity_pulse_voltage * polarity_m2_sign);
      }
      break;
    case '5':
    case '6':
    case '7':
    case '8':
      if (!polarity_test_active) {
        Serial.println("IGNORED: enter LQR polarity test mode with 'j' first.");
      } else if (abs(LQR_angle - polarity_reference_angle) >
                 kLqrTestTiltLimitDeg) {
        polarity_pulse_active = false;
        polarity_pulse_voltage = 0.0f;
        setBothTargetsToZero();
        Serial.println("IGNORED: polarity pulse tilt exceeds 10 degrees.");
      } else {
        polarity_right_velocity_sum = 0.0f;
        polarity_left_velocity_sum = 0.0f;
        polarity_right_velocity_peak = 0.0f;
        polarity_left_velocity_peak = 0.0f;
        polarity_velocity_sample_count = 0;
        polarity_pulse_command = command;

        polarity_drive_right = command == '5' || command == '6';
        polarity_drive_left = command == '7' || command == '8';

        polarity_right_start_angle = motor1.shaft_angle;
        polarity_left_start_angle = motor2.shaft_angle;

        const bool positive_pulse = command == '5' || command == '7';
        polarity_pulse_voltage =
            positive_pulse ? polarity_test_voltage : -polarity_test_voltage;

        polarity_pulse_start_ms = millis();
        polarity_pulse_active = true;

        const float polarity_m1_sign =
            m1_direction < 0.0f ? -1.0f : 1.0f;
        const float polarity_m2_sign =
            m2_direction < 0.0f ? -1.0f : 1.0f;

        const float right_pulse_output =
            polarity_drive_right
                ? polarity_pulse_voltage * polarity_m1_sign
                : 0.0f;
        const float left_pulse_output =
            polarity_drive_left
                ? polarity_pulse_voltage * polarity_m2_sign
                : 0.0f;

        Serial.println("SINGLE-WHEEL POLARITY PULSE:");
        Serial.printf("command=%c\n", command);
        Serial.printf("right_output=%.6f\n", right_pulse_output);
        Serial.printf("left_output=%.6f\n", left_pulse_output);
      }
      break;
    case '[':
    case ']':
      if (!polarity_test_active) {
        Serial.println(
            "IGNORED: enter LQR polarity test mode with 'j' first.");
      } else if (polarity_pulse_active) {
        Serial.println(
            "IGNORED: wait for the current polarity pulse to finish.");
      } else {
        if (command == '[') {
          polarity_test_voltage -= kPolarityTestVoltageStep;
        } else {
          polarity_test_voltage += kPolarityTestVoltageStep;
        }

        polarity_test_voltage =
            constrain(polarity_test_voltage,
                      kPolarityTestVoltageMin,
                      kPolarityTestVoltageMax);

        // Remove floating-point accumulation noise at 0.01 V steps.
        polarity_test_voltage =
            roundf(polarity_test_voltage * 100.0f) / 100.0f;

        Serial.printf("polarity_test_voltage=%.2f V\n",
                      polarity_test_voltage);
      }
      break;
    case 'k':
    case 'K':
      if (!polarity_test_active) {
        Serial.println("LQR polarity test mode is not active.");
      } else {
        polarity_test_active = false;
        polarity_pulse_active = false;
        polarity_drive_right = false;
        polarity_drive_left = false;
        polarity_pulse_voltage = 0.0f;
        stopBothMotorsAtZero();
        Serial.println("LQR POLARITY TEST ended; both targets are 0 V.");
      }
      break;
    case 'b':
    case 'B':
      if (!balance_wait_active) {
        Serial.println("Balance brake toggle is available only during Balance Wait.");
      } else {
        balance_brake_enabled = !balance_brake_enabled;
        balance_stable_start_ms = 0;
        balance_stable_elapsed_ms = 0;
        balance_candidate_angle = 0.0f;
        balance_angle_sum = 0.0f;
        balance_distance_sum = 0.0f;
        balance_sample_count = 0;
        balance_timer_reset = true;
        balance_brake_right = 0.0f;
        balance_brake_left = 0.0f;
        setBothTargetsToZero();
        Serial.printf("Balance brake: %s\n",
                      balance_brake_enabled ? "ON" : "OFF");
      }
      break;
    case 'd':
    case 'D':
      if (stopBalanceTest()) {
        printVelocityTrace();
        printLoopTimingTrace();
        printLqrControlTrace();
        emergency_stop_active = true;
        disableBothDrivers();
        Serial.println(
            "SAFETY STOP: mode=FULL_STOP reason=DISARM drivers=DISABLED balance=OFF");
        disableLegServoTorqueForTestEnd("DISARM");
      }
      break;
    case 'h':
    case 'H':
    case '?':
      printHelp();
      break;
    case '\r':
    case '\n':
    case ' ':
      break;
    default:
      Serial.print("Unknown command: ");
      Serial.println(command);
      setBothTargetsToZero();
      break;
  }
}
}  // namespace
