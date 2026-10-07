#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include <stdarg.h>
#include <string.h>
#include <atomic>
#include <esp_timer.h>
#include <driver/gpio.h>

namespace {
// Wiring and the provisional 7 pole pairs follow main.cpp, not the 2208 label.
constexpr int kPolePairs = 7;
constexpr float kSupplyVoltage = 8.0f;  // Match actual supply before powering.
constexpr float kAlignVoltage = 0.5f;   // Lower than main.cpp's 2 V; may not align.
constexpr float kDriveVoltage = 0.3f;   // Voltage torque, NOT current limiting.
constexpr uint32_t kRunMs = 1500;       // Below 2 s, allowing loop/I2C overhead.
constexpr uint32_t kAlignMs = 6000;     // Guard for blocking initFOC, not motor.init.
constexpr uint32_t kStreamMs = 200;
constexpr uint16_t kI2cTimeoutMs = 5;
constexpr uint8_t kRightEnable = 27, kLeftEnable = 32;
constexpr float kTau = 6.283185307179586f;

void gateOff() {
  // Arduino-ESP32 3.x digitalWrite() ignores pins not yet registered by
  // pinMode(). IDF gpio_set_level() really preloads the output latch at boot.
  gpio_set_level(static_cast<gpio_num_t>(kRightEnable), LOW);
  gpio_set_level(static_cast<gpio_num_t>(kLeftEnable), LOW);
}

esp_timer_handle_t outputTimer = nullptr;
std::atomic<bool> timerExpired{false};
bool timerReady = false;
void outputTimeout(void*) {
  // esp_timer task: no I2C, PWM library calls, or Serial. Independently gate
  // OFF while the Arduino loop is blocked; never enable from this callback.
  gateOff();
  timerExpired.store(true);
}
bool armTimer(uint32_t ms) {
  if (!timerReady) return false;
  esp_timer_stop(outputTimer);
  timerExpired.store(false);
  return esp_timer_start_once(outputTimer, uint64_t(ms) * 1000) == ESP_OK;
}

// No heap/String or blocking Print in the control loop. On overflow drop a
// whole record. Status exposes the count. Only pump what UART can accept.
char tx[2048];
size_t txRead = 0, txWrite = 0;
uint32_t droppedLogs = 0;
void logLine(const char* format, ...) {
  char line[192];
  va_list args;
  va_start(args, format);
  const int n = vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  if (n < 0 || n >= static_cast<int>(sizeof(line))) { ++droppedLogs; return; }
  const size_t free = (txRead + sizeof(tx) - txWrite - 1) % sizeof(tx);
  if (free < static_cast<size_t>(n)) { ++droppedLogs; return; }
  for (int i = 0; i < n; ++i) {
    tx[txWrite] = line[i];
    txWrite = (txWrite + 1) % sizeof(tx);
  }
}
void pumpLog() {
  int room = Serial.availableForWrite();
  if (room > 64) room = 64;
  while (room-- > 0 && txRead != txWrite) {
    Serial.write(static_cast<uint8_t>(tx[txRead]));
    txRead = (txRead + 1) % sizeof(tx);
  }
}

// MagneticSensorI2C 2.4.0 records endTransmission() but does not validate
// requestFrom() length. Sensor::update() silently retains the previous angle
// for a negative return. Explicitly latch errors and gate OFF in the callback,
// including when initFOC() itself is polling. Never display retained data as OK.
class CheckedAS5600 : public Sensor {
 public:
  explicit CheckedAS5600(TwoWire& bus) : bus_(bus) {}
  bool fault = false;
  bool ready = false;
  const char* error = "NOT_INITIALIZED";
  uint32_t lastGoodMs = 0;

  void begin(int sda, int scl) {
    if (!bus_.begin(sda, scl, 400000)) { fail("BUS_INIT"); return; }
    bus_.setTimeOut(kI2cTimeoutMs);
    // Avoid Sensor::init() committing negative samples to its internal fields.
    const float a = getSensorAngle();
    if (fault) return;
    angle_prev = vel_angle_prev = a;
    angle_prev_ts = vel_angle_prev_ts = micros();
    ready = true;
  }

  void update() override {
    if (!ready || fault) return;
    const float a = getSensorAngle();
    if (fault) return;
    // Shortest signed delta across 0/360. Poll continuously, even with stream
    // off. Valid provided motion between samples is less than half a turn.
    const float delta = a - angle_prev;
    if (delta > kTau / 2) --full_rotations;
    else if (delta < -kTau / 2) ++full_rotations;
    angle_prev = a;
    angle_prev_ts = micros();
  }

 protected:
  float getSensorAngle() override {
    if (fault) return -1.0f;
    bus_.beginTransmission(0x36);
    bus_.write(0x0B);  // STATUS, RAW_ANGLE high, RAW_ANGLE low (auto increment)
    if (bus_.endTransmission(false) != 0) return fail("I2C_ADDRESS/NACK");
    if (bus_.requestFrom(uint8_t(0x36), uint8_t(3)) != 3 || bus_.available() != 3)
      return fail("I2C_SHORT_READ/TIMEOUT");
    const int status = bus_.read(), high = bus_.read(), low = bus_.read();
    if (status < 0 || high < 0 || low < 0 || (high & 0xF0))
      return fail("INVALID_DATA");
    if (!(status & 0x20) || (status & 0x18)) return fail("MAGNET_MISSING/WEAK/STRONG");
    const float a = ((high << 8) | low) * (kTau / 4096.0f);
    if (!isfinite(a) || a < 0 || a >= kTau) return fail("INVALID_ANGLE");
    lastGoodMs = millis();
    error = "OK";
    return a;
  }
 private:
  TwoWire& bus_;
  float fail(const char* reason) {
    fault = true;
    error = reason;
    gateOff();  // Immediately gate both wheels; PWM cleanup follows in loop.
    return -1.0f;
  }
};

TwoWire rightBus(0), leftBus(1);
CheckedAS5600 rightSensor(rightBus), leftSensor(leftBus);
BLDCMotor rightMotor(kPolePairs), leftMotor(kPolePairs);
BLDCDriver3PWM rightDriver(13, 12, 14, kRightEnable);
BLDCDriver3PWM leftDriver(26, 25, 33, kLeftEnable);

struct Wheel {
  const char* name;
  CheckedAS5600& sensor;
  BLDCMotor& motor;
  BLDCDriver3PWM& driver;
  uint8_t enable;
  bool driverReady = false;
  bool calibrated = false;
  bool initFault = false;
};
Wheel right{"right", rightSensor, rightMotor, rightDriver, kRightEnable};
Wheel left{"left", leftSensor, leftMotor, leftDriver, kLeftEnable};
Wheel* running = nullptr;
bool streaming = false;
uint32_t runStarted = 0, streamAt = 0;
float commandVoltage = 0;
char command[64];
size_t commandLength = 0;
bool discardLine = false;

void stopAll(const char* reason) {
  gateOff();  // Gate first, before any PWM work.
  if (timerReady) esp_timer_stop(outputTimer);
  for (Wheel* w : {&right, &left}) {
    w->motor.target = 0;
    w->motor.voltage.q = w->motor.voltage.d = 0;
    if (w->driverReady) w->motor.disable();
  }
  running = nullptr;
  commandVoltage = 0;
  logLine("STOP: %s; both outputs=0, drivers=DISABLED\n", reason);
}

bool sensorsOK() {
  return rightSensor.ready && leftSensor.ready && !rightSensor.fault && !leftSensor.fault;
}
void reportSensor(Wheel& w) {
  if (!w.sensor.ready || w.sensor.fault) {
    logLine("%s sensor=FAULT %s; angle/continuous/turns=INVALID (latched; reboot)\n",
            w.name, w.sensor.error);
    return;
  }
  logLine("%s angle=%.3f deg continuous=%.3f deg turns=%.5f wraps=%ld age=%lu ms\n",
          w.name, w.sensor.getMechanicalAngle() * 360 / kTau,
          w.sensor.getPreciseAngle() * 360 / kTau,
          w.sensor.getPreciseAngle() / kTau,
          static_cast<long>(w.sensor.getFullRotations()),
          static_cast<unsigned long>(millis() - w.sensor.lastGoodMs));
}
void reportStatus() {
  logLine("state=%s selected=%s command=%+.2f V stream=%s dropped_logs=%lu\n",
          running ? "RUNNING" : "IDLE", running ? running->name : "none",
          commandVoltage, streaming ? "on" : "off", static_cast<unsigned long>(droppedLogs));
  logLine("pp=%d (provisional) supply=%.2f V align=%.2f V drive=%.2f V run=%lu ms I2C_timeout=%u ms\n",
          kPolePairs, kSupplyVoltage, kAlignVoltage, kDriveVoltage,
          static_cast<unsigned long>(kRunMs), kI2cTimeoutMs);
  logLine("output_timer=%s initFOC_guard=%lu ms (motor.init excluded)\n",
          timerReady ? "READY" : "FAULT", static_cast<unsigned long>(kAlignMs));
  for (Wheel* w : {&left, &right}) {
    logLine("%s sensor=%s driver=%s pwm_init=%d FOC=%d init_fault=%d\n", w->name,
            w->sensor.error, digitalRead(w->enable) ? "ENABLED" : "DISABLED",
            w->driverReady, w->calibrated, w->initFault);
    reportSensor(*w);
  }
}
void help() {
  logLine("help | status | read left | read right | read both | stream on | stream off | stop\n");
  logLine("motor left + | motor left - | motor right + | motor right -\n");
  logLine("First motor command aligns/moves that wheel. +/- is voltage sign, not vehicle direction.\n");
  logLine("Blocking init/initFOC: serial stop unavailable; FOC guard=6000ms, run timer=1500ms. Use power cut.\n");
}

void startWheel(Wheel& w, float sign) {
  if (running) { logLine("REJECT: already running; stop first, no queued drive\n"); return; }
  rightSensor.update();
  leftSensor.update();
  if (!sensorsOK() || w.initFault || !timerReady) {
    stopAll("sensor/init fault; reboot after correcting cause");
    return;
  }
  gateOff();
  if (!w.calibrated) {
    // Warning is sent before enabling any motor. Blocking output is allowed
    // ONLY here, while both enable pins are LOW.
    while (txRead != txWrite) { pumpLog(); delay(1); }
    Serial.printf("WARNING: %s first initFOC can MOVE at %.2f V; blocking ~4+ s. "
                  "Serial stop unavailable; FOC gate-off guard=6000ms; use physical power cut.\n",
                  w.name, kAlignVoltage);
    Serial.flush();
    w.driver.voltage_power_supply = kSupplyVoltage;
    // Driver limit is phase/PWM range; motor limit bounds commanded Uq.
    // Keep centering at half the actual supply, avoiding phase clipping.
    w.driver.voltage_limit = kSupplyVoltage;
    if (!w.driver.init()) {
      w.initFault = true;
      stopAll("driver init failed");
      return;
    }
    w.driverReady = true;
    w.driver.disable();
    w.motor.linkDriver(&w.driver);
    w.motor.linkSensor(&w.sensor);
    w.motor.voltage_limit = kAlignVoltage;
    w.motor.voltage_sensor_align = kAlignVoltage;
    w.motor.controller = MotionControlType::torque;
    w.motor.torque_controller = TorqueControlType::voltage;
    // motor.init() enables after a 500ms delay. An error BEFORE that enable
    // is excluded by fresh checks above; errors during initFOC gate off in
    // CheckedAS5600. This synchronous path performs no subsequent enable.
    const bool initOK = w.motor.init();
    const bool guardOK = initOK && armTimer(kAlignMs);
    const bool focOK = guardOK && w.motor.initFOC();
    gateOff();
    if (timerReady) esp_timer_stop(outputTimer);
    w.motor.disable();
    bool inputDuringInit = false;
    bool atBoundary = true;
    while (Serial.available()) {
      inputDuringInit = true;
      atBoundary = (Serial.read() == '\n');
    }
    if (inputDuringInit) {
      commandLength = 0;
      discardLine = !atBoundary;
    }
    // SimpleFOC treats pole-pair mismatch as a warning, not failure. For this
    // diagnostic reject it; do not silently accept an estimated pole count.
    if (!focOK || timerExpired.load() || !w.motor.pp_check_result || !sensorsOK()) {
      w.initFault = true;
      stopAll("FOC/sensor/pole-pair check failed (no voltage increase/retry)");
      return;
    }
    w.calibrated = true;
    logLine("%s FOC OK direction=%d zero=%.5f rad pp_check=OK\n",
            w.name, static_cast<int>(w.motor.sensor_direction), w.motor.zero_electric_angle);
    // Do not replay commands received during blocking init. Any input cancels
    // the pulse (including queued stop). Discard partial input through newline.
    if (inputDuringInit) {
      stopAll("input during initialization; pulse cancelled, commands discarded");
      return;
    }
  }
  // Fresh communication check after all blocking library delays, before enable.
  rightSensor.update();
  leftSensor.update();
  if (!sensorsOK()) { stopAll("sensor fault before drive"); return; }
  w.motor.voltage_limit = kDriveVoltage;
  w.motor.target = 0;
  if (!armTimer(kRunMs)) { stopAll("output timer failed"); return; }
  runStarted = millis();
  w.motor.enable();
  running = &w;
  commandVoltage = sign * kDriveVoltage;
  logLine("RUN %s command=%+.2f V for %lu ms; other driver=DISABLED\n", w.name,
          commandVoltage, static_cast<unsigned long>(kRunMs));
}

void dispatch(const char* s) {
  if (!strcmp(s, "stop")) stopAll("command");
  else if (!strcmp(s, "help")) help();
  else if (!strcmp(s, "status")) reportStatus();
  else if (!strcmp(s, "read left")) reportSensor(left);
  else if (!strcmp(s, "read right")) reportSensor(right);
  else if (!strcmp(s, "read both")) { reportSensor(left); reportSensor(right); }
  else if (!strcmp(s, "stream on")) { streaming = true; logLine("stream=on (5 Hz)\n"); }
  else if (!strcmp(s, "stream off")) { streaming = false; logLine("stream=off\n"); }
  else if (!strcmp(s, "motor left +")) startWheel(left, 1);
  else if (!strcmp(s, "motor left -")) startWheel(left, -1);
  else if (!strcmp(s, "motor right +")) startWheel(right, 1);
  else if (!strcmp(s, "motor right -")) startWheel(right, -1);
  else logLine("REJECT: unknown/invalid command; no motor action\n");
}

// Bounded RX work; floods cannot prevent the next stop/deadline check.
void receiveCommands() {
  for (int budget = 0; budget < 64 && Serial.available(); ++budget) {
    const int c = Serial.read();
    if (c == '\n') {
      if (!discardLine) {
        if (commandLength && command[commandLength - 1] == '\r') --commandLength;
        command[commandLength] = '\0';
        // Reset BEFORE dispatch: initialization may intentionally set
        // discardLine to reject the remainder of a partially received line.
        commandLength = 0;
        dispatch(command);
      } else {
        logLine("REJECT: oversized/non-printable/initialization input\n");
        commandLength = 0;
        discardLine = false;
      }
    } else if (!discardLine) {
      if ((c < 32 && c != '\r') || c > 126 || commandLength >= sizeof(command) - 1) {
        discardLine = true;
      } else command[commandLength++] = static_cast<char>(c);
    }
  }
}

void checkStop() {
  if (!running) return;
  if (timerExpired.load()) stopAll("independent output timer");
  else if (!sensorsOK()) stopAll("sensor fault latched");
  else if (static_cast<uint32_t>(millis() - runStarted) >= kRunMs) stopAll("timeout");
}
}  // namespace

void setup() {
  // Preload LOW output latch BEFORE changing pin direction. No driver.init(),
  // motor.init(), initFOC(), or PWM attach at startup. Boot ROM/high-Z before
  // setup cannot be controlled here; use hardware enable pulldowns.
  gateOff();
  pinMode(kRightEnable, OUTPUT);
  pinMode(kLeftEnable, OUTPUT);
  Serial.begin(115200);
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback = outputTimeout;
  timerArgs.name = "wheel_gate_off";
  timerReady = (esp_timer_create(&timerArgs, &outputTimer) == ESP_OK);
  rightSensor.begin(19, 18);
  leftSensor.begin(23, 5);
  logLine("Wheel diagnostic: drivers DISABLED; sensors only; no startup alignment.\n");
  help();
  reportStatus();
}

void loop() {
  checkStop();
  rightSensor.update();
  checkStop();
  leftSensor.update();
  checkStop();
  receiveCommands();
  checkStop();
  if (running) {
    running->motor.loopFOC();  // Checked sensor callback can gate off here too.
    checkStop();
    if (running) running->motor.move(commandVoltage);
  }
  if (streaming && static_cast<uint32_t>(millis() - streamAt) >= kStreamMs) {
    streamAt = millis();
    reportSensor(left);
    reportSensor(right);
  }
  pumpLog();
}
