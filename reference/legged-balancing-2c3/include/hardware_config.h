#pragma once

// Seeed Studio XIAO ESP32C3 pin map.  Keep every GPIO assignment here.
constexpr int LASER_OUTPUT_PIN = 2;       // D0, active HIGH
constexpr int TILT_PWM_PIN = 3;           // D1, reserved
constexpr int I2S_DIN_PIN = 4;            // D2
constexpr int I2S_BCLK_PIN = 5;           // D3
constexpr int I2S_LRC_PIN = 6;            // D4
constexpr int FIRING_LED_PIN = 7;          // D5, active HIGH
constexpr int HALF_DUPLEX_TX_PIN = 21;     // D6
constexpr int HALF_DUPLEX_RX_PIN = 20;     // D7
constexpr int HALF_DUPLEX_OE_PIN = 10;     // D10, active LOW

constexpr bool kLaserActiveHigh = true;
constexpr bool kFiringLedActiveHigh = true;
constexpr uint32_t HALF_DUPLEX_BAUD = 1000000;

// Set this single value to false to suppress bus RX/echo/PING diagnostics on
// the USB serial port.  The USB '?' status query remains available.
constexpr bool HALF_DUPLEX_RX_LOG_ENABLED = true;
