# Legged balancing 2v1

## 1 Mbps bus diagnostic firmware

The normal balancing firmware remains the default `esp32dev` environment. The
bus diagnostic is a separate, explicitly selected firmware which does not
initialize or drive either wheel motor, the MPU6050, Wi-Fi, or the gamepad.

```sh
pio run -e bus_diagnostic
pio run -e bus_diagnostic -t upload
pio device monitor -b 115200
```

Do not upload as part of review. Select `bus_diagnostic` explicitly only when
the robot is secured and wheel power is safe. The existing source establishes
the shared bus as 1 Mbps 8N1 with ESP32 RX GPIO16 and TX GPIO17. It contains no
direction-enable GPIO, so the diagnostic preserves that wiring assumption; the
external half-duplex direction/echo behavior must be checked on the machine.

### Menu

Commands are newline-terminated:

| Command | Action / expected result |
| --- | --- |
| `help` | Print the menu; no bus transmission. |
| `ping1`, `ping2` | Ping the right ID1 or left ID2 STS3215 without moving it. A valid, checksummed non-echo status packet is `RESPONDED`; otherwise it times out. |
| `ping3` | Send one standard STS-style Ping to ESP32C3 ID3. A matching, zero-error, length-2 status packet is required; a transmitted-packet echo is logged and ignored. |
| `pingall` | Run one standard Ping each for ID1, ID2, and ID3. |
| `pos1`, `pos2` | Read the current raw position at STS address 56 from ID1 or ID2. |
| `rhome`, `rext` | Move only right ID1 to HOME 2061 or small extension 2044. |
| `lhome`, `lext` | Move only left ID2 to HOME 2026 or small extension 2044. |
| `c3ping` | Send the ID3 `0xA0` ASCII command `PING` (expected payload: `OK PONG`). |
| `c3help` | Send the ID3 `0xA0` ASCII command `HELP` and display its ASCII response. |
| `c3status` | Send the ID3 `0xA0` ASCII command `STATUS` and display its ASCII response. |
| `c3arm` | Send the ID3 `0xA0` ASCII command `ARM` exactly once and display the response. It does not fire and no ARM state is cached locally. |
| `c3fire <n>` | Send the ID3 `0xA0` ASCII command `FIRE <n>` exactly once, where `<n>` is exactly one digit from 1 through 9. Missing, signed, out-of-range, whitespace-containing, or suffixed values are rejected before transmission. There is no automatic ARM or retry. |
| `c3disarm` | Send the ID3 `0xA0` ASCII command `DISARM` exactly once and display the response. |
| `c3stop` | Send the ID3 `0xA0` ASCII command `STOP` exactly once and display the response. |
| `c3tilt <us>` | Send one ID3 `0xA0` ASCII command `TILT <us>` to set the barrel servo pulse. `<us>` must contain only decimal digits and be in the inclusive range 500–2400; invalid input is not transmitted. Success requires the exact ASCII response `OK TILT <us> us`. |
| `c3mute on` | Send the ID3 `0xA0` ASCII command `MUTE ON` exactly once. Success requires the exact ASCII response `OK MUTE ON`. This mutes only the automatic firing audio produced by `FIRE`. |
| `c3mute off` | Send the ID3 `0xA0` ASCII command `MUTE OFF` exactly once. Success requires the exact ASCII response `OK MUTE OFF`. |
| `battery` | Print averaged GPIO36 ADC mV, converted battery voltage, and GPIO22 LED state (`NORMAL`, `LOW`, `CRITICAL`, or `INVALID`). |
| `battery stream on` | Enable the same battery report at 1 Hz; measurement and LED operation are independent of streaming. |
| `battery stream off` | Disable the 1 Hz battery report. |

Leg moves use the values already present in the balancing firmware: speed 150
and acceleration 15. Startup, help, position reads, and Ping never issue a move.
Every move prints its target ID and position and is sent once, with no retry.
The ID3 commands were checked against the source under
`reference/legged-balancing-2c3`; that directory remains reference-only and is
not part of either PlatformIO build. The diagnostic validates the ID, declared
length, checksum, and zero error byte before accepting an ID3 response. A
timeout, malformed packet, TX echo, or packet from another ID is never reported
as success. Response parameters from `0xA0` commands must be printable ASCII and
are displayed on the USB monitor.

The diagnostic does not send either MUTE command at startup, while displaying
help, or during Ping. It neither saves nor infers MUTE state: that state belongs
to the C3's RAM, and restarting the C3 restores its default of audio enabled.
MUTE applies only to the automatic firing sound made by `FIRE`; a manual C3
`SOUND` command still plays while MUTE is on. (The diagnostic does not expose a
manual `SOUND` command.) Missing arguments, values other than exactly `on` or
`off`, extra whitespace, and suffixes are rejected before bus transmission.

### Battery voltage and status LED

The new-board battery divider is monitored only by this diagnostic firmware.
GPIO36 uses the calibrated `analogReadMilliVolts()` API with the Arduino-ESP32
3.x `ADC_ATTEN_DB_12` attenuation setting (the range formerly called 11 dB).
Sixteen samples, spaced 10 ms apart without a blocking wait, are averaged.
Battery voltage is `ADC volts * 133 / 33`; a separate calibration multiplier in
`src/bus_diagnostic.cpp` is initially `1.0`. GPIO22 drives the active-HIGH LED:

| Averaged voltage / condition | LED and state |
| --- | --- |
| At least 7.2 V | Steady on, `NORMAL` |
| At least 7.0 V and below 7.2 V | Toggle every 500 ms, `LOW` |
| Below 7.0 V | Toggle every 125 ms, `CRITICAL` |
| ADC at/below 50 mV or at/above 3000 mV | Off, `INVALID` |

A transition toward a warning must remain below its threshold for one second.
Recovery is immediate only after 0.1 V hysteresis: LOW returns to NORMAL at
7.3 V, while CRITICAL returns to LOW at 7.1 V (or directly to NORMAL at 7.3 V).
Until the first complete valid average, the LED is off and state is `INVALID`.
These are provisional low-voltage warnings, not a state-of-charge or “10%
remaining” measurement. They cannot detect faults not inferable from voltage
and do not inhibit wheels, leg movement, or firing.

#### Meter comparison and threshold test

1. Secure the robot and make the wheel and firing mechanisms safe. Connect a
   multimeter across battery positive and ground.
2. With a normally charged 2S LiPo connected, run `battery` and compare it with
   the meter. At 8.4 V battery voltage, expect about 2.084 V (2084 mV) on
   GPIO36. Do not apply more than the board permits to the ADC pin.
3. If a consistent ratio remains after checking wiring and resistor values,
   calculate `meter voltage / reported voltage` and use it as the diagnostic's
   calibration multiplier. Its checked-in value intentionally remains `1.0`.
4. **Do not discharge a LiPo to test warnings.** Disconnect it and use a
   current-limited adjustable bench supply on the battery input. Sweep around
   7.2 V and 7.0 V, allowing over one second below each threshold, and verify
   states and LED periods. Raise the supply through 7.1 V and 7.3 V to verify
   hysteresis. Disconnect the supply and verify `INVALID` with the LED off.

Sampling, state timing, and blinking use `millis()` with no added delay. The
optional stream writes once per second only when the USB serial buffer has
room, so it does not intentionally hold up the 1 Mbps bus path.

The diagnostic also exposes the existing C3 `ARM`, `FIRE`, `DISARM`, and `STOP`
commands for deliberate manual testing. It never sends `ARM` or `FIRE` during
startup, help, or any Ping, never automatically arms or repeats a fire request,
and does not infer or retain ARM or firing state. Use `c3status` to ask the C3
for its actual state. Even when a FIRE response times out or is rejected, the
diagnostic does not claim that firing did not occur and does not retry it.

The reference-only 2c3 copy implements these command/state rules: it starts
`DISARMED`; `ARM` changes any state except `FIRING` to `ARMED`; `FIRE 1` through
`FIRE 9` is accepted only while `ARMED` and changes the state to `FIRING` after
the firing output starts; completion returns to `ARMED` on success or `ERROR`
on failure. `DISARM` and `STOP` both stop firing and audio and change the state
to `DISARMED`. Its response strings are `OK ARMED`, `OK FIRE`, `OK DISARMED`,
`OK STOPPED`, `ERR BUSY`, `ERR NOT ARMED`, `ERR FIRE START`, and `ERR COMMAND`;
`STATUS` returns `OK STATUS` followed by `DISARMED`, `ARMED`, `FIRING`, or
`ERROR`. Because the checked-in reference may predate the C3 installed on the
robot, the diagnostic displays the received body but does not guess or require
one of those strings as the success condition for these four commands. It only
reports a valid C3 response after checking packet ID, declared length,
checksum, zero error byte, printable ASCII, and excluding a TX echo.

### Recommended on-machine order

1. Secure the robot, keep the wheels clear, select and upload only
   `bus_diagnostic`, and open the 115200 baud USB monitor.
2. Run `help`. Confirm that neither wheel motor is driven at startup.
3. Run `ping1`, then `ping2`, then `ping3`. Confirm each TX echo is labelled and
   ignored rather than counted as success, and that `ping3` accepts only a
   zero-error ID3 status packet with length 2. Run `pingall` to repeat all three
   Pings once each.
4. Run `c3ping`, `c3help`, and `c3status` individually. Confirm each report says
   `ID=3`, `checksum=OK`, and `error=00`, and inspect the printed ASCII response.
   Expected values include `OK PONG`, a help line beginning `OK HELP`, and an
   `OK STATUS ...` line. A timeout or a response classified as another ID,
   malformed, non-ASCII, or nonzero-error is a failed check.
5. **Never aim the laser at the eyes of a person or animal.** Secure the robot,
   point it in a safe direction, and initially observe the firing LED, laser,
   and audio behavior using this exact manual sequence: `c3ping`, `c3status`,
   `c3arm`, `c3status`, `c3fire 1`, `c3status`, `c3disarm`, `c3status`. Inspect
   every printed response. One `c3fire 1` input must produce exactly one TX.
   Also try `c3fire`, `c3fire 0`, `c3fire 10`, `c3fire -1`, `c3fire +1`,
   `c3fire 1x`, and `c3fire 1 2`; each must be rejected without a `TX:` line.
   A timeout or invalid response does not prove the mechanism did not fire, so
   do not repeat FIRE automatically. If anything abnormal occurs, manually
   issue `c3stop` followed by `c3disarm`.
6. With the barrel mechanism secured and clear, run `c3tilt 1500` once. Confirm
   that exactly one `TILT 1500` packet is transmitted and that success is shown
   only after a valid, non-echo ID3 response containing exactly
   `OK TILT 1500 us`. Also try missing, signed, suffixed, and out-of-range inputs
   such as `c3tilt`, `c3tilt +1500`, `c3tilt 1500x`, `c3tilt 499`, and
   `c3tilt 2401`; each must print an error without a `TX:` line or servo motion.
7. Run `pos1`, then `pos2`, and compare the raw readings with the physical legs.
8. Run `c3mute on` and confirm that exactly one `MUTE ON` packet is transmitted
   and that success is shown only for the exact response `OK MUTE ON`. Then run
   `c3mute off` and perform the equivalent checks for `MUTE OFF` and
   `OK MUTE OFF`. Try `c3mute`, `c3mute yes`, `c3mute on `, and `c3mute offx`;
   each must be rejected without a `TX:` line. MUTE affects FIRE's automatic
   audio only; manually requested SOUND playback remains audible. These are
   on-machine checks, not claims that this repository change was hardware-tested.
9. Only after securing each leg, run one command at a time in this order:
   `rhome`, `rext`, `rhome`, then `lhome`, `lext`, `lhome`.

## Unverified hardware/protocol items

- The external half-duplex transmit-enable circuit and whether it echoes TX.
- Any direction-enable pin (none is used by the existing source).
- All responses and physical motion; this repository change has not been tested
  on the robot.
