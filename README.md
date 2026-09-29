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
| `c3tilt <us>` | Send one ID3 `0xA0` ASCII command `TILT <us>` to set the barrel servo pulse. `<us>` must contain only decimal digits and be in the inclusive range 500–2400; invalid input is not transmitted. Success requires the exact ASCII response `OK TILT <us> us`. |

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

In addition to the non-actuating C3 queries `PING`, `HELP`, and `STATUS`, the
diagnostic exposes only the `TILT` actuator command. It does not provide `ARM`,
`FIRE`, `TEST`, `SOUND`, or `LASER`. The authoritative 2c3 firmware already
implements `TILT`; the older reference-only copy in this repository is not
modified or used to implement it.

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
5. With the barrel mechanism secured and clear, run `c3tilt 1500` once. Confirm
   that exactly one `TILT 1500` packet is transmitted and that success is shown
   only after a valid, non-echo ID3 response containing exactly
   `OK TILT 1500 us`. Also try missing, signed, suffixed, and out-of-range inputs
   such as `c3tilt`, `c3tilt +1500`, `c3tilt 1500x`, `c3tilt 499`, and
   `c3tilt 2401`; each must print an error without a `TX:` line or servo motion.
6. Run `pos1`, then `pos2`, and compare the raw readings with the physical legs.
7. Only after securing each leg, run one command at a time in this order:
   `rhome`, `rext`, `rhome`, then `lhome`, `lext`, `lhome`.

## Unverified hardware/protocol items

- The external half-duplex transmit-enable circuit and whether it echoes TX.
- Any direction-enable pin (none is used by the existing source).
- All responses and physical motion; this repository change has not been tested
  on the robot.
