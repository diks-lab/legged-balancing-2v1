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
| `ping3` | Currently transmits nothing and reports that the 2c3 protocol is unavailable. |
| `pingall` | Run ID1 and ID2 Ping once each, then report ID3 unavailable. |
| `pos1`, `pos2` | Read the current raw position at STS address 56 from ID1 or ID2. |
| `rhome`, `rext` | Move only right ID1 to HOME 2061 or small extension 2044. |
| `lhome`, `lext` | Move only left ID2 to HOME 2026 or small extension 2044. |
| `c3help`, `c3status`, `c3laser`, `c3tilt`, `c3sound` | Disabled; no packet is sent until the current 2c3 implementation is source-verified. |

Leg moves use the values already present in the balancing firmware: speed 150
and acceleration 15. Startup, help, position reads, and Ping never issue a move.
Every move prints its target ID and position and is sent once, with no retry.

### Recommended on-machine order

1. Secure the robot, keep the wheels clear, select and upload only
   `bus_diagnostic`, and open the 115200 baud USB monitor.
2. Run `help`. Confirm that neither wheel motor is driven at startup.
3. Run `ping1`, then `ping2`, then `pingall`. Confirm TX echo is labelled and
   ignored rather than counted as success.
4. Run `pos1`, then `pos2`, and compare the raw readings with the physical legs.
5. Only after securing each leg, run one command at a time in this order:
   `rhome`, `rext`, `rhome`, then `lhome`, `lext`, `lhome`.
6. Do not test ID3 commands until `legged-balancing-2c3` source supplies the
   exact frame, checksum, response, and implemented command list. Current ID3
   menu entries deliberately transmit nothing.

## Unverified hardware/protocol items

- The external half-duplex transmit-enable circuit and whether it echoes TX.
- Any direction-enable pin (none is used by the existing source).
- ID3 packet framing, checksum, response framing, and supported command syntax.
- All responses and physical motion; this repository change has not been tested
  on the robot.
