# jetnano_python

Joystick control for the Jetson Nano robot over Wi-Fi, with automatic failover
to HC-12 radios and fail-safes that stop the robot when control is lost.

This is a rewrite of the original `drivenano.py`, `pijoy.py` and
`sync_server.py` scripts from this repository. Same intent and same hardware,
new structure: one small package (`jetnano_control`), a program for each end,
tests that run without a robot. The original scripts are still in the
history; browse them at the last commit before the rewrite:
https://github.com/stevej52/jetnano_python/tree/7c850efb5fde13f9d79c798a0ce4b0c410fb91b1

```
  operator (Pi or PC)                          robot (Jetson Nano)
  ┌──────────────────┐   UDP over Wi-Fi   ┌──────────────────────────┐
  │ jetnano-tx       │ ─────────────────▶ │ jetnano-rx               │
  │ joystick → frame │   HC-12 radio      │ pick freshest link       │
  │ send on ALL links│ ─────────────────▶ │ → PCA9685 servos / ESC   │
  │                  │ ◀ ─ status (UDP) ─ │ watchdog → neutral       │
  └──────────────────┘                    └──────────────────────────┘
```

## How it works

* **One frame format everywhere.** A frame is one joystick sample: sequence
  number, four axes, a button mask, flags (armed, e-stop, neutral, turbo) and a
  CRC-16. The same nine 16-bit words go out as a UDP datagram, as a text line
  on the radio, or as Modbus registers. A frame damaged in transit fails its
  CRC and is dropped, never applied.
* **The transmitter sends on every link at once.** Each link has its own
  sender thread running at its own rate (Wi-Fi every frame, radio 15 Hz), so a
  slow radio or a hung TCP write never delays the others. The transmitter does
  no link selection at all.
* **The receiver picks the freshest frame from the best link.** Every link
  delivers into an arbiter that keeps the newest frame per link. Each control
  tick uses the highest-priority link whose last frame is younger than
  `fresh_for_s` (0.5 s by default). When Wi-Fi stops, the radio's frames were
  already arriving, so failover is one tick with no probing; when Wi-Fi returns
  it takes over the same way.
* **When nothing is fresh, the robot goes to neutral.** That decision is made
  on the same tick as the servo command, not in a separate watchdog thread, so
  a stale command can never race past it.
* **Status flows back.** The receiver sends a one-line status back over UDP
  every second, and the transmitter prints it, so the operator can see which
  link is really in control and whether the robot is in fail-safe.

## Fail-safes and fallbacks

| Situation | What happens |
|---|---|
| No fresh frame on any link for `fresh_for_s` | Neutral, logged once, resumes when frames return |
| Arm button not held | Neutral (dead-man switch); `arm_button: -1` disables this |
| E-stop button pressed | Neutral until released |
| Transmitter exits normally (Ctrl-C) | E-stop frames on every link first, then the watchdog covers the rest |
| Joystick unplugged on the operator side | Neutral frames sent at once; controller re-attaches automatically |
| Corrupted or duplicated frame | Dropped by CRC and sequence check |
| Transmitter restarted (sequence reset) | Accepted as soon as the link had gone quiet |
| Servo bus (I2C) errors | Counted, output re-initialised after three in a row, loop keeps running |
| Serial device or socket error | Worker thread reopens it with backoff; other links unaffected |
| Throttle | Limited to `throttle_limit` (turbo button lifts it); increases ramp at `throttle_slew_per_s`, decreases are immediate |
| Obstacle ahead (optional RealSense guard) | Forward throttle held at zero; reverse and steering still work; a stale camera reading never blocks |
| Config typo | Rejected at start-up with the list of valid settings |

Health is logged every few seconds as JSON lines to a rotating file (link
counters, active link, fail-safe state, CPU temperature, BNO055 readings when
the IMU is present).

## Hardware

* **PCA9685** at I2C address 0x40, 100 Hz PWM: channel 0 throttle ESC,
  channel 1 front steering, channel 2 rear steering (turns the opposite way
  for four-wheel steering). Change channels and angles under `drive` in the
  config.
* **HC-12 radios** at 9600 baud (their factory default). Pi side on
  `/dev/serial0` (enable the UART and disable the serial console with
  `raspi-config`). Nano side on `/dev/ttyTHS1` (J41 header, pin 8 TX, pin 10
  RX). Both modules must be set to the same channel and baud rate.
* **Joystick** on the operator machine, read by pygame headlessly, so it works
  over SSH.
* Optional: BNO055 IMU (logged), RealSense D435 (obstacle guard).

## Install

Python 3.8 or newer. The stock Jetson Nano image (Ubuntu 18.04) ships 3.6;
install 3.8 from the deadsnakes PPA or use a newer JetPack.

```bash
git clone https://github.com/stevej52/jetnano_python
cd jetnano_python
# operator machine
pip3 install -e ".[pi]"
# robot (needs Adafruit Blinka for I2C: see Adafruit's Jetson Nano Blinka guide)
pip3 install -e ".[robot]"
# optional extras: .[modbus] .[realsense] .[test]
```

## Run

Bench test on one machine, no hardware:

```bash
jetnano-rx --dry-run -c config.bench.json        # terminal 1: mock servos, radio link off
jetnano-tx --fake-joystick -c config.bench.json  # terminal 2: sends a steering sweep
```

On the robot and the operator machine, with a shared config file:

```bash
jetnano-rx -c config.json          # on the Nano
jetnano-tx -c config.json          # on the Pi or PC
jetnano-joy                         # prints live axes and buttons, to fill in the config
```

Both programs print a status line every couple of seconds. On the transmitter
it looks like:

```
joystick=ok seq=55408 wifi=tx99/err0 radio=tx30/err0 | robot (0.3s ago): active=wifi failsafe=0 armed=1 thr=+0.00 steer=+0.45 wifi=rx85/fresh/0.01s radio=rx26/fresh/0.05s
```

Pull the Wi-Fi and `active=wifi` becomes `active=radio`; kill the transmitter
and it becomes `active=none failsafe=1`.

## Configuration

`config.example.json` lists every setting with its default. Your own file only
needs the keys you change; everything else keeps its default. Links are merged
by name, so this is a complete file for a robot at a new address:

```json
{
  "links": [
    {"name": "wifi", "robot_host": "192.168.0.201"},
    {"name": "radio", "tx_port": "/dev/serial0", "rx_port": "/dev/ttyTHS1"}
  ],
  "joystick": {"axes": [0, 1, 2, 3], "arm_button": 0, "estop_button": 1, "turbo_button": 2},
  "drive": {
    "throttle": {"channel": 0, "center": 90, "minimum": 65, "maximum": 125},
    "steer_front": {"channel": 1, "center": 85, "minimum": 30, "maximum": 135},
    "steer_rear": {"channel": 2, "center": 85, "minimum": 30, "maximum": 135, "reverse": true},
    "throttle_limit": 0.6
  }
}
```

The settings you are most likely to touch:

| Setting | Meaning |
|---|---|
| `links[].robot_host`, `port` | Where the transmitter sends UDP (and Modbus); the receiver listens on `port` |
| `links[].tx_port`, `rx_port` | Serial device on each end for the radio |
| `links[].priority` | Lower wins on the receiver (wifi 1, modbus 2, radio 3) |
| `links[].enabled` | Switch a link off without deleting it (Modbus is off by default) |
| `receiver.fresh_for_s` | How long a link is trusted after its last frame; also the fail-safe delay |
| `joystick.axes` | Which pygame axes become frame axes 0 to 3 (`drive.steer_axis` and `throttle_axis` pick from those) |
| `joystick.deadzone`, `calibrate_rest` | Dead zone, and whether the stick position at start-up counts as zero |
| `drive.*` | Servo channels, centre and end angles, limits, ramp, PWM frequency |
| `telemetry.log_path` | Rotating JSON health log (empty string disables) |
| `obstacle.enabled`, `stop_distance_m` | RealSense guard |

The Modbus link keeps the original idea alive for anyone who wants to poke
registers with a Modbus tool: the receiver runs the server itself (port 5020,
frame at holding registers 128 to 136) and the transmitter writes to it. There
is no separate `sync_server.py` any more.

## Frame format

| Word | Content |
|---|---|
| 0 | magic `0x4A43` |
| 1 | sequence number, wraps at 65535 |
| 2 to 5 | axes 0 to 3 as signed 16-bit, -10000 to 10000 for -1.0 to 1.0 |
| 6 | button mask, bit n = button n |
| 7 | flags: 1 armed, 2 e-stop, 4 neutral, 8 turbo |
| 8 | CRC-16/CCITT-FALSE over words 0 to 7 |

UDP carries the nine words big-endian (18 bytes). The radio carries `$`, the
same 18 bytes as 36 upper-case hex digits, and a newline, so a frame is
readable in a serial terminal and the receiver resynchronises on every line.
Lines on the radio port that do not start with `$` are passed to an
`on_aux_line` hook, so a Teensy reporting battery voltage on the same port can
still be read.

## Tests

```bash
pip3 install -e ".[test]"
pytest
```

The suite covers the frame encoding, the arbiter, servo mapping and ramping,
the config loader, each link over real sockets and a loopback serial port,
the joystick reader against a fake pygame, the obstacle guard, and an
end-to-end run of both programs in threads where the Wi-Fi link is cut and
restored and the transmitter is stopped.

## What changed from the old scripts

* The serial parser no longer reads the queue twice per pass, which dropped
  every other radio line and could block.
* Radio and Wi-Fi now carry identical frames. The old radio path fed -1 to 1
  floats into scaling maths tuned for the byte-swapped Modbus registers, so
  steering over the radio never mapped right.
* The old Modbus decoding only worked because pymodbus 2 swapped register
  bytes; the new frames are explicit about byte order and carry a CRC.
* There is a watchdog: losing every link puts the robot in neutral instead of
  leaving the last servo command in place.
* The RealSense obstacle scan checked only one column per row on the right
  side; the guard now samples the whole band, and it actually acts on what it
  sees.
* pygame runs without a display, so the joystick reader works over SSH, which
  was the reason for the Modbus detour in the first place.
* Everything hardware-specific sits behind an interface with a mock, so the
  logic is tested on a laptop.
