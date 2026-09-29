# slimy-tools

Personal tools and firmware patches for measuring and fixing IMU yaw drift on
SlimeVR trackers, plus fixes for early v1.2 boards (I2C IMU, battery divider).

I started this because full-body tracking kept falling apart whenever I slept
in VR. Sleeping is about the worst case for IMU trackers: you lie still for
hours, so drift piles up; you roll over, so anything that assumes a known pose
breaks; and Stay Aligned has no idea what lying on your side is.

To fix that I first had to measure drift properly, and that turned into these
tools. The measurements also turned up a few things I couldn't find documented
anywhere, see [Findings](#findings).

Everything talks to a stock SlimeVR server over SolarXR. Only the firmware
patches are optional.

## Disclaimer

- This is a personal project. Don't expect production-quality code or future updates.
- Everything was measured on one set of six LSM6DSV trackers on early v1.2
  boards, in one house. Your numbers will differ, but the methods should carry
  over.
- The firmware patches aren't upstream and the SlimeVR project doesn't endorse
  them. Flash at your own risk. Recovery is over USB.

## What's in here

| | |
| `slimevr_client.py` | SolarXR client. Connects to `ws://localhost:21110`, yields tracker samples, reset events and connection events, and reconnects by itself. Can also send resets. |
| `record.py` | Records a session (samples, battery, events) to `sessions/<timestamp>_<label>/`. Survives a power cut, and you can stop it with a `STOP` file. |
| `analyze.py` | Turns one or more sessions into a drift report, or a comparison table. |
| `live.py` | Live table: drift rate per tracker, Stay Aligned state, temperature, battery. |
| `quatmath.py` | Quaternion helpers and robust statistics. |
| `tools/ota_update.py` | Flashes a firmware image to one tracker over Wi-Fi, the same way the SlimeVR server does. |
| `tools/recalibrate.py` | Wipes and redoes a tracker's gyro calibration over serial, and shows what the old one was getting wrong. |
| `tools/raw_stream_log.py` | Logs the raw IMU stream (needs the firmware patch below). |
| `tools/serial_boot_log.py` | Captures a tracker's boot log, optionally after running some commands. |
| `tools/serial_via_server.py` | Sends serial console commands through the SlimeVR server, for when it's holding the COM port. |
| `firmware/*.patch` | Patches against SlimeVR-Tracker-ESP v0.7.3, see [Firmware patches](#firmware-patches). |

## Requirements

```
pip install websockets flatbuffers pyserial
```

You need a SlimeVR server running locally. The serial tools need SlimeVR
**closed**, since it holds the COM port.

`solarxr_fb.py` is checked in, generated from the SolarXR schema. To
regenerate it (flatc 22.10.26 or later):

```
flatc --python --gen-all --gen-onefile -o . -I SolarXR-Protocol/schema SolarXR-Protocol/schema/all.fbs
```

flatc's regular Python output has broken cross-namespace imports, and
`--gen-onefile` is what makes it usable.

## Measuring drift

```
python record.py --label night --note "hip 03389, chest C3A86"
python analyze.py sessions/20260911-004843_night
```

The main metric is what I call quiet drift. I take the raw fused yaw, look at
2-minute windows where the tracker never moved more than 8 degrees from its
starting orientation, fit the trend in each one, and report the median across
windows.

I ended up with this instead of something simpler for a few reasons:

- It uses the raw rotation, before resets and before Stay Aligned, so the
  server's corrections can't hide the drift I'm trying to measure.
- Yaw is the twist around the world vertical of the *delta* rotation, not
  something read from Euler angles, so it stays valid with the tracker lying
  down or on its side.
- Breathing and small movements oscillate around zero and cancel out over two
  minutes. A bias doesn't.

Look at the sign of each window, not just the median. A tracker with a real
bias puts almost every window on the same side of zero. Mixed signs around a
small median are just noise, even if some individual windows are large. I used
this several times to tell a broken tracker from normal variation.

## Firmware patches

The patches apply on top of
[SlimeVR-Tracker-ESP](https://github.com/SlimeVR/SlimeVR-Tracker-ESP) v0.7.3.
They're cumulative, so apply one or the other, not both.

```
git clone --depth 1 --branch v0.7.3 https://github.com/SlimeVR/SlimeVR-Tracker-ESP
cd SlimeVR-Tracker-ESP
git apply ../firmware/i2cbat-raw.patch
FIRMWARE_VERSION="0.7.3+i2cbat" pio run -e BOARD_SLIMEVR_V1_2
```

Flash with `tools/ota_update.py --tracker "Tracker XXXXX" --bin <image>`.

### i2cbat.patch

The release image expects the IMU on SPI and a 5:1 battery divider. On earlier
v1.2 boards the LSM6DSV is on I2C at address `0x6B` and the divider is 11:1.
With the official image, the IMU shows up as a second "extension" tracker and
the battery reads about half its real voltage (4.14 V becomes 1.89 V, so no
battery percentage at all). This patch sets the board defaults to match. It
doesn't disable charging or anything else: the charge controller isn't wired
to the MCU, so firmware can't limit charging.

### i2cbat-raw.patch

Everything above, plus the raw IMU stream, over serial and Wi-Fi.

`STREAM ON [ms]` / `STREAM OFF` on the serial console prints averaged raw
gyro, accel and temperature. Stock firmware never sends a raw gyro packet,
which is why SolarXR's `raw_angular_velocity` field is always empty.

Over the network, the tracker listens for `RAW ON [ms]` / `RAW OFF` /
`RAW PING` on UDP port 6971 and streams back to whoever asked, so there's no
address to configure and I can measure a tracker while it's being worn.

## Findings

All of this is from original SlimeVR v1.2 boards with LSM6DSV IMUs. One LSB is
0.035 deg/s at 1000 dps, so 1 LSB of gyro bias is 2.1 deg/min of yaw drift.

### Gyro bias isn't linear with temperature

I let a tracker cool from 40 C to 34 C on a table, undisturbed. The X axis bias
went -15.05 at 34 C, -15.33 at 36.5 C, and back to -15.01 at 40 C. SlimeVR's
runtime calibration stores two bias points and draws a straight line between
them. Against this curve, that line is off by up to 0.62 deg/min in the
middle, and the further apart the two points are, the worse the middle gets.

### Factory calibration tends to pick the worst pair of points

All six trackers had exactly two points: one near 20 C (a cold first boot on a
table) and one between 35 and 42 C. Trackers worn on the torso run at 39-42 C,
which lands right in the middle of that interval, where the line is least
accurate.

### Bias also depends on thermal history

The same tracker at the same temperature read -14.12 sitting at equilibrium and
-15.05 while cooling down from 40 C. That's about 1.9 deg/min apart, so no
model that only looks at the current temperature can capture it.

### Calibrate in the conditions you'll actually use it

Wiping the calibration (`DELCAL`) and letting it re-measure while the tracker
is at its working temperature fixed a tracker that had drifted about 1 deg/min
in every session, with every window on the same side of zero. After
recalibrating at operating temperature, the same tracker measured -0.25 deg/min
with mixed window signs, in a like-for-like session.

Each position has its own working temperature. Here, shins run at 33-34 C,
thighs 37-38 C, chest 39 C, and the hip 40-42 C because it sits against the
abdomen. Moving a tracker to another position invalidates its calibration.

### Mounting calibration in the ski pose with legs apart rotates the result

The mounting calibration works out how a tracker is strapped on from the
direction the limb moves between poses, and it assumes that movement happens in
the body's sagittal plane. If you do it with your legs splayed, the stored
mounting ends up rotated by roughly the splay angle.

You can't see the error while standing, because a twist around a vertical limb
doesn't change where the limb points. It only shows once the limb leaves
vertical: sitting or lying down, one leg reads lower than the other. Redoing
the calibration with feet parallel and close together took one shin from 23.2
degrees of left-right asymmetry down to 3.2.

To check for this: record 30 s standing, then 30 s lying on your back with legs
straight and ankles touching, and compare the angle between the two shins' long
axes (tracker local +Y from `rotation_reference_adjusted`, expressed in the hip
frame). In that pose the shins are physically parallel, so anything above a few
degrees is a mounting error, not your pose. I get about 2 degrees standing and
3 lying.

### `hipsWidth` doesn't scale with height

Every other bone is derived from the user's height, but the distance between
the hip joints is a flat 0.26 m default for everyone. For a small person that's
far too wide, and the symptom is that the knees never touch in the skeleton
even when they do in real life. There's also no per-side bone length: both legs
share one `upperLegLength` and one `lowerLegLength`, so real left-right
asymmetry can't be represented.

### What didn't hold up

I compared the stored calibration to the measured bias per axis, hoping it
would predict drift. It didn't. The worst tracker by that measure (3.35 deg/min
of implied error) drifted 0.15 deg/min in practice. Only the axis that happens
to be vertical matters for yaw, and the fusion re-estimates part of the bias at
rest anyway.

## License

MIT, see [LICENSE](LICENSE). Third-party code and generated files are listed in
[NOTICE.md](NOTICE.md).
