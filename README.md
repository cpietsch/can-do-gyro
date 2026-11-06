# A One-Axis Gimbal with the XIAO STM32C5 and a CAN Motor

The Seeed Studio [XIAO STM32C5](https://wiki.seeedstudio.com/xiao_stm32c5_getting_started/)'s on-board IMU tracks the
top's orientation in 3D, and over CAN it drives a BLDC motor that turns the base against every disturbance. However you turn
the bottom, the top keeps facing the same direction in the room. It works standing upright, tilted, sideways and
upside down.

<p align="center">
  <a href="https://youtu.be/w_WODAGTqgk"><img src="https://img.youtube.com/vi/w_WODAGTqgk/maxresdefault.jpg" alt="Demo video on YouTube: CAN do GYRO" width="640"></a>
  <br><a href="https://youtu.be/w_WODAGTqgk">▶ Watch the demo video on YouTube</a>
</p>

An M5Stack [Unit RollerCAN](https://docs.m5stack.com/en/unit/Unit_RollerCAN) stands on its motor: the round rotor is
the bottom, on the table. On its body sit a small 2S LiHV battery and the XIAO. The XIAO is on the part that has to stay
still, so it measures its own correction: a closed loop on the gyro, running at about 1 kHz.

## How it came about

I was looking for something to build with the XIAO STM32C5 and noticed two things on the board: an IMU
(accelerometer and gyro), and CAN with the transceiver on board, its killer feature. What can you do with knowing your
orientation and being able to talk to a motor? I had always wanted to try a BLDC motor, so: a gimbal in one axis. The
XIAO rides on the motor, and however you turn it, the top keeps facing the same direction.

The motor was an easy pick: the RollerCAN is a complete BLDC unit (motor, FOC driver, encoder) that takes its commands
over CAN. I soldered DuPont pigtails to the XIAO's CAN pads and to 5V and GND and glued them down, so everything plugs
in and out. The RollerCAN ran from a bench supply at first, then from a LiPo plugged straight into it, and the unit
became standalone. After some tuning it held the way I had in mind, and with orientation tracking in 3D it no longer
matters how the motor and the XIAO are oriented.

It shows how the XIAO STM32C5 can drive a CAN motor with everything, from the sensor fusion to the control loop,
computed on the chip.

## Why the XIAO STM32C5

Everything this needs is on the thumb-sized board, so the whole controller is the XIAO plus four wires:

- **CAN with an on-board transceiver**: the STM32C5A3CG has two FDCAN controllers; one is wired to a TI TCAN3403 with
  CANH/CANL pads and a 120 Ω termination (on as delivered). Two wires go to the motor. The RollerCAN only
  speaks classic CAN 2.0, so this uses 1 Mbit/s classic frames; the controller can do CAN FD.
- **On-board 6-axis IMU** (LSM6DS3TR-C, I2C): gyro at 416 Hz through its FIFO, every sample used.
- **Cortex-M33 at 144 MHz with FPU**: a quaternion update per gyro sample, the control law and CAN traffic every loop
  leave most of the time asleep (the loop takes ~0.5 ms of work).
- **1 MB dual-bank flash**: the firmware runs from bank 1 and records ~90 s of telemetry into bank 2 while it runs,
  without stalling, so you can see afterwards what it did standalone on the battery.
- **Native USB**: serial console for commands and status, and drag-and-drop UF2 flashing.

## Hardware

- Seeed Studio XIAO STM32C5
- M5Stack Unit RollerCAN (STM32G431 + DRV8311 FOC driver, D3504 200 KV gimbal motor, magnetic encoder, 6–16 V on
  XT30)
- A 2S LiHV battery (580 mAh used here, 7.6 V nominal) with an XT30 plug
- Female DuPont pigtails, soldered to the XIAO's CANH/CANL pads and 5V/GND and glued down on top with superglue, so
  CAN and power plug in and out
- Something to hold battery and XIAO on the RollerCAN's body (tape works)

<p align="center">
  <img src="docs/photo.jpg" alt="The unit: RollerCAN at the bottom, battery and XIAO STM32C5 taped on top" width="420">
</p>

Wiring:

<p align="center">
  <img src="docs/wiring.svg" alt="Wiring: LiPo + and − to the RollerCAN's XT30 power pins, the XIAO's CAN H and L pads to its CAN pins, the RollerCAN's Grove 5V and GND to the XIAO's VBUS and GND" width="780">
</p>

The RollerCAN's XT30 (2+2) connector carries both power and CAN:

- Battery on its power pins.
- XIAO **CANH / CANL** pads to its CAN_H / CAN_L pins (the ground is shared through the Grove cable).
- XIAO **5V** and GND from the RollerCAN's Grove port, so it runs standalone without USB.

The XIAO can sit at any angle: calibration takes the motor axis from gravity. Mounted here: the battery lies flat on
the RollerCAN's body, the XIAO is taped flat on the battery.

RollerCAN settings (in its own screen menu): communication **CAN**, **1 Mbit/s**, ID **0xA8** (168, the default),
stall protection on. Leave the PID preset at the default: the stiffness the firmware sends only applies there. The
firmware doesn't save anything on the RollerCAN.

## Build and flash

The STM32C5 has no Arduino core yet, so this is a [Zephyr](https://zephyrproject.org/) application, built with
[PlatformIO](https://platformio.org/) and Seeed's platform (pinned to a tested commit in `platformio.ini`, installed by
the first build):

```
pio run -e xiao -t upload
```

That resets the XIAO into its UF2 bootloader (1200-baud touch) and copies the firmware to its USB drive `XIAOC5BOOT`,
which has to be mounted (desktops do that by themselves). If the firmware hangs and the reset doesn't work,
double-tap the XIAO's reset button and upload again.

`platformio.ini` also pins a newer ARM GCC (12.3) than the platform asks for: the requested 8.2.1 has no Linux ARM64
build. Before every build `patch_zephyr.py` fixes a crash in Zephyr 4.4's CAN driver (see Notes below); it edits the
framework package in `~/.platformio` once and keeps the original as `can_mcan.c.orig`.

## Using it

1. Put it upright on the table and plug in the battery. It wants to stand still for 2 s: it measures the gyro's offset
   and the motor axis. If it moves, it says so and tries again. The heater warms the IMU meanwhile.
2. From then on it holds: the top keeps facing the direction it faced when holding started. Turn the bottom, pick it
   up, tilt it: the motor turns the top back.
3. Held by the top in the air, the bottom hangs free and the motor would spin it up: once the rotor turns more than
   180° within 0.2 s the motor switches off. Put it down: after 2 s standing still it holds the direction it faces
   then.

The green LED blinks once a second while all is well, and fast while the RollerCAN or the IMU is missing.

Serial commands (USB, 115200, e.g. `pio device monitor`), one per line. Anything else on a line is ignored without an
answer, so a terminal that echoes the XIAO's own output back (Linux does until the port is raw) can't start the motor.

| Command | What it does |
| --- | --- |
| `h` | hold: the top keeps the direction it faces now |
| `s` | motor off (turns freely) |
| `c` | calibrate the gyro again (upright and still for 2 s), then hold |
| `p <mA/°>` | stiffness, 5 to 100, until reset |
| `d <ms>` | damping, 0 to 500, until reset |
| `f` | flip the motor direction until reset |
| `t` | IMU heater off / on until reset (to compare) |
| `i` | status: deviation, rotor, current, voltage, temperature, gyro offset and noise, standing-still windows, tilt, IMU temperature and heater, loop timing, last reset (crash record), CAN errors, battery, recording |
| `dump` | print the recording (motor off meanwhile), see below |
| `erase` | clear the recording; it records from the next hold |
| `?` | list the commands |

## How it works

- **Orientation**: a quaternion integrated from all three gyro axes, with its tilt pulled towards gravity whenever the
  accelerometer reads about 1 g (Mahony-style, ~2 s time constant). The accelerometer can't see the heading, so that
  comes from the gyro alone.
- **Facing**: a vector across the motor axis is the top's front. The motor can only turn the top about the motor axis,
  so the target is the original direction projected across the motor axis, and the deviation is the signed angle
  between front and target about it. Counting only the gyro's turn about the motor axis would miss what tilting,
  turning and tilting back does (in a simulation, 20 tilt circles of 20° turned the top −12.7°, the single axis
  counted −141°). Within 15° of the motor axis pointing at the original direction, turning can't help; afterwards it
  holds the direction it faces then.
- **Control**: every loop, the rotor's target = where the rotor is now − (deviation + turning rate × damping +
  integral). The RollerCAN's own position controller pushes with the stiffness (mA per degree) towards it, up to
  500 mA. The XIAO sends the target and the next position request without waiting for the answers, which it picks up
  at the next loop.
- **Integral**: a deviation that stays makes the push grow until the motor breaks free of its friction (~50 mA) and
  turns the top back; capped at 300 mA.
- **Standing still**: every 1/4 s with the gyro within 2.5 °/s of its offset counts as standing still. That time
  isn't integrated and the offset follows the gyro, so the direction doesn't drift at rest.
- **IMU heater**: the gyro's offset wanders with temperature, so the IMU is kept at 40 °C by the heater resistors
  around it: PI regulation on the IMU's own temperature sensor every 0.5 s, PWM on PA8, at most 60% duty (gains and
  limits from Seeed's `zephyr-imu-temp-pid-6axis` example, but the integral may use the whole 60%: with Seeed's cap
  it got stuck at ~36 °C in open air). The gyro is calibrated right after power-on, while the IMU warms up: on the
  way to 40 °C the offset shifts ~1 °/s, slowly enough for the standing-still correction to follow it (waiting for
  40 °C first took up to 2 minutes). Above 50 °C or without a temperature reading the heater is off.
- **Crash record**: a CPU fault reboots the board, and a watchdog reboots it when the loop stops for 2 s. Before that,
  the fault handler (or the watchdog's early warning interrupt) notes the address, the loop stage and the uptime in
  RAM that survives the reset; `i` shows it as "last reset". Look the address up with
  `arm-none-eabi-addr2line -f -e .pio/build/xiao/firmware.elf <pc> <lr>`.
- **Never gives up**: no turn limit (the RollerCAN counts ~59,000 turns), the top more than 90° off (held by hand)
  makes its current direction the new one, and after the RollerCAN's stall protection (blocked ~3 s) it holds again
  after 3 s. Only an empty battery and a free-spinning bottom switch the motor off.
- **Free spin**: held by the top, the bottom has nothing to push against, and the motor spun it up to thousands of
  °/s. The rotor turning more than 180° within 0.2 s switches the motor off (normal use: at most 132°; free spins and
  runaways 670–1190°); after 2 s standing still (tilt under 10°) it holds again, facing where it faces then. A rotor
  speed above 1000 °/s for 0.2 s was tried first: the speed dipped below now and then and it cut off only after 1 s.

## Settings

Defaults in `src/main.c`, tuned by hand and from recordings, standalone on the battery:

- **Stiffness 15 mA/°** (`POSITION_KP`): at 10 the top lagged 4–7° while turning the bottom by hand. At 20 and 25 it
  holds tighter (3.6 and 2.1° RMS instead of 4.1°), but 25 buzzes at 50 Hz and 20 once ran off at rest.
- **Damping 100 ms** (`DAMPING_MS`): 50 overshot after fast turns.
- **Integral 4 s** (`INTEGRAL_S`): 1.5 s overshot by more than 90°.
- **Gyro 416 Hz**: at 104 Hz its readings come ~15 ms late and the unit wobbled at ~17 Hz. 416 Hz is noisier
  (~3.7 °/s RMS) but held best.
- **Loop ~1 ms** (`LOOP_US` 500 µs sleep; IMU 0.3 ms, CAN 0.1 ms).
- **IMU at 40 °C** (`HEATER_TARGET_C`), as in Seeed's example: above room temperature, so the heater can always hold
  it.
- **Max current 500 mA** (`MAX_CURRENT_MA`): the motor's continuous rating without cooling, so it can even stay
  blocked. The RollerCAN's factory limit is 1000 mA.

## Recording

To see what it does standalone, without USB. From the first hold on (not during calibration), every loop
writes the time, the top's turning rate and deviation, the rotor position, the motor current, the tilt and flags to the
XIAO's flash (every 3rd loop, ~3 ms apart, flags of the loops in between merged in; the unused OTA slot in bank 2,
480 KB: ~90 s, then it stops). A recording survives resets, power off
and reflashing; it only records when the area is empty. After `erase` it records from the next hold: right away if the
battery is on (`erase` switches the motor off, it holds again by itself).

1. `erase` over USB with the battery unplugged, unplug USB.
2. Power it up on the battery and let it run.
3. Unplug the battery (the XIAO powers off, the recording stays), plug in USB.
4. `python log.py rec.csv` (needs pyserial, PlatformIO's Zephyr Python has it:
   `~/.platformio/penv/.zephyr-4.4.0/bin/python`).

CSV columns: `t_s, rate_dps, turned_deg, rotor_deg, current_ma, tilt_deg, holding, new_gyro, still, sent`.

## Battery

The RollerCAN measures the battery at its input (no diode in between, good to about ±2%: compare `i` with a multimeter
once). The firmware averages it over ~10 s, so a short dip under load doesn't count (`BATTERY_*`, `CELL_*`):

- 3.82 V per cell (about 30% left) or less: "battery low" on serial every 30 s, the RollerCAN's LEDs orange. For a
  normal LiPo (4.20 V full) use 3.77 V and adjust the table in `batteryPercent()`.
- 3.50 V per cell or less: motor off, LEDs blink red. Unplug the battery then: the RollerCAN keeps drawing current, and
  a LiPo is damaged below 3.0 V per cell.
- Once the battery is above 30% again (charged or swapped), it holds again by itself.

With the battery unplugged, USB feeds the RollerCAN back through the Grove 5V (it reads ~4.8 V: "battery empty", motor
off).

## Limits and safety

- The RollerCAN enforces the 500 mA limit. Supply max 16 V.
- If the XIAO stops sending (unplugged, reset), the RollerCAN keeps its last target: its protocol has no timeout.
- The motor can turn the bottom fast when the top is held: keep cables clear.

## Notes on the XIAO STM32C5

Things learned along the way, in case they save you time:

- **CAN**: Zephyr's FDCAN driver with the on-board transceiver (its standby pin is handled by `can-transceiver-gpio`).
  Set `CONFIG_CAN_INIT_PRIORITY=80` as in Seeed's examples, so CAN starts after the GPIOs. The M_CAN controller has
  no one-shot mode: frames nobody acknowledges (motor unpowered) stay queued and block the TX buffers, so
  `can_stop()` / `can_start()` before looking for the motor again.
- **CAN driver crash**: Zephyr 4.4's M_CAN driver calls a TX buffer's callback from the TX event without checking it.
  After a bus-off, the bus-off handler has already completed that buffer, and the late TX event jumps to address 0:
  a fault in the CAN interrupt, here every few minutes while holding (LED stopped, USB silent, the motor kept its last
  target). Upstream Zephyr added the check after 4.4; `patch_zephyr.py` applies it.
- **USB serial**: Zephyr's CDC ACM (`device_next`). Read it with the interrupt API: with `uart_poll_in()` only, no
  receive is ever queued and writes from the host hang.
- **Stack**: the default 2 KB main stack overflowed silently once printk got many floats (the board hung, no USB, the
  1200-baud reset dead: double-tap reset). 8 KB here.
- **IMU FIFO**: read it in one burst: with address auto-increment the LSM6DS3TR-C rolls from `FIFO_DATA_OUT_H` back
  to `_L`. Reading only the newest sample each loop skips samples at the phase of the loop's own jitter, and that adds
  up to drift. Count each sample for the gyro's measured sample period, not the loop's time.
- **IMU block data update**: with BDU on, reading the accelerometer's registers alone left the FIFO's gyro samples
  ~6 °/s off on one axis. Off, the accelerometer can be read alone and less often.
- **Gyro offset**: it changes by up to ~3 °/s depending on whether the motor's supply is on, and wanders with
  temperature, so calibrate with everything powered as it will run, and let the standing-still correction follow the offset while the IMU warms up.
- **IMU heater**: resistors around the IMU, switched by PWM on PA8 (TIM1_CH1, board alias `imu-heater`, 1 kHz). The
  IMU's die temperature is in its registers 0x20/0x21 (256 LSB per °C, 0 = 25 °C). Seeed's example
  `zephyr-imu-temp-pid-6axis` shows the regulation.

## Known issues

- **Slow drift standalone**: on the battery, without USB, the gyro reads much noisier at rest (±20 °/s per sample
  instead of ±2 on USB), so the standing-still check rarely passes, the offset isn't corrected, and the top slowly
  turns. Suspected: the XIAO's supply from the RollerCAN's Grove 5V. On USB plus battery it holds within 0.4° per
  minute.
- **Loops faster than 1 ms** (shorter `LOOP_US`) made the tilt estimate go wrong and the motor run off. Not
  understood yet.

## What else it could be

Anything that has to keep pointing one way on something that turns:

- **Hubcaps that stand still** while the car drives. On a wheel the motor axis lies flat, where the accelerometer sees
  the angle too, so it wouldn't drift. The motor would have to keep up with the wheel, though: about 14 turns per
  second at 100 km/h.
- **A Predator costume**: a shoulder cannon that stays aimed while you turn and walk, with a laser pointer on it.

## License

MIT, see [LICENSE](LICENSE).
