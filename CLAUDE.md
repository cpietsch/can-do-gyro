# can-do-gyro

Firmware for a one-axis gimbal: a Seeed XIAO STM32C5 tracks the top's 3D orientation with its on-board IMU and
drives an M5Stack Unit RollerCAN over CAN, which turns the bottom so the top keeps facing one direction. One PlatformIO
project (env `xiao`, Zephyr 4.4, Seeed's platform pinned in platformio.ini), all firmware in `src/main.c`.
README.md has the wiring, behavior, commands, settings and lessons learned: keep it updated with every change.

## Hardware facts

- XIAO STM32C5: STM32C5A3CG, USB 2886:00c5 (no USB serial number), CDC ACM serial at 115200. Upload: 1200-baud touch
  -> UF2 drive XIAOC5BOOT (must be mounted). Firmware hung: double-tap reset.
- CAN: FDCAN2 with the on-board TCAN3403 (standby pin via can-transceiver-gpio), 120 Ω on. Classic CAN 2.0,
  1 Mbit/s, extended IDs: the RollerCAN's firmware only accepts and sends classic 8-byte frames.
- IMU: LSM6DS3TR-C on I2C2 (0x6A, alias imu0, 400 kHz in app.overlay), raw registers, no Zephyr sensor driver.
  Gyro noise grows with the data rate: ~0.8 dps RMS at 104 Hz, ~3.7 at 416, ~8 at 833. The gyro offset differs by
  ~3 dps with the RollerCAN powered or not and wanders ~1 dps with temperature. IMU heater: PWM on PA8 (TIM1_CH1,
  alias imu-heater), held at 40 °C by PI on the IMU's temperature (0x20), gains and limits (60% max) from Seeed's
  zephyr-imu-temp-pid-6axis example, but the integral cap raised to 300 (Seeed's 180 got stuck at ~36 °C in open air);
  the gyro is calibrated after warm-up. The reading swings ±1 °C at 40, so "±0.5 for 10 s" rarely holds and it
  calibrates via the 2 min fallback. `t` toggles it for comparisons.
- Zephyr 4.4's CAN driver (can_mcan.c) jumped to address 0 on a TX event for an already completed buffer (after a
  bus-off): fault in the CAN ISR every 2-4 min while holding (LED stopped, USB silent, motor kept its target).
  patch_zephyr.py (PlatformIO pre-script) applies upstream's NULL check to framework-zephyr-nrf54lm20 (the board's
  build.zephyr.package, not framework-zephyr), original kept as can_mcan.c.orig.
- Crash record: fault handler / IWDG early warning (2 s) note pc, lr, loop stage, uptime in __noinit RAM and reboot;
  `i` shows "last reset". The UF2 bootloader clears the reset flags (cause reads 0). `i` also counts CAN error states.
- RollerCAN: STM32G431 + DRV8311, D3504 200 KV, 0.5 A continuous / 1 A short, 6–16 V. Protocol checked against its
  firmware (github.com/m5stack/M5Unit-RollerCAN-Internal-FW, can_protocol.c), see the comments in main.c. Position
  mode: degrees x100 multi-turn; its position PID turns the error straight into current (Kp 0x7023, only with the
  default PID preset), capped by 0x7017. No command timeout. Stall protection trips after ~3 s. RGB colors are queued
  on the RollerCAN and black is ignored: write only on change.
- Battery: 2S LiHV on the RollerCAN's XT30. With it unplugged, USB backfeeds the RollerCAN through the Grove 5V
  (~4.8 V: "battery empty", motor off).

## Motor safety

Max current 500 mA (the motor's continuous rating). Don't raise current, stiffness or speed limits without asking.
Before suggesting to flash motor code, state the limits it uses and what the motor will do. DIRECTION -1: the other
sign is positive feedback and spins it.

## Tuning results (why the defaults are what they are)

- Gyro 416 Hz: 104 Hz came ~15 ms late (17 Hz limit cycle standalone); 416 held best despite the noise.
- Stiffness 15 mA/°: 10 lagged; 20 shook at slower loops and once ran off at rest at the 1 ms loop; 25 buzzes at
  50 Hz. Damping 100 ms (50 overshot). Integral 4 s, capped 300 mA (1.5 s overshot >90°). A leak (lasting deviation
  becomes the new direction) was tried and dropped: it turned lag into a lasting shift.
- Every gyro sample through the FIFO (one burst read), each counted for the measured sample period: skipping samples
  or splitting the loop time among them drifted. BDU off (with it on, accelerometer-only reads corrupted the FIFO's
  gyro samples). Still threshold 2.5 dps (1 dps never matched once the offset wandered).
- Loop 1 ms with CAN pipelined (no waiting for answers). Shorter sleeps made the tilt estimate go wrong and the motor
  run off: open.
- The motor must not switch itself off: no turn limit, >90° off re-holds, stall retries after 3 s.

Open: standalone (XIAO on the RollerCAN's Grove 5V, no USB) the gyro reads ~10x noisier at rest, still windows rarely
match, the offset isn't corrected and the top slowly turns. After the CAN fix: 7 min holding (6 handled) without a crash or
any CAN error state (crashes had come after 10-30 s). Gyro noise at rest while holding 3.7/8.7/10.8 dps RMS, motor off
1.8/3.2/2.7, heater on or off the same: the motor's holding jitter, mostly about the motor axis (z).
Open: put back on the table after handling, the bottom turned a bit by itself (integral wound up in the air, where
the bottom turns without resistance?). Recording now starts at the first hold (or after erase at the next), every
3rd loop, ~90 s.

## Conventions

- Heartbeat on the green user LED; no periodic serial output (it gets in the way of typing), status on `i`.
- Commands are whole lines only, input ignored for 300 ms after DTR goes on: Linux echoes the XIAO's output back
  until the port is raw, so echoed text must never start the motor.
- RollerCAN or IMU missing: LED blinks fast, serial repeats the error every 2 s.
- Behavior questions about standalone use: record (`erase`, run on battery, `log.py`), then analyze the CSV.
- Mechanical setups: ask for a photo early, words like "top" and "down" are ambiguous.
