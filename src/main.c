// can-do-gyro: a self-righting platform with the Seeed XIAO STM32C5.
//
// An M5Stack Unit RollerCAN (BLDC motor with its own FOC driver) stands on its motor. Its body, a battery and the XIAO
// sit on top. However the bottom is turned, the top keeps facing the same direction in the room: the XIAO tracks the
// top's 3D orientation with its on-board LSM6DS3TR-C IMU, and about every millisecond it sends the RollerCAN a new
// rotor target over CAN, through the XIAO's on-board TCAN3403 transceiver. The loop is closed on the part that has to
// stay still: the correction moves the XIAO, and the XIAO measures it.
//
// Zephyr (there is no Arduino core for the STM32C5). Serial commands over USB, see handleCommand() and README.md.
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

// ---- Settings (tuned by hand and from recordings, standalone on the battery) ----

#define ROLLER_ID 0xA8  // the RollerCAN's CAN ID (its factory default, shown on its screen)
#define HOST_ID 0x00    // ours, as in M5Stack's library
#define CAN_BITRATE 1000000

#define MAX_CURRENT_MA 500  // the motor's continuous rating without cooling: it can even stay blocked
// Stiffness: the RollerCAN's position controller pushes with this many mA per degree the top is off ("p" changes it
// until reset). 10 lagged 4-7° while turning by hand; 20 and more start to shake.
#define POSITION_KP 15.0f
#define POSITION_KP_MAX 100.0f
// Damping: the target also counts the top's turning rate times this ("d" changes it until reset). 50 overshot.
#define DAMPING_MS 100.0f
#define DAMPING_MS_MAX 500.0f
// Integral: a deviation that stays makes the push grow (by the deviation every INTEGRAL_S) until the motor breaks free
// of its friction (~50 mA) and turns the top back. Capped at INTEGRAL_MAX_MA. 1.5 s overshot by more than 90°.
#define INTEGRAL_S 4.0f
#define INTEGRAL_MAX_MA 300.0f
// The motor's turning direction relative to the gyro's ("f" flips it until reset). The wrong one spins it right away.
#define DIRECTION -1
// The top this far off while holding (held by hand, blocked): its current direction becomes the new one. No turn
// limit: the RollerCAN's position range is ±2^31 centidegrees, ~59,000 turns.
#define RUNAWAY_DEG 90.0
#define STALL_RETRY_MS 3000  // after the RollerCAN's stall protection switched the motor off: hold again after this

// Sleep per loop: ~1 ms loop (IMU 0.3 ms, CAN 0.1 ms). Shorter sleeps (0.3 ms and less) made the tilt estimate go wrong
// and the motor run off; not understood yet.
#define LOOP_US 500
#define CALIBRATION_MS 2000  // at boot (and "c"): the unit has to stand still, motor axis vertical, this long
// Standing still: the gyro's 1/4 s averages within this of each other (calibration) or of its offset (while holding).
// The offset wanders by up to ~3 dps (RollerCAN powered or not, temperature), so not much less.
#define STILL_DPS 2.5f
#define STILL_MS 250
#define OFFSET_FOLLOW 0.05f  // while standing still the offset follows the gyro (time constant ~5 s)
#define INPUT_QUIET_MS 300   // input ignored after a terminal connects, see main()

// Battery: a 2S LiHV (7.6 V nominal, 4.35 V/cell full) on the RollerCAN's XT30, measured by the RollerCAN. For a normal
// LiPo (4.20 V full): CELL_LOW_V 3.77 and the table in batteryPercent().
#define BATTERY_CELLS 2
#define CELL_LOW_V 3.82f    // ~30% left: warning, RollerCAN LEDs orange
#define CELL_EMPTY_V 3.50f  // motor off, LEDs blink red, until the battery is above CELL_LOW_V again
#define BATTERY_CHECK_MS 1000
#define BATTERY_AVERAGE 10       // checks averaged (exponentially): a short dip under load doesn't count
#define BATTERY_REPEAT_MS 30000  // a warning is repeated this often
#define LED_ORANGE 0x602000      // RollerCAN LED colors, dimmed: they draw from the battery too
#define LED_RED 0x600000
#define LED_DARK_RED 0x080000  // the blink's off phase (black isn't accepted)

#define ANSWER_MS 50     // wait this long for the RollerCAN's answer
#define LOST_AFTER 3     // unanswered frames in a row until it counts as gone
#define PIPE_LOST_MS 200 // while holding: no position answer this long, it counts as gone
#define RETRY_MS 2000    // look for the RollerCAN / the IMU again this often

// ---- RollerCAN protocol ----
// Extended frames, ID = type << 24 | option << 16 | host << 8 | target, 8 data bytes: parameter index (le16), value
// (le32 at byte 4). Read answers carry the RollerCAN's ID in ID bits 0-7, all other answers in bits 8-15.
// Checked against its firmware (github.com/m5stack/M5Unit-RollerCAN-Internal-FW, can_protocol.c).
#define TYPE_DEVICE_ID 0x00
#define TYPE_FEEDBACK 0x02  // answer to writes: faults in ID bits 16-18, data: rpm, position, current (mA), voltage
#define TYPE_READ 0x11
#define TYPE_WRITE 0x12
#define P_RELEASE_STALL 0x7003    // write 2: release the stall protection
#define P_OUTPUT 0x7004           // 1 = motor on, 0 = off (free)
#define P_MODE 0x7005             // 2 = position mode
#define P_POSITION 0x7016         // target, degrees x100 (multi-turn)
#define P_POS_MAX_CURRENT 0x7017  // mA x100
#define P_SPEED_MAX_CURRENT 0x7018
#define P_POS_KP 0x7023           // mA per degree x100000 (only applies with the default PID preset)
#define P_ACTUAL_POSITION 0x7031  // degrees x100
#define P_ACTUAL_CURRENT 0x7032   // mA x100
#define P_VIN 0x7034              // V x100
#define P_TEMP 0x7035             // °C
#define P_RGB_MODE 0x7050         // the RollerCAN's two LEDs: 0 = its own display, 1 = the color below
#define P_RGB_COLOR 0x7051        // 0xRRGGBB (queued on the RollerCAN: write only on change)
#define MODE_POSITION 2
#define FAULT_OVERVOLTAGE 0x1
#define FAULT_STALL 0x2
#define FAULT_RANGE 0x4

// ---- On-board IMU: LSM6DS3TR-C on I2C (raw registers) ----
#define IMU_WHO_AM_I 0x0F
#define IMU_ID 0x6A
#define IMU_CTRL1_XL 0x10
#define IMU_CTRL2_G 0x11
#define IMU_CTRL3_C 0x12
#define IMU_OUTX_L_G 0x22
#define IMU_OUTX_L_XL 0x28
#define XL_104HZ_2G 0x40  // accelerometer 104 Hz, ±2 g (only for the slow tilt correction)
// Gyro 416 Hz, ±2000 dps. At 104 Hz its readings came ~15 ms late and the unit wobbled at ~17 Hz; 416 Hz held best
// despite more noise (~3.7 dps RMS, ~0.8 at 104 Hz).
#define G_416HZ_2000DPS 0x6C
// FIFO, gyro only, 416 Hz: every sample counts, also the ones that arrive while the loop is busy. Reading only the newest
// sample per loop skipped samples at the phase of the loop's own jitter, and that added up to drift and tilt errors.
#define IMU_FIFO_CTRL3 0x08
#define IMU_FIFO_CTRL5 0x0A
#define IMU_FIFO_STATUS1 0x3A  // 1-2: unread words (11 bits), overrun = bit 6 of 2; 3-4: next word in the pattern
#define IMU_FIFO_DATA_OUT_L 0x3E
#define FIFO_GYRO_ONLY 0x08
#define FIFO_416HZ_CONTINUOUS 0x36
#define FIFO_BYPASS 0x00
#define FIFO_MAX_SAMPLES 32  // per loop; more means the loop stalled: start over
// Address auto-increment, block data update off: with BDU on, reading the accelerometer registers alone left the gyro's
// FIFO samples ~6 dps off on one axis. Off, the accelerometer can be read alone and less often.
#define CTRL3_IF_INC 0x04
#define ACCEL_EVERY 4       // read the accelerometer every this many loops
#define GYRO_DPS 0.070f     // per LSB at ±2000 dps
#define ACCEL_G 0.000061f   // per LSB at ±2 g
// The gyro's real sample period (its own clock, nominally 1/416 s), measured over this long. Each sample counts for
// exactly that: splitting the loop's time among its samples biased the integration the same way as skipping them.
#define SAMPLE_PERIOD_S 5.0f

// ---- 3D orientation ----
// A quaternion from XIAO axes to room axes (room z = up), integrated from all three gyro axes; gravity (when the
// accelerometer reads about 1 g) pulls its tilt back with TILT_GAIN. Counting only the turn about the motor axis misses
// what tilting, turning and tilting back does.
// Facing: `across` (a XIAO-axis vector across the motor axis, the top's front) should point at `facing` (where it pointed
// in the room when holding started). The motor can only turn it about the motor axis, so the target is facing's part
// across the motor axis, and `turned` counts how far front and target turned apart. Within FACING_MIN_DEG of the motor
// axis pointing at `facing`, turning can't help: steps don't count there, afterwards it holds the direction it faces then.
#define TILT_GAIN 0.5f              // rad/s per rad of tilt error: ~2 s time constant
#define TILT_ACCEL_TOLERANCE 0.15f  // the accelerometer counts if it reads 1 g within this (not shaken)
#define FACING_MIN_DEG 15.0f
#define DEG 57.29578f

// ---- Recording ----
// To see what it does standalone, without USB: from boot, every loop writes a LogRecord to the XIAO's internal flash,
// in the OTA slot (480 KB in flash bank 2: the code runs from bank 1, so writing doesn't stall it): ~30 s at the 1 ms
// loop. Kept until "erase" (reset, power off and reflashing don't touch it); a boot records only when it's empty.
// "dump" prints it, log.py turns that into a CSV.
#define LOG_MAGIC 0x474F4C52  // "RLOG"
#define LOG_VERSION 2
typedef struct __packed {
  uint32_t magic;
  uint16_t version;
  int16_t kp10;  // stiffness x10
  int16_t dampingMs;
  int8_t direction;
  uint8_t reserved[5];
} LogHeader;
typedef struct __packed {
  uint32_t us;      // since the recording started
  int16_t rate;     // the top's turning rate, 0.1 deg/s
  int16_t turned;   // how far the top is off, 0.01 deg
  int32_t rotor;    // rotor position, 0.01 deg (the last one answered while holding)
  int16_t current;  // motor current, mA (from the RollerCAN's answer to the last write)
  uint8_t flags;    // LOG_*
  uint8_t tilt;     // the motor axis off vertical, degrees
} LogRecord;
#define LOG_HOLDING 0x01   // motor on, following the gyro
#define LOG_NEW_GYRO 0x02  // new gyro samples this loop
#define LOG_STILL 0x04     // a standing-still window ended here (offset adjusted)
#define LOG_SENT 0x08      // a new target was sent
BUILD_ASSERT(sizeof(LogHeader) == 16 && sizeof(LogRecord) == 16, "one flash write unit (16 bytes) each");

static const struct device *const can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
static const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct i2c_dt_spec imu = I2C_DT_SPEC_GET(DT_ALIAS(imu0));
CAN_MSGQ_DEFINE(rx, 16);
K_MSGQ_DEFINE(input, 1, 64, 1);  // typed characters

// RollerCAN
static bool found = false;  // it answers
static int unanswered = 0;
static uint8_t faults = 0;        // from the last feedback frame
static int16_t lastCurrent = 0;   // mA, from the last feedback frame
static int64_t stallRetryAt = 0;  // uptime, ms: hold again from then on (after a stall)
typedef enum { BATTERY_OK, BATTERY_LOW, BATTERY_EMPTY } Battery;
static Battery battery = BATTERY_OK;
static float batteryV = 0;  // averaged, 0 = no reading yet
static int ledsShown = -1;  // Battery shown on the RollerCAN's LEDs, -1 = not set yet

// IMU and orientation
static bool imuOk = false;
static bool calibrated = false;
static float bias[3];  // gyro zero offset, dps
static float up[3];    // unit vector, XIAO axes: up = the motor axis
typedef struct {
  float w, x, y, z;
} Quat;
static Quat attitude = {1, 0, 0, 0};  // XIAO axes -> room axes
static float across[3];               // unit vector in XIAO axes, across the motor axis: the top's front
static float tilt = 0;                // the motor axis off vertical, degrees
static float facing[3] = {1, 0, 0};   // room: where the front should point (set when holding starts)
static float facingAngle = 0;         // degrees, front -> target about the motor axis
static bool facingValid = false;
static float rate = 0;  // the top's turning rate about the motor axis, degrees/s, as turned
static float samplePeriod = 1 / 416.0f;
static float periodTime = 0;  // s, since the sample period measurement started
static int periodSamples = 0;
static uint32_t lastCycles;  // the last loop's time

// Control
static float positionKp = POSITION_KP;
static float dampingMs = DAMPING_MS;
static int direction = DIRECTION;
static bool wantHold = true;  // hold whenever possible ("s" and an empty battery switch it off, "h" on)
static bool holding = false;  // motor on, rotor following the gyro
static double turned = 0;     // how far the top is off, degrees as the rotor counts them (double: tiny steps, big sums)
static double integral = 0;   // degrees added to turned for the push: grows while the top stays off
static int32_t lastActual = 0;    // the last rotor position answered, degrees x100
// Without waiting for CAN answers: each loop sends the target and the next position request and goes on; the answers
// arrive meanwhile and drainAnswers() takes them at the next loop (waiting cost ~0.7 ms per loop)
static int32_t pipeActual;      // the last position answer, degrees x100
static bool pipeFresh = false;  // ... not used for a target yet
static bool pipePending = false;  // a position request is out
static int64_t pipeSentAt = 0, pipeAnsweredAt = 0;

// Diagnostics for "i", since the last "i"
enum { T_IMU, T_READ, T_WRITE, T_SLEEP, T_LOOP, T_N };  // where a holding loop's time goes, cycles
static uint64_t timing[T_N];
static uint32_t timingLoops = 0;
static uint32_t timingMark, loopMark;
static float loopAccel[3];  // the accelerometer as the loop last saw it
static uint32_t tiltCorrected = 0, tiltSamples = 0;
static double loopGyroSum[3];  // gyro samples minus the offset, summed
static uint32_t fifoSkippedWords = 0, fifoResets = 0;  // since boot

// Recording
static const struct flash_area *logArea;
static uint32_t logOffset = 0;  // next record while recording, 0 = not recording
static uint32_t logRecords = 0;
static uint32_t logUs = 0;

static void timeTo(int part) {
  uint32_t c = k_cycle_get_32();
  timing[part] += c - timingMark;
  timingMark = c;
}

// ---- CAN ----

// Not used: an unanswered frame stays queued (the controller retries until it's acknowledged), see findRoller()
static void txDone(const struct device *dev, int error, void *user_data) {}

static void transmit(uint8_t type, uint16_t index, int32_t value, k_timeout_t timeout) {
  struct can_frame f = {
      .id = (uint32_t)type << 24 | HOST_ID << 8 | ROLLER_ID,
      .dlc = 8,
      .flags = CAN_FRAME_IDE,
  };
  memset(f.data, 0, sizeof f.data);
  sys_put_le16(index, f.data);
  sys_put_le32((uint32_t)value, f.data + 4);
  can_send(can, &f, timeout, txDone, NULL);
}

static uint8_t frameType(const struct can_frame *f) { return f->id >> 24 & 0x1F; }

static void logFaults(uint8_t now) {
  uint8_t added = now & ~faults;
  if (added & FAULT_OVERVOLTAGE) printk("RollerCAN: overvoltage (max 16 V)\n");
  if (added & FAULT_STALL)
    printk("RollerCAN: stall protection, motor off, holds again in %d s\n", STALL_RETRY_MS / 1000);
  if (added & FAULT_RANGE) printk("RollerCAN: out of range\n");
  if (!now && faults) printk("RollerCAN: faults cleared\n");
  faults = now;
  if (now & FAULT_STALL && holding) {
    holding = false;
    stallRetryAt = k_uptime_get() + STALL_RETRY_MS;
  }
}

static void takeFeedback(const struct can_frame *f) {
  logFaults(f->id >> 16 & 0x7);
  lastCurrent = (int16_t)sys_get_le16(f->data + 4);
}

// Send a frame and wait for its answer
static bool request(uint8_t type, uint16_t index, int32_t value, struct can_frame *f) {
  k_msgq_purge(&rx);
  transmit(type, index, value, K_MSEC(5));
  uint8_t answerType = type == TYPE_WRITE ? TYPE_FEEDBACK : type;
  int64_t until = k_uptime_get() + ANSWER_MS;
  bool ok = false;
  while (!ok && k_msgq_get(&rx, f, K_MSEC(MAX(until - k_uptime_get(), 0))) == 0) {
    if (frameType(f) != answerType || f->dlc != 8) continue;
    if (answerType == TYPE_READ) ok = (f->id & 0xFF) == ROLLER_ID && sys_get_le16(f->data) == index;
    else ok = (f->id >> 8 & 0xFF) == ROLLER_ID;
  }
  if (!ok) {
    if (found && ++unanswered >= LOST_AFTER) {
      found = false;
      holding = false;
      printk("RollerCAN lost: no answer on CAN\n");
    }
    return false;
  }
  if (answerType == TYPE_FEEDBACK) takeFeedback(f);
  unanswered = 0;
  return true;
}

static bool readParam(uint16_t index, int32_t *value) {
  struct can_frame f;
  if (!request(TYPE_READ, index, 0, &f)) return false;
  *value = (int32_t)sys_get_le32(f.data + 4);
  return true;
}

static bool writeParam(uint16_t index, int32_t value) {
  struct can_frame f;
  return request(TYPE_WRITE, index, value, &f);
}

// While holding: take the answers that arrived (positions and feedback)
static void drainAnswers(void) {
  struct can_frame f;
  while (k_msgq_get(&rx, &f, K_NO_WAIT) == 0) {
    if (f.dlc != 8) continue;
    if (frameType(&f) == TYPE_READ && (f.id & 0xFF) == ROLLER_ID && sys_get_le16(f.data) == P_ACTUAL_POSITION) {
      pipeActual = (int32_t)sys_get_le32(f.data + 4);
      pipeFresh = true;
      pipePending = false;
      pipeAnsweredAt = k_uptime_get();
      unanswered = 0;
    } else if (frameType(&f) == TYPE_FEEDBACK && (f.id >> 8 & 0xFF) == ROLLER_ID) {
      takeFeedback(&f);
    }
  }
}

static void motorOff(void) {
  writeParam(P_OUTPUT, 0);
  holding = false;
}

// Ask for the RollerCAN's ID. The STM32C5's FDCAN (M_CAN) has no one-shot mode: frames nobody acknowledged (RollerCAN
// without power) stay queued and block the TX buffers, so restart the controller first. Once found: motor off, position
// mode, current limits, stiffness.
static void findRoller(void) {
  can_stop(can);
  can_start(can);
  struct can_frame f;
  if (!request(TYPE_DEVICE_ID, 0, 0, &f)) {
    printk("RollerCAN not found (no answer from CAN ID 0x%02X): power on its XT30? CAN_H/CAN_L swapped?\n",
           ROLLER_ID);
    return;
  }
  found = true;
  unanswered = 0;
  faults = 0;
  batteryV = 0;  // maybe another battery
  ledsShown = -1;
  motorOff();
  int32_t current = MAX_CURRENT_MA * 100, check = 0, vin = 0, temp = 0;
  bool ok = writeParam(P_MODE, MODE_POSITION) && writeParam(P_POS_MAX_CURRENT, current) &&
            writeParam(P_SPEED_MAX_CURRENT, current) && writeParam(P_POS_KP, lroundf(positionKp * 100000)) &&
            readParam(P_POS_MAX_CURRENT, &check) && readParam(P_VIN, &vin) && readParam(P_TEMP, &temp);
  if (!ok || check != current) {
    printk("RollerCAN found but setting it up failed (max current reads %d mA): motor stays off\n", check / 100);
    found = false;
    return;
  }
  printk("RollerCAN found (CAN ID 0x%02X): %.1f V, %d C, motor off, position mode, max current %d mA, %.0f mA/deg\n",
         ROLLER_ID, vin / 100.0, temp, check / 100, (double)positionKp);
}

// ---- IMU ----

static bool imuInit(void) {
  uint8_t id = 0;
  if (!i2c_is_ready_dt(&imu) || i2c_reg_read_byte_dt(&imu, IMU_WHO_AM_I, &id) || id != IMU_ID ||
      i2c_reg_write_byte_dt(&imu, IMU_CTRL3_C, CTRL3_IF_INC) ||
      i2c_reg_write_byte_dt(&imu, IMU_CTRL1_XL, XL_104HZ_2G) ||
      i2c_reg_write_byte_dt(&imu, IMU_CTRL2_G, G_416HZ_2000DPS) ||
      i2c_reg_write_byte_dt(&imu, IMU_FIFO_CTRL3, FIFO_GYRO_ONLY)) {
    printk("IMU not found (WHO_AM_I 0x%02X): motor stays off\n", id);
    return false;
  }
  k_msleep(200);  // gyro start-up
  return true;
}

// Empty the FIFO and start collecting again (at boot and after the loop paused)
static void imuFifoReset(void) {
  i2c_reg_write_byte_dt(&imu, IMU_FIFO_CTRL5, FIFO_BYPASS);
  i2c_reg_write_byte_dt(&imu, IMU_FIFO_CTRL5, FIFO_416HZ_CONTINUOUS);
  periodTime = 0;
  periodSamples = 0;
}

// The gyro samples collected since the last call (dps, offset not removed): how many, -1 on an error. One burst read:
// with address auto-increment the IMU rolls back from FIFO_DATA_OUT_H to _L, so consecutive words follow.
static int readGyroFifo(float samples[][3], int max) {
  static uint8_t b[6 * FIFO_MAX_SAMPLES + 4];
  uint8_t st[4];
  if (i2c_burst_read_dt(&imu, IMU_FIFO_STATUS1, st, sizeof st)) return -1;
  int words = (st[1] & 0x07) << 8 | st[0];
  int pattern = (st[3] & 0x03) << 8 | st[2];  // 0 = the next word is a sample's x
  if (st[1] & 0x40 || words > 3 * max + 2) {  // overrun, or the loop stalled
    imuFifoReset();
    fifoResets++;
    return 0;
  }
  int skip = (3 - pattern % 3) % 3, n = words > skip ? (words - skip) / 3 : 0;
  if (!n) return 0;
  fifoSkippedWords += skip;
  if (i2c_burst_read_dt(&imu, IMU_FIFO_DATA_OUT_L, b, 2 * (skip + 3 * n))) return -1;
  for (int k = 0; k < n; k++)
    for (int i = 0; i < 3; i++) samples[k][i] = (int16_t)sys_get_le16(b + 2 * (skip + 3 * k + i)) * GYRO_DPS;
  return n;
}

// Three axes from the IMU's output registers at `reg`, scaled
static bool readVector(uint8_t reg, float scale, float v[3]) {
  uint8_t b[6];
  if (i2c_burst_read_dt(&imu, reg, b, sizeof b)) return false;
  for (int i = 0; i < 3; i++) v[i] = (int16_t)sys_get_le16(b + 2 * i) * scale;
  return true;
}

// ---- Orientation ----

static void cross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static float dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// v rotated by q
static void rotate(const Quat *q, const float v[3], float out[3]) {
  float u[3] = {q->x, q->y, q->z}, t[3], ut[3];
  cross(u, v, t);
  for (int i = 0; i < 3; i++) t[i] *= 2;
  cross(u, t, ut);
  for (int i = 0; i < 3; i++) out[i] = v[i] + q->w * t[i] + ut[i];
}

// Upright (from the calibration's gravity, `up`)
static void attitudeReset(void) {
  // the rotation taking up to room z: (1 + up . z, up x z), normalized
  Quat q = {1 + up[2], up[1], -up[0], 0};
  if (q.w < 1e-3f) q = (Quat){0, 1, 0, 0};  // upside down: half a turn about x
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  attitude = (Quat){q.w / n, q.x / n, q.y / n, q.z / n};
  // across: the XIAO axis least along the motor axis, minus its part along it
  int k = 0;
  for (int i = 1; i < 3; i++)
    if (fabsf(up[i]) < fabsf(up[k])) k = i;
  float e[3] = {0};
  e[k] = 1;
  for (int i = 0; i < 3; i++) across[i] = e[i] - up[k] * up[i];
  float n2 = sqrtf(dot(across, across));
  for (int i = 0; i < 3; i++) across[i] /= n2;
  rotate(&attitude, across, facing);  // facing as now (set again when holding starts)
  facingValid = false;
  tilt = 0;
}

// One gyro sample: g without its offset (dps), a accelerometer (g). Returns how far front and target turned apart
// (degrees as turned, 0 while the motor axis points at `facing`) and sets the top's turning rate about the motor axis.
static double attitudeUpdate(const float g[3], const float a[3], float dt) {
  float w[3], an = sqrtf(dot(a, a));
  for (int i = 0; i < 3; i++) {
    w[i] = g[i] / DEG;
    loopGyroSum[i] += (double)g[i];
  }
  memcpy(loopAccel, a, sizeof loopAccel);
  tiltSamples++;
  if (fabsf(an - 1) < TILT_ACCEL_TOLERANCE) {
    tiltCorrected++;
    // room up in XIAO axes; turning by (measured x estimated) moves the estimate towards the measurement
    Quat inv = {attitude.w, -attitude.x, -attitude.y, -attitude.z};
    float z[3] = {0, 0, 1}, v[3], m[3], e[3];
    rotate(&inv, z, v);
    for (int i = 0; i < 3; i++) m[i] = a[i] / an;
    cross(m, v, e);
    for (int i = 0; i < 3; i++) w[i] += TILT_GAIN * e[i];
  }
  Quat q = attitude;  // q' = q + q * (0, w) * dt / 2
  attitude.w += 0.5f * dt * (-q.x * w[0] - q.y * w[1] - q.z * w[2]);
  attitude.x += 0.5f * dt * (q.w * w[0] + q.y * w[2] - q.z * w[1]);
  attitude.y += 0.5f * dt * (q.w * w[1] + q.z * w[0] - q.x * w[2]);
  attitude.z += 0.5f * dt * (q.w * w[2] + q.x * w[1] - q.y * w[0]);
  float n = sqrtf(attitude.w * attitude.w + attitude.x * attitude.x + attitude.y * attitude.y +
                  attitude.z * attitude.z);
  attitude.w /= n;
  attitude.x /= n;
  attitude.y /= n;
  attitude.z /= n;

  float m[3], r[3], t[3], c[3];
  rotate(&attitude, up, m);      // motor axis in the room
  rotate(&attitude, across, r);  // front in the room
  tilt = acosf(CLAMP(m[2], -1.0f, 1.0f)) * DEG;
  rate = direction * dot(g, up);
  float fm = dot(facing, m);
  for (int i = 0; i < 3; i++) t[i] = facing[i] - fm * m[i];  // target: facing's part across the motor axis
  if (sqrtf(dot(t, t)) < sinf(FACING_MIN_DEG / DEG)) {
    facingValid = false;
    return 0;
  }
  cross(r, t, c);
  float angle = atan2f(dot(c, m), dot(r, t)) * DEG;
  double step = 0;
  if (facingValid) {
    float d = angle - facingAngle;
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    step = -direction * d;  // the front turning one way = the target turning the other way relative to it
  }
  facingAngle = angle;
  facingValid = true;
  return step;
}

// From now on the front should keep pointing where it points now
static void facingReset(void) {
  rotate(&attitude, across, facing);
  facingValid = false;
}

// Average the gyro (its zero offset) and the accelerometer (gravity: the motor axis is vertical, so up is along it)
// over CALIBRATION_MS. Only counts if the unit stood still the whole time: the gyro's averages over each 1/4 s have to
// agree within STILL_DPS (single readings are too noisy for that).
static void calibrate(void) {
  float g[3], a[3], gSum[3] = {0}, aSum[3] = {0}, wSum[3] = {0}, wMin[3], wMax[3];
  int n = 0, w = 0, windows = 0;
  printk("calibrating the gyro: keep it still for %d s\n", CALIBRATION_MS / 1000);
  int64_t start = k_uptime_get();
  while (k_uptime_get() - start < CALIBRATION_MS) {
    if (!readVector(IMU_OUTX_L_G, GYRO_DPS, g) || !readVector(IMU_OUTX_L_XL, ACCEL_G, a)) {
      printk("IMU read failed\n");
      imuOk = false;
      return;
    }
    for (int i = 0; i < 3; i++) {
      gSum[i] += g[i];
      aSum[i] += a[i];
      wSum[i] += g[i];
    }
    n++;
    w++;
    if (k_uptime_get() - start >= (windows + 1) * CALIBRATION_MS / 8) {  // a 1/4 s window is done
      for (int i = 0; i < 3; i++) {
        float mean = wSum[i] / w;
        wMin[i] = windows ? MIN(wMin[i], mean) : mean;
        wMax[i] = windows ? MAX(wMax[i], mean) : mean;
        wSum[i] = 0;
      }
      w = 0;
      windows++;
    }
    k_usleep(LOOP_US);
  }
  float spread = 0, gravity = sqrtf(dot(aSum, aSum)) / n;
  for (int i = 0; i < 3; i++) spread = MAX(spread, wMax[i] - wMin[i]);
  if (spread > STILL_DPS || gravity < 0.9f || gravity > 1.1f) {
    printk("calibration: it moved (gyro averages %.1f dps apart, gravity %.2f g), trying again\n", (double)spread,
           (double)gravity);
    return;
  }
  for (int i = 0; i < 3; i++) {
    bias[i] = gSum[i] / n;
    up[i] = aSum[i] / n / gravity;
  }
  attitudeReset();
  calibrated = true;
  printk("gyro calibrated: offset %.2f %.2f %.2f dps, motor axis (up) %.2f %.2f %.2f in XIAO axes\n",
         (double)bias[0], (double)bias[1], (double)bias[2], (double)up[0], (double)up[1], (double)up[2]);
}

// ---- Battery ----

// Rough charge left from the voltage per cell (LiHV at rest; typical values, they vary between packs)
static int batteryPercent(float cellV) {
  static const float table[][2] = {{3.40f, 0},  {3.65f, 5},  {3.72f, 10}, {3.78f, 20}, {3.82f, 30}, {3.87f, 40},
                                   {3.92f, 50}, {3.98f, 60}, {4.05f, 70}, {4.14f, 80}, {4.24f, 90}, {4.35f, 100}};
  if (cellV <= table[0][0]) return 0;
  for (size_t i = 1; i < ARRAY_SIZE(table); i++) {
    if (cellV < table[i][0])
      return lroundf(table[i - 1][1] + (cellV - table[i - 1][0]) / (table[i][0] - table[i - 1][0]) *
                                           (table[i][1] - table[i - 1][1]));
  }
  return 100;
}

// Read the battery voltage. Below CELL_LOW_V: warn. Below CELL_EMPTY_V: motor off until the battery is above
// CELL_LOW_V again (charged or swapped), then hold again.
static void checkBattery(int64_t now) {
  static int64_t lastWarning = 0;
  static bool blink = false;
  int32_t vin;
  if (!readParam(P_VIN, &vin)) return;
  batteryV = batteryV == 0 ? vin / 100.0f : batteryV + (vin / 100.0f - batteryV) / BATTERY_AVERAGE;
  float cell = batteryV / BATTERY_CELLS;
  Battery state = cell < CELL_EMPTY_V ? BATTERY_EMPTY : cell < CELL_LOW_V ? BATTERY_LOW : BATTERY_OK;
  if (battery == BATTERY_EMPTY && state == BATTERY_LOW) state = BATTERY_EMPTY;
  if (state != battery) {
    if (state == BATTERY_EMPTY) {
      wantHold = false;
      motorOff();
    } else if (battery == BATTERY_EMPTY) {
      wantHold = true;
      printk("battery charged again: %.2f V\n", (double)batteryV);
    }
    battery = state;
    lastWarning = now - BATTERY_REPEAT_MS;  // say it now
  }
  if (battery != BATTERY_OK && now - lastWarning >= BATTERY_REPEAT_MS) {
    lastWarning = now;
    if (battery == BATTERY_LOW)
      printk("battery low: %.2f V (~%d%%), charge it soon\n", (double)batteryV, batteryPercent(cell));
    else
      printk("battery empty: %.2f V, motor off. Unplug it: the RollerCAN still draws current, and a LiPo is "
             "damaged below %.1f V\n", (double)batteryV, 3.0 * BATTERY_CELLS);
  }
  blink = !blink;
  if (battery != ledsShown || battery == BATTERY_EMPTY) {
    if (battery == BATTERY_OK) writeParam(P_RGB_MODE, 0);
    else if (writeParam(P_RGB_MODE, 1))
      writeParam(P_RGB_COLOR, battery == BATTERY_LOW ? LED_ORANGE : blink ? LED_RED : LED_DARK_RED);
    ledsShown = battery;
  }
}

// ---- Holding ----

// Keep the top's current orientation: from now on the motor turns it back whenever it's off
static void hold(void) {
  int32_t actual;
  if (!readParam(P_ACTUAL_POSITION, &actual)) return;
  if (faults & FAULT_STALL) {
    writeParam(P_RELEASE_STALL, 2);
    printk("stall protection released, ");
  }
  if (!writeParam(P_POSITION, actual) || !writeParam(P_OUTPUT, 1)) return;
  turned = 0;
  integral = 0;
  facingReset();
  pipeFresh = pipePending = false;
  pipeAnsweredAt = k_uptime_get();
  holding = true;
  printk("holding: the top keeps its orientation\n");
}

// ---- Recording ----

static bool logEmpty(uint32_t offset) {
  uint8_t b[16];
  if (flash_area_read(logArea, offset, b, sizeof b)) return false;
  for (size_t i = 0; i < sizeof b; i++)
    if (b[i] != 0xFF) return false;
  return true;
}

static void logStart(void) {
  LogHeader h = {.magic = LOG_MAGIC, .version = LOG_VERSION, .kp10 = lroundf(positionKp * 10),
                 .dampingMs = lroundf(dampingMs), .direction = direction};
  memset(h.reserved, 0xFF, sizeof h.reserved);
  if (flash_area_write(logArea, 0, &h, sizeof h)) {
    printk("recording: flash write failed\n");
    return;
  }
  logOffset = sizeof h;
  logRecords = 0;
  logUs = 0;
  printk("recording: started (%u records, ~%u s, kept until erase)\n",
         (unsigned)((logArea->fa_size - sizeof h) / sizeof(LogRecord)),
         (unsigned)((logArea->fa_size - sizeof h) / sizeof(LogRecord) / 1000));
}

static bool logErase(void) {
  printk("recording: erasing...\n");
  logOffset = logRecords = 0;
  if (flash_area_erase(logArea, 0, logArea->fa_size)) {
    printk("recording: erase failed\n");
    return false;
  }
  return true;
}

// At boot: record if the area is empty (or holds something else), keep a recording otherwise
static void logInit(void) {
  if (flash_area_open(PARTITION_ID(slot1_partition), &logArea)) {
    printk("recording: no flash area\n");
    logArea = NULL;
    return;
  }
  LogHeader h;
  if (logEmpty(0)) {
    logStart();
  } else if (!flash_area_read(logArea, 0, &h, sizeof h) && h.magic == LOG_MAGIC && h.version == LOG_VERSION) {
    while (sizeof h + (logRecords + 1) * sizeof(LogRecord) <= logArea->fa_size &&
           !logEmpty(sizeof h + logRecords * sizeof(LogRecord)))
      logRecords++;
    LogRecord last = {0};
    if (logRecords) flash_area_read(logArea, sizeof h + (logRecords - 1) * sizeof last, &last, sizeof last);
    logUs = last.us;
    printk("recording: %u records (%.1f s) kept, not recording. dump prints them, erase clears (the next boot "
           "records)\n", logRecords, logUs / 1e6);
  } else if (logErase()) {
    logStart();
  }
}

static void logWrite(uint8_t flags) {
  if (!logOffset) return;
  LogRecord r = {
      .us = logUs,
      .rate = CLAMP(lroundf(rate * 10), INT16_MIN, INT16_MAX),
      .turned = CLAMP(lround(turned * 100), INT16_MIN, INT16_MAX),
      .rotor = lastActual,
      .current = lastCurrent,
      .flags = flags | (holding ? LOG_HOLDING : 0),
      .tilt = (uint8_t)MIN(lroundf(tilt), 255),
  };
  if (flash_area_write(logArea, logOffset, &r, sizeof r)) {
    printk("recording: flash write failed, stopped\n");
    logOffset = 0;
    return;
  }
  logOffset += sizeof r;
  logRecords++;
  if (logOffset + sizeof r > logArea->fa_size) {
    logOffset = 0;
    printk("recording: full (%u records), stopped\n", logRecords);
  }
}

// Print the recording: a header line, one line of hex per record (the LogRecord's bytes), an end line. The motor goes
// off meanwhile. Blocks while the host doesn't read: with flow control on the USB serial port the output waits
// instead of being dropped (only during the dump: without a host it would block forever).
static void logDump(void) {
  static const char hex[] = "0123456789abcdef";
  if (!logArea) return;
  motorOff();
  LogHeader h;
  flash_area_read(logArea, 0, &h, sizeof h);
  struct uart_config cfg;
  bool flow = !uart_config_get(console, &cfg);
  if (flow) {
    cfg.flow_ctrl = UART_CFG_FLOW_CTRL_RTS_CTS;
    uart_configure(console, &cfg);
  }
  printk("log begin %u records, kp %.1f damping %d direction %d version %u\n", logRecords, h.kp10 / 10.0,
         h.dampingMs, h.direction, h.version);
  for (uint32_t i = 0; i < logRecords; i++) {
    uint32_t dtr = 0;
    uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
    if (!dtr) break;  // the terminal went away
    uint8_t b[sizeof(LogRecord)];
    char line[2 * sizeof b + 1];
    flash_area_read(logArea, sizeof h + i * sizeof b, b, sizeof b);
    for (size_t j = 0; j < sizeof b; j++) {
      line[2 * j] = hex[b[j] >> 4];
      line[2 * j + 1] = hex[b[j] & 0xF];
    }
    line[2 * sizeof b] = 0;
    printk("%s\n", line);
  }
  printk("log end %u\n", logRecords);
  if (flow) {
    cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    uart_configure(console, &cfg);
  }
  lastCycles = k_cycle_get_32();  // the gyro wasn't integrated meanwhile
  imuFifoReset();
}

// ---- Serial commands ----

static void status(void) {
  printk("%s, top %.1f deg off, direction %d, %.0f mA/deg, damping %.0f ms\n",
         holding ? "holding" : "not holding (motor off)", turned, direction, (double)positionKp, (double)dampingMs);
  int32_t position = 0, current = 0, vin = 0, temp = 0;
  if (found && readParam(P_ACTUAL_POSITION, &position) && readParam(P_ACTUAL_CURRENT, &current) &&
      readParam(P_VIN, &vin) && readParam(P_TEMP, &temp)) {
    printk("rotor %.1f deg, %d mA, %.1f V, %d C\n", position / 100.0, current / 100, vin / 100.0, temp);
  } else {
    printk("RollerCAN not found\n");
  }
  if (calibrated)
    printk("gyro offset %.2f %.2f %.2f dps, motor axis %.2f %.2f %.2f, tilted %.1f deg, front %.1f deg off the target%s, "
           "integral %.1f deg, gyro samples every %.3f ms\n", (double)bias[0], (double)bias[1], (double)bias[2],
           (double)up[0], (double)up[1], (double)up[2], (double)tilt, (double)-facingAngle,
           facingValid ? "" : " (undefined: motor axis points at it)", integral, (double)(samplePeriod * 1000));
  else
    printk("gyro not calibrated%s\n", imuOk ? "" : ", IMU not found");
  if (tiltSamples) {
    float an = sqrtf(dot(loopAccel, loopAccel));
    float c = an > 0 ? dot(loopAccel, up) / an : 1;
    printk("IMU: tilt from the accelerometer %.1f deg (estimate %.1f), tilt corrected in %u of %u samples, gyro minus "
           "offset %.2f %.2f %.2f dps\n", (double)(acosf(CLAMP(c, -1.0f, 1.0f)) * DEG), (double)tilt, tiltCorrected,
           tiltSamples, loopGyroSum[0] / tiltSamples, loopGyroSum[1] / tiltSamples, loopGyroSum[2] / tiltSamples);
    memset(loopGyroSum, 0, sizeof loopGyroSum);
    tiltCorrected = tiltSamples = 0;
  }
  if (timingLoops) {
    double us = 1e6 / sys_clock_hw_cycles_per_sec() / timingLoops;
    printk("loop %.0f us while holding: IMU %.0f, CAN read %.0f, CAN write %.0f, sleep %.0f, rest %.0f; FIFO since boot: "
           "%u words skipped, %u resets\n",
           timing[T_LOOP] * us, timing[T_IMU] * us, timing[T_READ] * us, timing[T_WRITE] * us, timing[T_SLEEP] * us,
           (timing[T_LOOP] - timing[T_IMU] - timing[T_READ] - timing[T_WRITE] - timing[T_SLEEP]) * us,
           fifoSkippedWords, fifoResets);
    memset(timing, 0, sizeof timing);
    timingLoops = 0;
  }
  if (faults) printk("faults: 0x%x (1 overvoltage, 2 stall, 4 out of range)\n", faults);
  if (logOffset) printk("recording: %u records (%.1f s)\n", logRecords, logUs / 1e6);
  else if (logArea) printk("recording: %u records kept, not recording (dump, erase)\n", logRecords);
  if (batteryV > 0)
    printk("battery %.2f V averaged, ~%d%% of a %dS LiHV%s\n", (double)batteryV,
           batteryPercent(batteryV / BATTERY_CELLS), BATTERY_CELLS,
           battery == BATTERY_EMPTY ? ", empty: motor off" : battery == BATTERY_LOW ? ", low" : "");
}

// One command per line. A command has to be the whole line and anything else is ignored without an answer: a terminal
// that echoes our own output back (Linux does until the port is set to raw) must not act on it or play ping-pong.
static void handleCommand(const char *line) {
  if (!line[0]) return;
  if (!strcmp(line, "dump")) {
    logDump();
    return;
  }
  if (!strcmp(line, "erase")) {
    motorOff();
    if (logErase()) printk("recording: cleared, the next boot records\n");
    lastCycles = k_cycle_get_32();
    return;
  }
  char *end;
  float value = strtof(line + 1, &end);
  bool hasValue = end != line + 1;
  while (*end == ' ') end++;
  if (*end || !isfinite(value)) return;  // more than a command
  if (line[0] == 'p') {
    if (hasValue && value >= 5 && value <= POSITION_KP_MAX) {
      positionKp = value;
      if (found) writeParam(P_POS_KP, lroundf(positionKp * 100000));
    }
    printk("stiffness: %.0f mA per degree (p 5 to p %.0f)\n", (double)positionKp, (double)POSITION_KP_MAX);
  } else if (line[0] == 'd') {
    if (hasValue && value >= 0 && value <= DAMPING_MS_MAX) dampingMs = value;
    printk("damping: %.0f ms (d 0 to d %.0f)\n", (double)dampingMs, (double)DAMPING_MS_MAX);
  } else if (hasValue) {
    return;
  } else if (line[0] == 'h') {
    if (battery == BATTERY_EMPTY) printk("battery empty: holds once it's charged\n");
    wantHold = true;
    if (holding) hold();  // the orientation from now on
  } else if (line[0] == 's') {
    wantHold = false;
    motorOff();
    printk("motor off (turns freely)\n");
  } else if (line[0] == 'c') {
    motorOff();
    calibrated = false;
  } else if (line[0] == 'f') {
    direction = -direction;
    printk("direction %d\n", direction);
    if (holding) hold();
  } else if (line[0] == 'i') {
    status();
  } else if (line[0] == '?') {
    printk("commands: h = hold the orientation, s = motor off, c = calibrate the gyro (keep still), "
           "f = flip direction, p <mA/deg> = stiffness, d <ms> = damping, i = status, "
           "dump = print the recording, erase = clear it (the next boot records)\n");
  }
}

// Typed characters into `input`. Interrupt driven: with polling only, Zephyr's USB CDC ACM never queues a receive and
// the host's writes hang.
static void consoleIrq(const struct device *dev, void *user_data) {
  unsigned char c;
  while (uart_irq_update(dev) && uart_irq_rx_ready(dev) && uart_fifo_read(dev, &c, 1) == 1) {
    k_msgq_put(&input, &c, K_NO_WAIT);
  }
}

// ---- Main loop ----

int main(void) {
  gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
  uart_irq_callback_set(console, consoleIrq);
  uart_irq_rx_enable(console);
  k_msleep(1000);  // give the host a moment to open the USB serial port
  printk("can-do-gyro: max current %d mA\n", MAX_CURRENT_MA);

  struct can_filter all = {.flags = CAN_FILTER_IDE, .id = 0, .mask = 0};  // every extended frame
  if (!device_is_ready(can) || can_set_bitrate(can, CAN_BITRATE) || can_add_rx_filter_msgq(can, &rx, &all) < 0 ||
      can_start(can)) {
    printk("CAN setup failed\n");
  }
  findRoller();
  imuOk = imuInit();
  logInit();
  handleCommand("?");  // print the commands

  int64_t lastTry = k_uptime_get(), lastBlink = 0, windowStart = 0, quietUntil = 0, lastBattery = 0;
  lastCycles = k_cycle_get_32();
  double windowTurned = 0;
  float windowSum[3] = {0};
  int windowN = 0;
  char line[32];
  size_t len = 0;
  bool tooLong = false, connected = false;
  while (true) {
    int64_t now = k_uptime_get();

    // Heartbeat on the user LED, fast while the RollerCAN or the IMU is missing
    if (now - lastBlink >= (found && imuOk ? 500 : 100)) {
      lastBlink = now;
      gpio_pin_toggle_dt(&led);
    }
    if ((!found || !imuOk) && now - lastTry >= RETRY_MS) {
      lastTry = now;
      if (!found) findRoller();
      if (!imuOk) imuOk = imuInit();
    }
    if (imuOk && !calibrated) {
      if (holding) motorOff();
      calibrate();
      lastCycles = k_cycle_get_32();
      imuFifoReset();
    }

    // Track the orientation; turned = how far front and target turned apart. A STILL_MS window in which it stood still
    // doesn't count (what the gyro integrated then is drift) and moves the offset towards the gyro.
    static float gs[FIFO_MAX_SAMPLES][3];
    uint8_t logFlags = 0;
    uint32_t cycles = k_cycle_get_32();
    float dt = (float)(cycles - lastCycles) / sys_clock_hw_cycles_per_sec();
    lastCycles = cycles;
    logUs += lroundf(dt * 1e6f);
    if (imuOk && calibrated) {
      uint32_t loopStart = k_cycle_get_32();
      timingMark = loopStart;
      int n = readGyroFifo(gs, FIFO_MAX_SAMPLES);
      static int accelCount = 0;
      static float accel[3] = {0, 0, 1};
      bool imuRead = n >= 0;
      if (imuRead && accelCount++ % ACCEL_EVERY == 0) imuRead = readVector(IMU_OUTX_L_XL, ACCEL_G, accel);
      if (holding) {
        timing[T_LOOP] += loopStart - loopMark;
        timeTo(T_IMU);
        timingLoops++;
      }
      loopMark = loopStart;
      if (imuRead) {
        periodTime += dt;
        periodSamples += n;
        if (periodTime >= SAMPLE_PERIOD_S && periodSamples) {
          samplePeriod = CLAMP(periodTime / periodSamples, 0.9f / 416, 1.1f / 416);
          periodTime = 0;
          periodSamples = 0;
        }
        double step = 0;
        float rateSum = 0;
        for (int k = 0; k < n; k++) {
          float gc[3] = {gs[k][0] - bias[0], gs[k][1] - bias[1], gs[k][2] - bias[2]};
          step += attitudeUpdate(gc, accel, samplePeriod);
          rateSum += rate;
          for (int i = 0; i < 3; i++) windowSum[i] += gs[k][i];
        }
        if (n) {
          rate = rateSum / n;
          logFlags |= LOG_NEW_GYRO;
        }
        windowN += n;
        turned += step;
        if (holding)
          integral = CLAMP(integral + turned * (double)(dt / INTEGRAL_S), -(double)(INTEGRAL_MAX_MA / positionKp),
                           (double)(INTEGRAL_MAX_MA / positionKp));
        else
          integral = 0;
        windowTurned += step;
        if (now - windowStart >= STILL_MS && windowN) {
          bool still = true;
          for (int i = 0; i < 3; i++) still = still && fabsf(windowSum[i] / windowN - bias[i]) <= STILL_DPS;
          if (still) {
            logFlags |= LOG_STILL;
            turned -= windowTurned;
            for (int i = 0; i < 3; i++) bias[i] += (windowSum[i] / windowN - bias[i]) * OFFSET_FOLLOW;
          }
          windowStart = now;
          windowTurned = 0;
          windowSum[0] = windowSum[1] = windowSum[2] = 0;
          windowN = 0;
        }
      } else {
        printk("IMU read failed: motor off\n");
        imuOk = calibrated = false;
        motorOff();
      }
    }

    // A terminal connected (DTR on): the output buffered meanwhile reaches it at once, and Linux echoes it back until
    // the port is raw. Ignore what comes in during INPUT_QUIET_MS, the half line it leaves would eat the first command.
    uint32_t dtr = 0;
    uart_line_ctrl_get(console, UART_LINE_CTRL_DTR, &dtr);
    if (dtr && !connected) quietUntil = now + INPUT_QUIET_MS;
    connected = dtr;
    unsigned char c;
    while (k_msgq_get(&input, &c, K_NO_WAIT) == 0) {
      if (now < quietUntil) {
        len = 0;
        tooLong = false;
      } else if (c == '\r' || c == '\n') {
        line[len] = 0;
        if (!tooLong) handleCommand(line);
        len = 0;
        tooLong = false;
      } else if (len < sizeof line - 1) {
        line[len++] = c;
      } else {
        tooLong = true;  // not a command either
      }
    }

    if (found && now - lastBattery >= BATTERY_CHECK_MS) {
      lastBattery = now;
      checkBattery(now);
    }
    if (wantHold && !holding && found && imuOk && calibrated && battery != BATTERY_EMPTY && now >= stallRetryAt)
      hold();

    // Holding: the rotor's target is where it is now, minus how far the top is off, plus its turning rate times the
    // damping, plus the integral. The RollerCAN's position controller pushes with positionKp mA per degree of that.
    if (holding && fabs(turned) > RUNAWAY_DEG) {
      printk("the top got %.0f deg off (held?): its direction now counts as the new one\n", turned);
      hold();
    } else if (holding) {
      drainAnswers();
      timeTo(T_READ);
      if (holding && pipeFresh) {
        pipeFresh = false;
        lastActual = pipeActual;
        transmit(TYPE_WRITE, P_POSITION, pipeActual - lround((turned + integral + (double)(rate * dampingMs / 1000)) * 100),
                 K_MSEC(2));
        logFlags |= LOG_SENT;
      }
      // the next position: when the last one was answered, or again after a lost one (e.g. a blocking call took it)
      if (holding && (!pipePending || now - pipeSentAt > 5)) {
        transmit(TYPE_READ, P_ACTUAL_POSITION, 0, K_MSEC(2));
        pipePending = true;
        pipeSentAt = now;
      }
      if (holding && now - pipeAnsweredAt > PIPE_LOST_MS) {
        found = holding = false;
        printk("RollerCAN lost: no position answer for %d ms\n", PIPE_LOST_MS);
      }
      timeTo(T_WRITE);
    }
    logWrite(logFlags);

    timingMark = k_cycle_get_32();
    k_usleep(LOOP_US);
    if (holding) timeTo(T_SLEEP);
  }
}
