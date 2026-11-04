#!/usr/bin/env python3
"""Read the recording from the XIAO into a CSV (see LogRecord in src/main.c).

Usage: log.py [out.csv] [port]
Needs pyserial; PlatformIO's Zephyr Python has it:
  ~/.platformio/penv/.zephyr-4.4.0/bin/python log.py rec.csv
"""
import struct
import sys
import time

import serial

PORT = "/dev/serial/by-id/usb-Seeed_Studio_XIAO_STM32C5-if00"
FLAGS = {"holding": 0x01, "new_gyro": 0x02, "still": 0x04, "sent": 0x08}


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "rec.csv"
    port = sys.argv[2] if len(sys.argv) > 2 else PORT
    with serial.Serial(port, 115200, timeout=5) as s:
        time.sleep(0.5)  # the firmware ignores input for 300 ms after the port opens
        s.reset_input_buffer()
        s.write(b"dump\n")
        header, records = None, []
        while True:
            line = s.readline().decode(errors="replace").strip()
            if not line:
                sys.exit("timeout: no 'log end' from the XIAO")
            if line.startswith("log begin"):
                header = line
            elif line.startswith("log end"):
                break
            elif header and len(line) == 32:
                records.append(struct.unpack("<IhhihBB", bytes.fromhex(line)))
    print(header)
    with open(out, "w") as f:
        f.write("t_s,rate_dps,turned_deg,rotor_deg,current_ma,tilt_deg," + ",".join(FLAGS) + "\n")
        for us, rate, turned, rotor, current, flags, tilt in records:
            bits = ",".join(str(int(bool(flags & b))) for b in FLAGS.values())
            f.write(f"{us / 1e6:.6f},{rate / 10:.1f},{turned / 100:.2f},{rotor / 100:.2f},{current},{tilt},{bits}\n")
    print(f"{len(records)} records -> {out}")


if __name__ == "__main__":
    main()
