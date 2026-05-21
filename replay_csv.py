#!/usr/bin/env python3
"""
Replay a sensor CSV as MCC-protocol UDP packets.

Supports CSV files from:
  - hbk_cv7_cli (cv7_* columns)
  - sample_navigator (nav_* columns)
  - MCC raw logs (accel_*, gyro_*, mag_* columns)

MCC protocol: |ts,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw*

Usage:
    python replay_csv.py /path/to/data.csv --udp-addr 127.0.0.1 --udp-port 50100 --speed 3.0
"""

import argparse
import csv
import socket
import time
import sys


# Column name mappings for different CSV sources
COL_MAPS = [
    # MCC native
    {'ts':'timestamp_s','ax':'accel_x','ay':'accel_y','az':'accel_z',
     'gx':'gyro_x','gy':'gyro_y','gz':'gyro_z',
     'mx':'mag_x','my':'mag_y','mz':'mag_z',
     'roll':'roll_rad','pitch':'pitch_rad','yaw':'yaw_rad'},
    # CV7 (hbk_cv7_cli)
    {'ts':'timestamp_s','ax':'cv7_accel_x_ms2','ay':'cv7_accel_y_ms2','az':'cv7_accel_z_ms2',
     'gx':'cv7_gyro_x_rads','gy':'cv7_gyro_y_rads','gz':'cv7_gyro_z_rads',
     'mx':'cv7_mag_x_gauss','my':'cv7_mag_y_gauss','mz':'cv7_mag_z_gauss',
     'roll':'cv7_roll_rad','pitch':'cv7_pitch_rad','yaw':'cv7_yaw_rad'},
    # Navigator (sample_navigator)
    {'ts':'timestamp_s','ax':'nav_accel_x_ms2','ay':'nav_accel_y_ms2','az':'nav_accel_z_ms2',
     'gx':'nav_gyro_x_rads','gy':'nav_gyro_y_rads','gz':'nav_gyro_z_rads',
     'mx':'nav_mag_x_ut','my':'nav_mag_y_ut','mz':'nav_mag_z_ut',
     'roll':'nav_roll_rad','pitch':'nav_pitch_rad','yaw':'nav_yaw_rad'},
]


def detect_columns(header):
    """Auto-detect which column mapping matches the CSV header."""
    header_set = set(c.strip() for c in header)
    for cmap in COL_MAPS:
        if all(v in header_set for v in cmap.values()):
            return cmap
    return None


def main():
    p = argparse.ArgumentParser(description='Replay sensor CSV as MCC UDP stream')
    p.add_argument('csv_file', help='Path to sensor CSV')
    p.add_argument('--udp-addr', default='127.0.0.1')
    p.add_argument('--udp-port', type=int, default=50100)
    p.add_argument('--speed', type=float, default=1.0, help='Playback speed multiplier')
    p.add_argument('--loop', action='store_true', help='Loop indefinitely')
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    dest = (args.udp_addr, args.udp_port)

    # Read CSV and detect format
    with open(args.csv_file, 'r') as f:
        reader = csv.DictReader(f)
        header = [c.strip() for c in reader.fieldnames]
        rows = list(reader)

    cmap = detect_columns(header)
    if cmap is None:
        print(f"Error: Could not detect CSV format from columns: {header}")
        sys.exit(1)

    # Identify source by which map matched
    src = 'unknown'
    if 'cv7_mag_x_gauss' in [cmap[k] for k in cmap]:
        src = 'CV7'
    elif 'nav_mag_x_ut' in [cmap[k] for k in cmap]:
        src = 'Navigator'
    elif 'mag_x' in [cmap[k] for k in cmap]:
        src = 'MCC'
    print(f"Detected: {src} ({len(rows)} rows)")

    sent = 0
    while True:
        csv_start = None
        wall_start = time.monotonic()

        for row in rows:
            # Strip whitespace from keys
            row = {k.strip(): v for k, v in row.items()}

            ts = float(row[cmap['ts']])
            if csv_start is None:
                csv_start = ts

            # Timing
            csv_elapsed = (ts - csv_start) / args.speed
            wall_elapsed = time.monotonic() - wall_start
            delay = csv_elapsed - wall_elapsed
            if delay > 0:
                time.sleep(delay)

            # Build MCC packet: |ts,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw*
            pkt = "|{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}*".format(
                ts,
                float(row[cmap['ax']]), float(row[cmap['ay']]), float(row[cmap['az']]),
                float(row[cmap['gx']]), float(row[cmap['gy']]), float(row[cmap['gz']]),
                float(row[cmap['mx']]), float(row[cmap['my']]), float(row[cmap['mz']]),
                float(row[cmap['roll']]), float(row[cmap['pitch']]), float(row[cmap['yaw']]),
            )
            sock.sendto(pkt.encode('ascii'), dest)
            sent += 1

            if sent % 500 == 0:
                print(f"\r{sent} packets, t={ts:.2f}s", end='', flush=True)

        print(f"\n{sent} packets — complete.")
        if not args.loop:
            break

    sock.close()


if __name__ == '__main__':
    main()
