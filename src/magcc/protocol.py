# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Raymond Turrisi, Massachusetts Institute of Technology

"""
magcc packet schemas.

v0: |ts,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw*
    sensor frame; gyro rad/s, angles rad, mag in the UI-selected unit

Later schemas lead with a tag field (|MAGCC1,...*). Add parsers to SCHEMAS.
"""

V0_FIELDS = ['ts', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz',
             'filt_roll', 'filt_pitch', 'filt_yaw']


def _strip(s):
    s = s.strip()
    if s.startswith('|'): s = s[1:]
    if s.endswith('*'): s = s[:-1]
    return s


def parse_v0(s):
    fields = _strip(s).split(',')
    if len(fields) != 13: return None
    try:
        f = [float(x) for x in fields]
    except ValueError:
        return None
    return dict(zip(V0_FIELDS, f))


SCHEMAS = {'v0': parse_v0}


def detect(s):
    for name, parse in SCHEMAS.items():
        if parse(s) is not None:
            return name
    return None
