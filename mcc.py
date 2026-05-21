#!/usr/bin/env python3
"""
MCC — Mag Cal Cockpit

Usage:
    mcc.py --udp-port 50100 [--mount-pitch 90]
    mcc.py --inspect mcc_session_20260521.json
"""

import asyncio
import json
import argparse
import subprocess
import time
import webbrowser
from pathlib import Path
from datetime import datetime
import numpy as np
from scipy.optimize import least_squares
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
clients: set[WebSocket] = set()
mag_data: list = []          # [[mx,my,mz], ...] raw sensor frame
mag_ts: list = []            # [ts, ...] epoch timestamps per sample
rp_data: list = []           # [[roll,pitch,yaw], ...] radians
full_rows: list = []         # full parsed dicts for CSV
logging_active = False
gyro_threshold = 0.05
lvm_result = None            # (hi, si, quality, calibrated) after calibrate
last_cal_mask = None         # boolean mask of which samples were used in last calibration
all_display_pts: list = []   # accumulated display points for reconnect

emfi_ut = 51.1217
inclination_deg = 66.231
declination_deg = -13.7716
mag_unit = 'gauss'  # 'gauss', 'ut', 'nt' — set by frontend
cal_method = 'lm'   # 'lm' or 'lse' — set by frontend

UNIT_LABELS = {'gauss': 'Gauss', 'ut': 'µT', 'nt': 'nT'}
UNIT_SCALE = {'gauss': 1e-2, 'ut': 1.0, 'nt': 1e3}  # multiply emfi_ut by this

def get_emfi():
    return emfi_ut * UNIT_SCALE[mag_unit]

def get_unit_label():
    return UNIT_LABELS[mag_unit]

mount_config = {'mount_roll': 0.0, 'mount_pitch': 0.0, 'mount_yaw': 0.0}
def make_hist_bins(*err_arrays):
    """Compute histogram bins from data range, 2% bin width."""
    all_err = np.concatenate([e for e in err_arrays if len(e) > 0])
    lo = np.floor(np.min(all_err) / 2) * 2
    hi = np.ceil(np.max(all_err) / 2) * 2
    bins = np.arange(lo, hi + 2, 2)
    centers = ((bins[:-1] + bins[1:]) / 2).tolist()
    return bins, centers

# Boston inclination
INCL_RAD = np.deg2rad(66 + 16/60 + 7/3600.0)

# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------
def _rpy_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return (np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]]) @
            np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]) @
            np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]))

def get_mount_inv():
    r = np.deg2rad(mount_config['mount_roll'])
    p = np.deg2rad(mount_config['mount_pitch'])
    y = np.deg2rad(mount_config['mount_yaw'])
    return _rpy_matrix(r, p, y).T

def body_to_level(V, roll, pitch):
    """Transform Nx3 body-frame vectors to level frame using roll/pitch arrays."""
    out = np.empty_like(V)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    # Vectorized Ry(p) @ Rx(r) @ v
    x, y, z = V[:,0], V[:,1], V[:,2]
    # Rx
    y1 = cr*y - sr*z
    z1 = sr*y + cr*z
    # Ry
    out[:,0] = cp*x + sp*z1
    out[:,1] = y1
    out[:,2] = -sp*x + cp*z1
    return out

def mag_heading_deg(M_level):
    """Magnetic heading from level-frame mag: atan2(-My, Mx) in [0,360)."""
    h = np.degrees(np.arctan2(-M_level[:,1], M_level[:,0]))
    return np.mod(h + 360.0, 360.0)

# ---------------------------------------------------------------------------
# RLS Sphere Estimator
# ---------------------------------------------------------------------------
class RLSSphereEstimator:
    """
    Incremental RLS for sphere center fitting.
    Model: 2*cx*mx + 2*cy*my + 2*cz*mz + d = mx^2+my^2+mz^2
    theta = [2cx, 2cy, 2cz, d] where d = r^2 - |c|^2
    """
    def __init__(self, forgetting=0.999):
        self.theta = np.zeros(4)
        self.P = np.eye(4) * 1000.0
        self.lam = forgetting
        self.n = 0

    def update(self, mx, my, mz):
        phi = np.array([mx, my, mz, 1.0])
        y = mx**2 + my**2 + mz**2
        Pphi = self.P @ phi
        K = Pphi / (self.lam + phi @ Pphi)
        self.theta += K * (y - phi @ self.theta)
        self.P = (self.P - np.outer(K, Pphi)) / self.lam
        self.n += 1

    def get_center(self):
        return self.theta[:3] / 2.0

    def get_radius(self):
        c = self.get_center()
        return np.sqrt(max(self.theta[3] + np.dot(c, c), 1e-12))

    def get_hard_soft(self, expected_field):
        c = self.get_center()
        r = self.get_radius()
        scale = expected_field / r if r > 1e-8 else 1.0
        return c, np.eye(3) * scale

    def reset(self):
        self.theta = np.zeros(4)
        self.P = np.eye(4) * 1000.0
        self.n = 0

rls = RLSSphereEstimator()

# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def _lse_init(data, ef, n_iter=3):
    X = data.copy()
    cum_hi = np.zeros(3)
    for _ in range(n_iter):
        n = X.shape[0]
        D = np.column_stack([X[:,0]**2, X[:,1]**2, X[:,2]**2,
            2*X[:,0]*X[:,1], 2*X[:,0]*X[:,2], 2*X[:,1]*X[:,2],
            2*X[:,0], 2*X[:,1], 2*X[:,2]])
        v = np.linalg.lstsq(D, np.ones(n), rcond=None)[0]
        A_q = np.array([[v[0],v[3],v[4]],[v[3],v[1],v[5]],[v[4],v[5],v[2]]])
        b_v = np.array([v[6],v[7],v[8]])
        try:
            center = -0.5 * np.linalg.solve(A_q, b_v)
        except np.linalg.LinAlgError:
            center = -0.5 * np.linalg.pinv(A_q) @ b_v
        cum_hi += center; X = X - center

    hi = cum_hi; Xc = data - hi; n = len(Xc)
    D_M = np.column_stack([Xc[:,0]**2, Xc[:,1]**2, Xc[:,2]**2,
        2*Xc[:,0]*Xc[:,1], 2*Xc[:,0]*Xc[:,2], 2*Xc[:,1]*Xc[:,2]])
    mp = np.linalg.lstsq(D_M, np.full(n, ef**2), rcond=None)[0]
    M = np.array([[mp[0],mp[3],mp[4]],[mp[3],mp[1],mp[5]],[mp[4],mp[5],mp[2]]])
    try:
        ev = np.linalg.eigvals(M)
        if np.min(ev) <= 0: M += (1e-6 - np.min(ev) + 1e-6) * np.eye(3)
        si = np.linalg.cholesky(M).T
    except np.linalg.LinAlgError:
        ev, evec = np.linalg.eigh(M)
        si = evec @ np.diag(np.sqrt(np.maximum(ev, 1e-6)))
    return hi, si


def emfi_pct(data, ef, thresholds=[1, 2, 5]):
    """Percentage of points within ±threshold% of EMFI."""
    mags = np.linalg.norm(data, axis=1)
    result = {}
    for t in thresholds:
        lo, hi_b = ef * (1 - t/100), ef * (1 + t/100)
        result[t] = float(np.sum((mags >= lo) & (mags <= hi_b)) / len(mags) * 100)
    return result


def calibrate_lse(mag_arr, ef, n_iter=5):
    """LSE-only calibration (no nonlinear optimization)."""
    hi, si = _lse_init(mag_arr, ef, n_iter=n_iter)
    cal = (si @ (mag_arr - hi).T).T
    radii = np.linalg.norm(cal, axis=1)
    raw_emfi = emfi_pct(mag_arr, ef)
    cal_emfi = emfi_pct(cal, ef)
    q = {'mean_radius':float(np.mean(radii)),'std_radius':float(np.std(radii)),
         'relative_std':float(np.std(radii)/np.mean(radii)) if np.mean(radii)>0 else 999,
         'n_samples':len(mag_arr),
         'raw_emfi_pct':raw_emfi, 'cal_emfi_pct':cal_emfi}
    return hi, si, q, cal


def calibrate_lm(mag_arr, ef, max_iter=100):
    def residuals(params, data, target):
        hi = params[:3]; sp = params[3:9]
        A = np.array([[sp[0],sp[3],sp[4]],[sp[3],sp[1],sp[5]],[sp[4],sp[5],sp[2]]])
        return np.linalg.norm((A @ (data - hi).T).T, axis=1) - target
    hi0, si0 = _lse_init(mag_arr, ef)
    p0 = np.array([hi0[0],hi0[1],hi0[2],si0[0,0],si0[1,1],si0[2,2],si0[0,1],si0[0,2],si0[1,2]])
    result = least_squares(residuals, p0, args=(mag_arr, ef), method='lm', max_nfev=max_iter*9)
    hi = result.x[:3]; sp = result.x[3:9]
    si = np.array([[sp[0],sp[3],sp[4]],[sp[3],sp[1],sp[5]],[sp[4],sp[5],sp[2]]])
    ev = np.linalg.eigvals(si)
    if np.min(ev) <= 0: si += (1e-6 - np.min(ev) + 1e-6) * np.eye(3)
    cal = (si @ (mag_arr - hi).T).T
    radii = np.linalg.norm(cal, axis=1)
    raw_emfi = emfi_pct(mag_arr, ef)
    cal_emfi = emfi_pct(cal, ef)
    q = {'mean_radius':float(np.mean(radii)),'std_radius':float(np.std(radii)),
         'relative_std':float(np.std(radii)/np.mean(radii)) if np.mean(radii)>0 else 999,
         'n_samples':len(mag_arr),
         'raw_emfi_pct':raw_emfi, 'cal_emfi_pct':cal_emfi}
    return hi, si, q, cal

# ---------------------------------------------------------------------------
# File I/O
# ---------------------------------------------------------------------------
CSV_COLS = ['timestamp_s',
    'accel_x','accel_y','accel_z',
    'gyro_x','gyro_y','gyro_z',
    'mag_x','mag_y','mag_z',
    'roll_rad','pitch_rad','yaw_rad',
    'used']

def save_csv_log(rows, path, mask=None):
    with open(path, 'w') as f:
        f.write(','.join(CSV_COLS) + '\n')
        for i, r in enumerate(rows):
            used = int(mask[i]) if mask is not None and i < len(mask) else 1
            f.write(','.join([f"{r['ts']:.6f}",
                f"{r['ax']:.6f}",f"{r['ay']:.6f}",f"{r['az']:.6f}",
                f"{r['gx']:.6f}",f"{r['gy']:.6f}",f"{r['gz']:.6f}",
                f"{r['mx']:.6f}",f"{r['my']:.6f}",f"{r['mz']:.6f}",
                f"{r['filt_roll']:.6f}",f"{r['filt_pitch']:.6f}",f"{r['filt_yaw']:.6f}",
                str(used)]) + '\n')

def save_calibration(hi, si, quality, path, csv_path=None):
    with open(path, 'w') as f:
        f.write("# Magnetometer Calibration Parameters\n")
        f.write("# Generated by mag_cal_cockpit v1\n")
        f.write(f"# Method: {cal_method.upper()}\n")
        f.write(f"# Sensor: generic\n")
        f.write(f"# Units: Gauss\n")
        if csv_path:
            f.write(f"# Raw data: {csv_path}\n")
        f.write(f"# Samples: {quality['n_samples']}\n")
        f.write(f"# Mean radius: {quality['mean_radius']:.6f} Gauss\n")
        f.write(f"# Std radius: {quality['std_radius']:.6f} Gauss\n")
        f.write(f"# Relative std: {quality['relative_std']:.4f}\n")
        f.write(f"# Expected field strength: {get_emfi():.6f} {get_unit_label()}\n")
        f.write("#\n")
        f.write("# EMFI Percentage Analysis:\n")
        raw_pct = quality.get('raw_emfi_pct', {})
        cal_pct = quality.get('cal_emfi_pct', {})
        for t in sorted(raw_pct.keys()):
            f.write(f"# Within ±{t}% of EMFI: Raw {raw_pct[t]:.2f}% → Calibrated {cal_pct.get(t,0):.2f}%\n")
        f.write("\n")
        f.write(f"b = {hi[0]:.12f},{hi[1]:.12f},{hi[2]:.12f}\n")
        f.write(f"# Note: Hard iron values in Gauss\n")
        f.write("A = " + ",".join(f"{x:.12f}" for x in si.flatten()) + "\n")

def save_session(path, hi, si, quality, calibrated):
    """Save full session state for --inspect reload."""
    session = {
        'mag_data': [list(m) for m in mag_data],
        'mag_ts': list(mag_ts),
        'rp_data': [list(r) for r in rp_data],
        'full_rows': full_rows,
        'hard_iron': hi.tolist(),
        'soft_iron': si.tolist(),
        'quality': quality,
        'calibrated': calibrated.tolist(),
        'method': cal_method,
        'unit': mag_unit,
        'emfi_ut': emfi_ut,
        'inclination_deg': inclination_deg,
        'declination_deg': declination_deg,
        'mask': last_cal_mask.tolist() if last_cal_mask is not None else None,
        'mount_config': mount_config,
    }
    class NpEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, np.integer): return int(obj)
            if isinstance(obj, np.floating): return float(obj)
            if isinstance(obj, np.ndarray): return obj.tolist()
            return super().default(obj)
    with open(path, 'w') as f:
        json.dump(session, f, cls=NpEncoder)
    print(f"Session saved: {path}")


def load_session(path):
    """Load session from JSON, populate all globals."""
    global mag_data, mag_ts, rp_data, full_rows, lvm_result, last_cal_mask
    global cal_method, mag_unit, emfi_ut, inclination_deg, declination_deg, mount_config
    with open(path) as f:
        s = json.load(f)
    mag_data[:] = s['mag_data']
    mag_ts[:] = s['mag_ts']
    rp_data[:] = s['rp_data']
    full_rows[:] = s['full_rows']
    cal_method = s.get('method', 'lm')
    mag_unit = s.get('unit', 'gauss')
    emfi_ut = s.get('emfi_ut', 51.1217)
    inclination_deg = s.get('inclination_deg', 66.231)
    declination_deg = s.get('declination_deg', -13.7716)
    mount_config.update(s.get('mount_config', {}))

    hi = np.array(s['hard_iron'])
    si = np.array(s['soft_iron'])
    quality = s['quality']
    calibrated = np.array(s['calibrated'])
    last_cal_mask = np.array(s['mask']) if s.get('mask') is not None else np.ones(len(mag_data), dtype=bool)
    lvm_result = (hi, si, quality, calibrated)

    # Feed all data into RLS estimator
    rls.reset()
    for m in mag_data:
        rls.update(m[0], m[1], m[2])

    print(f"Session loaded: {path} ({len(mag_data)} samples, method={cal_method})")


# ---------------------------------------------------------------------------
# WMMHR
# ---------------------------------------------------------------------------
def get_wmm(lat, lon, alt=0.0):
    global emfi_ut, inclination_deg, declination_deg, INCL_RAD
    try:
        r = subprocess.run(['wmmhr_cli', str(lat), str(lon), str(alt)],
                           capture_output=True, text=True, timeout=5)
        for line in r.stdout.splitlines():
            if 'F  (Total Intensity)' in line:
                emfi_ut = float(line.split(':')[1].split('µT')[0].strip())
                pass  # emfi_ut updated, get_emfi() derives from it
            elif 'Declination:' in line:
                declination_deg = float(line.split(':')[1].split('±')[0].strip())
            elif 'Inclination:' in line:
                inclination_deg = float(line.split(':')[1].split('±')[0].strip())
                INCL_RAD = np.deg2rad(inclination_deg)
    except Exception:
        pass
    return {'emfi':get_emfi(),'emfi_ut':emfi_ut,
            'inclination_deg':inclination_deg,'declination_deg':declination_deg}

# ---------------------------------------------------------------------------
# UDP — MCC protocol: |ts,ax,ay,az,gx,gy,gz,mx,my,mz,roll,pitch,yaw*
# 13 fields, |/* delimited, sensor frame, units defined by broadcaster
# ---------------------------------------------------------------------------
def parse_mcc_packet(s):
    s = s.strip()
    if s.startswith('|'): s = s[1:]
    if s.endswith('*'): s = s[:-1]
    fields = s.split(',')
    if len(fields) < 13: return None
    try:
        f = [float(x) for x in fields[:13]]
    except ValueError:
        return None
    return {'ts':f[0],
        'ax':f[1],'ay':f[2],'az':f[3],
        'gx':f[4],'gy':f[5],'gz':f[6],
        'mx':f[7],'my':f[8],'mz':f[9],
        'filt_roll':f[10],'filt_pitch':f[11],'filt_yaw':f[12]}


class UDPProtocol(asyncio.DatagramProtocol):
    def __init__(self, q):
        self.q = q; self.buf = ""
    def datagram_received(self, data, addr):
        try:
            self.buf += data.decode('ascii', errors='ignore')
            while '|' in self.buf and '*' in self.buf:
                s = self.buf.find('|'); e = self.buf.find('*', s)
                if e == -1: break
                try: self.q.put_nowait(self.buf[s:e+1])
                except asyncio.QueueFull: pass
                self.buf = self.buf[e+1:]
        except Exception: pass

# ---------------------------------------------------------------------------
# Compute diagnostic data for Tab 2
# ---------------------------------------------------------------------------
def compute_diagnostics(mag_arr, rp_arr, hi, si, lvm_hi=None, lvm_si=None):
    """Compute level-frame data, headings, inclination/radius for diagnostics tab.
    hi/si is the RLS estimate. lvm_hi/lvm_si is optional LVM result."""
    centered = mag_arr - hi
    cal = (si @ centered.T).T

    roll = rp_arr[:,0]; pitch = rp_arr[:,1]
    raw_lvl = body_to_level(mag_arr, roll, pitch)
    cal_lvl = body_to_level(cal, roll, pitch)

    # Inclination & radius helpers
    def incl_radius(lvl):
        xy = np.sqrt(lvl[:,0]**2 + lvl[:,1]**2)
        incl = np.degrees(np.arctan2(lvl[:,2], xy))
        radius = np.linalg.norm(lvl, axis=1)
        return incl, radius

    raw_incl, raw_radius = incl_radius(raw_lvl)
    cal_incl, cal_radius = incl_radius(cal_lvl)
    ideal_incl = np.degrees(INCL_RAD)

    # Downsample
    n = len(mag_arr)
    idx = np.arange(n)
    if n > 5000:
        idx = np.linspace(0, n-1, 5000, dtype=int)

    result = {
        'raw_lvl_xy': [raw_lvl[idx,0].tolist(), raw_lvl[idx,1].tolist()],
        'rls_lvl_xy': [cal_lvl[idx,0].tolist(), cal_lvl[idx,1].tolist()],
        'roll_abs': np.abs(np.degrees(roll[idx])).tolist(),
        'pitch_abs': np.abs(np.degrees(pitch[idx])).tolist(),
        'raw_incl_err': (raw_incl[idx] - ideal_incl).tolist(),
        'rls_incl_err': (cal_incl[idx] - ideal_incl).tolist(),
        'raw_radius_err': (raw_radius[idx] - get_emfi()).tolist(),
        'rls_radius_err': (cal_radius[idx] - get_emfi()).tolist(),
        'ring_r': float(get_emfi() * np.cos(INCL_RAD)),
        'ring_z': float(get_emfi() * np.sin(INCL_RAD)),
        'ideal_incl': float(ideal_incl),
        # 3D level-frame points for animation ring view
        'raw_lvl_3d': [raw_lvl[idx,0].tolist(), raw_lvl[idx,1].tolist(), raw_lvl[idx,2].tolist()],
        'rls_lvl_3d': [cal_lvl[idx,0].tolist(), cal_lvl[idx,1].tolist(), cal_lvl[idx,2].tolist()],
    }

    # LVM level-frame if available
    if lvm_hi is not None and lvm_si is not None:
        lvm_cal = (lvm_si @ (mag_arr - lvm_hi).T).T
        lvm_lvl = body_to_level(lvm_cal, roll, pitch)
        lvm_incl, lvm_radius = incl_radius(lvm_lvl)
        result['lm_lvl_xy'] = [lvm_lvl[idx,0].tolist(), lvm_lvl[idx,1].tolist()]
        result['lm_incl_err'] = (lvm_incl[idx] - ideal_incl).tolist()
        result['lm_radius_err'] = (lvm_radius[idx] - get_emfi()).tolist()
        result['lm_lvl_3d'] = [lvm_lvl[idx,0].tolist(), lvm_lvl[idx,1].tolist(), lvm_lvl[idx,2].tolist()]

    return result

# ---------------------------------------------------------------------------
# Broadcast
# ---------------------------------------------------------------------------
async def bcast(obj):
    global clients
    if not clients: return
    msg = json.dumps(obj)
    dead = set()
    for ws in list(clients):
        try: await ws.send_text(msg)
        except Exception: dead.add(ws)
    clients -= dead

# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------
app = FastAPI()

static_dir = Path(__file__).parent
app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

@app.get("/")
async def root():
    return FileResponse(str(static_dir / "index.html"), headers={"Cache-Control": "no-cache, no-store, must-revalidate"})

@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    has_mount = any(mount_config[k] != 0 for k in mount_config)
    await websocket.send_text(json.dumps({'type':'config', **mount_config, 'has_mount':has_mount}))
    # Send accumulated points on reconnect
    if all_display_pts:
        await websocket.send_text(json.dumps({'type':'points','points':all_display_pts,
                                              'n_logged':len(mag_data),'logging':logging_active}))
    # If calibration exists (inspect mode or post-calibrate), re-run calibrate to populate all tabs
    if lvm_result is not None:
        await handle_msg(websocket, json.dumps({'cmd':'calibrate'}))
    try:
        while True:
            message = await websocket.receive_text()
            await handle_msg(websocket, message)
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
    finally:
        clients.discard(websocket)


async def handle_msg(ws, message):
    global logging_active, mag_data, mag_ts, rp_data, full_rows, lvm_result, last_cal_mask, gyro_threshold, cal_method, mag_unit
    try:
        msg = json.loads(message)
    except json.JSONDecodeError:
        return

    cmd = msg.get('cmd')

    if cmd in ('start', 'resume'):
        logging_active = True
        await bcast({'type':'status','logging':True})

    elif cmd == 'pause':
        logging_active = False
        await bcast({'type':'status','logging':False})

    elif cmd == 'reset':
        logging_active = False
        lvm_result = None
        mag_data.clear(); mag_ts.clear(); rp_data.clear(); full_rows.clear(); all_display_pts.clear()
        rls.reset()
        await bcast({'type':'status','logging':False})
        await bcast({'type':'reset'})

    elif cmd == 'calibrate':
        if len(mag_data) < 50:
            await bcast({'type':'cal_result','error':'Need at least 50 samples'})
            return

        # Region filtering: list of {start_t, end_t} relative to first sample
        regions = msg.get('regions')  # None = use all data
        ts_arr = np.array(mag_ts)
        t0 = ts_arr[0] if len(ts_arr) > 0 else 0
        ts_rel = ts_arr - t0

        if regions and len(regions) > 0:
            mask = np.zeros(len(mag_data), dtype=bool)
            for r in regions:
                mask |= (ts_rel >= r['start']) & (ts_rel <= r['end'])
        else:
            mask = np.ones(len(mag_data), dtype=bool)

        # Roll/pitch magnitude filter
        rp_filter = msg.get('rp_filter')
        if rp_filter:
            all_rp = np.array(rp_data)
            roll_deg = np.abs(np.degrees(all_rp[:, 0]))
            pitch_deg = np.abs(np.degrees(all_rp[:, 1]))
            roll_max = rp_filter.get('roll_max', 90)
            pitch_max = rp_filter.get('pitch_max', 90)
            mask &= (roll_deg <= roll_max) & (pitch_deg <= pitch_max)

        if mask.sum() < 50:
            await bcast({'type':'cal_result','error':f'Only {mask.sum()} samples after filtering (need 50)'})
            return

        arr = np.array(mag_data)[mask]
        rp_arr = np.array(rp_data)[mask]
        try:
            if cal_method == 'lse':
                hi, si, quality, calibrated = calibrate_lse(arr, get_emfi())
            else:
                hi, si, quality, calibrated = calibrate_lm(arr, get_emfi())
            lvm_result = (hi, si, quality, calibrated)
            last_cal_mask = mask

            # Histograms
            raw_r = np.linalg.norm(arr, axis=1)
            cal_r = np.linalg.norm(calibrated, axis=1)
            raw_err = (raw_r - get_emfi())/get_emfi()*100
            cal_err = (cal_r - get_emfi())/get_emfi()*100

            # Downsample for display (max 5000)
            disp_idx = np.arange(len(arr))
            if len(arr) > 5000:
                disp_idx = np.linspace(0, len(arr)-1, 5000, dtype=int)

            has_mount = any(mount_config[k] != 0 for k in mount_config)
            R_inv = get_mount_inv() if has_mount else None

            raw_disp = arr[disp_idx]
            cal_disp = calibrated[disp_idx]

            # LSE intermediate for animation (raw→LSE→LM)
            lse_disp = None
            if cal_method == 'lm':
                lse_hi, lse_si = _lse_init(arr, get_emfi())
                lse_cal = (lse_si @ (arr - lse_hi).T).T
                lse_disp = lse_cal[disp_idx]

            if has_mount:
                raw_disp = (R_inv @ raw_disp.T).T
                cal_disp = (R_inv @ cal_disp.T).T
                if lse_disp is not None:
                    lse_disp = (R_inv @ lse_disp.T).T

            # Diagnostics
            rls_hi_tmp, rls_si_tmp = rls.get_hard_soft(get_emfi())
            diag = compute_diagnostics(arr, rp_arr, rls_hi_tmp, rls_si_tmp, lvm_hi=hi, lvm_si=si)

            # LSE level-frame ring for animation
            if lse_disp is not None:
                lse_hi_anim, lse_si_anim = _lse_init(arr, get_emfi())
                lse_cal_all = (lse_si_anim @ (arr - lse_hi_anim).T).T
                lse_lvl = body_to_level(lse_cal_all, rp_arr[:,0], rp_arr[:,1])
                diag_idx = np.arange(len(arr))
                if len(diag_idx) > 5000:
                    diag_idx = np.linspace(0, len(arr)-1, 5000, dtype=int)
                diag['lse_lvl_3d'] = [lse_lvl[diag_idx,0].tolist(), lse_lvl[diag_idx,1].tolist(), lse_lvl[diag_idx,2].tolist()]

            # RLS for comparison histogram
            rls_cal = (rls_si_tmp @ (arr - rls_hi_tmp).T).T
            rls_r = np.linalg.norm(rls_cal, axis=1)
            rls_err = (rls_r - get_emfi())/get_emfi()*100

            # LM MFI on ALL data (not just filtered) for time-series plot
            all_arr = np.array(mag_data)
            all_lm_cal = (si @ (all_arr - hi).T).T
            all_lm_r = np.linalg.norm(all_lm_cal, axis=1)
            all_ts = np.array(mag_ts)
            all_t0 = all_ts[0] if len(all_ts) > 0 else 0
            mfi_idx2 = np.arange(len(all_ts))
            if len(mfi_idx2) > 2000:
                mfi_idx2 = np.linspace(0, len(all_ts)-1, 2000, dtype=int)

            # Per-point metadata for precision inspection (same disp_idx)
            ts_filt = np.array(mag_ts)[mask]
            t0_filt = ts_filt[0] if len(ts_filt) > 0 else 0
            pt_meta = {
                'time': (ts_filt[disp_idx] - t0_filt).tolist(),
                'roll_deg': np.degrees(rp_arr[disp_idx, 0]).tolist(),
                'pitch_deg': np.degrees(rp_arr[disp_idx, 1]).tolist(),
                'raw_mag': np.linalg.norm(arr[disp_idx], axis=1).tolist(),
                'cal_mag': np.linalg.norm(calibrated[disp_idx], axis=1).tolist(),
                'cal_err_pct': ((np.linalg.norm(calibrated[disp_idx], axis=1) - get_emfi()) / get_emfi() * 100).tolist(),
            }

            cal_msg = {
                'type':'cal_result', 'method':cal_method,
                'hard_iron':hi.tolist(), 'soft_iron':si.tolist(), 'quality':quality,
                'raw_points':raw_disp.tolist(),
                'calibrated':cal_disp.tolist(),
                'pt_meta':pt_meta,}
            if lse_disp is not None:
                cal_msg['lse_points'] = lse_disp.tolist()
            await bcast({**cal_msg,
                'raw_hist':np.histogram(raw_err, bins=(hb:=make_hist_bins(raw_err, cal_err, rls_err))[0])[0].tolist(),
                'rls_hist':np.histogram(rls_err, bins=hb[0])[0].tolist(),
                'lm_hist':np.histogram(cal_err, bins=hb[0])[0].tolist(),
                'bin_centers':hb[1],
                'diagnostics':diag,
                'mfi_lm': all_lm_r[mfi_idx2].tolist(),
                'mfi_t': (all_ts[mfi_idx2] - all_t0).tolist(),
            })
        except Exception as e:
            await bcast({'type':'cal_result','error':str(e)})

    elif cmd == 'export':
        if lvm_result is None:
            await bcast({'type':'export_result','error':'Run calibrate first'})
            return
        try:
            hi, si, quality, cal = lvm_result
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            out_dir = Path(__file__).parent
            csv_file = out_dir / f'mcc_raw_{ts}.csv'
            dat_file = out_dir / 'mag_cal.dat'
            session_file = out_dir / f'mcc_session_{ts}.json'
            save_csv_log(full_rows, csv_file, mask=last_cal_mask)
            save_calibration(hi, si, quality, dat_file, csv_path=str(csv_file))
            save_session(session_file, hi, si, quality, cal)
            await bcast({'type':'export_result','file':str(dat_file),'csv_file':str(csv_file),'session_file':str(session_file)})
        except Exception as e:
            print(f"Export error: {e}")
            await bcast({'type':'export_result','error':str(e)})

    elif cmd == 'geolocation':
        result = get_wmm(msg.get('lat',42.357), msg.get('lon',-71.087))
        await bcast({'type':'wmm', **result})

    elif cmd == 'set_gyro_threshold':
        gyro_threshold = float(msg.get('value', 0.05))

    elif cmd == 'set_method':
        val = msg.get('value', 'lm').lower()
        if val in ('lm', 'lse'):
            cal_method = val
            await bcast({'type':'method_changed','method':cal_method})

    elif cmd == 'set_unit':
        unit = msg.get('value', 'gauss').lower()
        if unit in UNIT_SCALE:
            mag_unit = unit
            await bcast({'type':'unit_changed','unit':mag_unit,'label':get_unit_label(),'emfi':get_emfi()})


# ---------------------------------------------------------------------------
# Packet processor
# ---------------------------------------------------------------------------
async def process_packets(packet_queue):
    global mag_data, rp_data, full_rows, all_display_pts
    last_update = time.time()
    last_rls = time.time()
    pending = []
    has_mount = any(mount_config[k] != 0 for k in mount_config)
    R_inv = get_mount_inv() if has_mount else None

    while True:
        try:
            sentence = await asyncio.wait_for(packet_queue.get(), timeout=0.05)
        except asyncio.TimeoutError:
            now = time.time()
            if pending and now - last_update >= 0.066:
                last_update = now
                await bcast({'type':'points','points':pending,
                             'n_logged':len(mag_data),'logging':logging_active})
                pending = []
            continue

        parsed = parse_mcc_packet(sentence)
        if parsed is None: continue

        mx, my, mz = parsed['mx'], parsed['my'], parsed['mz']
        gx, gy, gz = parsed['gx'], parsed['gy'], parsed['gz']
        gyro_mag = float(np.sqrt(gx**2 + gy**2 + gz**2))
        in_motion = gyro_mag > gyro_threshold

        # EMFI % error for coloring
        mag_norm = float(np.sqrt(mx**2 + my**2 + mz**2))
        emfi_err = (mag_norm - get_emfi()) / get_emfi() * 100 if get_emfi() > 0 else 0

        pt = {'mx':mx,'my':my,'mz':mz,'in_motion':bool(in_motion),'gyro_mag':gyro_mag,'err':round(emfi_err,2),
              'roll':parsed['filt_roll'],'pitch':parsed['filt_pitch'],'ts':parsed['ts']}
        if has_mount:
            v = R_inv @ np.array([mx, my, mz])
            pt['vx'],pt['vy'],pt['vz'] = float(v[0]),float(v[1]),float(v[2])

        pending.append(pt)
        all_display_pts.append(pt)

        if logging_active and in_motion:
            mag_data.append([mx, my, mz])
            mag_ts.append(parsed['ts'])
            rp_data.append([parsed['filt_roll'], parsed['filt_pitch'], parsed['filt_yaw']])
            full_rows.append(parsed)
            rls.update(mx, my, mz)

        now = time.time()

        if now - last_update >= 0.066:
            last_update = now
            await bcast({'type':'points','points':pending,
                         'n_logged':len(mag_data),'logging':logging_active})
            pending = []

        # RLS display at 1 Hz
        if now - last_rls >= 1.0 and rls.n >= 20:
            last_rls = now
            try:
                hi, si = rls.get_hard_soft(get_emfi())
                arr = np.array(mag_data)
                rp_arr = np.array(rp_data)
                centered = arr - hi
                cal = (si @ centered.T).T
                raw_r = np.linalg.norm(arr, axis=1)
                cal_r = np.linalg.norm(cal, axis=1)
                raw_err = (raw_r - get_emfi())/get_emfi()*100
                cal_err = (cal_r - get_emfi())/get_emfi()*100

                cal_disp = cal
                if len(cal_disp) > 5000:
                    idx = np.linspace(0, len(cal_disp)-1, 5000, dtype=int)
                    cal_disp = cal_disp[idx]
                if has_mount:
                    cal_disp = (R_inv @ cal_disp.T).T

                # Compute diagnostics for tab 2, include LVM if available
                lvm_hi_d = lvm_result[0] if lvm_result else None
                lvm_si_d = lvm_result[1] if lvm_result else None
                diag = compute_diagnostics(arr, rp_arr, hi, si, lvm_hi=lvm_hi_d, lvm_si=lvm_si_d)

                # MFI time-series (downsample to max 2000 for transport)
                ts_arr = np.array(mag_ts)
                mfi_idx = np.arange(len(ts_arr))
                if len(mfi_idx) > 2000:
                    mfi_idx = np.linspace(0, len(ts_arr)-1, 2000, dtype=int)
                # Relative time from first sample
                t0 = ts_arr[0] if len(ts_arr) > 0 else 0
                mfi_t = (ts_arr[mfi_idx] - t0).tolist()
                mfi_raw = raw_r[mfi_idx].tolist()
                mfi_rls = cal_r[mfi_idx].tolist()
                mfi_roll = np.abs(np.degrees(rp_arr[mfi_idx, 0])).tolist()
                mfi_pitch = np.abs(np.degrees(rp_arr[mfi_idx, 1])).tolist()

                msg_out = {
                    'type':'rls_update',
                    'hard_iron':hi.tolist(), 'soft_iron':si.tolist(),
                    'mean_radius':float(np.mean(cal_r)),
                    'std_radius':float(np.std(cal_r)),
                    'relative_std':float(np.std(cal_r)/np.mean(cal_r)),
                    'calibrated':cal_disp.tolist(),
                    'raw_hist':np.histogram(raw_err, bins=(hb2:=make_hist_bins(raw_err, cal_err))[0])[0].tolist(),
                    'rls_hist':np.histogram(cal_err, bins=hb2[0])[0].tolist(),
                    'bin_centers':hb2[1],
                    'emfi':get_emfi(),
                    'diagnostics':diag,
                    'mfi_t':mfi_t, 'mfi_raw':mfi_raw, 'mfi_rls':mfi_rls, 'mfi_roll':mfi_roll, 'mfi_pitch':mfi_pitch,
                    'ts_range':[float(t0), float(ts_arr[-1])] if len(ts_arr)>0 else [0,0],
                }
                # Include LVM histogram if we have a calibration result
                if lvm_result is not None:
                    lvm_hi, lvm_si, _, _ = lvm_result
                    lvm_cal = (lvm_si @ (arr - lvm_hi).T).T
                    lvm_r = np.linalg.norm(lvm_cal, axis=1)
                    lvm_err = (lvm_r - get_emfi())/get_emfi()*100
                    msg_out['lm_hist'] = np.histogram(lvm_err, bins=hb2[0])[0].tolist()
                await bcast(msg_out)
            except Exception as e:
                print(f"RLS error: {e}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description='MCC — Mag Cal Cockpit')
    p.add_argument('--udp-port', type=int, default=50100)
    p.add_argument('--udp-addr', default='0.0.0.0')
    p.add_argument('--web-port', type=int, default=8080)
    p.add_argument('--no-browser', action='store_true')
    p.add_argument('--mount-roll', type=float, default=0.0)
    p.add_argument('--mount-pitch', type=float, default=0.0)
    p.add_argument('--mount-yaw', type=float, default=0.0)
    p.add_argument('--inspect', type=str, default=None, help='Load session JSON for offline inspection')
    args = p.parse_args()

    global mount_config
    mount_config = {'mount_roll':args.mount_roll,'mount_pitch':args.mount_pitch,'mount_yaw':args.mount_yaw}

    inspect_mode = args.inspect is not None

    if inspect_mode:
        load_session(args.inspect)
        # Override mount config from session if not explicitly set on CLI
        if args.mount_roll == 0 and args.mount_pitch == 0 and args.mount_yaw == 0:
            pass  # use session's mount_config (already loaded)
        else:
            mount_config = {'mount_roll':args.mount_roll,'mount_pitch':args.mount_pitch,'mount_yaw':args.mount_yaw}

    packet_queue = asyncio.Queue(maxsize=2000)

    @app.on_event("startup")
    async def startup():
        if not inspect_mode:
            loop = asyncio.get_event_loop()
            transport, _ = await loop.create_datagram_endpoint(
                lambda: UDPProtocol(packet_queue),
                local_addr=(args.udp_addr, args.udp_port))
            print(f"UDP: {args.udp_addr}:{args.udp_port}")
            asyncio.create_task(process_packets(packet_queue))
        if not args.no_browser:
            webbrowser.open(f'http://localhost:{args.web_port}')

    print(f"MCC — Mag Cal Cockpit on http://localhost:{args.web_port}")
    if inspect_mode:
        print(f"Inspect mode: {args.inspect}")
    uvicorn.run(app, host='0.0.0.0', port=args.web_port, log_level='warning')


if __name__ == '__main__':
    main()
