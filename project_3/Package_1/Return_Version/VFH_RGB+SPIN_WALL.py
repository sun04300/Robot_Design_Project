import os
import atexit
import signal
import sys
import math
import threading
import serial
import time
import cv2
import numpy as np

from color_v2 import (ColorDetector, load_calibration,
                        get_red_mask, get_yellow_mask, get_blue_mask)

PORT_ARDU  = "/dev/ttyS0"
PORT_LIDAR = "/dev/ttyUSB0"
CAM_INDEX  = 0
CAM_W, CAM_H = 640, 480
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "camera_calibration.pkl")

PAPER_W_MM = 300.0
PAPER_H_MM = 300.0

# 주행 파라미터
MAX_STEER       = 1.0
SPEED_FAR       = 0.70
SPEED_NEAR      = 0.45
DIST_SLOW_MM    = 150.0
AREA_PEAK_THRES = 0.08
STEER_GAIN      = 0.015
CONFIRM_FRAMES  = 4
STOP_DURATION   = 1.0
ENTRY_STOP_MM   = 250.0

WEAK_MIN_AREA      = 200
WEAK_SPEED         = 0.55
WEAK_STEER_GAIN    = 0.60
NUDGE_SPEED        = 0.30
COLOR_MEMORY_TIME  = 0.30
STEER_SMOOTH_ALPHA = 0.40

ALIGN_STEER_ENTER = 0.30
ALIGN_STEER_EXIT  = 0.10
ALIGN_MIN_DIST    = 400.0

# 360° 스캔 파라미터
SPIN_SCAN_SPEED = 0.28    # 제자리 회전 속도
SPIN_SCAN_TIME  = 6.0     # 360° 완료 시간(s) — 실측 후 조정
SPIN_SCAN_DIR   = 1.0     # +1 = 우측 회전

# 벽 추종 파라미터
WALL_TARGET_DIST  = 330.0   # 목표 벽 거리 (mm)
WALL_FOLLOW_SPEED = 0.50    # 벽 추종 전진 속도
WALL_KP           = 0.0030  # 거리 오차 → 조향 비례 게인
WALL_FRONT_THRESH = 450.0   # 전방 장애물 회전 임계값 (mm)
WALL_TURN_STEER   = 0.80    # 전방 막힘 시 조향값
WALL_SIDE         = 'left'  # 추종 벽 방향 ('left' 또는 'right')

TARGETS = ['red', 'yellow', 'blue']
DEBUG   = True

# LiDAR VFH 파라미터
BIN_DEG       = 4.0
N_BINS        = int(360 / BIN_DEG)
GAP_MIN_PASS  = 80.0
DETECT        = 750.0
VELO_DOWN     = 600.0
EMERGENCY     = 210.0
LID_MAX_STEER = 1.2
ROT_THRESH    = 100.0
ROBOT_RADIUS  = 80.0

_lidar_lock  = threading.Lock()
_lidar_state = {
    'has_data'    : False,
    'emg_near'    : 9999.0,
    'front_near'  : 9999.0,
    'left_dist'   : 9999.0,
    'right_dist'  : 9999.0,
    'vfh_action'  : 'FWD',
    'vfh_steer'   : 0.0,
    'vfh_speed'   : 0.65,
    'rot_dir'     : 1.0,
    'goal_bearing': None,
}

_display_lock  = threading.Lock()
_display_frame = [None]
_quit_flag     = [False]


# ── LiDAR 히스토그램 / VFH ────────────────────────────────────────────

def _build_hist(scan_buf):
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt


def _nearest(hist, has_pt, center_cw, arc_half=55):
    cb = int(center_cw / BIN_DEG) % N_BINS
    nc = max(1, int(arc_half / BIN_DEG))
    md = 9999.0
    for k in range(-nc, nc + 1):
        idx = (cb + k) % N_BINS
        if has_pt[idx] and hist[idx] < md:
            md = hist[idx]
    return md


def _find_gaps(hist, has_pt):
    blocked = [has_pt[i] and hist[i] <= DETECT for i in range(N_BINS)]
    smoothed = blocked[:]
    for i in range(N_BINS):
        if blocked[i] and not blocked[(i-1)%N_BINS] and not blocked[(i+1)%N_BINS]:
            smoothed[i] = False
    blocked = smoothed
    inflated = blocked[:]
    for i in range(N_BINS):
        if blocked[i] and hist[i] < 9999.0:
            ar = math.asin(min(1.0, ROBOT_RADIUS / max(hist[i], ROBOT_RADIUS)))
            ab = int(math.degrees(ar) / BIN_DEG) + 1
            for k in range(-ab, ab + 1):
                inflated[(i + k) % N_BINS] = True
    blocked = inflated
    gaps = []; seen = set(); i = 0
    while i < 2 * N_BINS:
        if not blocked[i % N_BINS]:
            j = i + 1
            while j < i + N_BINS and not blocked[j % N_BINS]:
                j += 1
            span = j - i
            if span < N_BINS:
                ccw = ((i + j) / 2.0 * BIN_DEG) % 360.0
                ck  = round(ccw)
                if ck not in seen:
                    seen.add(ck)
                    dg = span * BIN_DEG
                    dL = min(hist[(i-1)%N_BINS] if has_pt[(i-1)%N_BINS] else DETECT, DETECT)
                    dR = min(hist[j%N_BINS]     if has_pt[j%N_BINS]     else DETECT, DETECT)
                    gw = (dL + dR) * math.sin(math.radians(dg / 2.0))
                    dp = min(hist[k%N_BINS] for k in range(i, j))
                    cs = ccw if ccw <= 180.0 else ccw - 360.0
                    gaps.append({'center': cs, 'center_cw': ccw, 'width': gw,
                                 'passable': gw >= GAP_MIN_PASS, 'delta_deg': dg,
                                 'd_L': dL, 'd_R': dR, 'depth': dp})
            i = j
        else:
            i += 1
    return gaps


def _best_gap(gaps, goal_bearing=None):
    if not gaps:
        return None
    pool = [g for g in gaps if g['passable']] or gaps
    ref  = 0.0 if goal_bearing is None else goal_bearing
    return max(pool, key=lambda g: g['width']*0.3 - abs(g['center']-ref)*1.6
                                    + min(g['depth'], DETECT)/DETECT*25.0)


def _compute_vfh(hist, has_pt, goal_bearing=None):
    emg   = _nearest(hist, has_pt, 0.0, arc_half=80)
    front = _nearest(hist, has_pt, 0.0, arc_half=40)

    if not any(has_pt):
        return 'FWD', 0.0, 0.70, 1.0, emg, front

    gaps = _find_gaps(hist, has_pt)
    best = _best_gap(gaps, goal_bearing)

    if best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
        imb  = (best['d_R'] - best['d_L']) / (best['d_L'] + best['d_R'] + 1e-9)
        bias = imb * (best['delta_deg'] / 2.9)
        WR   = 150.0
        lL   = _nearest(hist, has_pt, 270.0, arc_half=45)
        lR   = _nearest(hist, has_pt,  90.0, arc_half=45)
        rep  = (max(0.0, WR-lL)/WR - max(0.0, WR-lR)/WR) * 20.0
        CR   = 350.0
        cL   = _nearest(hist, has_pt, 320.0, arc_half=25)
        cR   = _nearest(hist, has_pt,  40.0, arc_half=25)
        crn  = (max(0.0, CR-cL)/CR - max(0.0, CR-cR)/CR) * 45.0
        tgt  = best['center'] + bias + rep + crn
        nd   = _nearest(hist, has_pt, best['center_cw'], arc_half=35)
        rt   = min(max((VELO_DOWN-nd)/(VELO_DOWN-EMERGENCY), 0.0), 1.0)
        st   = max(-LID_MAX_STEER, min(LID_MAX_STEER,
                   tgt * (1.0+rt*0.5) / 90.0 * LID_MAX_STEER))
        spd  = 0.85 * (1.0 - rt * 0.55)
        return 'FWD', float(st), float(spd), 1.0, emg, front

    FARC = 60.0
    if gaps:
        if goal_bearing is not None:
            td = min(gaps, key=lambda g: abs(g['center']-goal_bearing))['center']
            td = max(-90.0, min(90.0, td))
        else:
            fg = [g for g in gaps if abs(g['center']) <= FARC]
            td = max(fg, key=lambda g: g['width'])['center'] if fg \
                 else max(-FARC, min(FARC, max(gaps, key=lambda g: g['width'])['center']))
    else:
        td = 0.0
    st = max(-LID_MAX_STEER, min(LID_MAX_STEER, td/90.0*LID_MAX_STEER*0.5))
    return 'FWD', float(st), 0.40, 1.0, emg, front


def _lidar_worker(ser_l):
    scan_buf = []
    while True:
        try:
            data = ser_l.read(5)
            if len(data) != 5:
                continue
            s_flag     = data[0] & 0x01
            s_inv_flag = (data[0] & 0x02) >> 1
            if s_inv_flag != (1 - s_flag):
                continue
            if (data[1] & 0x01) != 1:
                continue
            quality  = data[0] >> 2
            angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
            distance = (data[3] | (data[4] << 8)) / 4.0
            if quality == 0 or distance < 80:
                continue
            scan_buf.append((angle, distance))
            if s_flag == 1 and scan_buf:
                hist, has_pt = _build_hist(scan_buf)
                with _lidar_lock:
                    gb = _lidar_state['goal_bearing']
                act, st, spd, rd, emg, front = _compute_vfh(hist, has_pt, gb)
                left_d  = _nearest(hist, has_pt, 270.0, arc_half=40)
                right_d = _nearest(hist, has_pt,  90.0, arc_half=40)
                with _lidar_lock:
                    _lidar_state['has_data']   = True
                    _lidar_state['emg_near']   = emg
                    _lidar_state['front_near'] = front
                    _lidar_state['left_dist']  = left_d
                    _lidar_state['right_dist'] = right_d
                    _lidar_state['vfh_action'] = act
                    _lidar_state['vfh_steer']  = st
                    _lidar_state['vfh_speed']  = spd
                    _lidar_state['rot_dir']    = rd
                scan_buf = []
        except Exception as e:
            print(f"[LIDAR] {e}")
            scan_buf = []


def _lidar_read():
    with _lidar_lock:
        return dict(_lidar_state)


def _speed_limit(cam_speed: float) -> float:
    ls = _lidar_read()
    if not ls['has_data']:
        return cam_speed
    front = ls['front_near']
    if front <= EMERGENCY:
        return 0.0
    if front < VELO_DOWN:
        rt = max(0.0, min(1.0, (VELO_DOWN - front) / (VELO_DOWN - EMERGENCY)))
        return cam_speed * (1.0 - rt * 0.45)
    return cam_speed


def _wall_follow_cmd(ls):
    """벽 추종 조향·속도 계산 → (steer, speed)."""
    if not ls['has_data']:
        return 0.0, 0.0

    front_dist = ls['front_near']

    if front_dist <= EMERGENCY:
        return 0.0, 0.0

    # 전방 장애물 → 벽 반대 방향 회전
    if front_dist < WALL_FRONT_THRESH:
        steer = WALL_TURN_STEER if WALL_SIDE == 'left' else -WALL_TURN_STEER
        return float(steer), WALL_FOLLOW_SPEED * 0.5

    wall_dist = ls['left_dist'] if WALL_SIDE == 'left' else ls['right_dist']
    error = wall_dist - WALL_TARGET_DIST
    # 좌벽: 멀면 좌(음수 조향), 가까우면 우(양수 조향)
    steer = -WALL_KP * error if WALL_SIDE == 'left' else WALL_KP * error
    return float(np.clip(steer, -MAX_STEER, MAX_STEER)), WALL_FOLLOW_SPEED


# ── 종이 자세 추정 ────────────────────────────────────────────────────

_OBJ_PTS = np.array([
    [0,          0,          0],
    [PAPER_W_MM, 0,          0],
    [PAPER_W_MM, PAPER_H_MM, 0],
    [0,          PAPER_H_MM, 0],
], dtype=np.float32)


def _order_points(pts):
    s    = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(s)], pts[np.argmin(diff)],
                     pts[np.argmax(s)], pts[np.argmax(diff)]], dtype=np.float32)


def _extract_quad(contour):
    hull = cv2.convexHull(contour)
    peri = cv2.arcLength(hull, True)
    best = hull.reshape(-1, 2).astype(np.float32)
    for eps_ratio in np.arange(0.01, 0.40, 0.01):
        approx = cv2.approxPolyDP(hull, float(eps_ratio) * peri, True)
        pts = approx.reshape(-1, 2).astype(np.float32)
        if len(pts) == 4:
            return _order_points(pts)
        if len(pts) < 4:
            break
        best = pts
    while len(best) > 4:
        dists = [np.linalg.norm(best[i] - best[(i + 1) % len(best)])
                 for i in range(len(best))]
        idx = int(np.argmin(dists))
        nxt = (idx + 1) % len(best)
        best[idx] = (best[idx] + best[nxt]) / 2
        best = np.delete(best, nxt, axis=0)
    if len(best) < 4:
        best = cv2.boxPoints(cv2.minAreaRect(contour)).astype(np.float32)
    return _order_points(best)


def solve_paper_pose(contour, cam_mat, dist_coeffs):
    if cam_mat is None:
        return None
    quad_pts = _extract_quad(contour)
    try:
        ok, rvec, tvec = cv2.solvePnP(_OBJ_PTS, quad_pts, cam_mat, dist_coeffs,
                                       flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    center_obj = np.array([[PAPER_W_MM/2], [PAPER_H_MM/2], [0.0]], dtype=np.float64)
    center_cam = R @ center_obj + tvec
    z_mm = float(center_cam[2])
    x_mm = float(center_cam[0])
    if not (np.isfinite(z_mm) and np.isfinite(x_mm) and z_mm > 0):
        return None
    angle = np.degrees(np.arctan2(x_mm, z_mm))
    steer = float(np.clip(angle * STEER_GAIN, -MAX_STEER, MAX_STEER))
    return z_mm, x_mm, steer, quad_pts


# ── 색상 탐지 보조 ────────────────────────────────────────────────────

def _get_mask(hsv, color):
    if color == 'red':    return get_red_mask(hsv)
    if color == 'yellow': return get_yellow_mask(hsv)
    return get_blue_mask(hsv)


def get_weak_contour(hsv, color):
    mask = _get_mask(hsv, color)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) > WEAK_MIN_AREA]
    return max(cnts, key=cv2.contourArea) if cnts else None


def _contour_offset(cnt, frame_w):
    M = cv2.moments(cnt)
    if M['m00'] == 0:
        return 0.0
    return (M['m10'] / M['m00'] - frame_w / 2) / (frame_w / 2)


def _draw_center(vis, cx, cy, color):
    cv2.circle(vis, (cx, cy), 6, color, -1)
    cv2.line(vis, (cx - 15, cy), (cx + 15, cy), color, 1)
    cv2.line(vis, (cx, cy - 15), (cx, cy + 15), color, 1)


def _display_worker():
    while True:
        with _display_lock:
            frame = _display_frame[0]
        if frame is not None:
            cv2.imshow('Robot View', frame)
            if (cv2.waitKey(1) & 0xFF) == ord('q'):
                _quit_flag[0] = True
        else:
            time.sleep(0.005)


# ── 메인 ─────────────────────────────────────────────────────────────

def main():
    ser   = serial.Serial(PORT_ARDU,  460800, timeout=1)
    ser_l = serial.Serial(PORT_LIDAR, 460800, timeout=1)

    ser_l.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    ser_l.write(bytes([0xA5, 0x20]))

    t = threading.Thread(target=_lidar_worker, args=(ser_l,), daemon=True)
    t.start()

    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    cam_mat, dist_coeffs, calib_res = load_calibration(CALIB_FILE)
    if calib_res and calib_res != (fw, fh):
        print(f"[경고] 캘리브 해상도 불일치({calib_res} vs {fw}×{fh}) → PnP 비활성화")
        cam_mat = dist_coeffs = None

    detector = ColorDetector(frame_w=fw, frame_h=fh,
                             camera_matrix=cam_mat, dist_coeffs=dist_coeffs)

    if detector._map1 is not None:
        pnp_mat  = detector._new_mtx
        pnp_dist = np.zeros((4, 1), dtype=np.float64)
    else:
        pnp_mat  = cam_mat
        pnp_dist = dist_coeffs if dist_coeffs is not None else np.zeros((4, 1))

    disp_t = threading.Thread(target=_display_worker, daemon=True)
    disp_t.start()

    def _cleanup():
        try:
            ser.write(b"S\n"); time.sleep(0.1)
            ser_l.write(bytes([0xA5, 0x25])); time.sleep(0.1)
            cap.release(); cv2.destroyAllWindows()
            ser.close(); ser_l.close()
        except Exception:
            pass
    atexit.register(_cleanup)

    def _sig(_s, _f):
        _cleanup(); sys.exit(0)
    signal.signal(signal.SIGINT,  _sig)
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGTSTP, _sig)

    # ── 상태 변수 ─────────────────────────────────────────────────────
    target_idx         = 0
    state              = 'SPIN_SCAN'   # 시작 시 즉시 360° 스캔
    spin_start         = time.time()
    on_zone_count      = 0
    stop_start         = None
    last_seen          = time.time()
    last_steer         = 0.0
    smoothed_steer     = 0.0
    align_mode         = False
    area_peak_seen     = False
    peak_area_r        = 0.0
    peak_confirm_count = 0
    avoiding           = False

    # ── 메인 루프 ─────────────────────────────────────────────────────
    while True:
        ret, raw = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        # ── DONE ──────────────────────────────────────────────────────
        if state == 'DONE':
            ser.write(b"S\n")
            vis = raw.copy()
            cv2.putText(vis, "MISSION COMPLETE", (fw//2-120, fh//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
            with _display_lock:
                _display_frame[0] = vis
            time.sleep(0.1)
            continue

        ls     = _lidar_read()
        result = detector.detect(raw)
        color  = TARGETS[target_idx]
        vis    = detector.draw_debug(raw, result)
        hsv_u  = cv2.cvtColor(result['undistorted'], cv2.COLOR_BGR2HSV)
        det    = result.get(color, {})

        # ── STOP ──────────────────────────────────────────────────────
        if state == 'STOP':
            ser.write(b"S\n")
            elapsed = time.time() - stop_start
            remain  = max(0.0, STOP_DURATION - elapsed)
            cv2.putText(vis, f"STOP {color.upper()}  {remain:.1f}s",
                        (fw//2-90, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 255), 2)
            with _display_lock:
                _display_frame[0] = vis
            if elapsed >= STOP_DURATION:
                if target_idx < len(TARGETS) - 1:
                    target_idx        += 1
                    state              = 'SPIN_SCAN'
                    spin_start         = time.time()
                    on_zone_count      = 0
                    area_peak_seen     = False
                    peak_area_r        = 0.0
                    peak_confirm_count = 0
                    align_mode         = False
                    avoiding           = False
                    last_seen          = time.time()
                    last_steer         = 0.0
                    smoothed_steer     = 0.0
                    with _lidar_lock:
                        _lidar_state['goal_bearing'] = None
                    print(f"  ✅ {TARGETS[target_idx-1].upper()} 완료 → {TARGETS[target_idx].upper()} 탐색")
                else:
                    state = 'DONE'
                    print("  ✅ 전체 미션 완료!")
            continue

        # ── SPIN_SCAN ─────────────────────────────────────────────────
        # 360° 회전하며 색상 탐색. 발견 → SEEK, 완료 미발견 → WALL_FOLLOW.
        # 색 발견 시 continue 하지 않고 이 프레임에서 바로 SEEK 블록으로 낙하.
        if state == 'SPIN_SCAN':
            weak_cnt     = get_weak_contour(hsv_u, color)
            color_found  = det.get('found') or (weak_cnt is not None)
            elapsed_spin = time.time() - spin_start

            if color_found:
                state = 'SEEK'
                last_seen = time.time()
                print(f"  🔍 [{color.upper()}] 스캔 중 발견 → SEEK")
                # SEEK 블록으로 낙하 (continue 없음)

            elif elapsed_spin >= SPIN_SCAN_TIME:
                state = 'WALL_FOLLOW'
                ser.write(b"S\n")
                print(f"  🔄 [{color.upper()}] 360° 완료 미발견 → WALL_FOLLOW")
                # WALL_FOLLOW 블록으로 낙하 (continue 없음)

            else:
                ser.write(f"T {SPIN_SCAN_DIR * SPIN_SCAN_SPEED:.2f}\n".encode())
                cv2.putText(vis, f"SPIN_SCAN {elapsed_spin:.1f}/{SPIN_SCAN_TIME:.0f}s",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 200, 0), 2)
                emg_txt = f" EMG:{ls['emg_near']:.0f}mm" if ls['has_data'] else ""
                cv2.putText(vis, f"SPIN_SCAN|{color.upper()}{emg_txt}",
                            (5, fh-10), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 2)
                with _display_lock:
                    _display_frame[0] = vis
                continue

        # ── WALL_FOLLOW ───────────────────────────────────────────────
        # 벽 추종으로 이동. 색 발견 시 SEEK로 전환.
        if state == 'WALL_FOLLOW':
            weak_cnt    = get_weak_contour(hsv_u, color)
            color_found = det.get('found') or (weak_cnt is not None)

            if color_found:
                state = 'SEEK'
                last_seen = time.time()
                print(f"  🔍 [{color.upper()}] 벽 추종 중 발견 → SEEK")
                # SEEK 블록으로 낙하 (continue 없음)

            else:
                steer, speed = _wall_follow_cmd(ls)
                if speed > 0:
                    ser.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                else:
                    ser.write(b"S\n")
                wall_d = ls['left_dist'] if WALL_SIDE == 'left' else ls['right_dist']
                cv2.putText(vis, f"WALL({WALL_SIDE}) d={wall_d:.0f}mm st={steer:+.2f}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
                if DEBUG:
                    print(f"  [WALL] [{color.upper()}] d={wall_d:.0f}mm st={steer:+.2f} "
                          f"front={ls['front_near']:.0f}mm")
                emg_txt = f" EMG:{ls['emg_near']:.0f}mm" if ls['has_data'] else ""
                cv2.putText(vis, f"WALL_FOLLOW|{color.upper()}{emg_txt}",
                            (5, fh-10), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 2)
                with _display_lock:
                    _display_frame[0] = vis
                continue

        # ── SEEK ──────────────────────────────────────────────────────
        # VFH 갭 주행 + 색상 추종. 색 소실 시 SPIN_SCAN으로 복귀.

        # ① 강탐지
        if det.get('found'):
            last_seen     = time.time()
            on_zone_count = 0
            cnt           = det['contour']
            pose          = solve_paper_pose(cnt, pnp_mat, pnp_dist)
            area_r        = det['area'] / (fw * fh)

            if area_r > AREA_PEAK_THRES and not align_mode:
                peak_confirm_count += 1
                if peak_confirm_count >= 3:
                    area_peak_seen = True
                    peak_area_r    = max(peak_area_r, area_r)
            else:
                peak_confirm_count = 0

            if pose is not None:
                z_mm, x_mm, steer, quad_pts = pose
                speed       = _speed_limit(SPEED_NEAR if z_mm < DIST_SLOW_MM else SPEED_FAR)
                cam_bearing = math.degrees(math.atan2(x_mm, z_mm))
                cv2.putText(vis, f"Z={z_mm:.0f}mm X={x_mm:+.0f}mm A={area_r:.3f}",
                            (fw//2-160, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
                cv2.polylines(vis, [quad_pts.astype(np.int32)], True, (0, 220, 255), 2)
                ctr = quad_pts.mean(axis=0).astype(int)
                _draw_center(vis, int(ctr[0]), int(ctr[1]), (0, 220, 255))

                if z_mm < ENTRY_STOP_MM and not align_mode:
                    state = 'STOP'; stop_start = time.time()
                    ser.write(b"S\n")
                    print(f"  🎯 [{color.upper()}] 중심 도달! z={z_mm:.0f}mm")
                    with _display_lock:
                        _display_frame[0] = vis
                    continue

                if abs(steer) > ALIGN_STEER_ENTER and z_mm > ALIGN_MIN_DIST:
                    align_mode = True
                elif abs(steer) < ALIGN_STEER_EXIT:
                    align_mode = False

                if align_mode:
                    rot_sign = 0.30 if steer > 0 else -0.30
                    ser.write(f"T {rot_sign:.2f}\n".encode())
                    if DEBUG:
                        print(f"  [ALIGN] [{color.upper()}] rot={rot_sign:+.2f} z={z_mm:.0f}mm")
                    last_seen = time.time()
                    with _display_lock:
                        _display_frame[0] = vis
                    continue

            else:
                offset      = det['offset']
                steer       = float(np.clip(offset * 0.80, -MAX_STEER, MAX_STEER))
                speed       = _speed_limit(SPEED_NEAR if area_peak_seen else SPEED_FAR)
                cam_bearing = offset * 30.0
                align_mode  = False
                cv2.putText(vis, f"A={area_r:.3f} pk={peak_area_r:.3f}",
                            (fw//2-80, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 2)
                M_c = cv2.moments(cnt)
                if M_c['m00'] > 0:
                    _draw_center(vis, int(M_c['m10']/M_c['m00']),
                                 int(M_c['m01']/M_c['m00']), (200, 200, 200))

            with _lidar_lock:
                _lidar_state['goal_bearing'] = cam_bearing

            # 장애물 근접 시 VFH 조향으로 이양
            if ls['has_data']:
                if   ls['emg_near'] < DETECT:       avoiding = True
                elif ls['emg_near'] > DETECT * 1.2: avoiding = False
            else:
                avoiding = False

            if avoiding:
                steer = float(np.clip(ls['vfh_steer'], -MAX_STEER, MAX_STEER))
                speed = _speed_limit(ls['vfh_speed'])  # VFH 속도도 반드시 제한 적용

            # 광각(80°) 비상 정지 오버라이드 — avoiding 여부 무관
            if ls['has_data'] and ls['emg_near'] <= EMERGENCY:
                speed = 0.0

            last_steer     = steer
            smoothed_steer = steer
            if speed > 0:
                ser.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            else:
                ser.write(b"S\n")
            if DEBUG:
                avd = " AVOID" if avoiding else ""
                print(f"  [SEEK{avd}] [{color.upper()}] st={steer:+.2f} spd={speed:.2f}")

        # ② 피크 후 미탐지 (종이 진입 중)
        elif area_peak_seen:
            weak_cnt = get_weak_contour(hsv_u, color)

            if weak_cnt is not None:
                weak_offset    = _contour_offset(weak_cnt, fw)
                steer          = float(np.clip(weak_offset * WEAK_STEER_GAIN,
                                               -MAX_STEER, MAX_STEER))
                last_steer     = steer
                last_seen      = time.time()
                steer_cmd      = STEER_SMOOTH_ALPHA * steer + (1.0 - STEER_SMOOTH_ALPHA) * smoothed_steer
                smoothed_steer = steer_cmd
                ser.write(f"F {steer_cmd:.2f} {WEAK_SPEED:.2f}\n".encode())
                cv2.drawContours(vis, [weak_cnt], -1, (180, 180, 0), 1)
                M_w = cv2.moments(weak_cnt)
                if M_w['m00'] > 0:
                    _draw_center(vis, int(M_w['m10']/M_w['m00']),
                                 int(M_w['m01']/M_w['m00']), (180, 255, 0))
                cv2.putText(vis, f"ENTERING {color.upper()} off={weak_offset:+.2f}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 0), 2)
                if DEBUG:
                    print(f"  [ENTER] [{color.upper()}] off={weak_offset:+.2f}")
            else:
                if not align_mode:
                    on_zone_count += 1
                nudge_spd = _speed_limit(NUDGE_SPEED)
                if nudge_spd > 0:
                    ser.write(f"F 0.00 {nudge_spd:.2f}\n".encode())
                else:
                    ser.write(b"S\n")
                cv2.putText(vis, f"ON PAPER cnt:{on_zone_count}/{CONFIRM_FRAMES}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                if DEBUG:
                    print(f"  [ON] [{color.upper()}] cnt={on_zone_count}")
                if on_zone_count >= CONFIRM_FRAMES:
                    state = 'STOP'; stop_start = time.time()
                    ser.write(b"S\n")
                    print(f"  🎯 [{color.upper()}] 도달!")
                    with _display_lock:
                        _display_frame[0] = vis
                    continue

        # ③ 색 소실 (약탐지 포함)
        else:
            on_zone_count = max(0, on_zone_count - 1)
            weak_cnt = get_weak_contour(hsv_u, color)

            if weak_cnt is not None:
                # 약탐지 → 저속 유도
                weak_offset    = _contour_offset(weak_cnt, fw)
                steer          = float(np.clip(weak_offset * WEAK_STEER_GAIN,
                                               -MAX_STEER, MAX_STEER))
                last_steer     = steer
                last_seen      = time.time()
                steer_cmd      = STEER_SMOOTH_ALPHA * steer + (1.0 - STEER_SMOOTH_ALPHA) * smoothed_steer
                smoothed_steer = steer_cmd
                ser.write(f"F {steer_cmd:.2f} {WEAK_SPEED:.2f}\n".encode())
                M_w = cv2.moments(weak_cnt)
                if M_w['m00'] > 0:
                    _draw_center(vis, int(M_w['m10']/M_w['m00']),
                                 int(M_w['m01']/M_w['m00']), (180, 180, 0))
                cv2.putText(vis, f"WEAK {color.upper()} off={weak_offset:+.2f}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 0), 2)
                if DEBUG:
                    print(f"  [WEAK] [{color.upper()}] off={weak_offset:+.2f}")

            else:
                elapsed = time.time() - last_seen
                if elapsed < COLOR_MEMORY_TIME:
                    # 소실 직후: 마지막 조향 방향 유지
                    steer_cmd      = STEER_SMOOTH_ALPHA * last_steer + (1.0 - STEER_SMOOTH_ALPHA) * smoothed_steer
                    smoothed_steer = steer_cmd
                    mem_speed      = _speed_limit(SPEED_NEAR * 0.7)
                    if mem_speed > 0:
                        ser.write(f"F {steer_cmd:.2f} {mem_speed:.2f}\n".encode())
                    else:
                        ser.write(b"S\n")
                    cv2.putText(vis, f"MEM {elapsed:.2f}s st={steer_cmd:+.2f}",
                                (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 220, 100), 2)
                    if DEBUG:
                        print(f"  [MEM] [{color.upper()}] t={elapsed:.2f}s")
                else:
                    # 완전 소실 → SPIN_SCAN으로 복귀
                    avoiding   = False
                    align_mode = False
                    with _lidar_lock:
                        _lidar_state['goal_bearing'] = None
                    state      = 'SPIN_SCAN'
                    spin_start = time.time()
                    smoothed_steer *= (1.0 - STEER_SMOOTH_ALPHA)
                    ser.write(b"S\n")
                    cv2.putText(vis, f"색 소실 → SPIN_SCAN",
                                (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 200, 255), 2)
                    if DEBUG:
                        print(f"  [LOST] [{color.upper()}] {elapsed:.1f}s → SPIN_SCAN")

        # ── 공통 HUD ──────────────────────────────────────────────────
        emg_txt = f" EMG:{ls['emg_near']:.0f}mm" if ls['has_data'] else ""
        cv2.putText(vis,
                    f"{state}|{color.upper()}|cnt:{on_zone_count}/{CONFIRM_FRAMES}{emg_txt}",
                    (5, fh-10), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 2)
        with _display_lock:
            _display_frame[0] = vis

        if _quit_flag[0]:
            print("  [종료] q 입력")
            break

    ser.write(b"S\n")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
