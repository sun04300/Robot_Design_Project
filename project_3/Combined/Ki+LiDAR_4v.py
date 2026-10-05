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


# ═══════════════════════════════════════════════════════════════════
#  하드웨어 설정
# ═══════════════════════════════════════════════════════════════════
PORT_ARDU  = "/dev/ttyS0"
PORT_LIDAR = "/dev/ttyUSB0"
CAM_INDEX  = 0
CAM_W, CAM_H = 640, 480
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "camera_calibration.pkl")

# ═══════════════════════════════════════════════════════════════════
#  현장 측정값 ← 직접 측정 후 수정
# ═══════════════════════════════════════════════════════════════════
PAPER_W_MM = 300.0
PAPER_H_MM = 300.0

# ═══════════════════════════════════════════════════════════════════
#  색상 탐지 / 주행 파라미터 (color_Lidar_1.py 동일)
# ═══════════════════════════════════════════════════════════════════
MAX_STEER       = 1.1
SPEED_FAR       = 0.55
SPEED_NEAR      = 0.35
AREA_PEAK_THRES = 0.04
AREA_SLOW_THRES = 0.02       # 이 면적 이상이면 감속 (종이에 가까워진 것으로 판단)
STEER_GAIN_CENT = 0.70       # Hough 무게중심 기반 조향 계수
CONFIRM_FRAMES  = 4
STOP_DURATION   = 1.1

WEAK_MIN_AREA      = 200
WEAK_SPEED         = 0.35
WEAK_STEER_GAIN    = 0.60
COLOR_MEMORY_TIME  = 0.30   # 색 소실 후 마지막 조향 유지 시간 (s)
STEER_SMOOTH_ALPHA = 0.40  # 조향 EMA 평활화 계수 (낮을수록 부드러움)

TARGETS = ['red', 'yellow', 'blue']

# ═══════════════════════════════════════════════════════════════════
#  LiDAR VFH 파라미터 (LiDAR_detection_L.py 동일)
# ═══════════════════════════════════════════════════════════════════
BIN_DEG       = 4.0
N_BINS        = int(360 / BIN_DEG)
GAP_MIN_PASS  = 150.0
DETECT        = 600.0
VELO_DOWN     = 400.0
EMERGENCY     = 200.0
LID_MAX_STEER = 1.2
ROT_THRESH    = 110.0
ROBOT_RADIUS  = 50.0

# ═══════════════════════════════════════════════════════════════════
#  LiDAR 공유 상태 (스레드 간)
# ═══════════════════════════════════════════════════════════════════
_lidar_lock  = threading.Lock()
_lidar_state = {
    'has_data'   : False,
    'emg_near'   : 9999.0,   # 전방 80° 최근접 거리
    'front_near' : 9999.0,   # 전방 35° 최근접 거리 (감속용)
    'vfh_action' : 'FWD',    # 'FWD' | 'ROT' | 'BACK'
    'vfh_steer'  : 0.0,      # 정규화 조향값 (-1~1)
    'vfh_speed'  : 0.65,
    'rot_dir'    : 1.0,
}


# ═══════════════════════════════════════════════════════════════════
#  LiDAR VFH 헬퍼 (LiDAR_detection_L.py 이식, 내부용 _ prefix)
# ═══════════════════════════════════════════════════════════════════

def _build_hist(scan_buf):
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt


def _nearest(hist, has_pt, center_cw, arc_half=25):
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
            ar  = math.asin(min(1.0, ROBOT_RADIUS / max(hist[i], ROBOT_RADIUS)))
            ab  = int(math.degrees(ar) / BIN_DEG) + 1
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


def _best_gap(gaps):
    if not gaps:
        return None
    pool = [g for g in gaps if g['passable']] or gaps
    return max(pool, key=lambda g: g['width']*0.3 - abs(g['center'])*1.6
                                    + min(g['depth'],DETECT)/DETECT*25.0)


def _compute_vfh(hist, has_pt):
    """VFH 분석 → (action, steer, speed, rot_dir, emg_near, front_near)."""
    emg   = _nearest(hist, has_pt, 0.0, arc_half=80)
    front = _nearest(hist, has_pt, 0.0, arc_half=35)

    if not any(has_pt):
        return 'FWD', 0.0, 0.70, 1.0, emg, front

    gaps = _find_gaps(hist, has_pt)
    best = _best_gap(gaps)

    if emg <= EMERGENCY and (best is None or not best['passable']
                              or abs(best['center']) > ROT_THRESH):
        return 'BACK', 0.0, 0.80, 1.0, emg, front

    if best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
        imb  = (best['d_R'] - best['d_L']) / (best['d_L'] + best['d_R'] + 1e-9)
        bias = imb * (best['delta_deg'] / 2.9)
        WR   = 130.0
        lL   = _nearest(hist, has_pt, 270.0, arc_half=45)
        lR   = _nearest(hist, has_pt,  90.0, arc_half=45)
        rep  = (max(0.0,WR-lL)/WR - max(0.0,WR-lR)/WR) * 14.0
        CR   = 250.0
        cL   = _nearest(hist, has_pt, 320.0, arc_half=25)
        cR   = _nearest(hist, has_pt,  40.0, arc_half=25)
        crn  = (max(0.0,CR-cL)/CR - max(0.0,CR-cR)/CR) * 22.0
        tgt  = best['center'] + bias + rep + crn
        nd   = _nearest(hist, has_pt, best['center_cw'], arc_half=35)
        rt   = min(max((VELO_DOWN-nd)/(VELO_DOWN-EMERGENCY), 0.0), 1.0)
        st   = max(-LID_MAX_STEER, min(LID_MAX_STEER,
                   tgt * (1.0+rt*0.5) / 90.0 * LID_MAX_STEER))
        spd  = 0.85 * (1.0 - rt * 0.55)
        return 'FWD', float(st), float(spd), 1.0, emg, front

    FARC = 60.0
    if gaps:
        fg = [g for g in gaps if abs(g['center']) <= FARC]
        td = max(fg, key=lambda g: g['width'])['center'] if fg \
             else max(-FARC, min(FARC, max(gaps, key=lambda g: g['width'])['center']))
    else:
        td = 0.0
    st = max(-LID_MAX_STEER, min(LID_MAX_STEER, td/90.0*LID_MAX_STEER*0.5))
    return 'FWD', float(st), 0.40, 1.0, emg, front


# ═══════════════════════════════════════════════════════════════════
#  LiDAR 백그라운드 스레드
# ═══════════════════════════════════════════════════════════════════

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
                act, st, spd, rd, emg, front = _compute_vfh(hist, has_pt)
                with _lidar_lock:
                    _lidar_state['has_data']   = True
                    _lidar_state['emg_near']   = emg
                    _lidar_state['front_near'] = front
                    _lidar_state['vfh_action'] = act
                    _lidar_state['vfh_steer']  = st
                    _lidar_state['vfh_speed']  = spd
                    _lidar_state['rot_dir']    = rd
                scan_buf = []
        except Exception as e:
            print(f"[LIDAR] {e}")
            scan_buf = []


def _lidar_read():
    """현재 LiDAR 상태 스냅샷 반환."""
    with _lidar_lock:
        return dict(_lidar_state)


def _vfh_drive(ser):
    """VFH 계산 결과로 Arduino 주행 명령 전송 (미탐지 탐색용)."""
    ls = _lidar_read()
    if not ls['has_data']:
        ser.write(b"S\n")
        return "NO_LIDAR"
    act = ls['vfh_action']
    if act == 'BACK':
        ser.write(b"B 0.80\n")
        return f"VFH_BACK emg={ls['emg_near']:.0f}mm"
    ser.write(f"F {ls['vfh_steer']:.2f} {ls['vfh_speed']:.2f}\n".encode())
    return f"VFH_FWD steer={ls['vfh_steer']:+.2f} spd={ls['vfh_speed']:.2f}"


def _speed_limit(cam_speed: float) -> float:
    """전방 장애물 거리에 따라 카메라 속도 상한 제한."""
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


# ═══════════════════════════════════════════════════════════════════
#  탐색 보조 함수
# ═══════════════════════════════════════════════════════════════════

def _get_mask(hsv, color: str):
    if color == 'red':    return get_red_mask(hsv)
    if color == 'yellow': return get_yellow_mask(hsv)
    return get_blue_mask(hsv)


def get_weak_contour(hsv, color: str):
    mask = _get_mask(hsv, color)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) > WEAK_MIN_AREA]
    return max(cnts, key=cv2.contourArea) if cnts else None



def _detect_paper_lines(mask: np.ndarray):
    """색상 마스크에서 수평/수직 경계선을 Hough로 검출."""
    k     = np.ones((3, 3), np.uint8)
    edges = cv2.Canny(cv2.dilate(mask, k, iterations=1), 30, 100)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180,
                             threshold=20, minLineLength=40, maxLineGap=25)
    if lines is None:
        return [], []
    h_lines, v_lines = [], []
    for x1, y1, x2, y2 in lines[:, 0]:
        if np.hypot(x2 - x1, y2 - y1) < 30:
            continue
        ang = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
        if ang < 25 or ang > 155:
            h_lines.append((x1, y1, x2, y2))
        elif 65 < ang < 115:
            v_lines.append((x1, y1, x2, y2))
    return h_lines, v_lines


def reconstruct_paper_center(mask: np.ndarray, contour: np.ndarray,
                              fw: int, fh: int,
                              cam_mat=None, z_hint: float = None):
    """
    부분적으로 보이는 색종이에서 경계선 + 알려진 크기로 전체 중심점 추정.

    px/mm 스케일 추정 우선순위:
      1) 카메라 행렬 + LiDAR z 거리
      2) 보이는 수평선 두 개 간격 (= 종이 높이)
      3) 보이는 수직선 두 개 간격 (= 종이 너비)
      4) bounding box 최대 치수 기반 근사

    중심 추정:
      - 수평선 2개 → 그 사이 중간
      - 수평선 1개 → 화면 상반부면 far edge(중심 = 선 + h/2),
                     화면 하반부면 near edge(중심 = 선 - h/2)
      - X 방향도 동일 논리로 수직선 적용
    """
    h_lines, v_lines = _detect_paper_lines(mask)
    bx, by, bw, bh   = cv2.boundingRect(contour)

    # px_per_mm 추정
    px_per_mm = None
    if cam_mat is not None and z_hint is not None and 50.0 < z_hint < 3000.0:
        px_per_mm = float(cam_mat[1, 1]) / z_hint

    h_ys = sorted((y1 + y2) / 2.0 for _,  y1, _,  y2 in h_lines) if h_lines else []
    v_xs = sorted((x1 + x2) / 2.0 for x1, _,  x2, _  in v_lines) if v_lines else []

    if px_per_mm is None and len(h_ys) >= 2:
        span = h_ys[-1] - h_ys[0]
        if span > 15:
            px_per_mm = span / PAPER_H_MM
    if px_per_mm is None and len(v_xs) >= 2:
        span = v_xs[-1] - v_xs[0]
        if span > 15:
            px_per_mm = span / PAPER_W_MM
    if px_per_mm is None:
        px_per_mm = max(bw, bh) / max(PAPER_W_MM, PAPER_H_MM)

    paper_h_px = PAPER_H_MM * px_per_mm
    paper_w_px = PAPER_W_MM * px_per_mm

    # Y 중심 추정
    if len(h_ys) >= 2 and (h_ys[-1] - h_ys[0]) > paper_h_px * 0.25:
        cy_est = (h_ys[0] + h_ys[-1]) / 2.0
    elif len(h_ys) == 1:
        y0     = h_ys[0]
        cy_est = (y0 + paper_h_px / 2.0) if y0 < fh * 0.5 else (y0 - paper_h_px / 2.0)
    else:
        cy_est = by + bh / 2.0

    # X 중심 추정
    if len(v_xs) >= 2 and (v_xs[-1] - v_xs[0]) > paper_w_px * 0.25:
        cx_est = (v_xs[0] + v_xs[-1]) / 2.0
    elif len(v_xs) == 1:
        x0     = v_xs[0]
        cx_est = (x0 + paper_w_px / 2.0) if x0 < fw / 2.0 else (x0 - paper_w_px / 2.0)
    else:
        cx_est = bx + bw / 2.0

    return (int(np.clip(cx_est, 0, fw - 1)),
            int(np.clip(cy_est, 0, fh - 1)))


def _draw_center(vis, cx: int, cy: int, color):
    cv2.circle(vis, (cx, cy), 6, color, -1)
    cv2.line(vis, (cx - 15, cy), (cx + 15, cy), color, 1)
    cv2.line(vis, (cx, cy - 15), (cx, cy + 15), color, 1)


# ═══════════════════════════════════════════════════════════════════
#  메인
# ═══════════════════════════════════════════════════════════════════

def main():
    # ── 시리얼 포트 ───────────────────────────────────────────────
    ser   = serial.Serial(PORT_ARDU,  460800, timeout=1)
    ser_l = serial.Serial(PORT_LIDAR, 460800, timeout=1)

    # LiDAR 초기화 및 스캔 시작
    ser_l.write(bytes([0xA5, 0x40]))
    time.sleep(1)
    ser_l.write(bytes([0xA5, 0x20]))

    # LiDAR 백그라운드 스레드 시작
    t = threading.Thread(target=_lidar_worker, args=(ser_l,), daemon=True)
    t.start()

    # ── 카메라 ────────────────────────────────────────────────────
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    cam_mat, dist_coeffs, calib_res = load_calibration(CALIB_FILE)
    if calib_res and calib_res != (fw, fh):
        print(f"[경고] 캘리브 해상도 불일치({calib_res} vs {fw}×{fh}) → 왜곡 보정 비활성화")
        cam_mat = dist_coeffs = None

    detector = ColorDetector(frame_w=fw, frame_h=fh,
                             camera_matrix=cam_mat, dist_coeffs=dist_coeffs)

    # ── 종료 핸들러 ───────────────────────────────────────────────
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

    # ── 상태 변수 ─────────────────────────────────────────────────
    target_idx     = 0
    state          = 'SEEK'
    on_zone_count  = 0
    stop_start     = None
    last_seen      = time.time()
    last_steer     = 0.0
    smoothed_steer = 0.0
    area_peak_seen = False
    peak_area_r    = 0.0

    print("=" * 65)
    print("  color_lidar_v2  |  Hough 무게중심 + LiDAR VFH")
    print(f"  목표: RED → YELLOW → BLUE   색지:{PAPER_W_MM:.0f}×{PAPER_H_MM:.0f}mm")
    print(f"  조향: Hough 경계선 기반 무게중심  |  미탐지: VFH 전진")
    print("=" * 65)

    while True:
        ret, raw = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        # ── DONE ──────────────────────────────────────────────────
        if state == 'DONE':
            ser.write(b"S\n")
            vis = raw.copy()
            cv2.putText(vis, "MISSION COMPLETE", (fw//2-120, fh//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
            cv2.imshow('Robot View', vis)
            cv2.waitKey(1)
            time.sleep(0.1)
            continue

        # ── LiDAR 긴급 후진 (최우선 — SEEK 중에만 적용) ──────────
        ls = _lidar_read()
        if state == 'SEEK' and ls['has_data'] and ls['emg_near'] <= EMERGENCY:
            ser.write(b"B 0.80\n")
            vis = raw.copy()
            cv2.putText(vis, f"LIDAR EMERGENCY {ls['emg_near']:.0f}mm",
                        (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
            cv2.imshow('Robot View', vis)
            cv2.waitKey(1)
            print(f"  [EMG] 전방={ls['emg_near']:.0f}mm → 후진")
            continue

        result = detector.detect(raw)
        color  = TARGETS[target_idx]
        vis    = detector.draw_debug(raw, result)

        # ── STOP (정지 대기) ───────────────────────────────────────
        if state == 'STOP':
            ser.write(b"S\n")
            elapsed = time.time() - stop_start
            remain  = max(0.0, STOP_DURATION - elapsed)
            cv2.putText(vis, f"STOP {color.upper()}  {remain:.1f}s",
                        (fw//2-90, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 255), 2)
            cv2.imshow('Robot View', vis)
            cv2.waitKey(1)
            if elapsed >= STOP_DURATION:
                if target_idx < len(TARGETS) - 1:
                    target_idx    += 1
                    state          = 'SEEK'
                    on_zone_count  = 0
                    area_peak_seen      = False
                    peak_area_r         = 0.0
                    last_seen           = time.time()
                    print(f"  ✅ {color.upper()} 완료 → {TARGETS[target_idx].upper()}")
                else:
                    state = 'DONE'
                    print("  ✅ 전체 미션 완료!")
            continue

        # ── SEEK ──────────────────────────────────────────────────
        det = result.get(color, {})

        # ① 강탐지 ─────────────────────────────────────────────────
        if det.get('found'):
            last_seen = time.time()
            cnt       = det['contour']
            area_r    = det['area'] / (fw * fh)

            if area_r > AREA_PEAK_THRES:
                area_peak_seen = True
                peak_area_r    = max(peak_area_r, area_r)

            # Hough 경계선 기반 무게중심으로 조향
            _mask_fb       = _get_mask(
                cv2.cvtColor(result['undistorted'], cv2.COLOR_BGR2HSV), color)
            rec_cx, rec_cy = reconstruct_paper_center(_mask_fb, cnt, fw, fh)
            offset         = (rec_cx - fw / 2) / (fw / 2)
            steer          = float(np.clip(offset * STEER_GAIN_CENT, -MAX_STEER, MAX_STEER))
            speed          = _speed_limit(SPEED_NEAR if area_r > AREA_SLOW_THRES else SPEED_FAR)

            last_steer     = steer
            smoothed_steer = steer
            if speed > 0:
                ser.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            else:
                ser.write(b"S\n")
            cv2.putText(vis, f"A={area_r:.3f} off={offset:+.2f}",
                        (fw//2-80, 38), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 220, 255), 2)
            _draw_center(vis, rec_cx, rec_cy, (0, 220, 255))
            print(f"  [SEEK] {color.upper()} off={offset:+.2f} area={area_r:.2f} steer={steer:+.2f} spd={speed:.2f}")

        # ② 피크 후 미탐지 (종이 위 진입 중) ───────────────────────
        elif area_peak_seen:
            hsv_u    = cv2.cvtColor(result['undistorted'], cv2.COLOR_BGR2HSV)
            weak_cnt = get_weak_contour(hsv_u, color)

            if weak_cnt is not None:
                _mask_u        = _get_mask(hsv_u, color)
                rec_cx, rec_cy = reconstruct_paper_center(_mask_u, weak_cnt, fw, fh)
                weak_offset    = (rec_cx - fw / 2) / (fw / 2)
                steer          = float(np.clip(weak_offset * WEAK_STEER_GAIN,
                                               -MAX_STEER, MAX_STEER))
                last_steer     = steer
                last_seen      = time.time()
                steer_cmd      = STEER_SMOOTH_ALPHA * steer + (1.0 - STEER_SMOOTH_ALPHA) * smoothed_steer
                smoothed_steer = steer_cmd
                ser.write(f"F {steer_cmd:.2f} {WEAK_SPEED:.2f}\n".encode())
                cv2.drawContours(vis, [weak_cnt], -1, (180, 180, 0), 1)
                _draw_center(vis, rec_cx, rec_cy, (180, 255, 0))
                cv2.putText(vis, f"ENTERING {color.upper()} off={weak_offset:+.2f}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 0), 2)
                print(f"  [ENTER] {color.upper()} off={weak_offset:+.2f} steer={steer_cmd:+.2f}")
            else:
                on_zone_count += 1
                ser.write(b"S\n")
                cv2.putText(vis, f"ON PAPER  cnt:{on_zone_count}/{CONFIRM_FRAMES}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                print(f"  [ON] {color.upper()} pk={peak_area_r:.2f} cnt={on_zone_count}")
                if on_zone_count >= CONFIRM_FRAMES:
                    state = 'STOP'; stop_start = time.time()
                    ser.write(b"S\n")
                    print(f"  🎯 {color.upper()} 도달!")
                    cv2.imshow('Robot View', vis); cv2.waitKey(1)
                    continue

        # ③ 미탐지 → VFH 탐색 (호회전 대체) ──────────────────────
        else:
            on_zone_count = max(0, on_zone_count - 1)
            hsv_u    = cv2.cvtColor(result['undistorted'], cv2.COLOR_BGR2HSV)
            weak_cnt = get_weak_contour(hsv_u, color)

            if weak_cnt is not None:
                # 약탐지: 해당 방향으로 저속 유도
                _mask_u        = _get_mask(hsv_u, color)
                rec_cx, rec_cy = reconstruct_paper_center(_mask_u, weak_cnt, fw, fh)
                weak_offset    = (rec_cx - fw / 2) / (fw / 2)
                steer          = float(np.clip(weak_offset * WEAK_STEER_GAIN,
                                               -MAX_STEER, MAX_STEER))
                last_steer     = steer
                last_seen      = time.time()
                steer_cmd      = STEER_SMOOTH_ALPHA * steer + (1.0 - STEER_SMOOTH_ALPHA) * smoothed_steer
                smoothed_steer = steer_cmd
                ser.write(f"F {steer_cmd:.2f} {WEAK_SPEED:.2f}\n".encode())
                cv2.drawContours(vis, [weak_cnt], -1, (180, 180, 0), 1)
                _draw_center(vis, rec_cx, rec_cy, (180, 180, 0))
                cv2.putText(vis, f"WEAK {color.upper()} off={weak_offset:+.2f}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 0), 2)
                print(f"  [WEAK] {color.upper()} off={weak_offset:+.2f} steer={steer_cmd:+.2f}")
            else:
                elapsed = time.time() - last_seen

                if elapsed < COLOR_MEMORY_TIME:
                    # 색 소실 직후: 마지막 조향 방향 유지 → 좌우 우왕자왕 방지
                    steer_cmd      = STEER_SMOOTH_ALPHA * last_steer + (1.0 - STEER_SMOOTH_ALPHA) * smoothed_steer
                    smoothed_steer = steer_cmd
                    mem_speed      = _speed_limit(SPEED_NEAR * 0.7)
                    if mem_speed > 0:
                        ser.write(f"F {steer_cmd:.2f} {mem_speed:.2f}\n".encode())
                    else:
                        ser.write(b"S\n")
                    cv2.putText(vis, f"MEM {elapsed:.2f}s st={steer_cmd:+.2f}",
                                (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 220, 100), 2)
                    print(f"  [MEM] {color.upper()} t={elapsed:.2f}s st={steer_cmd:+.2f}")
                else:
                    # VFH 탐색 (정지·제자리 회전 없이 전진 위주)
                    log = _vfh_drive(ser)
                    smoothed_steer *= (1.0 - STEER_SMOOTH_ALPHA)
                    cv2.putText(vis, f"VFH {log}",
                                (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (100, 200, 255), 2)
                    print(f"  [VFH] {color.upper()} {elapsed:.1f}s → {log}")


        # ── 공통 HUD ──────────────────────────────────────────────
        emg_txt = f" EMG:{ls['emg_near']:.0f}mm" if ls['has_data'] else ""
        cv2.putText(vis,
                    f"{state}|{color.upper()}|cnt:{on_zone_count}/{CONFIRM_FRAMES}{emg_txt}",
                    (5, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 2)
        cv2.imshow('Robot View', vis)
        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            print("  [종료] q 입력")
            break

    ser.write(b"S\n")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
