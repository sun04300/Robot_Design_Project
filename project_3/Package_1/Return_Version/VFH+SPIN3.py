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

# 시리얼 포트 설정 (환경에 맞게 조정)
PORT_ARDU  = "/dev/ttyS0"
PORT_LIDAR = "/dev/ttyUSB0"
CAM_INDEX  = 0
CAM_W, CAM_H = 640, 480
CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "camera_calibration.pkl")

# 종이 크기 (실제 mm 단위) — solvePnP에서 사용
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
ENTRY_STOP_MM   = 250.0  # 카메라→앞바퀴 거리 (실측 후 조정, 종이 중심 도달 시 정지)

WEAK_MIN_AREA      = 200
WEAK_SPEED         = 0.55
WEAK_STEER_GAIN    = 0.60
NUDGE_SPEED        = 0.30   # 피크 후 미탐지 시 앞으로 밀어 넣는 속도
COLOR_MEMORY_TIME  = 0.30   # 색 소실 후 마지막 조향 유지 시간 (s)
STEER_SMOOTH_ALPHA = 0.40   # 조향 EMA 평활화 계수 (낮을수록 부드러움)

ALIGN_STEER_ENTER = 0.30   # 제자리 정렬 회전 시작 임계값 (|steer| 이상)
ALIGN_STEER_EXIT  = 0.10   # 정렬 완료 임계값 (|steer| 이하)
ALIGN_MIN_DIST    = 400.0  # 이 거리(mm) 이상에서만 정렬 (근거리는 바로 전진)

# ── 탐색(SEARCH) 파라미터 ─────────────────────────────────────────
# 색상영역 미발견 시: 즉시 360° 제자리 스핀(넓게 스캔) → VFH 전진 →
# 재스핀 을 반복한다. 2m×2m·장애물 6개·로봇 200mm 환경 시뮬레이션
# (sim_search.py)에서 가장 높은 발견율을 보인 전략.
#  · 스핀: 한 자리에서 카메라 1500mm 반경을 전방향 스캔
#  · VFH : 장애물을 피해 ~1m 전진 → 새로운 시야각 확보
#  · 재스핀: 직전 스핀에서 가려졌던 영역 재탐지
SEARCH_SPIN_SPEED = 0.30   # 제자리 회전 T 명령값 (align_mode와 동일 크기)
SEARCH_SPIN_TIME  = 5.0    # 1회 360° 스핀 소요 시간(s) — 실측 후 조정
SEARCH_VFH_TIME   = 5.5    # 스핀 사이 VFH 전진 시간(s, 약 1m 전진)

TARGETS = ['red', 'yellow', 'blue']

# [최적화 1] True로 설정하면 매 프레임 상세 로그 출력 (기본 비활성화)
DEBUG = True

# LiDAR VFH 파라미터
BIN_DEG       = 4.0
N_BINS        = int(360 / BIN_DEG)
GAP_MIN_PASS  = 90.0
DETECT        = 750.0
VELO_DOWN     = 600.0
EMERGENCY     = 210.0
LID_MAX_STEER = 1.2
ROT_THRESH    = 100.0
ROBOT_RADIUS  = 90.0

# LiDAR 상태 공유 변수 및 락
_lidar_lock  = threading.Lock()
_lidar_state = {
    'has_data'   : False,
    'emg_near'   : 9999.0,   # 전방 80° 최근접 거리
    'front_near' : 9999.0,   # 전방 35° 최근접 거리 (감속용)
    'vfh_action' : 'FWD',
    'vfh_steer'  : 0.0,      # 정규화 조향값 (-1~1)
    'vfh_speed'  : 0.65,
    'rot_dir'    : 1.0,
    'goal_bearing': None,    # 색상영역 방향(deg, +=우측, 0=전방). None=편향 없음
}

# [최적화 5] 디스플레이 스레드 공유 변수
_display_lock  = threading.Lock()
_display_frame = [None]
_quit_flag     = [False]


# LiDAR 스캔 → VFH 분석 → 주행 명령 변환 함수

# LiDAR 스캔 버퍼 → 히스토그램 구축 (각도 구간별 최소 거리)
def _build_hist(scan_buf):
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt

# 주어진 중심 각도 주변에서 가장 가까운 장애물 거리 반환 (탐색용)
def _nearest(hist, has_pt, center_cw, arc_half=55):
    cb = int(center_cw / BIN_DEG) % N_BINS
    nc = max(1, int(arc_half / BIN_DEG))
    md = 9999.0
    for k in range(-nc, nc + 1):
        idx = (cb + k) % N_BINS
        if has_pt[idx] and hist[idx] < md:
            md = hist[idx]
    return md

# 히스토그램 분석 → 통과 가능한 간격(gap) 탐색 및 평가
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

# 탐색된 간격 평가 → 최적 간격 선택 (통과 가능성, 중앙 위치, 깊이 등 고려)
def _best_gap(gaps, goal_bearing=None):
    if not gaps:
        return None
    pool = [g for g in gaps if g['passable']] or gaps
    # goal_bearing 이 있으면 정면(0°) 대신 색 방향에 가까운 gap 을 선호
    ref = 0.0 if goal_bearing is None else goal_bearing
    return max(pool, key=lambda g: g['width']*0.3 - abs(g['center']-ref)*1.6
                                    + min(g['depth'],DETECT)/DETECT*25.0)

# VFH 분석 → 주행 명령 (조향, 속도, 회전 여부) 계산
def _compute_vfh(hist, has_pt, goal_bearing=None):
    """VFH 분석 → (action, steer, speed, rot_dir, emg_near, front_near).

    goal_bearing(deg, +=우측) 가 주어지면 그 방향에 가까운 gap 을 우선 선택
    → 색상 추종 중에도 장애물을 '색 방향으로' 우회 (반발/교착 방지)."""
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
        rep  = (max(0.0,WR-lL)/WR - max(0.0,WR-lR)/WR) * 20.0
        CR   = 350.0
        cL   = _nearest(hist, has_pt, 320.0, arc_half=25)
        cR   = _nearest(hist, has_pt,  40.0, arc_half=25)
        crn  = (max(0.0,CR-cL)/CR - max(0.0,CR-cR)/CR) * 45.0
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
            # 색 방향에 가장 가까운 gap 으로 우회 (정면 고집 대신)
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


# LiDAR 스캔 수집 및 VFH 분석 백그라운드 스레드
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
    ser.write(f"F {ls['vfh_steer']:.2f} {ls['vfh_speed']:.2f}\n".encode())
    return f"VFH_FWD steer={ls['vfh_steer']:+.2f} spd={ls['vfh_speed']:.2f}"


def _search_step(ser, phase: str, phase_start: float, spin_dir: float):
    """탐색 상태머신: SPIN(제자리 360° 스캔) ↔ VFH(전진) 교대.

    반환 (phase, phase_start, log). 매 프레임 카메라 탐지가 우선하므로
    색이 보이면 상위 분기가 가로채 자동으로 탐색을 빠져나간다."""
    now = time.time()
    if phase == 'SPIN':
        if now - phase_start >= SEARCH_SPIN_TIME:
            return 'VFH', now, "SPIN->VFH"
        ser.write(f"T {spin_dir * SEARCH_SPIN_SPEED:.2f}\n".encode())
        return 'SPIN', phase_start, f"SPIN {now - phase_start:.1f}s"
    # VFH 전진 위상
    if now - phase_start >= SEARCH_VFH_TIME:
        return 'SPIN', now, "VFH->SPIN"
    log = _vfh_drive(ser)
    return 'VFH', phase_start, f"VFH {log}"


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


# 종이 위치 추정용 3D 모델 포인트 (mm 단위, Z=0 평면)

_OBJ_PTS = np.array([
    [0,          0,          0],
    [PAPER_W_MM, 0,          0],
    [PAPER_W_MM, PAPER_H_MM, 0],
    [0,          PAPER_H_MM, 0],
], dtype=np.float32)


# 컨투어에서 사각형 추출 → 4개 꼭지점 반환 (반시계 방향, 좌상부터)
def _order_points(pts: np.ndarray) -> np.ndarray:
    s    = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(s)], pts[np.argmin(diff)],
                     pts[np.argmax(s)], pts[np.argmax(diff)]], dtype=np.float32)

# 컨투어에서 사각형 추출 → 4개 꼭지점 반환 (반시계 방향, 좌상부터)
def _extract_quad(contour: np.ndarray) -> np.ndarray:
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

    # 병합 폴백 (버그 수정: 최고 → best)
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

# 컨투어에서 사각형 추출 → solvePnP로 종이 위치 및 조향 계산 → (거리, 좌우 오프셋, 조향) 반환
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

    # 종이 무게중심 (150, 150, 0)을 카메라 좌표계로 변환
    R, _ = cv2.Rodrigues(rvec)
    center_obj = np.array([[PAPER_W_MM / 2], [PAPER_H_MM / 2], [0.0]], dtype=np.float64)
    center_cam = R @ center_obj + tvec
    z_mm = float(center_cam[2])
    x_mm = float(center_cam[0])
    if not (np.isfinite(z_mm) and np.isfinite(x_mm) and z_mm > 0):
        return None
    angle = np.degrees(np.arctan2(x_mm, z_mm))
    steer = float(np.clip(angle * STEER_GAIN, -MAX_STEER, MAX_STEER))
    return z_mm, x_mm, steer, quad_pts


# ═══════════════════════════════════════════════════════════════════
#  탐색 보조 함수 (color_Lidar_1.py 동일)
# ═══════════════════════════════════════════════════════════════════

def _get_mask(hsv, color: str):
    if color == 'red':    return get_red_mask(hsv)
    if color == 'yellow': return get_yellow_mask(hsv)
    return get_blue_mask(hsv)

# 색 영역에서 가장 큰 컨투어 반환 (면적 기준, WEAK_MIN_AREA 이상)
def get_weak_contour(hsv, color: str):
    mask = _get_mask(hsv, color)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) > WEAK_MIN_AREA]
    return max(cnts, key=cv2.contourArea) if cnts else None

# 컨투어 무게중심의 좌우 오프셋 계산 (정규화된 -1~1 값, 음수=좌측, 양수=우측)
def _contour_offset(cnt, frame_w: int) -> float:
    M = cv2.moments(cnt)
    if M['m00'] == 0:
        return 0.0
    return (M['m10'] / M['m00'] - frame_w / 2) / (frame_w / 2)

# 디버그용: 이미지에 중심점과 십자선 그리기
def _draw_center(vis, cx: int, cy: int, color):
    cv2.circle(vis, (cx, cy), 6, color, -1)
    cv2.line(vis, (cx - 15, cy), (cx + 15, cy), color, 1)
    cv2.line(vis, (cx, cy - 15), (cx, cy + 15), color, 1)


# [최적화 5] 디스플레이 전용 스레드: imshow/waitKey를 메인 루프 블로킹에서 분리
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
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # [최적화 2] 최신 프레임만 유지하여 제어 지연 제거
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

    # [최적화 5] 디스플레이 스레드 시작
    disp_t = threading.Thread(target=_display_worker, daemon=True)
    disp_t.start()

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
    align_mode         = False
    area_peak_seen     = False
    peak_area_r        = 0.0
    peak_confirm_count = 0     # area_peak_seen 확정에 필요한 연속 프레임 수
    searching          = False  # 탐색 상태머신 진입 여부
    search_phase   = 'SPIN'    # 'SPIN' | 'VFH'
    search_t0      = 0.0       # 현재 위상 시작 시각
    search_dir     = 1.0       # 스핀 회전 방향 (+1 = 한 방향 고정)
    avoiding       = False     # 색 추종 중 장애물 우회 모드 (히스테리시스)

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
            with _display_lock:
                _display_frame[0] = vis
            time.sleep(0.1)
            continue

        ls = _lidar_read()

        result = detector.detect(raw)
        color  = TARGETS[target_idx]
        vis    = detector.draw_debug(raw, result)

        # [최적화 3] HSV 변환을 루프 상단에서 한 번만 수행 후 재사용
        hsv_u  = cv2.cvtColor(result['undistorted'], cv2.COLOR_BGR2HSV)

        # ── STOP (정지 대기) ───────────────────────────────────────
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
                    target_idx    += 1
                    state          = 'SEEK'
                    on_zone_count      = 0
                    area_peak_seen     = False
                    peak_area_r        = 0.0
                    peak_confirm_count = 0
                    align_mode         = False
                    searching          = False
                    avoiding           = False
                    last_seen      = time.time()
                    print(f"  ✅ {color.upper()} 완료 → {TARGETS[target_idx].upper()}")
                else:
                    state = 'DONE'
                    print("  ✅ 전체 미션 완료!")
            continue



        # ── SEEK ──────────────────────────────────────────────────
        det = result.get(color, {})

        # ① 강탐지 ─────────────────────────────────────────────────
        if det.get('found'):
            last_seen     = time.time()
            searching     = False      # 색 발견 → 탐색 종료
            on_zone_count = 0          # 강탐지 중엔 "종이 위" 아님 → 카운트 초기화
            cnt           = det['contour']
            pose          = solve_paper_pose(cnt, pnp_mat, pnp_dist)
            area_r        = det['area'] / (fw * fh)

            # align_mode 중 원근 왜곡으로 인한 면적 스파이크 방지
            # → 3프레임 연속으로 임계치 초과해야 peak 확정
            if area_r > AREA_PEAK_THRES and not align_mode:
                peak_confirm_count += 1
                if peak_confirm_count >= 3:
                    area_peak_seen = True
                    peak_area_r    = max(peak_area_r, area_r)
            else:
                peak_confirm_count = 0

            if pose is not None:
                z_mm, x_mm, steer, quad_pts = pose
                speed   = _speed_limit(SPEED_NEAR if z_mm < DIST_SLOW_MM else SPEED_FAR)
                cam_bearing = math.degrees(math.atan2(x_mm, z_mm))  # 색 방향(deg, +=우측)
                log_msg = f"PnP z={z_mm:.0f}mm x={x_mm:+.0f}mm area={area_r:.2f}"

                cv2.putText(vis, f"Z={z_mm:.0f}mm X={x_mm:+.0f}mm A={area_r:.3f}",
                            (fw//2-160, 38), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 220, 255), 2)
                cv2.polylines(vis, [quad_pts.astype(np.int32)], True, (0, 220, 255), 2)
                ctr = quad_pts.mean(axis=0).astype(int)
                _draw_center(vis, int(ctr[0]), int(ctr[1]), (0, 220, 255))

                # ── 무게중심 도달 → 즉시 STOP ─────────────────────
                # align_mode 중 PnP 왜곡으로 인한 거짓 도달 판정 방지
                if z_mm < ENTRY_STOP_MM and not align_mode:
                    state = 'STOP'; stop_start = time.time()
                    ser.write(b"S\n")
                    print(f"  🎯 {color.upper()} 중심 도달! z={z_mm:.0f}mm")
                    with _display_lock:
                        _display_frame[0] = vis
                    continue

                # ── 정렬 히스테리시스 ──────────────────────────────
                if abs(steer) > ALIGN_STEER_ENTER and z_mm > ALIGN_MIN_DIST:
                    align_mode = True
                elif abs(steer) < ALIGN_STEER_EXIT:
                    align_mode = False

                if align_mode:
                    rot_sign = 0.30 if steer > 0 else -0.30
                    ser.write(f"T {rot_sign:.2f}\n".encode())
                    if DEBUG:
                        print(f"  [ALIGN] {color.upper()} rot={rot_sign:+.0f} steer={steer:+.2f} z={z_mm:.0f}mm")
                    last_seen = time.time()
                    continue  # HUD 스킵 후 다음 프레임

            else:
                offset  = det['offset']
                steer   = float(np.clip(offset * 0.80, -MAX_STEER, MAX_STEER))
                speed   = _speed_limit(SPEED_NEAR if area_peak_seen else SPEED_FAR)
                cam_bearing = offset * 30.0   # 픽셀 오프셋 → 근사 방향(반화각 30°)
                log_msg = f"fallback offset={offset:+.2f} area={area_r:.2f}"
                align_mode = False   # PnP 없으면 정렬 모드 해제
                cv2.putText(vis, f"A={area_r:.3f} pk={peak_area_r:.3f}",
                            (fw//2-80, 38), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (200, 200, 200), 2)
                M_c = cv2.moments(cnt)
                if M_c['m00'] > 0:
                    _draw_center(vis, int(M_c['m10']/M_c['m00']),
                                 int(M_c['m01']/M_c['m00']), (200, 200, 200))

            # ── 색 방향을 VFH에 전달 (다음 스캔의 gap 선택을 색 쪽으로 편향) ──
            with _lidar_lock:
                _lidar_state['goal_bearing'] = cam_bearing

            # ── 장애물 근접 시 목표지향 VFH로 조향 이양 (반발/교착 방지) ──
            #   히스테리시스: emg<DETECT 진입, emg>DETECT*1.2 해제 → 모드 깜빡임 방지
            if ls['has_data']:
                if   ls['emg_near'] < DETECT:        avoiding = True
                elif ls['emg_near'] > DETECT * 1.2:  avoiding = False
            else:
                avoiding = False

            if avoiding:
                # 색 방향으로 편향된 gap 을 따라 장애물 우회 (단일 결정자)
                steer   = float(np.clip(ls['vfh_steer'], -MAX_STEER, MAX_STEER))
                speed   = ls['vfh_speed']
                log_msg = f"AVOID(vfh) bearing={cam_bearing:+.0f} emg={ls['emg_near']:.0f}"

            last_steer     = steer
            smoothed_steer = steer  # 강탐지 시 평활화 값 즉시 동기화
            if speed > 0:
                ser.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            else:
                ser.write(b"S\n")
            if DEBUG:
                print(f"  [SEEK] {color.upper()} {log_msg} steer={steer:+.2f} spd={speed:.2f}")

        # ② 피크 후 미탐지 (종이 위 진입 중) ───────────────────────
        elif area_peak_seen:
            searching = False          # 진입 중 → 탐색 종료
            # [최적화 3] 미리 변환된 hsv_u 재사용 (이중 변환 제거)
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
                    print(f"  [ENTER] {color.upper()} off={weak_offset:+.2f} steer={steer_cmd:+.2f}")
            else:
                # align_mode 중 탐지 단절로 인한 on_zone_count 오누적 방지
                if not align_mode:
                    on_zone_count += 1
                nudge_spd = _speed_limit(NUDGE_SPEED)
                if nudge_spd > 0:
                    ser.write(f"F 0.00 {nudge_spd:.2f}\n".encode())
                else:
                    ser.write(b"S\n")
                cv2.putText(vis, f"ON PAPER  cnt:{on_zone_count}/{CONFIRM_FRAMES}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                if DEBUG:
                    print(f"  [ON] {color.upper()} pk={peak_area_r:.2f} cnt={on_zone_count} nudge={nudge_spd:.2f}")
                if on_zone_count >= CONFIRM_FRAMES:
                    state = 'STOP'; stop_start = time.time()
                    ser.write(b"S\n")
                    print(f"  🎯 {color.upper()} 도달!")
                    with _display_lock:
                        _display_frame[0] = vis
                    continue

        # ③ 미탐지 → VFH 탐색 (호회전 대체) ──────────────────────
        else:
            on_zone_count = max(0, on_zone_count - 1)
            # [최적화 3] 미리 변환된 hsv_u 재사용 (이중 변환 제거)
            weak_cnt = get_weak_contour(hsv_u, color)

            if weak_cnt is not None:
                # 약탐지: 해당 방향으로 저속 유도
                searching      = False     # 색 흔적 포착 → 탐색 종료
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
                                 int(M_w['m01']/M_w['m00']), (180, 180, 0))
                cv2.putText(vis, f"WEAK {color.upper()} off={weak_offset:+.2f}",
                            (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 0), 2)
                if DEBUG:
                    print(f"  [WEAK] {color.upper()} off={weak_offset:+.2f} steer={steer_cmd:+.2f}")
            else:
                elapsed = time.time() - last_seen

                if elapsed < COLOR_MEMORY_TIME:
                    # 색 소실 직후: 마지막 조향 방향 유지 → 좌우 호회전 대신 부드러운 전진 유도
                    searching      = False     # 아직 "방금 본" 상태 → 탐색 보류
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
                        print(f"  [MEM] {color.upper()} t={elapsed:.2f}s st={steer_cmd:+.2f}")
                else:
                    # 색상영역 완전 소실 → 즉시 스핀→VFH→재스핀 탐색
                    # (sim_search.py 시뮬레이션 최우수 전략)
                    avoiding = False
                    with _lidar_lock:        # 색 편향 해제 → VFH 중립(정면 우선)
                        _lidar_state['goal_bearing'] = None
                    if not searching:
                        searching    = True
                        search_phase = 'SPIN'
                        search_t0    = time.time()
                    search_phase, search_t0, slog = _search_step(
                        ser, search_phase, search_t0, search_dir)
                    smoothed_steer *= (1.0 - STEER_SMOOTH_ALPHA)
                    cv2.putText(vis, f"SEARCH {slog}",
                                (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (100, 200, 255), 2)
                    if DEBUG:
                        print(f"  [SEARCH] {color.upper()} {elapsed:.1f}s -> {slog}")

        # ── 공통 HUD ──────────────────────────────────────────────
        emg_txt = f" EMG:{ls['emg_near']:.0f}mm" if ls['has_data'] else ""
        cv2.putText(vis,
                    f"{state}|{color.upper()}|cnt:{on_zone_count}/{CONFIRM_FRAMES}{emg_txt}",
                    (5, fh - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 0), 2)
        with _display_lock:
            _display_frame[0] = vis

        if _quit_flag[0]:
            if DEBUG:
                print("  [종료] q 입력")
            break

    ser.write(b"S\n")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
