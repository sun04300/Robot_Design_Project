import serial
import time
import math
import threading
import cv2
import numpy as np

# ── 포트 설정 ──────────────────────────────────────────────────────────
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))


# ── VFH 파라미터 ───────────────────────────────────────────────────────
BIN_DEG      = 4.0
N_BINS       = int(360 / BIN_DEG)
GAP_MIN      = 80.0
GAP_MARGIN   = 10.0
GAP_MIN_PASS = GAP_MIN + GAP_MARGIN
DETECT       = 560.0
VELO_DOWN    = 400.0
EMERGENCY    = 150.0
P4_DIST      = 170.0
MAX_STEER    = 1.2
ROT_THRESH   = 110.0
ROBOT_RADIUS = 35.0


# ── 색상 탐색 파라미터 ─────────────────────────────────────────────────
COLOR_SEQUENCE   = ['red', 'yellow', 'blue']  # 탐색 순서
STOP_DURATION    = 2.0    # 종이 위 정지 시간 (초)
ON_PAPER_Z_MM    = 200.0  # solvePnP Z 이 값 이하면 "종이 위" 판정 (mm) — 실측 후 조정
MIN_CONTOUR_AREA = 3000   # 최소 윤곽선 면적 (픽셀²)
CAM_STEER_GAIN   = 0.8    # 카메라 X 오프셋 → 조향 변환 게인


# ── 카메라 내부 파라미터 ───────────────────────────────────────────────
# 실제 카메라 캘리브레이션 값으로 교체 필요
SQUARE_MM    = 300.0   # 종이 한 변 실제 크기 (mm)
IMG_W, IMG_H = 640, 480
focal_length = 700.0   # 캘리브레이션으로 얻은 fx (픽셀)

obj_points = np.array([
    [-SQUARE_MM / 2,  SQUARE_MM / 2, 0.0],
    [ SQUARE_MM / 2,  SQUARE_MM / 2, 0.0],
    [ SQUARE_MM / 2, -SQUARE_MM / 2, 0.0],
    [-SQUARE_MM / 2, -SQUARE_MM / 2, 0.0],
], dtype=np.float32)

camera_matrix = np.array([
    [focal_length, 0,            IMG_W / 2.0],
    [0,            focal_length, IMG_H / 2.0],
    [0,            0,            1.0         ],
], dtype=np.float32)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

# HSV 색상 범위 — 조명 환경에 맞게 조정
HSV_RANGES = {
    'red': [
        (np.array([  0, 100,  80]), np.array([ 10, 255, 255])),   # 순수 빨강
        (np.array([150,  70,  80]), np.array([179, 255, 255])),   # 핑크-마젠타 (170→150)
    ],
    'yellow': [
        (np.array([18, 30, 160]), np.array([35, 255, 255])),      # 글레어 대응 S↓ V↑
    ],
    'blue': [
        (np.array([100,  80, 100]), np.array([125, 170, 245])),   # S 상한 170 = 박스 차단
    ],
}

# 파란 종이 최소 비율 (박스는 보통 1~2% 수준이므로 2.5%로 구분)
BLUE_PAPER_RATIO = 0.025


# ── 공유 상태 (스레드 간) ──────────────────────────────────────────────
_cam_lock = threading.Lock()
_cam_data = {
    'detected': False,
    'x_norm'  : 0.0,    # 영상 중심 기준 좌우 오프셋 (-1=좌, +1=우)
    'z_mm'    : 9999.0, # solvePnP 카메라-종이 거리 (mm)
}

_state_lock  = threading.Lock()
_robot_state = {
    'mode'      : 'SEARCHING',  # 'SEARCHING' | 'ON_PAPER' | 'DONE'
    'color_idx' : 0,            # 현재 목표 색 인덱스
    'stop_start': 0.0,          # ON_PAPER 진입 시각
}


# ── 카메라 색상 검출 ────────────────────────────────────────────────────
def _detect_color(frame, color):
    """목표 색상 사각형을 검출해 (x_norm, z_mm) 반환. 없으면 None."""
    hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)

    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    
    ratio = np.count_nonzero(mask) / (frame.shape[0] * frame.shape[1])
    if ratio < BLUE_PAPER_RATIO:
        return None  # 픽셀 비율 미달 → 박스 측면이나 노이즈로 판단, 무시
    
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None  # (x_norm, z_mm)
    for cnt in cnts:
        if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
            continue
        eps    = 0.04 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, eps, True)
        if len(approx) != 4:
            continue

        pts = approx.reshape(4, 2).astype(np.float32)
        pts = pts[np.argsort(pts[:, 1])]      # y 오름차순 정렬
        top, bot = pts[:2], pts[2:]
        tl = top[np.argmin(top[:, 0])]
        tr = top[np.argmax(top[:, 0])]
        br = bot[np.argmax(bot[:, 0])]
        bl = bot[np.argmin(bot[:, 0])]
        img_pts = np.array([tl, tr, br, bl], dtype=np.float32)

        ok, _, tvec = cv2.solvePnP(
            obj_points, img_pts, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok or tvec[2][0] <= 0:
            continue

        z_mm   = float(tvec[2][0])
        cx     = float(np.mean(img_pts[:, 0]))
        x_norm = (cx - IMG_W / 2.0) / (IMG_W / 2.0)  # -1(좌) ~ +1(우)

        if best is None or z_mm < best[1]:
            best = (x_norm, z_mm)

    return best


# ── 카메라 스레드 ──────────────────────────────────────────────────────
def _camera_thread():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[CAM] 카메라 열기 실패")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue

        with _state_lock:
            mode = _robot_state['mode']
            cidx = _robot_state['color_idx']

        if mode == 'DONE' or cidx >= len(COLOR_SEQUENCE):
            with _cam_lock:
                _cam_data['detected'] = False
            time.sleep(0.05)
            continue

        result = _detect_color(frame, COLOR_SEQUENCE[cidx])

        with _cam_lock:
            if result:
                _cam_data['detected'] = True
                _cam_data['x_norm']   = result[0]
                _cam_data['z_mm']     = result[1]
            else:
                _cam_data['detected'] = False
                _cam_data['z_mm']     = 9999.0

    cap.release()


# ── VFH 헬퍼 함수 ─────────────────────────────────────────────────────
def build_polar_hist(scan_buf):
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt


def find_vfh_gaps(hist, has_pt, detect_dist, min_pass_mm):
    blocked = [has_pt[i] and hist[i] <= detect_dist for i in range(N_BINS)]

    smoothed = blocked[:]
    for i in range(N_BINS):
        if blocked[i] and not blocked[(i - 1) % N_BINS] and not blocked[(i + 1) % N_BINS]:
            smoothed[i] = False
    blocked = smoothed

    inflated = blocked[:]
    for i in range(N_BINS):
        if blocked[i] and hist[i] < 9999.0:
            alpha_rad  = math.asin(min(1.0, ROBOT_RADIUS / max(hist[i], ROBOT_RADIUS)))
            alpha_bins = int(math.degrees(alpha_rad) / BIN_DEG) + 1
            for k in range(-alpha_bins, alpha_bins + 1):
                inflated[(i + k) % N_BINS] = True
    blocked = inflated

    gaps = []
    seen = set()
    i = 0
    while i < 2 * N_BINS:
        bi = i % N_BINS
        if not blocked[bi]:
            j = i + 1
            while j < i + N_BINS and not blocked[j % N_BINS]:
                j += 1
            span = j - i
            if span < N_BINS:
                center_cw = ((i + j) / 2.0 * BIN_DEG) % 360.0
                ck = round(center_cw)
                if ck not in seen:
                    seen.add(ck)
                    delta_deg = span * BIN_DEG
                    d_L = hist[(i - 1) % N_BINS] if has_pt[(i - 1) % N_BINS] else detect_dist
                    d_R = hist[j % N_BINS]        if has_pt[j % N_BINS]        else detect_dist
                    d_L = min(d_L, detect_dist)
                    d_R = min(d_R, detect_dist)
                    gap_w    = (d_L + d_R) * math.sin(math.radians(delta_deg / 2.0))
                    depth    = min(hist[k % N_BINS] for k in range(i, j))
                    center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0
                    gaps.append({
                        'center'   : center_s,
                        'center_cw': center_cw,
                        'width'    : gap_w,
                        'passable' : gap_w >= min_pass_mm,
                        'delta_deg': delta_deg,
                        'd_L'      : d_L,
                        'd_R'      : d_R,
                        'depth'    : depth,
                    })
            i = j
        else:
            i += 1
    return gaps


def select_best_gap(gaps, min_pass_mm=GAP_MIN_PASS):
    if not gaps:
        return None
    passable = [g for g in gaps if g['width'] >= min_pass_mm]
    pool     = passable if passable else gaps
    def score(g):
        depth_norm = min(g['depth'], DETECT) / DETECT
        return g['width'] * 0.3 - abs(g['center']) * 1.6 + depth_norm * 25.0
    return max(pool, key=score)


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d


# ── 초기화 ────────────────────────────────────────────────────────────
import atexit

scan_buf = []

def _cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
    except Exception:
        pass

atexit.register(_cleanup)

cam_thread = threading.Thread(target=_camera_thread, daemon=True)
cam_thread.start()

print("=" * 65)
print("  색상 순서 탐색 + VFH 장애물 회피  (빨강 → 노랑 → 파랑)")
print(f"  감지:{int(DETECT)}mm  긴급:{int(EMERGENCY)}mm  최소통과폭:{int(GAP_MIN_PASS)}mm")
print(f"  종이크기:{int(SQUARE_MM)}mm  정지판정Z:{ON_PAPER_Z_MM:.0f}mm  정지시간:{STOP_DURATION:.0f}s")
print("=" * 65)


# ── 메인 루프 (LiDAR 기반) ────────────────────────────────────────────
while True:
    data = ser_L.read(5)
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
    if distance < 80 or quality == 0:
        continue

    scan_buf.append((angle, distance))

    if s_flag != 1:  # 1회전 미완료
        continue

    # ── 1회전 완료: 상태머신 판단 ────────────────────────────────────
    try:
        with _state_lock:
            mode       = _robot_state['mode']
            cidx       = _robot_state['color_idx']
            stop_start = _robot_state['stop_start']

        with _cam_lock:
            detected = _cam_data['detected']
            x_norm   = _cam_data['x_norm']
            z_mm     = _cam_data['z_mm']

        # ── [DONE] 전체 완료 ──────────────────────────────────────
        if mode == 'DONE':
            ser_Ardu.write(b"S\n")
            print("DONE — 모든 색상 탐색 완료, 정지 유지")
            scan_buf = []
            continue

        # ── [ON_PAPER] 종이 위 정지 대기 ─────────────────────────
        if mode == 'ON_PAPER':
            elapsed    = time.time() - stop_start
            color_name = COLOR_SEQUENCE[cidx]
            ser_Ardu.write(b"S\n")
            print(f"ON_PAPER [{color_name}] 정지 중 {elapsed:.1f}/{STOP_DURATION:.0f}s")

            if elapsed >= STOP_DURATION:
                with _state_lock:
                    _robot_state['color_idx'] += 1
                    next_idx = _robot_state['color_idx']
                    if next_idx >= len(COLOR_SEQUENCE):
                        _robot_state['mode'] = 'DONE'
                        print("모든 색상 탐색 완료!")
                    else:
                        _robot_state['mode'] = 'SEARCHING'
                        print(f"다음 목표: [{COLOR_SEQUENCE[next_idx]}]")

            scan_buf = []
            continue

        # ── [SEARCHING] 종이 위 판정 ──────────────────────────────
        if detected and z_mm < ON_PAPER_Z_MM:
            color_name = COLOR_SEQUENCE[cidx]
            print(f"도착! [{color_name}] Z={z_mm:.0f}mm → 정지")
            ser_Ardu.write(b"S\n")
            with _state_lock:
                _robot_state['mode']       = 'ON_PAPER'
                _robot_state['stop_start'] = time.time()
            scan_buf = []
            continue

        # ── [SEARCHING] VFH + 카메라 조향 블렌드 ─────────────────
        hist, has_pt = build_polar_hist(scan_buf)
        emg_near     = nearest_in_arc(hist, has_pt, 0.0, arc_half=80)
        color_name   = COLOR_SEQUENCE[cidx]

        # LiDAR 데이터 없음 → 카메라만으로 전진
        if not any(has_pt):
            cam_steer = max(-MAX_STEER, min(MAX_STEER, x_norm * CAM_STEER_GAIN)) if detected else 0.0
            ser_Ardu.write(f"F {cam_steer:.2f} 0.60\n".encode())
            scan_buf = []
            continue

        gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
        best = select_best_gap(gaps, GAP_MIN_PASS)

        # ── P1: VFH 전진 + 카메라 조향 블렌드 ───────────────────
        if best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
            d_L, d_R  = best['d_L'], best['d_R']
            imbalance = (d_R - d_L) / (d_L + d_R + 1e-9)
            bias      = imbalance * (best['delta_deg'] / 2.9)

            WALL_REP = 150.0
            lat_L    = nearest_in_arc(hist, has_pt, 270.0, arc_half=45)
            lat_R    = nearest_in_arc(hist, has_pt,  90.0, arc_half=45)
            rep_L    = max(0.0, WALL_REP - lat_L) / WALL_REP
            rep_R    = max(0.0, WALL_REP - lat_R) / WALL_REP
            repulsion = (rep_L - rep_R) * 20.0

            CORNER_REP = 350.0
            crn_L  = nearest_in_arc(hist, has_pt, 320.0, arc_half=25)
            crn_R  = nearest_in_arc(hist, has_pt,  40.0, arc_half=25)
            crnf_L = max(0.0, CORNER_REP - crn_L) / CORNER_REP
            crnf_R = max(0.0, CORNER_REP - crn_R) / CORNER_REP
            corner_rep = (crnf_L - crnf_R) * 45.0

            PULL_PEAK, PULL_RANGE = 300.0, 150.0
            pull_L    = max(0.0, 1.0 - abs(lat_L - PULL_PEAK) / PULL_RANGE)
            pull_R    = max(0.0, 1.0 - abs(lat_R - PULL_PEAK) / PULL_RANGE)
            side_pull = (pull_R - pull_L) * 10.0

            vfh_target = best['center'] + bias + repulsion + corner_rep + side_pull
            near_d     = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=35)
            ratio      = min(max((VELO_DOWN - near_d) / (VELO_DOWN - EMERGENCY), 0.0), 1.0)
            steer_gain = 1.0 + ratio * 0.5
            vfh_steer  = max(-MAX_STEER, min(MAX_STEER, vfh_target * steer_gain / 90.0 * MAX_STEER))
            speed      = 0.85 * (1.0 - ratio * 0.55)

            # 카메라 조향 블렌드: 장애물 가까울수록(ratio↑) VFH 우선
            if detected:
                cam_w = 0.35 * (1.0 - ratio)
                steer = vfh_steer * (1.0 - cam_w) + (x_norm * CAM_STEER_GAIN) * cam_w
                steer = max(-MAX_STEER, min(MAX_STEER, steer))
            else:
                steer = vfh_steer

            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            cam_info = f"cam=({x_norm:+.2f},{z_mm:.0f}mm)" if detected else "cam=없음"
            print(f"VFH_FWD [{color_name}] 갭={best['width']:.0f}mm@{best['center']:+.0f}도  "
                  f"{cam_info}  steer={steer:+.2f}  spd={speed:.2f}")

        # ── P2: 제자리 회전 (갭 후방 + 근접) ────────────────────
        elif best is not None and best['passable'] and emg_near <= P4_DIST:
            # 색상 감지 중이면 색상 방향으로 회전 우선
            if detected and abs(x_norm) > 0.3:
                rot_dir = 1.0 if x_norm > 0 else -1.0
            else:
                rot_dir = 1.0 if best['center'] > 0 else -1.0
            ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
            print(f"VFH_ROT [{color_name}] 갭 후방({best['center']:+.0f}도) 근접={emg_near:.0f}mm")

        # ── P3: 긴급 후진 ────────────────────────────────────────
        elif emg_near <= EMERGENCY:
            ser_Ardu.write(b"B 0.80\n")
            print(f"EMERGENCY_BACK [{color_name}] 근접={emg_near:.0f}mm")

        # ── P4: 통과 가능 갭 없음 → 저속 전진 ──────────────────
        else:
            FRONT_ARC = 60.0
            if gaps:
                front_gaps = [g for g in gaps if abs(g['center']) <= FRONT_ARC]
                if front_gaps:
                    open_g     = max(front_gaps, key=lambda g: g['width'])
                    target_dir = open_g['center']
                else:
                    open_g     = max(gaps, key=lambda g: g['width'])
                    target_dir = max(-FRONT_ARC, min(FRONT_ARC, open_g['center']))
                widest = open_g['width']
            else:
                target_dir = 0.0
                widest     = 0.0

            # 색상 감지 중이면 해당 방향으로 회전 유도
            if detected and abs(x_norm) > 0.2:
                target_dir = x_norm * FRONT_ARC

            steer = max(-MAX_STEER, min(MAX_STEER, target_dir / 90.0 * MAX_STEER * 0.5))
            ser_Ardu.write(f"F {steer:.2f} 0.40\n".encode())
            print(f"NO_GAP [{color_name}] 최대폭={widest:.0f}mm  steer={steer:+.2f}")

    except Exception as e:
        print(f"[ERROR] {e}")

    scan_buf = []
