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
GAP_MIN      = 120.0
GAP_MARGIN   = 10.0
GAP_MIN_PASS = GAP_MIN + GAP_MARGIN
DETECT       = 500.0
VELO_DOWN    = 400.0
EMERGENCY    = 150.0
P4_DIST      = 170.0
MAX_STEER          = 1.0    # 최대 조향값 (낮출수록 완만하게 회전)
STEER_SMOOTH_ALPHA = 0.40  # 조향 저역통과 필터 (0=없음, 높을수록 부드러움)
ROT_THRESH         = 110.0
ROBOT_RADIUS = 35.0


# ── 색상 탐색 파라미터 ─────────────────────────────────────────────────
COLOR_SEQUENCE   = ['red', 'yellow', 'blue']  # 탐색 순서
STOP_DURATION    = 1.5    # 종이 위 정지 시간 (초)
CREEP_DURATION   = 1.5    # 종이 감지 후 저속 전진 시간 (초) — 실측 후 조정
CREEP_SPEED      = 0.5   # 저속 전진 속도 (0~1)
ON_PAPER_Z_MM    = 450.0  # solvePnP Z 이 값 이하면 "종이 위" 판정 (mm) — 실측 후 조정
APPROACH_Z_MM    = 700.0  # 이 거리부터 접근 모드 진입 (카메라 중심 조향 우선)
X_NORM_THRESH    = 0.25   # 정지 트리거용 중심 정렬 기준 (|x_norm| 이하면 충분히 정렬됨)
SCAN_TIMEOUT     = 5.0    # 색상 미감지 N초 경과 시 SCANNING 진입
SCAN_ROT_SPEED   = 0.55   # 스캔 회전 속도 (T 명령 강도)
SCAN_FULL_TIME   = 7.0    # 360도 회전 예상 시간 (초) — 실측 후 조정
MIN_CONTOUR_AREA  = 3000   # 최소 윤곽선 면적 (픽셀²) — 1m 이상 기본값
CLOSE_RANGE_Z_MM  = 1000.0 # 이 거리 이내면 근접 탐지 모드 전환 (mm)
COLOR_RATIO_CLOSE = 0.008  # 근접 폴백: 화면의 0.8% 이상이면 비율 감지 시도
CAM_STEER_GAIN    = 0.8    # 카메라 X 오프셋 → 조향 변환 게인


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

# 디버그 화면용 윤곽선/점 색상 (BGR)
COLOR_BGR = {
    'red'   : (0,   0,   255),
    'yellow': (0,   220, 220),
    'blue'  : (255, 100, 0  ),
}


# ── 공유 상태 (스레드 간) ──────────────────────────────────────────────
_cam_lock = threading.Lock()
_cam_data = {
    'detected': False,
    'x_norm'  : 0.0,    # 영상 중심 기준 좌우 오프셋 (-1=좌, +1=우)
    'z_mm'    : 9999.0, # solvePnP 카메라-종이 거리 (mm)
}

_state_lock  = threading.Lock()
_robot_state = {
    'mode'           : 'SEARCHING',  # 'SEARCHING'|'SCANNING'|'CREEPING'|'ON_PAPER'|'DONE'
    'color_idx'      : 0,
    'stop_start'     : 0.0,
    'creep_start'    : 0.0,
    'last_detected_t': time.time(),  # 마지막 색상 감지 시각 (즉시 스캔 방지)
    'scan_start'     : 0.0,          # SCANNING 진입 시각
    'scan_dir'       : 1.0,          # 회전 방향 (+1=우, -1=좌)
}


# ── 카메라 색상 검출 헬퍼 ─────────────────────────────────────────────
def _sort_corners(pts):
    """4점을 좌상→우상→우하→좌하 순으로 정렬 (solvePnP 입력 순서)."""
    pts = pts[np.argsort(pts[:, 1])]
    top, bot = pts[:2], pts[2:]
    return np.array([
        top[np.argmin(top[:, 0])],
        top[np.argmax(top[:, 0])],
        bot[np.argmax(bot[:, 0])],
        bot[np.argmin(bot[:, 0])],
    ], dtype=np.float32)


def _contour_to_quad(cnt):
    """윤곽선에서 4꼭짓점 추출. 3단계 폴백 전략.
    반환: (img_pts, method_str, vis_contour) 또는 None
    """
    perimeter = cv2.arcLength(cnt, True)

    # 단계 1: approxPolyDP — epsilon 값 다수 시도
    for eps_ratio in [0.04, 0.02, 0.06, 0.08, 0.10]:
        approx = cv2.approxPolyDP(cnt, eps_ratio * perimeter, True)
        if len(approx) == 4:
            pts = _sort_corners(approx.reshape(4, 2).astype(np.float32))
            return pts, 'approx', approx

    # 단계 2: Convex Hull + approxPolyDP (볼록 외형으로 정규화 후 재시도)
    hull = cv2.convexHull(cnt)
    hull_perim = cv2.arcLength(hull, True)
    for eps_ratio in [0.05, 0.08, 0.12, 0.18]:
        approx = cv2.approxPolyDP(hull, eps_ratio * hull_perim, True)
        if len(approx) == 4:
            pts = _sort_corners(approx.reshape(4, 2).astype(np.float32))
            return pts, 'hull', approx

    # 단계 3: minAreaRect 강제 4점 (최후 수단 — 어떤 형태도 처리)
    rect = cv2.minAreaRect(cnt)
    box  = cv2.boxPoints(rect).astype(np.float32)
    pts  = _sort_corners(box)
    return pts, 'rect', cnt  # vis_contour에 원본 윤곽선 사용


# ── 카메라 색상 검출 ────────────────────────────────────────────────────
def _detect_color(frame, color, hint_z_mm=9999.0):
    """목표 색상 사각형 검출. (result_dict, mask) 반환.
    result_dict: None 또는 {'x_norm', 'z_mm', 'vis_contour', 'img_pts',
                            'rvec', 'tvec', 'method'}
    hint_z_mm: 직전 프레임 거리 추정값 — 근접 시 탐지 기준 완화에 사용.
    rvec/tvec/vis_contour/img_pts는 method='ratio' 폴백 시 None.
    """
    hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)

    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    # 근접 여부에 따라 사전 비율 게이트 완화
    min_ratio = COLOR_RATIO_CLOSE if hint_z_mm < CLOSE_RANGE_Z_MM else BLUE_PAPER_RATIO
    ratio = np.count_nonzero(mask) / (frame.shape[0] * frame.shape[1])
    if ratio < min_ratio:
        return None, mask

    # 거리에 비례해 윤곽선 최소 면적 완화 (1m 이내일수록 작은 컨투어도 수락)
    if hint_z_mm < 500:
        area_thresh = 800
    elif hint_z_mm < CLOSE_RANGE_Z_MM:
        area_thresh = 1800
    else:
        area_thresh = MIN_CONTOUR_AREA

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    for cnt in cnts:
        if cv2.contourArea(cnt) < area_thresh:
            continue

        quad = _contour_to_quad(cnt)
        if quad is None:
            continue
        img_pts, method, vis_contour = quad

        # solvePnP 시도
        ok, rvec, tvec = cv2.solvePnP(
            obj_points, img_pts, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if ok and tvec[2][0] > 0:
            z_mm = float(tvec[2][0])
        else:
            area = max(cv2.contourArea(cnt), 1.0)
            z_mm = focal_length * SQUARE_MM / math.sqrt(area)
            rvec = tvec = None

        cx     = float(np.mean(img_pts[:, 0]))
        x_norm = (cx - IMG_W / 2.0) / (IMG_W / 2.0)

        if best is None or z_mm < best['z_mm']:
            best = {
                'x_norm'     : x_norm,
                'z_mm'       : z_mm,
                'vis_contour': vis_contour,
                'img_pts'    : img_pts,
                'rvec'       : rvec,
                'tvec'       : tvec,
                'method'     : method,
            }

    # ── 근접 비율 폴백: 컨투어 추출 실패해도 마스크 무게중심으로 조향 유지 ──
    if best is None and hint_z_mm < CLOSE_RANGE_Z_MM and ratio >= COLOR_RATIO_CLOSE:
        filled = np.count_nonzero(mask)
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx     = M['m10'] / M['m00']
            x_norm = (cx - IMG_W / 2.0) / (IMG_W / 2.0)
            z_mm   = focal_length * SQUARE_MM / math.sqrt(max(float(filled), 1.0))
            best = {
                'x_norm'     : x_norm,
                'z_mm'       : z_mm,
                'vis_contour': None,
                'img_pts'    : None,
                'rvec'       : None,
                'tvec'       : None,
                'method'     : 'ratio',
            }

    return best, mask


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
            mode       = _robot_state['mode']
            cidx       = _robot_state['color_idx']
            stop_start = _robot_state['stop_start']
            scan_start = _robot_state['scan_start']

        display = frame.copy()

        # ── DONE: 카메라 정지 ──────────────────────────────────
        if mode == 'DONE' or cidx >= len(COLOR_SEQUENCE):
            with _cam_lock:
                _cam_data['detected'] = False
            cv2.putText(display, "DONE - All colors visited",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Color Detection Debug", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.05)
            continue

        color = COLOR_SEQUENCE[cidx]
        with _cam_lock:
            hint_z = _cam_data['z_mm']   # 직전 프레임 거리 (근접 탐지 완화에 사용)
        result, mask = _detect_color(frame, color, hint_z_mm=hint_z)

        # ── 공유 상태 업데이트 ─────────────────────────────────
        with _cam_lock:
            if result:
                _cam_data['detected'] = True
                _cam_data['x_norm']   = result['x_norm']
                _cam_data['z_mm']     = result['z_mm']
            else:
                _cam_data['detected'] = False
                _cam_data['z_mm']     = 9999.0

        if result:
            with _state_lock:
                _robot_state['last_detected_t'] = time.time()

        # ── 디버그 화면 그리기 ─────────────────────────────────
        dot_color = COLOR_BGR[color]

        if result:
            # 윤곽선 & 꼭짓점 (ratio 폴백 시 None)
            if result['vis_contour'] is not None:
                cv2.drawContours(display, [result['vis_contour']], -1, dot_color, 2)
            if result['img_pts'] is not None:
                for pt in result['img_pts']:
                    cv2.circle(display, tuple(pt.astype(int)), 7, dot_color, -1)

            # 3D 축 — solvePnP 성공 시만 표시
            if result['rvec'] is not None:
                axis = np.float32([[100, 0, 0], [0, 100, 0], [0, 0, -100]]).reshape(-1, 3)
                imgpts, _ = cv2.projectPoints(
                    axis, result['rvec'], result['tvec'], camera_matrix, dist_coeffs)
                origin = tuple(result['img_pts'][0].astype(int))
                cv2.line(display, origin, tuple(imgpts[0].ravel().astype(int)), (0,   0, 255), 2)
                cv2.line(display, origin, tuple(imgpts[1].ravel().astype(int)), (0, 255,   0), 2)
                cv2.line(display, origin, tuple(imgpts[2].ravel().astype(int)), (255, 0,   0), 2)
                x_mm  = float(result['tvec'][0][0])
                info  = (f"Z:{result['z_mm']:.0f}mm  X:{x_mm:+.0f}mm  "
                         f"x_norm:{result['x_norm']:+.2f}  [{result['method']}]")
            else:
                tag  = "RATIO-FALLBACK" if result['method'] == 'ratio' else result['method'] + '*'
                info = (f"Z(est):{result['z_mm']:.0f}mm  "
                        f"x_norm:{result['x_norm']:+.2f}  [{tag}]")

            cv2.putText(display, info, (10, IMG_H - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2)

        # 목표 색 & 검출 여부
        det_str    = "DETECTED" if result else "searching..."
        hdr_color  = (0, 255, 0) if result else (0, 80, 255)
        cv2.putText(display, f"[{color.upper()}] {det_str}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hdr_color, 2)

        # 상태머신 모드
        mode_str = f"Mode: {mode}"
        if mode == 'SCANNING':
            elapsed   = time.time() - scan_start
            mode_str += f"  ({elapsed:.1f}/{SCAN_FULL_TIME:.0f}s) SPIN"
        elif mode == 'CREEPING':
            elapsed   = time.time() - stop_start
            mode_str += f"  ({elapsed:.2f}/{CREEP_DURATION:.1f}s)"
        elif mode == 'ON_PAPER':
            elapsed   = time.time() - stop_start
            mode_str += f"  ({elapsed:.1f}/{STOP_DURATION:.0f}s)"
        cv2.putText(display, mode_str,
                    (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 화면 중앙 기준선 (조향 0 기준)
        cv2.line(display, (IMG_W // 2, 0), (IMG_W // 2, IMG_H), (80, 80, 80), 1)

        cv2.imshow("Color Detection Debug", display)
        cv2.imshow("Color Mask", mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


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

scan_buf   = []
_prev_steer = 0.0  # 조향 스무딩용 직전 값

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
            mode             = _robot_state['mode']
            cidx             = _robot_state['color_idx']
            stop_start       = _robot_state['stop_start']
            creep_start      = _robot_state['creep_start']
            last_detected_t  = _robot_state['last_detected_t']
            scan_start       = _robot_state['scan_start']
            scan_dir         = _robot_state['scan_dir']

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

        # ── [CREEPING] 종이 위 소량 전진 ────────────────────────
        if mode == 'CREEPING':
            elapsed    = time.time() - creep_start
            color_name = COLOR_SEQUENCE[cidx]
            if elapsed < CREEP_DURATION:
                ser_Ardu.write(f"F 0.00 {CREEP_SPEED:.2f}\n".encode())
                print(f"CREEPING [{color_name}] 전진 중 {elapsed:.2f}/{CREEP_DURATION:.1f}s")
            else:
                ser_Ardu.write(b"S\n")
                with _state_lock:
                    _robot_state['mode']       = 'ON_PAPER'
                    _robot_state['stop_start'] = time.time()
                print(f"CREEPING 완료 [{color_name}] → 정지")
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

        # ── [SCANNING] 색상 탐색 회전 ────────────────────────────
        if mode == 'SCANNING':
            color_name = COLOR_SEQUENCE[cidx]
            if detected:
                # 색상 감지됨 → SEARCHING 복귀
                with _state_lock:
                    _robot_state['mode']           = 'SEARCHING'
                    _robot_state['last_detected_t'] = time.time()
                print(f"SCANNING [{color_name}] 감지! → SEARCHING 복귀")
                scan_buf = []
                continue

            elapsed = time.time() - scan_start
            if elapsed >= SCAN_FULL_TIME:
                # 360도 완료 → 방향 반전 후 재시도
                with _state_lock:
                    _robot_state['scan_dir']   = -scan_dir
                    _robot_state['scan_start'] = time.time()
                print(f"SCANNING [{color_name}] 360도 완료 → 방향 반전")
                scan_buf = []
                continue

            ser_Ardu.write(f"T {scan_dir:.2f}\n".encode())
            print(f"SCANNING [{color_name}] 회전 중 {elapsed:.1f}/{SCAN_FULL_TIME:.0f}s dir={scan_dir:+.0f}")
            scan_buf = []
            continue

        # ── [SEARCHING] LiDAR 분석 (모든 경로 공통) ─────────────
        hist, has_pt = build_polar_hist(scan_buf)
        emg_near     = nearest_in_arc(hist, has_pt, 0.0, arc_half=80)
        color_name   = COLOR_SEQUENCE[cidx]

        # P3: 긴급 후진 — 접근 모드 중에도 항상 최우선
        if emg_near <= EMERGENCY:
            _prev_steer = 0.0
            ser_Ardu.write(b"B 0.80\n")
            print(f"EMERGENCY_BACK [{color_name}] 근접={emg_near:.0f}mm")
            scan_buf = []
            continue

        # ── [APPROACHING] 종이 감지 시 카메라 중심 조향 우선 ─────
        if detected and z_mm < APPROACH_Z_MM:
            approach_t = max(0.0, 1.0 - z_mm / APPROACH_Z_MM)  # 0(멀)~1(가까이)

            # 정지 조건: 가깝고 중앙 정렬됨 (또는 매우 가까워 더 이상 조정 불가)
            centered   = abs(x_norm) < X_NORM_THRESH
            very_close = z_mm < ON_PAPER_Z_MM * 0.55  # 극근접 시 x_norm 무시
            if z_mm < ON_PAPER_Z_MM and (centered or very_close):
                print(f"종이 감지! [{color_name}] Z={z_mm:.0f}mm x={x_norm:+.2f} → 전진 후 정지")
                ser_Ardu.write(f"F 0.00 {CREEP_SPEED:.2f}\n".encode())
                with _state_lock:
                    _robot_state['mode']        = 'CREEPING'
                    _robot_state['creep_start'] = time.time()
                scan_buf = []
                continue

            # 아직 정렬 중: 카메라 조향 우선 + 거리에 비례해 감속
            speed     = max(0.28, 0.65 * (1.0 - approach_t * 0.55))
            raw_steer = x_norm * CAM_STEER_GAIN * (1.0 + approach_t * 0.6)
            raw_steer = max(-MAX_STEER, min(MAX_STEER, raw_steer))
            steer     = STEER_SMOOTH_ALPHA * _prev_steer + (1.0 - STEER_SMOOTH_ALPHA) * raw_steer
            steer     = max(-MAX_STEER, min(MAX_STEER, steer))
            _prev_steer = steer
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"APPROACH [{color_name}] Z={z_mm:.0f}mm x={x_norm:+.2f} "
                  f"steer={steer:+.2f} spd={speed:.2f}")
            scan_buf = []
            continue

        # ── SCANNING 진입 트리거 ──────────────────────────────────
        if not detected and (time.time() - last_detected_t) > SCAN_TIMEOUT:
            with _state_lock:
                _robot_state['mode']       = 'SCANNING'
                _robot_state['scan_start'] = time.time()
                _robot_state['scan_dir']   = 1.0
            print(f"[{COLOR_SEQUENCE[cidx]}] {SCAN_TIMEOUT:.0f}초 미감지 → SCANNING 진입")
            scan_buf = []
            continue

        # ── [SEARCHING] 일반 VFH 주행 ────────────────────────────
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

            WALL_REP  = 150.0
            lat_L     = nearest_in_arc(hist, has_pt, 270.0, arc_half=45)
            lat_R     = nearest_in_arc(hist, has_pt,  90.0, arc_half=45)
            rep_L     = max(0.0, WALL_REP - lat_L) / WALL_REP
            rep_R     = max(0.0, WALL_REP - lat_R) / WALL_REP
            repulsion = (rep_L - rep_R) * 20.0

            CORNER_REP = 200.0
            crn_L  = nearest_in_arc(hist, has_pt, 320.0, arc_half=25)
            crn_R  = nearest_in_arc(hist, has_pt,  40.0, arc_half=25)
            crnf_L = max(0.0, CORNER_REP - crn_L) / CORNER_REP
            crnf_R = max(0.0, CORNER_REP - crn_R) / CORNER_REP
            corner_rep = (crnf_L - crnf_R) * 22.0

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

            if detected:
                cam_w = 0.35 * (1.0 - ratio)
                steer = vfh_steer * (1.0 - cam_w) + (x_norm * CAM_STEER_GAIN) * cam_w
                steer = max(-MAX_STEER, min(MAX_STEER, steer))
            else:
                steer = vfh_steer

            steer = STEER_SMOOTH_ALPHA * _prev_steer + (1.0 - STEER_SMOOTH_ALPHA) * steer
            steer = max(-MAX_STEER, min(MAX_STEER, steer))
            _prev_steer = steer

            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            cam_info = f"cam=({x_norm:+.2f},{z_mm:.0f}mm)" if detected else "cam=없음"
            print(f"VFH_FWD [{color_name}] 갭={best['width']:.0f}mm@{best['center']:+.0f}도  "
                  f"{cam_info}  steer={steer:+.2f}  spd={speed:.2f}")

        # ── P2: 제자리 회전 (갭 후방 + 근접) ────────────────────
        elif best is not None and best['passable'] and emg_near <= P4_DIST:
            _prev_steer = 0.0
            if detected and abs(x_norm) > 0.3:
                rot_dir = 1.0 if x_norm > 0 else -1.0
            else:
                rot_dir = 1.0 if best['center'] > 0 else -1.0
            ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
            print(f"VFH_ROT [{color_name}] 갭 후방({best['center']:+.0f}도) 근접={emg_near:.0f}mm")

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

            if detected and abs(x_norm) > 0.2:
                target_dir = x_norm * FRONT_ARC

            steer = max(-MAX_STEER, min(MAX_STEER, target_dir / 90.0 * MAX_STEER * 0.5))
            ser_Ardu.write(f"F {steer:.2f} 0.40\n".encode())
            print(f"NO_GAP [{color_name}] 최대폭={widest:.0f}mm  steer={steer:+.2f}")

    except Exception as e:
        print(f"[ERROR] {e}")

    scan_buf = []
