"""
color_wall_follow.py
────────────────────
색상 추종(Primary) + 벽 추종(Secondary) 통합 코드

동작 원리
  ┌ 색상 감지됨 ──→ TRACKING: 카메라 x_norm 으로 색상 방향 추종
  │                       ↓ 충분히 가까워지면
  │               CREEPING → ON_PAPER → 다음 색상 → WALL_FOLLOW
  └ 색상 미감지 ──→ WALL_FOLLOW: 벽 P-제어로 공간 순회 탐색
                   (색상이 발견될 때까지 실내 경계를 따라 이동)

색상 미감지 탐색 전략 — 벽 추종 탐색
  · 실내 폐쇄 공간에서 벽을 따라가면 전체 경계를 체계적으로 순회
  · 이동 중 새 시야각이 계속 확보되어 바닥 색상 종이를 자연스럽게 발견
  · 제자리 회전(SCANNING)보다 실제 위치 이동 → 더 넓은 탐색 범위
"""

import serial
import time
import math
import threading
import cv2
import numpy as np

# ── 포트 설정 ──────────────────────────────────────────────────────
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))


# ── LiDAR / 벽 추종 파라미터 ───────────────────────────────────────
BIN_DEG      = 4.0
N_BINS       = int(360 / BIN_DEG)
REAR_EXCL_LO = 165.0          # 후방 제외 시작 (180° - 15°)
REAR_EXCL_HI = 195.0          # 후방 제외 종료 (180° + 15°)
EMERGENCY    = 150.0           # 긴급 후진 거리 (mm)

WALL_SIDE   = 'left'           # 따라갈 벽: 'left' 또는 'right'
TARGET_DIST = 200.0            # 목표 벽 간격 (mm)
KP_WALL     = 0.007            # 벽 거리 오차 → 조향 게인
CORNER_DIST = 350.0            # 전방 코너 감지 거리 (mm)
BASE_SPEED  = 0.55             # 벽 추종 기본 속도
MAX_STEER   = 1.2
STEER_ALPHA = 0.35             # 조향 저역통과 필터

SIDE_CW = 270.0 if WALL_SIDE == 'left' else  90.0
W_SIGN  = -1.0  if WALL_SIDE == 'left' else   1.0


# ── 색상 탐지 파라미터 ─────────────────────────────────────────────
COLOR_SEQUENCE   = ['red', 'yellow', 'blue']
STOP_DURATION    = 1.5    # 종이 위 정지 시간 (초)
CREEP_DURATION   = 1.5    # 저속 전진 시간 (초)
CREEP_SPEED      = 0.5    # 저속 전진 속도
ON_PAPER_Z_MM    = 500.0  # 종이 위 판정 거리 (mm)
APPROACH_Z_MM    = 700.0  # 이 거리부터 카메라 조향 우선
X_NORM_THRESH    = 0.25   # 정지 트리거 중심 정렬 기준
CAM_STEER_GAIN   = 0.8    # x_norm → 조향 게인
TRACK_LOSS_T     = 2.0    # 색상 미감지 N초 후 WALL_FOLLOW 복귀

MIN_CONTOUR_AREA  = 3000
CLOSE_RANGE_Z_MM  = 700.0
COLOR_RATIO_CLOSE = 0.008
BLUE_PAPER_RATIO  = 0.025


# ── 카메라 내부 파라미터 ───────────────────────────────────────────
SQUARE_MM    = 300.0
IMG_W, IMG_H = 640, 480
focal_length = 700.0

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

HSV_RANGES = {
    'red': [
        (np.array([  0, 100,  80]), np.array([ 10, 255, 255])),
        (np.array([150,  70,  80]), np.array([179, 255, 255])),
    ],
    'yellow': [
        (np.array([18, 80, 150]), np.array([35, 255, 255])),
    ],
    'blue': [
        (np.array([95, 80, 70]), np.array([135, 255, 240])),
    ],
}

COLOR_BGR = {
    'red'   : (0,   0,   255),
    'yellow': (0,   220, 220),
    'blue'  : (255, 100, 0  ),
}


# ── 공유 상태 ──────────────────────────────────────────────────────
_cam_lock = threading.Lock()
_cam_data = {
    'detected': False,
    'x_norm'  : 0.0,
    'z_mm'    : 9999.0,
}

_state_lock  = threading.Lock()
_robot_state = {
    'mode'           : 'WALL_FOLLOW',  # WALL_FOLLOW|TRACKING|CREEPING|ON_PAPER|DONE
    'color_idx'      : 0,
    'stop_start'     : 0.0,
    'creep_start'    : 0.0,
    'last_detected_t': time.time(),
}


# ── 색상 검출 헬퍼 ─────────────────────────────────────────────────

def _sort_corners(pts):
    pts = pts[np.argsort(pts[:, 1])]
    top, bot = pts[:2], pts[2:]
    return np.array([
        top[np.argmin(top[:, 0])],
        top[np.argmax(top[:, 0])],
        bot[np.argmax(bot[:, 0])],
        bot[np.argmin(bot[:, 0])],
    ], dtype=np.float32)


def _contour_to_quad(cnt):
    perimeter = cv2.arcLength(cnt, True)
    for eps in [0.04, 0.02, 0.06, 0.08, 0.10]:
        approx = cv2.approxPolyDP(cnt, eps * perimeter, True)
        if len(approx) == 4:
            return _sort_corners(approx.reshape(4, 2).astype(np.float32)), 'approx', approx
    hull      = cv2.convexHull(cnt)
    hull_peri = cv2.arcLength(hull, True)
    for eps in [0.05, 0.08, 0.12, 0.18]:
        approx = cv2.approxPolyDP(hull, eps * hull_peri, True)
        if len(approx) == 4:
            return _sort_corners(approx.reshape(4, 2).astype(np.float32)), 'hull', approx
    rect = cv2.minAreaRect(cnt)
    box  = cv2.boxPoints(rect).astype(np.float32)
    return _sort_corners(box), 'rect', cnt


def _detect_color(frame, color, hint_z_mm=9999.0):
    """목표 색상 검출. (result_dict, mask) 반환."""
    hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    mask   = np.zeros(frame.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, lo, hi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    min_ratio = COLOR_RATIO_CLOSE if hint_z_mm < CLOSE_RANGE_Z_MM else BLUE_PAPER_RATIO
    ratio     = np.count_nonzero(mask) / (frame.shape[0] * frame.shape[1])
    if ratio < min_ratio:
        return None, mask

    area_thresh = (800 if hint_z_mm < 500
                   else 1800 if hint_z_mm < CLOSE_RANGE_Z_MM
                   else MIN_CONTOUR_AREA)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for cnt in cnts:
        if cv2.contourArea(cnt) < area_thresh:
            continue
        quad = _contour_to_quad(cnt)
        if quad is None:
            continue
        img_pts, method, vis_cnt = quad
        ok, rvec, tvec = cv2.solvePnP(
            obj_points, img_pts, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok and tvec[2][0] > 0:
            z_mm = float(tvec[2][0])
        else:
            z_mm = focal_length * SQUARE_MM / math.sqrt(max(cv2.contourArea(cnt), 1.0))
            rvec = tvec = None
        cx     = float(np.mean(img_pts[:, 0]))
        x_norm = (cx - IMG_W / 2.0) / (IMG_W / 2.0)
        if best is None or z_mm < best['z_mm']:
            best = {'x_norm': x_norm, 'z_mm': z_mm,
                    'vis_contour': vis_cnt, 'img_pts': img_pts,
                    'rvec': rvec, 'tvec': tvec, 'method': method}

    # 근접 비율 폴백
    if best is None and hint_z_mm < CLOSE_RANGE_Z_MM and ratio >= COLOR_RATIO_CLOSE:
        filled = np.count_nonzero(mask)
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx     = M['m10'] / M['m00']
            x_norm = (cx - IMG_W / 2.0) / (IMG_W / 2.0)
            z_mm   = focal_length * SQUARE_MM / math.sqrt(max(float(filled), 1.0))
            best   = {'x_norm': x_norm, 'z_mm': z_mm,
                      'vis_contour': None, 'img_pts': None,
                      'rvec': None, 'tvec': None, 'method': 'ratio'}
    return best, mask


# ── 카메라 스레드 ──────────────────────────────────────────────────
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
            mode  = _robot_state['mode']
            cidx  = _robot_state['color_idx']

        display = frame.copy()

        if mode == 'DONE' or cidx >= len(COLOR_SEQUENCE):
            with _cam_lock:
                _cam_data['detected'] = False
            cv2.putText(display, "DONE", (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
            cv2.imshow("Camera", display)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            time.sleep(0.05)
            continue

        color = COLOR_SEQUENCE[cidx]
        with _cam_lock:
            hint_z = _cam_data['z_mm']
        result, mask = _detect_color(frame, color, hint_z_mm=hint_z)

        # 공유 상태 업데이트
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

        # 디버그 화면
        dot_color = COLOR_BGR[color]
        if result:
            if result['vis_contour'] is not None:
                cv2.drawContours(display, [result['vis_contour']], -1, dot_color, 2)
            if result['img_pts'] is not None:
                for pt in result['img_pts']:
                    cv2.circle(display, tuple(pt.astype(int)), 7, dot_color, -1)
            tag  = 'RATIO' if result['method'] == 'ratio' else result['method']
            info = (f"Z:{result['z_mm']:.0f}mm  x:{result['x_norm']:+.2f}  [{tag}]")
            cv2.putText(display, info, (10, IMG_H - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        det_str   = "DETECTED" if result else "searching..."
        hdr_color = (0, 255, 0) if result else (0, 80, 255)
        cv2.putText(display, f"[{color.upper()}] {det_str}  Mode:{mode}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, hdr_color, 2)
        cv2.line(display, (IMG_W // 2, 0), (IMG_W // 2, IMG_H), (80, 80, 80), 1)
        cv2.imshow("Camera", display)
        cv2.imshow("Mask",   mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# ── LiDAR 헬퍼 ────────────────────────────────────────────────────

def build_polar_hist(scan_buf):
    """극좌표 히스토그램. 후방 30°(165°~195°) 제외."""
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        if REAR_EXCL_LO <= a <= REAR_EXCL_HI:
            continue
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d


# ── 카메라 스레드 시작 ─────────────────────────────────────────────
threading.Thread(target=_camera_thread, daemon=True).start()


# ── 메인 루프 ──────────────────────────────────────────────────────
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
    if distance < 80:
        continue

    # 1회성 초기화
    try:
        _ready
    except NameError:
        import atexit
        _ready     = True
        scan_buf   = []
        prev_steer = 0.0

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

        print("=" * 65)
        print("  Color + Wall Follow  (색상 추종 우선 / 벽 추종 탐색)")
        print(f"  색상 순서: {' → '.join(COLOR_SEQUENCE)}")
        print(f"  벽:{WALL_SIDE.upper()} 목표:{TARGET_DIST:.0f}mm  "
              f"긴급:{EMERGENCY:.0f}mm  추종손실:{TRACK_LOSS_T:.1f}s")
        print("=" * 65)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    # 1회전 완료 → 판단
    if s_flag == 1:
        try:
            hist, has_pt = build_polar_hist(scan_buf)

            # 공유 상태 읽기
            with _cam_lock:
                detected = _cam_data['detected']
                x_norm   = _cam_data['x_norm']
                z_mm     = _cam_data['z_mm']
            with _state_lock:
                mode             = _robot_state['mode']
                cidx             = _robot_state['color_idx']
                stop_start       = _robot_state['stop_start']
                creep_start      = _robot_state['creep_start']
                last_detected_t  = _robot_state['last_detected_t']

            front_dist = nearest_in_arc(hist, has_pt, 0.0, arc_half=80)
            color_name = COLOR_SEQUENCE[cidx] if cidx < len(COLOR_SEQUENCE) else '-'

            # ── DONE ──────────────────────────────────────────────
            if mode == 'DONE':
                ser_Ardu.write(b"S\n")
                print("DONE — 모든 색상 완료")
                scan_buf = []
                continue

            # ── 최우선: 긴급 후진 (모든 모드 공통) ───────────────
            if front_dist <= EMERGENCY:
                prev_steer = 0.0
                ser_Ardu.write(b"B 0.80\n")
                print(f"EMERGENCY_BACK  전방={front_dist:.0f}mm  [{mode}]")
                scan_buf = []
                continue

            # ── ON_PAPER: 정지 대기 ───────────────────────────────
            if mode == 'ON_PAPER':
                ser_Ardu.write(b"S\n")
                elapsed = time.time() - stop_start
                if elapsed >= STOP_DURATION:
                    next_idx = cidx + 1
                    if next_idx >= len(COLOR_SEQUENCE):
                        with _state_lock:
                            _robot_state['mode'] = 'DONE'
                        print("모든 색상 완료 → DONE")
                    else:
                        with _state_lock:
                            _robot_state['color_idx']       = next_idx
                            _robot_state['mode']            = 'WALL_FOLLOW'
                            _robot_state['last_detected_t'] = time.time()
                        print(f"ON_PAPER 완료 → 다음: [{COLOR_SEQUENCE[next_idx]}] → WALL_FOLLOW")
                else:
                    print(f"ON_PAPER [{color_name}]  {elapsed:.1f}/{STOP_DURATION:.0f}s")
                scan_buf = []
                continue

            # ── CREEPING: 저속 전진 ───────────────────────────────
            if mode == 'CREEPING':
                elapsed = time.time() - creep_start
                if elapsed >= CREEP_DURATION:
                    with _state_lock:
                        _robot_state['mode']       = 'ON_PAPER'
                        _robot_state['stop_start'] = time.time()
                    print(f"CREEPING 완료 [{color_name}] → ON_PAPER")
                else:
                    ser_Ardu.write(f"F 0.00 {CREEP_SPEED:.2f}\n".encode())
                    print(f"CREEPING [{color_name}]  {elapsed:.2f}/{CREEP_DURATION:.1f}s")
                scan_buf = []
                continue

            # ── TRACKING: 색상 방향으로 카메라 조향 ──────────────
            if mode == 'TRACKING':
                if detected:
                    # 종이에 충분히 가까워지면 저속 전진으로 전환
                    centered   = abs(x_norm) < X_NORM_THRESH
                    very_close = z_mm < ON_PAPER_Z_MM * 0.55
                    if z_mm < ON_PAPER_Z_MM and (centered or very_close):
                        ser_Ardu.write(f"F 0.00 {CREEP_SPEED:.2f}\n".encode())
                        with _state_lock:
                            _robot_state['mode']        = 'CREEPING'
                            _robot_state['creep_start'] = time.time()
                        print(f"TRACKING [{color_name}] → CREEPING  Z={z_mm:.0f}mm")
                    else:
                        # 거리에 비례 감속, x_norm 으로 조향
                        approach_t = max(0.0, 1.0 - z_mm / APPROACH_Z_MM)
                        raw   = x_norm * CAM_STEER_GAIN * (1.0 + approach_t * 0.5)
                        steer = STEER_ALPHA * prev_steer + (1.0 - STEER_ALPHA) * raw
                        steer = max(-MAX_STEER, min(MAX_STEER, steer))
                        prev_steer = steer
                        speed = max(0.28, BASE_SPEED * (1.0 - approach_t * 0.45))
                        ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                        print(f"TRACKING [{color_name}]  Z={z_mm:.0f}mm  "
                              f"x={x_norm:+.2f}  steer={steer:+.2f}  spd={speed:.2f}")
                else:
                    # 색상 미감지: TRACK_LOSS_T 초 후 WALL_FOLLOW 복귀
                    lost_t = time.time() - last_detected_t
                    if lost_t > TRACK_LOSS_T:
                        with _state_lock:
                            _robot_state['mode'] = 'WALL_FOLLOW'
                        print(f"TRACKING [{color_name}] 색상 손실 {lost_t:.1f}s → WALL_FOLLOW")
                    else:
                        # 짧은 손실: 마지막 조향 유지하며 저속 전진
                        ser_Ardu.write(f"F {prev_steer:.2f} 0.30\n".encode())
                        print(f"TRACKING [{color_name}] 일시 손실 {lost_t:.1f}s → 유지")
                scan_buf = []
                continue

            # ── WALL_FOLLOW: 벽 추종 탐색 ─────────────────────────
            # 색상 감지되면 즉시 TRACKING 전환
            if detected:
                with _state_lock:
                    _robot_state['mode'] = 'TRACKING'
                print(f"WALL_FOLLOW [{color_name}] 색상 발견! → TRACKING  Z={z_mm:.0f}mm")
                scan_buf = []
                continue

            if not any(has_pt):
                ser_Ardu.write(f"F 0.00 {BASE_SPEED:.2f}\n".encode())
                scan_buf = []
                continue

            side_dist = nearest_in_arc(hist, has_pt, SIDE_CW, arc_half=40)

            # 전방 코너 선회
            if front_dist < CORNER_DIST:
                t     = (CORNER_DIST - front_dist) / (CORNER_DIST - EMERGENCY)
                t     = max(0.0, min(1.0, t))
                raw   = W_SIGN * (-MAX_STEER) * t
                steer = STEER_ALPHA * prev_steer + (1.0 - STEER_ALPHA) * raw
                steer = max(-MAX_STEER, min(MAX_STEER, steer))
                prev_steer = steer
                speed = BASE_SPEED * (1.0 - t * 0.45)
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"WALL_CORNER [{color_name}]  front={front_dist:.0f}mm  "
                      f"steer={steer:+.2f}  spd={speed:.2f}")
            else:
                # 벽 P 제어
                error = side_dist - TARGET_DIST
                raw   = W_SIGN * KP_WALL * error
                steer = STEER_ALPHA * prev_steer + (1.0 - STEER_ALPHA) * raw
                steer = max(-MAX_STEER, min(MAX_STEER, steer))
                prev_steer = steer
                label = "WALL_LOST" if side_dist > TARGET_DIST * 3 else "WALL_FOLLOW"
                ser_Ardu.write(f"F {steer:.2f} {BASE_SPEED:.2f}\n".encode())
                print(f"{label} [{color_name}]  side={side_dist:.0f}mm  "
                      f"err={error:+.0f}mm  steer={steer:+.2f}")

        except Exception as e:
            print(f"[ERROR] {e}")

        scan_buf = []
