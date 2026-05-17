import serial
import time
import math

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))


# ── 전역 파라미터 ─────────────────────────────────────────────────
BIN_DEG      = 4.0
N_BINS       = int(360 / BIN_DEG)   # 90개 빈

# ★ 수정1: 로봇 실측값을 반드시 입력할 것
ROBOT_W      = 200.0               # 로봇 폭 (mm) — 실측 입력 필요
BODY_MARGIN  = 25.0                # 양쪽 최소 여유 (mm)
GAP_MIN_PASS = ROBOT_W + BODY_MARGIN * 2   # 260mm (원본 155mm에서 변경)
# ★ 원본 버그: GAP_MIN(145) + GAP_MARGIN(10) = 155mm (≪ 로봇폭 200mm)
#              주석에만 230mm로 표기돼 있어 실제값과 불일치

DETECT       = 550.0               # 감지 거리 (mm)
EMERGENCY    = 150.0               # 즉시 대응 거리 (mm)
P4_DIST      = 200.0               # 제자리 회전 발동 거리 (mm)
MAX_STEER    = 0.89
ROT_THRESH   = 110.0


# ── VFH 헬퍼 함수 (원본과 동일) ───────────────────────────────────

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
        if blocked[i] and not blocked[(i-1)%N_BINS] and not blocked[(i+1)%N_BINS]:
            smoothed[i] = False
    blocked = smoothed

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
                    d_L = hist[(i-1)%N_BINS] if has_pt[(i-1)%N_BINS] else detect_dist
                    d_R = hist[j%N_BINS]      if has_pt[j%N_BINS]      else detect_dist
                    d_L = min(d_L, detect_dist)
                    d_R = min(d_R, detect_dist)
                    gap_w = (d_L + d_R) * math.sin(math.radians(delta_deg / 2.0))
                    center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0
                    gaps.append({
                        'center'   : center_s,
                        'center_cw': center_cw,
                        'width'    : gap_w,
                        'passable' : gap_w >= min_pass_mm,
                        'delta_deg': delta_deg,
                        'd_L'      : d_L,
                        'd_R'      : d_R,
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
    return max(pool, key=lambda g: g['width'] * 0.45 - abs(g['center']) *1.5)


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d


# ★ 수정2: 갭 내 안전 중심 오프셋 계산 ──────────────────────────────
def safe_gap_offset_deg(gap):
    """
    갭이 비대칭일 때 로봇 몸체가 여유있는 쪽으로 치우치도록 목표각 보정.

    원리:
      갭 물리폭 gap_w에서 로봇폭 ROBOT_W를 뺀 값이 실제 이동 여유 공간.
      d_R이 크면 오른쪽(+각도)이 더 넓으므로 그쪽으로 오프셋.

      safe_offset_mm = (d_R - d_L) / (d_L + d_R) × (navigable / 2)
      safe_offset_deg ≈ atan(safe_offset_mm / mean_distance)

    Returns: 도 단위 오프셋 (+ 오른쪽 / - 왼쪽)
    """
    d_L, d_R = gap['d_L'], gap['d_R']
    navigable = gap['width'] - ROBOT_W   # 실제 여유 폭 (mm)
    if navigable <= 0 or (d_L + d_R) <= 0:
        return 0.0
    safe_offset_mm = (d_R - d_L) / (d_L + d_R) * (navigable / 2.0)
    mean_d = (d_L + d_R) / 2.0
    return math.degrees(math.atan2(safe_offset_mm, mean_d))


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

    try:
        _ready
    except NameError:
        import atexit
        _ready     = True
        scan_buf   = []

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
        print("  VFH 장애물 회피 — 로봇 몸체 폭 반영 버전")
        print(f"  로봇폭: {ROBOT_W:.0f}mm  최소통과폭: {GAP_MIN_PASS:.0f}mm")
        print(f"  감지:{DETECT:.0f}mm  긴급:{EMERGENCY:.0f}mm")
        print(f"  빈:{BIN_DEG:.0f}도x{N_BINS}개  회전전환:+-{ROT_THRESH:.0f}도")
        print("=" * 65)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    if s_flag == 1:
        try:
            hist, has_pt = build_polar_hist(scan_buf)
            emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=60)

            if not any(has_pt):
                ser_Ardu.write(b"F 0.00 0.70\n")

            else:
                gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
                best = select_best_gap(gaps, GAP_MIN_PASS)

                # ── P1: VFH 전진 ─────────────────────────────────
                if best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
                    d_L, d_R  = best['d_L'], best['d_R']
                    imbalance = (d_R - d_L) / (d_L + d_R + 1e-9)
                    bias      = imbalance * (best['delta_deg'] / 3.5)

                    lat_L = nearest_in_arc(hist, has_pt, 270.0, arc_half=45)
                    lat_R = nearest_in_arc(hist, has_pt,  90.0, arc_half=45)
                    rep_L = max(0.0, WALL_REP - lat_L) / WALL_REP if (WALL_REP:=350.0) else 0
                    rep_R = max(0.0, WALL_REP - lat_R) / WALL_REP
                    repulsion = (rep_L - rep_R) * 18.0

                    # ★ 수정2 적용: 로봇 몸체 기준 갭 안전 중심으로 치우침
                    offset = safe_gap_offset_deg(best)

                    target = best['center'] + bias + repulsion + offset
                    steer  = max(-MAX_STEER, min(MAX_STEER, target / 90.0 * MAX_STEER))

                    near_d = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=30)
                    ratio  = min(max((DETECT - near_d) / (DETECT - EMERGENCY + 5), 0.0), 1.0)
                    speed  = 0.60 * (1.0 - ratio * 0.55)

                    ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                    print(f"P1_FWD  갭={best['width']:.0f}mm@{best['center']:+.0f}deg  "
                          f"offset={offset:+.1f}deg  bias={bias:+.1f}  rep={repulsion:+.1f}  "
                          f"steer={steer:+.2f}  spd={speed:.2f}")

                # ── P2: 제자리 회전 ───────────────────────────────
                elif best is not None and best['passable'] and emg_near <= P4_DIST:
                    rot_dir = 1.0 if best['center'] > 0 else -1.0
                    ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
                    print(f"P2_ROT  갭후방({best['center']:+.0f}deg)  근접={emg_near:.0f}mm")

                # ── P3: 긴급 후진 ─────────────────────────────────
                elif emg_near <= EMERGENCY and (best is None or not best['passable'] or abs(best['center']) > ROT_THRESH):
                    ser_Ardu.write(b"B 0.80\n")
                    print(f"P3_BACK 근접={emg_near:.0f}mm")

                # ── P4: 통과 가능 갭 없음 → 저속 전진 ──────────────
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
                    steer = max(-MAX_STEER, min(MAX_STEER, target_dir / 90.0 * MAX_STEER * 0.5))
                    ser_Ardu.write(f"F {steer:.2f} 0.40\n".encode())
                    print(f"P4_NOGAP  최대폭={widest:.0f}mm  steer={steer:+.2f}")

        except Exception as e:
            print(f"[VFH ERROR] {e}")

        scan_buf = []
