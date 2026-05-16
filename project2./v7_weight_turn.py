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


# ═══════════════════════════════════════════════════════════════════
#  VFH  (후진 없음)
#
#  [상태 흐름]
#
#    매 사이클:
#      1. 360° 히스토그램 구성
#      2. 통과 가능 갭 탐색
#      3. 명령 결정:
#           +-ROT_THRESH 이내 통과 가능 갭  → VFH_FWD     (조향 전진)
#           +-ROT_THRESH 초과 통과 가능 갭  → VFH_TURN    (급선회 전진)
#           통과 가능 갭 없음               → NO_GAP_FWD  (전방 내 가장 열린 방향 조향)
#
#    ※ F 명령만 사용.
# ═══════════════════════════════════════════════════════════════════

# ── 전역 파라미터 ─────────────────────────────────────────────────
BIN_DEG      = 5.0
N_BINS       = int(360 / BIN_DEG)       # 72개 빈
ROBOT_WIDTH  = 200.0                    # 차량 폭 (mm)
GAP_MARGIN   = 0.0                      # 통과 안전 마진 (mm)
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN # 최소 통과 가능 폭: 205mm
DETECT       = 580.0    # [C] 500→750: 조기 감지로 긴급 상황 예방
EMERGENCY    = 142.0    # [C] 135→160: 벽 타기 진입 여유 확보
MAX_STEER    = 0.85
ROT_THRESH   = 100.0     # [C] 75→90: 전진 허용 범위 확대 (후진 대신 조향)


# ── VFH 헬퍼 함수 ─────────────────────────────────────────────────

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

                    d_ref = min(d_L, d_R)
                    gap_w = 2.0 * d_ref * math.sin(math.radians(delta_deg / 2.0))

                    center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0

                    gaps.append({
                        'center'   : center_s,
                        'center_cw': center_cw,
                        'width'    : gap_w,
                        'passable' : gap_w >= min_pass_mm,
                    })
            i = j
        else:
            i += 1
    return gaps


def select_best_gap(gaps):
    if not gaps:
        return None
    
    passable = [g for g in gaps if g['passable']]
    if not passable:
        return max(gaps, key=lambda g: g['width'])

    # [핵심 수정] 전방 +-90도(ROT_THRESH) 내에 갈 수 있는 길이 있다면 그것만 먼저 검토
    forward_pool = [g for g in passable if abs(g['center']) <= ROT_THRESH]
    
    if forward_pool:
        # 전방에 길이 있으면, 그 중에서 가장 정면에 가깝고(Angle Penalty 강화) 넓은 길 선택
        # abs(g['center'])에 더 큰 가중치(예: 2.0~3.0)를 곱해 정면 위주로 판단하게 합니다.
        return max(forward_pool, key=lambda g: g['width'] * 0.3 - abs(g['center']) * 2.0)
    else:
        # 전방이 다 막혔을 때만 측후방을 돌아보는(VFH_TURN) 갭을 선택
        return max(passable, key=lambda g: g['width'])


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d


def best_forward_open(hist, has_pt):
    """
    전방 ±ROT_THRESH 내에서 장애물까지 거리가 가장 먼 방향 반환.
    통과 불가 갭만 있을 때 그나마 열린 전방 방향으로 조향하기 위해 사용.
    Returns: center_s (부호있는 도, +오른쪽 / -왼쪽)
    """
    half_bins = int(ROT_THRESH / BIN_DEG)
    best_d, best_s = -1.0, 0.0
    for k in range(-half_bins, half_bins + 1):
        idx = k % N_BINS
        d   = hist[idx] if has_pt[idx] else 9999.0
        if d > best_d:
            best_d = d
            best_s = k * BIN_DEG
    return best_s


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

    # ── 1회성 초기화 ────────────────────────────────────────
    try:
        _ready
    except NameError:
        import atexit
        _ready   = True
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

        print("=" * 65)
        print("  VFH  (후진 없음)  Ctrl+C 종료")
        print(f"  감지:{int(DETECT)}mm  긴급:{int(EMERGENCY)}mm  "
              f"최소통과폭:{int(GAP_MIN_PASS)}mm")
        print(f"  빈:{BIN_DEG:.0f}도x{N_BINS}개  회전전환기준:+-{ROT_THRESH:.0f}도")
        print("=" * 65)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    # ── 1회전 완료 -> VFH 판단 ────────────────────────────
    if s_flag == 1:
        if len(scan_buf) > 10:
            hist, has_pt = build_polar_hist(scan_buf)
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps)

            # ── VFH_FWD: 전방 +-ROT_THRESH 이내 통과 가능 갭 ─
            if best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
                steer  = max(-MAX_STEER, min(MAX_STEER,
                             best['center'] / 90.0 * MAX_STEER))
                near_d = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=20)
                ratio  = min(max((DETECT - near_d) / (DETECT - EMERGENCY), 0.0), 1.0)
                speed  = 0.70 * (1.0 - ratio * 0.55)
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"VFH_FWD  갭={best['width']:.0f}mm@{best['center']:+.0f}도  "
                      f"근접={near_d:.0f}mm  steer={steer:+.2f}  spd={speed:.2f}")

            # ── VFH_TURN: +-ROT_THRESH 초과 통과 가능 갭 → 급선회 전진 ─
            elif best is not None and best['passable']:
                steer = MAX_STEER if best['center'] > 0 else -MAX_STEER
                ser_Ardu.write(f"F {steer:.2f} 0.30\n".encode())
                print(f"VFH_TURN  갭={best['center']:+.0f}도  steer={steer:+.2f}  "
                      f"폭={best['width']:.0f}mm")

            # ── NO_GAP_FWD: 통과 가능 갭 없음 → 전방 내 가장 열린 방향으로 조향 ─
            else:
                open_s = best_forward_open(hist, has_pt)
                steer  = max(-MAX_STEER, min(MAX_STEER,
                             open_s / 90.0 * MAX_STEER))
                ser_Ardu.write(f"F {steer:.2f} 0.30\n".encode())
                widest = max((g['width'] for g in gaps), default=0.0)
                print(f"NO_GAP_FWD  최대폭={widest:.0f}mm  open={open_s:+.0f}도  "
                      f"steer={steer:+.2f}")


        # ── 버퍼 초기화 ────────────────────────────────────
        scan_buf = []
