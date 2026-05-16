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
#  VFH + Wall-Following  (후진 없음)
#
#  [변경 사항 vs v6]
#    [C] DETECT 500→750mm, EMERGENCY 135→160mm, ROT_THRESH 75→90도
#        → 더 일찍 반응해 긴급 상황 자체를 줄임
#    [B] wall_follow_dir() 추가
#        → 긴급 상황·갭 없음 시 후진 대신 벽 접선 방향으로 전진
#
#  [상태 흐름]
#
#    매 사이클:
#      1. 360° 히스토그램 구성
#      2. 통과 가능 갭 탐색
#      3. 명령 결정:
#           긴급(전방 160mm 이내)
#             벽 접선 방향이 열려 있음 → WALL_FOLLOW (저속 전진+조향)
#             벽 접선도 막힘           → EMG_ROT     (가장 열린 방향 제자리 회전)
#           +-90도 이내 통과 가능 갭   → VFH_FWD     (조향 전진)
#           +-90도 초과 통과 가능 갭   → VFH_TURN    (급선회 전진)
#           통과 가능 갭 없음          → NO_GAP_WF   (벽 접선 전진)
#
#    ※ B(후진) 및 T(제자리 회전) 명령 불필요. F 명령만 사용.
# ═══════════════════════════════════════════════════════════════════

# ── 전역 파라미터 ─────────────────────────────────────────────────
BIN_DEG      = 5.0
N_BINS       = int(360 / BIN_DEG)       # 72개 빈
ROBOT_WIDTH  = 200.0                    # 차량 폭 (mm)
GAP_MARGIN   = 5.0                      # 통과 안전 마진 (mm)
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN # 최소 통과 가능 폭: 205mm
DETECT       = 400.0    # [C] 500→750: 조기 감지로 긴급 상황 예방
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
                        'delta_deg': delta_deg,
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


def wall_follow_dir(hist, has_pt):
    """
    [B] 가장 가까운 장애물에서 90도 벗어난 벽 접선 방향 반환.
    좌(+90도) / 우(-90도) 중 더 열린 쪽을 선택.
    Returns: (center_cw, center_s)  center_s: +오른쪽 / -왼쪽
    """
    min_d, min_bin = 9999.0, 0
    for i in range(N_BINS):
        if has_pt[i] and hist[i] < min_d:
            min_d, min_bin = hist[i], i

    offset = int(90.0 / BIN_DEG)           # 18 bins = 90도
    cw_a = (min_bin + offset) % N_BINS     # 장애물 기준 +90도
    cw_b = (min_bin - offset) % N_BINS     # 장애물 기준 -90도

    near_a = nearest_in_arc(hist, has_pt, cw_a * BIN_DEG, arc_half=30)
    near_b = nearest_in_arc(hist, has_pt, cw_b * BIN_DEG, arc_half=30)

    best_bin  = cw_a if near_a >= near_b else cw_b
    center_cw = best_bin * BIN_DEG
    center_s  = center_cw if center_cw <= 180.0 else center_cw - 360.0
    return center_cw, center_s


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
        print("  VFH + Wall-Follow  (후진 없음)  Ctrl+C 종료")
        print(f"  감지:{int(DETECT)}mm  긴급:{int(EMERGENCY)}mm  "
              f"최소통과폭:{int(GAP_MIN_PASS)}mm")
        print(f"  빈:{BIN_DEG:.0f}도x{N_BINS}개  회전전환기준:+-{ROT_THRESH:.0f}도")
        print("=" * 65)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    # ── 1회전 완료 -> VFH 판단 ────────────────────────────
    if s_flag == 1 and len(scan_buf) > 15:

        hist, has_pt = build_polar_hist(scan_buf)
        emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=70)

        # 완전 개방 -> 직진
        if not any(has_pt):
            ser_Ardu.write(b"F 0.00 0.70\n")

        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps)

            # ── WALL_FOLLOW / EMG_ROT: 긴급 거리 진입 ────────
            # 후진 대신 벽 접선 방향 전진 또는 열린 방향 제자리 회전
            if emg_near <= EMERGENCY:
                wf_cw, wf_s = wall_follow_dir(hist, has_pt)
                wf_near = nearest_in_arc(hist, has_pt, wf_cw, arc_half=20)

                if wf_near > EMERGENCY * 1.5:   # 접선 방향이 충분히 열려 있음
                    steer = max(-MAX_STEER, min(MAX_STEER,
                                wf_s / 90.0 * MAX_STEER))
                    ser_Ardu.write(f"F {steer:.2f} 0.30\n".encode())
                    print(f"WALL_FOLLOW  dir={wf_s:+.0f}도  near={wf_near:.0f}mm  "
                          f"steer={steer:+.2f}")
                else:                            # 접선도 막힘 -> 가장 열린 방향으로 급선회
                    open_bin = max(range(N_BINS),
                                   key=lambda i: hist[i] if has_pt[i] else 9999.0)
                    open_ang = open_bin * BIN_DEG
                    open_s   = open_ang if open_ang <= 180.0 else open_ang - 360.0
                    steer    = MAX_STEER if open_s >= 0 else -MAX_STEER
                    ser_Ardu.write(f"F {steer:.2f} 0.30\n".encode())
                    print(f"EMG_TURN  open={open_s:+.0f}도  steer={steer:+.2f}")

            # ── VFH_FWD: 전방 +-ROT_THRESH 이내 통과 가능 갭 ─
            elif best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
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

            # ── NO_GAP_WF: 통과 가능 갭 없음 -> 벽 타기 ──────
            else:
                wf_cw, wf_s = wall_follow_dir(hist, has_pt)
                steer = max(-MAX_STEER, min(MAX_STEER,
                            wf_s / 90.0 * MAX_STEER))
                ser_Ardu.write(f"F {steer:.2f} 0.30\n".encode())
                widest = max((g['width'] for g in gaps), default=0.0)
                print(f"NO_GAP_WF  최대폭={widest:.0f}mm  dir={wf_s:+.0f}도  "
                      f"steer={steer:+.2f}")

        # ── 버퍼 초기화 ────────────────────────────────────
        scan_buf = []
