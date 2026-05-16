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
#  VFH (Vector Field Histogram) 기반 장애물 회피
#
#  [핵심 아이디어]
#    매 스캔마다 360° 극좌표 히스토그램을 만들고,
#    장애물이 없는 '계곡(valley)' 방향으로 지속적으로 조향.
#    → 위험 거리에 도달하기 전에 미리 방향을 틀기 때문에
#      대부분의 상황에서 후진이 필요 없음.
#
#  [상태 흐름]
#
#    매 사이클:
#      1. 360° 히스토그램 구성
#      2. 통과 가능한 갭(폭 >= 230mm) 탐색
#      3. 최적 갭 선택 (전방에 가깝고 넓은 것)
#      4. 갭 방향에 따라 명령 결정:
#           +-75도 이내  -> F {steer} {speed}  (조향 전진)
#           +-75도 초과  -> T {dir}            (제자리 회전, 후진 없음)
#           갭 없음      -> 최후 수단 후진
#
#  [갭 폭 계산]
#
#           로봇(0,0)
#               |
#        Ath    |
#       <----   |
#      O         |         O   <- 장애물 엣지
#      d_L       |         d_R
#      |<--- gap_w ------->|
#
#      gap_w = 2 x min(d_L, d_R) x sin(Dth/2)
#
# ═══════════════════════════════════════════════════════════════════

# ── 전역 파라미터 ─────────────────────────────────────────────────
BIN_DEG      = 5.0               # 히스토그램 빈 해상도 (도)
N_BINS       = int(360 / BIN_DEG)  # 72개 빈
ROBOT_WIDTH  = 180.0             # 차량 폭 (mm)
GAP_MARGIN   = 10.0              # 통과 안전 마진 (mm)
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 최소 통과 가능 폭: 230mm
DETECT       = 550.0             # 감지 거리 (mm) — 이른 반응을 위해 확대
EMERGENCY    = 135.0             # 즉시 대응 거리 (mm)
MAX_STEER    = 0.85              # 최대 조향값
ROT_THRESH   = 100.0             # 이 각도 초과 시 제자리 회전 사용 (도)


# ── VFH 헬퍼 함수 ─────────────────────────────────────────────────

def build_polar_hist(scan_buf):
    """
    스캔 버퍼로 360도 극좌표 히스토그램 구성.
    각 빈(BIN_DEG도)에 해당 방향의 최근접 장애물 거리를 저장.
    포인트가 없는 빈은 9999(개방)으로 유지.

    Returns: (hist[N_BINS], has_pt[N_BINS])
    """
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt


def find_vfh_gaps(hist, has_pt, detect_dist, min_pass_mm):
    """
    히스토그램에서 통과 가능한 갭을 모두 탐색.

    0도/360도 경계 wraparound 처리:
      배열을 2배 확장(0~719도)해 연속 개방 구간을 선형 탐색.
      중복 갭은 center 각도 기준으로 제거.

    갭 폭 = 2 x min(d_L, d_R) x sin(Dth/2)

    Returns: list of gap dict
      center    : 갭 중심 (부호있는 도, +오른쪽 / -왼쪽)
      center_cw : 갭 중심 CW 각도 (0~360도)
      width     : 추정 물리 폭 (mm)
      passable  : width >= min_pass_mm 여부
      delta_deg : 갭 각도 폭 (도)
    """
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

            # span == N_BINS: 완전 개방(장애물 없음) -> 직진 처리를 위해 제외
            if span < N_BINS:
                center_cw = ((i + j) / 2.0 * BIN_DEG) % 360.0
                ck = round(center_cw)
                if ck not in seen:
                    seen.add(ck)
                    delta_deg = span * BIN_DEG

                    # 갭 양쪽 장애물 거리 (엣지 없으면 detect_dist 사용)
                    d_L = hist[(i - 1) % N_BINS] if has_pt[(i - 1) % N_BINS] else detect_dist
                    d_R = hist[j % N_BINS]        if has_pt[j % N_BINS]        else detect_dist
                    d_L = min(d_L, detect_dist)
                    d_R = min(d_R, detect_dist)

                    # 갭 폭: 가까운 엣지 기준 현(chord) 계산 (보수적 추정)
                    d_ref = min(d_L, d_R)
                    gap_w = 2.0 * d_ref * math.sin(math.radians(delta_deg / 2.0))

                    # CW 각도 -> 부호있는 각도 (+오른쪽 / -왼쪽)
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
    """
    최적 갭 선택.
    통과 가능한 갭 우선, 동점 시 전방(0도)에 가깝고 넓은 것 선호.

    점수 = 폭(mm) x 0.3 - |center|(도) x 1.0
      -> 전방에서 1도 멀어질 때마다 폭 3.3mm의 이점이 상쇄됨.
    """
    if not gaps:
        return None
    passable = [g for g in gaps if g['passable']]
    pool     = passable if passable else gaps
    return max(pool, key=lambda g: g['width'] * 0.3 - abs(g['center']) * 1.5)


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    """
    지정 방향(center_cw) +- arc_half도 내의 최근접 장애물 거리 반환.
    장애물 없으면 9999.0 반환.
    """
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d


# ── 메인 루프 ──────────────────────────────────────────────────────
while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    # ── RPLIDAR 패킷 유효성 검사 ───────────────────────────
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
        _ready     = True
        scan_buf   = []
        emg_cnt    = 0   # P2 긴급거리 후진 연속 횟수
        no_gap_cnt = 0   # P5 갭없음 후진 연속 횟수
        extra_back = 0

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
        print("  VFH 장애물 회피  (Vector Field Histogram)  Ctrl+C 종료")
        print(f"  감지:{int(DETECT)}mm  긴급:{int(EMERGENCY)}mm  "
              f"최소통과폭:{int(GAP_MIN_PASS)}mm")
        print(f"  빈:{BIN_DEG:.0f}도x{N_BINS}개  회전전환기준:+-{ROT_THRESH:.0f}도")
        print("  ※ 아두이노에서 'T {steer}\\n' 제자리 회전 명령 지원 필요")
        print("=" * 65)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    # ── 1회전 완료 -> VFH 판단 ────────────────────────────
    if s_flag == 1 and len(scan_buf) > 15:
        # ── VFH 분석 ────────────────────────────────────────
        hist, has_pt = build_polar_hist(scan_buf)
        emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=70)

        if not any(has_pt):
            ser_Ardu.write(b"F 0.00 0.70\n")

        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps)

            # ── P1: 확장 후진 진행 중 ─────────────────────
            if extra_back > 0:
                ser_Ardu.write(b"B 0.80\n")
                extra_back -= 1
                print(f"EXTENDED_BACK 잔여 {extra_back}사이클")

            # ── P2: 즉각 위험 + 전방 탈출 불가 -> 후진 ──
            elif (emg_near <= EMERGENCY and
                    (best is None or not best['passable'] or
                    abs(best['center']) > ROT_THRESH)):
                emg_cnt += 1
                if emg_cnt >= 6:
                    ser_Ardu.write(b"B 0.80\n")
                    extra_back = 3
                    emg_cnt    = 0
                    print("EXTENDED_BACK 시작! (3x) [긴급]")
                else:
                    ser_Ardu.write(b"B 0.80\n")
                    print(f"EMERGENCY! 근접={emg_near:.0f}mm ({emg_cnt}/6)")

            # ── P3: VFH 전진 — 갭이 전방 반구(+-ROT_THRESH) ─
            elif best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
                steer  = max(-MAX_STEER, min(MAX_STEER,
                                best['center'] / 90.0 * MAX_STEER))
                near_d = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=20)
                ratio  = min(max((DETECT - near_d) / (DETECT - EMERGENCY), 0.0), 1.0)
                speed  = 0.65 * (1.0 - ratio * 0.55)
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"VFH_FWD  갭={best['width']:.0f}mm@{best['center']:+.0f}도  "
                        f"D{best['delta_deg']:.0f}도  근접={near_d:.0f}mm  "
                        f"steer={steer:+.2f}  spd={speed:.2f}")

            # ── P4: VFH 제자리 회전 — 갭이 후방 반구 ──────
            elif best is not None and best['passable']:
                rot_dir = 1.0 if best['center'] > 0 else -1.0
                ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
                print(f"VFH_ROT  갭 후방({best['center']:+.0f}도) -> "
                        f"제자리 회전 dir={rot_dir:+.0f}  폭={best['width']:.0f}mm")

            # ── P5: 통과 가능 갭 없음 → 가장 열린 방향으로 저속 전진 ─
            else:
                open_g = max(gaps, key=lambda g: g['width']) if gaps else None
                steer  = max(-MAX_STEER, min(MAX_STEER,
                                open_g['center'] / 90.0 * MAX_STEER * 0.5)) if open_g else 0.0
                widest = open_g['width'] if open_g else 0.0
                ser_Ardu.write(f"F {steer:.2f} 0.20\n".encode())
                print(f"NO_GAP  최대폭={widest:.0f}mm  steer={steer:+.2f}")

    # ── 버퍼 항상 초기화 ───────────────────────────────
    scan_buf = []
