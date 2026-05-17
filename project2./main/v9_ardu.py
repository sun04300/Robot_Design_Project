import serial
import time
import math

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 115200, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))


# ── 전역 파라미터 ─────────────────────────────────────────────────
BIN_DEG      = 5.0               # 히스토그램 빈 해상도 (도)
N_BINS       = int(360 / BIN_DEG)  # 72개 빈
ROBOT_WIDTH  = 140.0             # 차량 폭 (mm)
GAP_MARGIN   = 10.0              # 통과 안전 마진 (mm)
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 최소 통과 가능 폭: 230mm
DETECT       = 500.0             # 감지 거리 (mm) — 이른 반응을 위해 확대
EMERGENCY    = 150.0             # 즉시 대응 거리 (mm) — P3 감속 기준
P4_DIST      = 200.0             # 이 거리 이하일 때만 제자리 회전(P4) 발동 (mm)
MAX_STEER    = 0.85              # 최대 조향값
ROT_THRESH   = 110.0             # 이 각도 초과 시 제자리 회전 사용 (도)


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

    # 단일 노이즈 빈 제거: 양쪽이 모두 열린 단일 blocked 빈은 노이즈로 무시
    smoothed = blocked[:]
    for i in range(N_BINS):
        if blocked[i] and not blocked[(i - 1) % N_BINS] and not blocked[(i + 1) % N_BINS]:
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

                    # 갭 폭: 양쪽 장애물 거리를 각각 반영한 현(chord) 합산
                    # 좌측 여유: d_L * sin(Δθ/2), 우측: d_R * sin(Δθ/2)
                    gap_w = (d_L + d_R) * math.sin(math.radians(delta_deg / 2.0))

                    # CW 각도 -> 부호있는 각도 (+오른쪽 / -왼쪽)
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
    """
    최적 갭 선택.
    min_pass_mm 이상인 갭을 우선 풀로 사용, 없으면 전체 갭에서 선택.
    점수 = 폭(mm) x 0.25 - |center|(도) x 1.8
    -> 폭이 넓고 전방에 가까운 갭 선호. 1도당 약 1.8mm 패널티로 ±45도 정도까지 허용.
    """
    if not gaps:
        return None
    passable = [g for g in gaps if g['width'] >= min_pass_mm]
    pool     = passable if passable else gaps
    return max(pool, key=lambda g: g['width'] * 0.25 - abs(g['center']) * 1.9)


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
    if s_flag == 1:
        # ── VFH 분석 ─────────────────────────────────────
        hist, has_pt = build_polar_hist(scan_buf)
        emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=75)

        if not any(has_pt):
            ser_Ardu.write(b"F 0.00 0.70\n")

        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps, GAP_MIN_PASS)

            # ── P1: 확장 후진 진행 중 ───────────────────
            if extra_back > 0:
                ser_Ardu.write(b"B 0.80\n")
                extra_back -= 1
                print(f"EXTENDED_BACK 잔여 {extra_back}사이클")

            # ── P3: VFH 전진 — 갭이 전방 반구(+-ROT_THRESH) ─
            elif best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
                d_L, d_R  = best['d_L'], best['d_R']
                imbalance = (d_R - d_L) / (d_L + d_R + 1e-9)
                bias      = imbalance * (best['delta_deg'] / 3.2)
                target    = best['center'] + bias
                steer     = max(-MAX_STEER, min(MAX_STEER, target / 90.0 * MAX_STEER))
                near_d = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=30)
                ratio  = min(max((DETECT - near_d) / (DETECT - EMERGENCY + 5), 0.0), 1.0)
                speed  = 0.65 * (1.0 - ratio * 0.55)
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"VFH_FWD  갭={best['width']:.0f}mm@{best['center']:+.0f}도  "
                        f"bias={bias:+.1f}도  D{best['delta_deg']:.0f}도  근접={near_d:.0f}mm  "
                        f"steer={steer:+.2f}  spd={speed:.2f}")

            # ── P4: VFH 제자리 회전 — 갭이 후방 반구 + 장애물 근접 ──
            elif best is not None and best['passable'] and emg_near <= P4_DIST:
                rot_dir = 1.0 if best['center'] > 0 else -1.0
                ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
                print(f"VFH_ROT  갭 후방({best['center']:+.0f}도) 근접={emg_near:.0f}mm -> "
                        f"제자리 회전 dir={rot_dir:+.0f}  폭={best['width']:.0f}mm")

            # ── P5: 통과 가능 갭 없음 → 전방 60도 이내 저속 전진 ─
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
                ser_Ardu.write(f"F {steer:.2f} 0.20\n".encode())
                print(f"NO_GAP  최대폭={widest:.0f}mm  target={target_dir:+.0f}도  steer={steer:+.2f}")

