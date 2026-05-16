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

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  VFH (Vector Field Histogram) 기반 장애물 회피
#
#  [수정 요약]
#    ① select_best_gap 점수식 개선: 폭 가중치 ↑, 각도 페널티 ↓
#    ② 탈출 방향 메모리(escape_dir) 추가
#       - 한 번 방향을 결정하면 ESCAPE_HOLD_CYCLES 동안 유지
#       - 갭 점수에 dir_bonus 반영 → 역방향 갭 선택 방지
#    ③ 코너 통과 후 역방향 재진입 차단
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# ── 파라미터 ────────────────────────────────────────────────────────
BIN_DEG      = 5.0
N_BINS       = int(360 / BIN_DEG)   # 72개 빈
ROBOT_WIDTH  = 200.0                # 로봇 폭 (mm)
GAP_MARGIN   = 10.0
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 통과 가능 최소 갭 = 210mm
DETECT       = 550.0                # 감지 거리 (mm)
EMERGENCY    = 135.0                # 긴급 후진 거리 (mm)
MAX_STEER    = 0.85
ROT_THRESH   = 75.0                 # 이 각도 초과 시 제자리 회전 사용 (°)

# [수정②] 방향 메모리 유지 사이클
ESCAPE_HOLD_CYCLES = 6              # 방향 결정 후 이 사이클 동안 같은 방향 선호
DIR_BONUS_WEIGHT   = 30.0          # 방향 보너스 강도 (mm 환산)


# ── VFH 보조 함수 ────────────────────────────────────────────────────

def build_polar_hist(scan_buf):
    """스캔 버퍼로 360° 폴라 히스토그램 생성. 각 빈의 최소 거리 저장."""
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
    wraparound 처리: 2N_BINS 길이 순회 후 중복 center 제거.
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


def select_best_gap(gaps, escape_dir=0.0):
    """
    최적 갭 선택.

    [수정①] 점수식 개선:
      score = width × 0.45  -  |center| × 0.75  +  dir_bonus
      - 폭 가중치 0.3 → 0.45 (넓은 갭 우선도 ↑)
      - 각도 페널티 1.0 → 0.75 (각도에 덜 민감)

    [수정②] dir_bonus:
      escape_dir = +1 → 우측 갭(center > 0) 선호
      escape_dir = -1 → 좌측 갭(center < 0) 선호
      escape_dir =  0 → 보너스 없음

      dir_bonus = escape_dir × sign(center) × DIR_BONUS_WEIGHT
      → 방향이 일치하면 +DIR_BONUS_WEIGHT, 반대면 -DIR_BONUS_WEIGHT
    """
    if not gaps:
        return None
    passable = [g for g in gaps if g['passable']]
    pool = passable if passable else gaps

    def score(g):
        width_score = g['width'] * 0.45
        angle_cost  = abs(g['center']) * 0.75
        # center 부호: 양수=우측(CW 0~180°), 음수=좌측(CW 180~360°)
        dir_sign    = 1.0 if g['center'] > 0 else (-1.0 if g['center'] < 0 else 0.0)
        dir_bonus   = escape_dir * dir_sign * DIR_BONUS_WEIGHT
        return width_score - angle_cost + dir_bonus

    return max(pool, key=score)


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    """지정 방향(center_cw) ±arc_half° 내 최단 장애물 거리 반환."""
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d


# ── 메인 루프 ────────────────────────────────────────────────────────
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

    # ── 1회성 초기화 ──────────────────────────────────────────────────
    try:
        _ready
    except NameError:
        import atexit
        _ready     = True
        scan_buf   = []
        emg_cnt    = 0
        no_gap_cnt = 0
        extra_back = 0

        # [수정②] 방향 메모리 상태 변수
        escape_dir   = 0.0   # +1=우측 선호, -1=좌측 선호, 0=자유
        escape_hold  = 0     # 남은 방향 유지 사이클

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
        print("  VFH 장애물 회피 (방향 메모리 + 갭 점수식 개선)")
        print(f"  감지:{int(DETECT)}mm  긴급:{int(EMERGENCY)}mm  "
              f"최소갭폭:{int(GAP_MIN_PASS)}mm")
        print(f"  빈:{BIN_DEG:.0f}°×{N_BINS}개  회전전환:±{ROT_THRESH:.0f}°")
        print(f"  방향메모리: {ESCAPE_HOLD_CYCLES}사이클  보너스강도: {DIR_BONUS_WEIGHT:.0f}mm")
        print("=" * 65)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    # ── 1 스캔 완료 → VFH 판단 ────────────────────────────────────────
    if s_flag == 1 and len(scan_buf) > 15:

        # ── VFH 계산 ────────────────────────────────────────────────
        hist, has_pt = build_polar_hist(scan_buf)
        emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=70)

        # [수정②] 방향 메모리 감쇠 (매 사이클 1씩 줄임)
        if escape_hold > 0:
            escape_hold -= 1
            if escape_hold == 0:
                escape_dir = 0.0
                print("DIR_MEMORY 해제 → 방향 자유")

        if not any(has_pt):
            # 장애물 없음 → 직진
            ser_Ardu.write(b"F 0.00 0.70\n")

        else:
            # [수정②] escape_dir를 갭 선택에 전달
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps, escape_dir)

            # ── P1: 연장 후진 진행 중 ───────────────────────────────
            if extra_back > 0:
                ser_Ardu.write(b"B 0.80\n")
                extra_back -= 1
                print(f"EXTENDED_BACK 잔여 {extra_back}사이클")

            # ── P2: 긴급 근접 + 갭이 너무 옆에 있거나 없음 → 후진 ────
            elif (emg_near <= EMERGENCY and
                  (best is None or not best['passable'] or
                   abs(best['center']) > ROT_THRESH)):
                emg_cnt += 1
                if emg_cnt >= 6:
                    ser_Ardu.write(b"B 0.90\n")
                    extra_back = 3
                    emg_cnt    = 0
                    print("EXTENDED_BACK 시작! (3x) [긴급]")
                else:
                    # [수정②] 비상 후진 중에도 통과 가능한 갭이 있으면 방향 기억
                    if best is not None and best['passable']:
                        new_dir = 1.0 if best['center'] > 0 else -1.0
                        if escape_dir == 0.0 or escape_hold == 0:
                            escape_dir  = new_dir
                            escape_hold = ESCAPE_HOLD_CYCLES
                            side = "우" if new_dir > 0 else "좌"
                            print(f"  → 탈출방향 사전 결정: {side}  "
                                  f"(갭중심:{best['center']:+.0f}°  폭:{best['width']:.0f}mm)")
                    ser_Ardu.write(b"B 0.90\n")
                    print(f"EMERGENCY! 근접={emg_near:.0f}mm ({emg_cnt}/6)")

            # ── P3: VFH 전진 회피 — 갭이 ±ROT_THRESH 이내 ─────────────
            elif best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
                steer  = max(-MAX_STEER, min(MAX_STEER,
                             best['center'] / 90.0 * MAX_STEER))
                near_d = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=20)
                ratio  = min(max((DETECT - near_d) / (DETECT - EMERGENCY), 0.0), 1.0)
                speed  = 0.70 * (1.0 - ratio * 0.55)

                # [수정②] 유의미한 조향이면 방향 메모리 갱신
                if abs(steer) > 0.25:
                    new_dir = 1.0 if steer > 0 else -1.0
                    if new_dir != escape_dir:
                        side = "우" if new_dir > 0 else "좌"
                        print(f"  → 방향메모리 갱신: {side}  hold={ESCAPE_HOLD_CYCLES}사이클")
                    escape_dir  = new_dir
                    escape_hold = ESCAPE_HOLD_CYCLES

                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"VFH_FWD  갭={best['width']:.0f}mm@{best['center']:+.0f}°  "
                      f"D{best['delta_deg']:.0f}°  근접={near_d:.0f}mm  "
                      f"steer={steer:+.2f}  spd={speed:.2f}  "
                      f"dir_mem={escape_dir:+.0f}({escape_hold})")

            # ── P4: VFH 제자리 회전 — 갭이 ±ROT_THRESH 초과 ──────────
            elif best is not None and best['passable']:
                rot_dir = 1.0 if best['center'] > 0 else -1.0

                # [수정②] 제자리 회전 방향도 메모리에 저장
                if rot_dir != escape_dir:
                    side = "우" if rot_dir > 0 else "좌"
                    print(f"  → 방향메모리 갱신(회전): {side}  hold={ESCAPE_HOLD_CYCLES}사이클")
                escape_dir  = rot_dir
                escape_hold = ESCAPE_HOLD_CYCLES

                ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
                print(f"VFH_ROT  갭({best['center']:+.0f}°) → "
                      f"제자리회전 dir={rot_dir:+.0f}  폭={best['width']:.0f}mm  "
                      f"dir_mem={escape_dir:+.0f}({escape_hold})")

            # ── P5: 통과 가능한 갭 없음 → 후진 ────────────────────────
            else:
                no_gap_cnt += 1
                ser_Ardu.write(b"B 0.90\n")
                widest = max((g['width'] for g in gaps), default=0.0)
                print(f"VFH_BACK  통과 갭 없음 "
                      f"(최대폭={widest:.0f}mm < {GAP_MIN_PASS:.0f}mm)  ({no_gap_cnt})")
                if no_gap_cnt >= 6:
                    extra_back = 3
                    no_gap_cnt = 0
                    print("EXTENDED_BACK 시작! (3x) [갭없음]")

        # ── 버퍼 초기화 ────────────────────────────────────────────────
        scan_buf = []