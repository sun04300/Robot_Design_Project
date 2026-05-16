import serial
import time
import math
import sys
import tty
import termios

def wait_for_start():
    """
    S 키를 누를 때까지 대기.
    메인 루프 시작 직전에 한 번만 호출.
    """
    print("\n[준비 완료] S 키를 누르면 시작합니다.")

    while True:
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1).lower()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

        if key == 's':
            print("\n[START] 스캔 시작\n")
            break


port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

wait_for_start()
ser_L.reset_input_buffer()
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  VFH 기반 장애물 회피 — 코너 진입 안정화 (v3c4)
#
#  [v3c3 → v3c4 변경 요약]
#    ④ 정면 우선(STRAIGHT_PRIO): 정면 ±FWD_CLEAR_ARC° 가 FWD_CLEAR_DIST mm
#       이상 트여있으면 갭 선택 무시하고 직진 → 옆 갭 쫓아가는 미세 회전 차단
#    ⑤ 조향 데드존: 갭 중심이 ±STEER_DEADZONE_DEG° 이내면 조향 강제 0
#       → 직진 중 누적되는 미세 기울임 차단 (코너 부정렬 진입의 주원인)
#    ⑥ 정면 갭 보너스: 점수식에 |center|<CENTER_BONUS_DEG° 갭 + 보너스
#       → 통과 가능한 정면 갭이 옆 갭한테 뺏기지 않게 함
#    ⑦ 조향 저역통과(LPF): steer = α·target + (1-α)·prev
#       → 회전 중 흔들림 완화, 갭 전환 시 급격한 조향 변동 감쇠
#    ⑧ 측면 근접 감속: 좌·우 전측방에 가까운 장애물 있으면 속도 ×배율
#       → 코너 내벽 클리핑 완화
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# ── 기본 파라미터 ────────────────────────────────────────────────────
BIN_DEG      = 5.0
N_BINS       = int(360 / BIN_DEG)
ROBOT_WIDTH  = 200.0
GAP_MARGIN   = 10.0
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN
DETECT       = 330.0
EMERGENCY    = 140.0
MAX_STEER    = 0.85
ROT_THRESH   = 75.0

# 방향 메모리 (v3c3)
ESCAPE_HOLD_CYCLES = 6
DIR_BONUS_WEIGHT   = 30.0

# ── 신규: 코너 진입 안정화 파라미터 (v3c4) ─────────────────────────
FWD_CLEAR_ARC      = 20.0     # 정면 안전 확인 호 반각 (°)
FWD_CLEAR_DIST     = 430.0    # 이 거리 이상 트이면 직진 우선 (mm)
STEER_DEADZONE_DEG = 12.0     # 갭 중심 |°| 이 이하면 조향 0
CENTER_BONUS_DEG   = 25.0     # 정면 갭 보너스 인정 범위 (°)
CENTER_BONUS_W     = 40.0     # 정면 갭 보너스 강도 (mm 환산)

STEER_ALPHA        = 0.6      # 조향 저역통과 (0=완전평활, 1=즉시반응)

SIDE_BRAKE_ARC     = 20.0     # 측면 검사 호 반각 (°)
SIDE_BRAKE_CENTERS = (60.0, 300.0)  # (우전측, 좌전측) CW 각도
SIDE_BRAKE_DIST    = 210.0    # 이 이하면 감속 (mm)
SIDE_BRAKE_FACTOR  = 0.55     # 감속 배율


# ── VFH 보조 함수 ────────────────────────────────────────────────────

def build_polar_hist(scan_buf):
    """스캔 버퍼로 360° 폴라 히스토그램 생성."""
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt


def find_vfh_gaps(hist, has_pt, detect_dist, min_pass_mm):
    """히스토그램에서 통과 가능한 갭을 모두 탐색 (wraparound 처리)."""
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

    점수식:
      score = width × 0.45  -  |center| × 0.75
             + dir_bonus(방향 메모리)
             + center_bonus(정면 갭 보너스)   ← v3c4 추가
    """
    if not gaps:
        return None
    passable = [g for g in gaps if g['passable']]
    pool = passable if passable else gaps

    def score(g):
        width_score  = g['width'] * 0.45
        angle_cost   = abs(g['center']) * 0.75
        dir_sign     = 1.0 if g['center'] > 0 else (-1.0 if g['center'] < 0 else 0.0)
        dir_bonus    = escape_dir * dir_sign * DIR_BONUS_WEIGHT
        # [v3c4] 정면 갭 보너스
        center_bonus = CENTER_BONUS_W if abs(g['center']) < CENTER_BONUS_DEG else 0.0
        return width_score - angle_cost + dir_bonus + center_bonus

    return max(pool, key=score)


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    """지정 방향 ±arc_half° 내 최단 장애물 거리."""
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
    if distance < 50.0:
        continue

    # ── 1회성 초기화 ──────────────────────────────────────────────────
    try:
        _ready
    except NameError:
        import atexit
        _ready       = True
        scan_buf     = []
        emg_cnt      = 0
        no_gap_cnt   = 0
        extra_back   = 0

        # 방향 메모리
        escape_dir   = 0.0
        escape_hold  = 0

        # [v3c4] 조향 저역통과 상태
        steer_prev   = 0.0

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

        print("=" * 70)
        print("  VFH 장애물 회피 v3c4 — 코너 진입 안정화")
        print(f"  감지:{int(DETECT)}mm  긴급:{int(EMERGENCY)}mm  최소갭폭:{int(GAP_MIN_PASS)}mm")
        print(f"  빈:{BIN_DEG:.0f}°×{N_BINS}개  회전전환:±{ROT_THRESH:.0f}°")
        print(f"  방향메모리: {ESCAPE_HOLD_CYCLES}사이클  보너스: {DIR_BONUS_WEIGHT:.0f}mm")
        print(f"  직진우선: ±{FWD_CLEAR_ARC:.0f}° / {FWD_CLEAR_DIST:.0f}mm 이상 트임시")
        print(f"  데드존: ±{STEER_DEADZONE_DEG:.0f}°  정면보너스: ±{CENTER_BONUS_DEG:.0f}°/{CENTER_BONUS_W:.0f}mm")
        print(f"  조향평활 α: {STEER_ALPHA:.2f}")
        print(f"  측면감속: {SIDE_BRAKE_DIST:.0f}mm 이내 → ×{SIDE_BRAKE_FACTOR:.2f}")
        print("=" * 70)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    # ── 1 스캔 완료 → VFH 판단 ────────────────────────────────────────
    if s_flag == 1 and len(scan_buf) > 15:

        hist, has_pt = build_polar_hist(scan_buf)
        emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=70)

        # 방향 메모리 감쇠
        if escape_hold > 0:
            escape_hold -= 1
            if escape_hold == 0:
                escape_dir = 0.0
                print("DIR_MEMORY 해제 → 방향 자유")

        if not any(has_pt):
            # 장애물 없음 → 직진
            ser_Ardu.write(b"F 0.00 0.70\n")
            steer_prev = 0.0   # [v3c4] 평활 상태 리셋

        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps, escape_dir)

            # ── P1: 연장 후진 진행 중 ───────────────────────────────
            if extra_back > 0:
                ser_Ardu.write(b"B 0.80\n")
                extra_back -= 1
                steer_prev = 0.0
                print(f"EXTENDED_BACK 잔여 {extra_back}사이클")

            # ── P2: 긴급 후진 ────────────────────────────────────────
            elif (emg_near <= EMERGENCY and
                  (best is None or not best['passable'] or
                   abs(best['center']) > ROT_THRESH)):
                emg_cnt += 1
                if emg_cnt >= 6:
                    ser_Ardu.write(b"B 0.90\n")
                    extra_back = 3
                    emg_cnt    = 0
                    steer_prev = 0.0
                    print("EXTENDED_BACK 시작! (3x) [긴급]")
                else:
                    # 비상 후진 중에도 통과 가능한 갭이 있으면 방향 기억
                    if best is not None and best['passable']:
                        new_dir = 1.0 if best['center'] > 0 else -1.0
                        if escape_dir == 0.0 or escape_hold == 0:
                            escape_dir  = new_dir
                            escape_hold = ESCAPE_HOLD_CYCLES
                            side = "우" if new_dir > 0 else "좌"
                            print(f"  → 탈출방향 사전 결정: {side}  "
                                  f"(갭중심:{best['center']:+.0f}°  폭:{best['width']:.0f}mm)")
                    ser_Ardu.write(b"B 0.90\n")
                    steer_prev = 0.0
                    print(f"EMERGENCY! 근접={emg_near:.0f}mm ({emg_cnt}/6)")

            # ── P3: VFH 전진 회피 [v3c4 핵심 수정] ─────────────────────
            elif best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:

                # ① 정면 클리어런스 확인
                fwd_clear = nearest_in_arc(hist, has_pt, 0.0,
                                           arc_half=int(FWD_CLEAR_ARC))

                # ② 조향 목표값 결정 (3-mode)
                if fwd_clear >= FWD_CLEAR_DIST:
                    # 정면 안전 → 갭 무시하고 직진
                    steer_target = 0.0
                    near_d       = fwd_clear
                    mode_tag     = "STRAIGHT_PRIO"
                elif abs(best['center']) <= STEER_DEADZONE_DEG:
                    # 갭이 거의 정면 → 데드존, 조향 0
                    steer_target = 0.0
                    near_d       = nearest_in_arc(hist, has_pt,
                                                  best['center_cw'], arc_half=20)
                    mode_tag     = "DEADZONE"
                else:
                    # 일반 비례 조향
                    steer_target = max(-MAX_STEER, min(MAX_STEER,
                                       best['center'] / 90.0 * MAX_STEER))
                    near_d       = nearest_in_arc(hist, has_pt,
                                                  best['center_cw'], arc_half=20)
                    mode_tag     = "STEER"

                # ③ 조향 저역통과
                steer = STEER_ALPHA * steer_target + (1.0 - STEER_ALPHA) * steer_prev
                steer_prev = steer

                # ④ 속도 결정 (전방 근접 → 감속)
                ratio = min(max((DETECT - near_d) / (DETECT - EMERGENCY), 0.0), 1.0)
                speed = 0.70 * (1.0 - ratio * 0.7)

                # ⑤ 측면 근접 감속 (코너 내벽 클리핑 완화)
                side_min = 9999.0
                for sc in SIDE_BRAKE_CENTERS:
                    sd = nearest_in_arc(hist, has_pt, sc,
                                        arc_half=int(SIDE_BRAKE_ARC))
                    if sd < side_min:
                        side_min = sd
                if side_min < SIDE_BRAKE_DIST:
                    speed *= SIDE_BRAKE_FACTOR
                    mode_tag += "+SIDE"

                # ⑥ 방향 메모리 갱신 (유의미한 조향일 때만)
                if abs(steer) > 0.25:
                    new_dir = 1.0 if steer > 0 else -1.0
                    if new_dir != escape_dir:
                        side = "우" if new_dir > 0 else "좌"
                        print(f"  → 방향메모리 갱신: {side}  hold={ESCAPE_HOLD_CYCLES}사이클")
                    escape_dir  = new_dir
                    escape_hold = ESCAPE_HOLD_CYCLES

                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"VFH_FWD[{mode_tag}]  갭={best['width']:.0f}mm@{best['center']:+.0f}°  "
                      f"근접={near_d:.0f}mm  측면={side_min:.0f}mm  "
                      f"steer={steer:+.2f}  spd={speed:.2f}  "
                      f"dir_mem={escape_dir:+.0f}({escape_hold})")

            # ── P4: VFH 제자리 회전 ──────────────────────────────────
            elif best is not None 및 best['passable']:
                rot_dir = 1.0 if best['center'] > 0 else -1.0

                if rot_dir != escape_dir:
                    side = "우" if rot_dir > 0 else "좌"
                    print(f"  → 방향메모리 갱신(회전): {side}  hold={ESCAPE_HOLD_CYCLES}사이클")
                escape_dir  = rot_dir
                escape_hold = ESCAPE_HOLD_CYCLES
                steer_prev  = 0.0   # 회전 후 전진 시 새로 시작

                ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
                print(f"VFH_ROT  갭({best['center']:+.0f}°) → "
                      f"제자리회전 dir={rot_dir:+.0f}  폭={best['width']:.0f}mm  "
                      f"dir_mem={escape_dir:+.0f}({escape_hold})")

            # ── P5: 통과 가능한 갭 없음 → 후진 ───────────────────────
            else:
                no_gap_cnt += 1
                ser_Ardu.write(b"B 0.70\n")
                steer_prev = 0.0
                widest = max((g['width'] for g in gaps), default=0.0)
                print(f"VFH_BACK  통과 갭 없음 "
                      f"(최대폭={widest:.0f}mm < {GAP_MIN_PASS:.0f}mm)  ({no_gap_cnt})")
                if no_gap_cnt >= 6:
                    extra_back = 3
                    no_gap_cnt = 0
                    print("EXTENDED_BACK 시작! (3x) [갭없음]")

        # ── 버퍼 초기화 ────────────────────────────────────────────────
        scan_buf = []
