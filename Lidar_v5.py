import serial
import time
import math
import sys
import tty
import termios
import atexit

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  VFH 기반 장애물 회피 — 최적화 버전
#
#  [v3c4 대비 주요 변경]
#  ① STRAIGHT_PRIO 제거  : "전방 완전 클리어" 가드(State A)가 이미 처리
#  ② STEER_DEADZONE 제거 : LPF가 미세진동을 자연스럽게 흡수; 하드컷은 필요한 보정도 막음
#  ③ CENTER_BONUS 제거   : DIR_BONUS와 역할 중복 → 두 가중치가 서로 간섭
#  ④ side_min 이중계산 버그 수정 (v3c4는 루프 결과를 곧바로 덮어썼음)
#  ⑤ 초기화 루프 밖으로 이동 (try:_ready except NameError 패턴 제거)
#  ⑥ 갭 점수식 단일화: width×0.45 − |center|×0.75 + dir_bonus
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━


# ── [1] 키보드 시작 대기 ─────────────────────────────────────────────
def wait_for_start():
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
            print("\n[START] 주행을 시작합니다.\n")
            break


# ── [2] 하드웨어 초기화 ──────────────────────────────────────────────
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

wait_for_start()

print("라이다 모터 안정화 중 (1.5초)...")
ser_L.reset_input_buffer()
time.sleep(1.5)
ser_L.reset_input_buffer()

def _cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
        print("\n[종료] 하드웨어 정지 완료.")
    except Exception:
        pass

atexit.register(_cleanup)


# ── [3] 파라미터 ─────────────────────────────────────────────────────
BIN_DEG      = 5.0
N_BINS       = int(360 / BIN_DEG)   # 72개 빈

ROBOT_WIDTH  = 200.0
GAP_MARGIN   = 60.0
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 최소 통과 너비: 260mm

DETECT_DIST  = 350.0   # 장애물 감지 거리 (mm)
EMERGENCY    = 150.0   # 긴급 후진 임계 거리 (mm)
ROT_THRESH   = 75.0    # 이 각도 초과 시 제자리 회전 (°)
MAX_STEER    = 0.85    # 최대 조향 값

# 방향 메모리 — 갭 전환 시 좌우 진동 방지
DIR_HOLD     = 6       # 메모리 유지 사이클 수
DIR_BONUS    = 30.0    # 동일 방향 갭 점수 보너스 (mm 환산)

# 조향 저역통과(LPF) — 급격한 핸들 꺾임 완화, STEER_DEADZONE 불필요하게 만듦
STEER_ALPHA  = 0.60    # 1.0=즉시반응, 0.0=완전평활

# 측면 근접 감속 — 코너 내벽 클리핑 방지
SIDE_ARC     = 20.0    # 측면 검사 호 반각 (°)
SIDE_DIST    = 210.0   # 이 거리 이내면 감속 (mm)
SIDE_FACTOR  = 0.55    # 감속 배율
SIDE_PUSH    = 0.18    # 가까운 벽의 반대 방향으로 밀어내는 조향 보정량


# ── [4] VFH 연산 함수 ────────────────────────────────────────────────
def build_polar_hist(scan_buf):
    """라이다 스캔 버퍼 → 360° 폴라 히스토그램 (각 빈의 최단 거리)."""
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt


def nearest_in_arc(hist, has_pt, center_cw, arc_half):
    """center_cw ± arc_half° 범위에서 가장 가까운 장애물 거리 반환."""
    n  = max(1, int(arc_half / BIN_DEG))
    cb = int(center_cw / BIN_DEG) % N_BINS
    min_d = 9999.0
    for k in range(-n, n + 1):
        idx = (cb + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d


def find_gaps(hist, has_pt):
    """
    장애물 히스토그램에서 빈 공간(갭)을 탐색.
    wraparound(0°=360° 연결) 처리 포함.
    """
    blocked = [has_pt[i] and hist[i] <= DETECT_DIST for i in range(N_BINS)]
    gaps, seen = [], set()
    i = 0
    while i < 2 * N_BINS:
        bi = i % N_BINS
        if not blocked[bi]:
            j = i + 1
            while j < i + N_BINS and not blocked[j % N_BINS]:
                j += 1
            span = j - i

            if span >= N_BINS:               # 360° 전체가 클리어
                return [{'center': 0.0, 'center_cw': 0.0,
                          'width': 9999.0,   'passable': True}]

            center_cw = ((i + j) / 2.0 * BIN_DEG) % 360.0
            ck = round(center_cw)
            if ck not in seen:
                seen.add(ck)
                d_L  = min(hist[(i-1)%N_BINS], DETECT_DIST) if has_pt[(i-1)%N_BINS] else DETECT_DIST
                d_R  = min(hist[j%N_BINS],     DETECT_DIST) if has_pt[j%N_BINS]     else DETECT_DIST
                d_ref = min(d_L, d_R)
                gap_w = 2.0 * d_ref * math.sin(math.radians(span * BIN_DEG / 2.0))
                center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0
                gaps.append({
                    'center'   : center_s,
                    'center_cw': center_cw,
                    'width'    : gap_w,
                    'passable' : gap_w >= GAP_MIN_PASS,
                })
            i = j
        else:
            i += 1
    return gaps


def select_best_gap(gaps, escape_dir):
    """
    최적 갭 선택.

    점수식 (단일, 충돌하는 보너스 없음):
      score = width × 0.45  -  |center| × 0.75  +  방향메모리_보너스
              ↑ 넓을수록 좋음   ↑ 정면에 가까울수록 좋음  ↑ 이전 방향 유지

    CENTER_BONUS / STEER_DEADZONE 을 제거한 이유:
    - CENTER_BONUS 는 DIR_BONUS 와 "정면 선호" 역할이 겹쳐 가중치가 두 번 들어감
    - 위 두 항이 이미 정면 근처 갭을 선호하게 설계되어 있음
    """
    pool = [g for g in gaps if g['passable']] or gaps
    if not pool:
        return None

    def score(g):
        dir_sign  = 1.0 if g['center'] > 0 else (-1.0 if g['center'] < 0 else 0.0)
        dir_bonus = escape_dir * dir_sign * DIR_BONUS
        return g['width'] * 0.45 - abs(g['center']) * 0.75 + dir_bonus

    return max(pool, key=score)


# ── [5] 메인 루프 변수 초기화 (루프 밖에서, 정상적으로) ───────────────
scan_buf    = []
emg_cnt     = 0
no_gap_cnt  = 0
extra_back  = 0
escape_dir  = 0.0
escape_hold = 0
steer_prev  = 0.0

# 워밍업: 라이다 스핀업 직후 첫 N 회전은 거리값이 부정확 → 건너뜀
# Code 1의 "시작하자마자 VFH_BACK" 원인 — 첫 스캔 노이즈 데이터가
# 히스토그램에 올라가 통과 가능한 갭이 없는 것처럼 보임
WARMUP_SCANS = 3          # 건너뛸 초기 스캔 수 (라이다 15Hz 기준 약 0.2초)
warmup_count = 0

print("=" * 65)
print("  VFH 장애물 회피 — 최적화 버전")
print(f"  감지:{int(DETECT_DIST)}mm  긴급:{int(EMERGENCY)}mm  최소갭:{int(GAP_MIN_PASS)}mm")
print(f"  회전전환:±{ROT_THRESH:.0f}°  LPF α:{STEER_ALPHA:.2f}")
print(f"  방향메모리: {DIR_HOLD}사이클 / 보너스: {DIR_BONUS:.0f}")
print(f"  측면감속: {SIDE_DIST:.0f}mm 이내 → ×{SIDE_FACTOR:.2f} / 보정:{SIDE_PUSH:.2f}")
print("=" * 65)


# ── [6] 메인 루프 ───────────────────────────────────────────────────
while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    # 라이다 패킷 검증 (체크섬 + 동기 비트)
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): continue
    if (data[1] & 0x01) != 1:      continue
    if (data[0] >> 2) == 0:        continue   # quality == 0 → 노이즈

    angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0
    if distance < 50.0:
        continue

    scan_buf.append((angle, distance))

    # 1회전 미완성이면 계속 수집
    if s_flag != 1 or len(scan_buf) <= 100:
        continue

    # ── 워밍업 스캔 건너뛰기 ────────────────────────────────────────
    #   라이다 스핀업 직후 첫 WARMUP_SCANS 회전은 쓰레기 데이터일 수 있음.
    #   이걸 그냥 쓰면 히스토그램이 오염 → 갭 없음 판단 → 즉시 후진 발생.
    #   (Code 1의 시작 직후 VFH_BACK 버그의 근본 원인)
    scan_buf = []
    if warmup_count < WARMUP_SCANS:
        warmup_count += 1
        ser_Ardu.write(b"S\n")   # 워밍업 중 정지 유지
        print(f"WARMUP  ({warmup_count}/{WARMUP_SCANS}) 스캔 안정화 중...")
        continue

    # ── 히스토그램 구축 ────────────────────────────────────────────
    hist, has_pt = build_polar_hist(scan_buf)

    # 방향 메모리 감쇠 (매 사이클 1씩 차감)
    if escape_hold > 0:
        escape_hold -= 1
        if escape_hold == 0:
            escape_dir = 0.0

    # ── [State A] 전방 완전 클리어 → 직진 ───────────────────────────
    #   "STRAIGHT_PRIO" 를 별도 파라미터로 만들 필요가 없는 이유:
    #   장애물이 DETECT_DIST 안에 하나도 없으면 여기서 바로 처리됨
    if not any(has_pt[i] and hist[i] <= DETECT_DIST for i in range(N_BINS)):
        ser_Ardu.write(b"F 0.00 0.70\n")
        steer_prev = 0.0
        print(f"CLEAR  직진")
        continue

    # ── 갭 탐색 (B~F 공통) ──────────────────────────────────────────
    gaps = find_gaps(hist, has_pt)
    best = select_best_gap(gaps, escape_dir)

    # 전방 근접 거리 (±70° — 측전방 포함)
    front_d = nearest_in_arc(hist, has_pt, 0.0, arc_half=70)

    # ── [State B] 연장 후진 진행 중 ─────────────────────────────────
    if extra_back > 0:
        ser_Ardu.write(b"B 0.85\n")
        extra_back -= 1
        steer_prev  = 0.0
        print(f"EXTENDED_BACK  잔여 {extra_back}사이클")
        continue

    # ── [State C] 긴급 후진 ─────────────────────────────────────────
    #   조건: 전방 너무 가까운데 꺾을 수 있는 갭도 없는 상황
    needs_emergency = (
        front_d <= EMERGENCY and
        (best is None or not best['passable'] or abs(best['center']) > ROT_THRESH)
    )
    if needs_emergency:
        emg_cnt += 1
        # 후진 중에도 탈출 방향을 미리 기억해둠
        if best is not None and best['passable'] and escape_hold == 0:
            escape_dir  = 1.0 if best['center'] > 0 else -1.0
            escape_hold = DIR_HOLD
            print(f"  → 탈출방향 사전결정: {'우' if escape_dir>0 else '좌'}")
        if emg_cnt >= 6:
            extra_back = 3
            emg_cnt    = 0
            print("EXTENDED_BACK 시작! (긴급, 3사이클)")
        else:
            ser_Ardu.write(b"B 0.90\n")
            steer_prev = 0.0
            print(f"EMERGENCY  front={front_d:.0f}mm ({emg_cnt}/6)")
        continue

    emg_cnt = 0   # 긴급 해제

    # ── [State D] VFH 전진 회피 ─────────────────────────────────────
    if best is not None 및 best['passable'] 및 abs(best['center']) <= ROT_THRESH:
        no_gap_cnt = 0

        # 비례 조향 목표
        steer_target = max(-MAX_STEER,
                       min( MAX_STEER, best['center'] / 90.0 * MAX_STEER))

        # LPF 적용 — STEER_DEADZONE 없이도 미세진동을 흡수
        steer = STEER_ALPHA * steer_target + (1.0 - STEER_ALPHA) * steer_prev
        steer_prev = steer

        # 속도: 전방 거리에 비례해 선형 감속 (0.25 ~ 0.70)
        ratio = min(max((DETECT_DIST - front_d) / (DETECT_DIST - EMERGENCY), 0.0), 1.0)
        speed = 0.70 * (1.0 - ratio * 0.65)

        # 측면 근접 감속 + 반발 조향 (버그 수정: 단일 계산)
        side_R = nearest_in_arc(hist, has_pt,  60.0, arc_half=SIDE_ARC)
        side_L = nearest_in_arc(hist, has_pt, 300.0, arc_half=SIDE_ARC)
        side_min = min(side_R, side_L)
        if side_min < SIDE_DIST:
            speed *= SIDE_FACTOR
            push   = SIDE_PUSH * (1.0 if side_R < side_L else -1.0)   # 가까운 쪽 반대로
            steer  = max(-MAX_STEER, min(MAX_STEER, steer + push))

        # 방향 메모리 갱신 (유의미한 조향에서만)
        if abs(steer) > 0.25:
            new_dir = 1.0 if steer > 0 else -1.0
            if new_dir != escape_dir:
                print(f"  → 방향메모리: {'우' if new_dir>0 else '좌'}  hold={DIR_HOLD}")
            escape_dir  = new_dir
            escape_hold = DIR_HOLD

        ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
        print(f"FWD  갭@{best['center']:+.0f}°({best['width']:.0f}mm)  "
              f"front={front_d:.0f}mm  side={side_min:.0f}mm  "
              f"steer={steer:+.2f}  spd={speed:.2f}  "
              f"mem={escape_dir:+.0f}({escape_hold})")
        continue

    # ── [State E] 제자리 회전 ────────────────────────────────────────
    #   갭이 ROT_THRESH(75°)보다 측면에 있어 전진으로는 진입 불가
    if best is not None 및 best['passable']:
        rot_dir = 1.0 if best['center'] > 0 else -1.0
        if rot_dir != escape_dir:
            print(f"  → 방향메모리(회전): {'우' if rot_dir>0 else '좌'}  hold={DIR_HOLD}")
        escape_dir  = rot_dir
        escape_hold = DIR_HOLD
        steer_prev  = 0.0   # 회전 후 전진 시 LPF를 새로 시작
        ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
        print(f"ROT  갭@{best['center']:+.0f}°  dir={'우' if rot_dir>0 else '좌'}  "
              f"폭={best['width']:.0f}mm  mem={escape_dir:+.0f}({escape_hold})")
        continue

    # ── [State F] 통과 가능한 갭 없음 → 후진 ──────────────────────
    no_gap_cnt += 1
    steer_prev  = 0.0
    widest = max((g['width'] for g in gaps), default=0.0)
    ser_Ardu.write(b"B 0.70\n")
    print(f"BACK  최대갭={widest:.0f}mm < {GAP_MIN_PASS:.0f}mm  ({no_gap_cnt})")
    if no_gap_cnt >= 6:
        extra_back = 3
        no_gap_cnt = 0
        print("EXTENDED_BACK 시작! (갭없음, 3사이클)")
