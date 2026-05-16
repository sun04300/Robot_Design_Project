import serial
import time
import math
import sys
import atexit

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  VFH 실시간 장애물 회피 (RPLidar C1 최적화 - 초고속 기동 버전)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# ── [1] 하드웨어 즉시 초기화 (리셋/키보드 대기 제거) ─────────────────────
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=0.05)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=0.1)

print("라이다 SCAN 모드 즉시 진입 중...")
# [중요] SCAN 명령 전에 버퍼를 비워 잔여 노이즈 데이터 제거
ser_L.reset_input_buffer() 

# RESET 없이 바로 SCAN 명령 전송 (대기시간 0초)
ser_L.write(bytes([0xA5, 0x20]))   
desc = ser_L.read(7)               # 응답 디스크립터 7바이트 소비

if len(desc) != 7 or desc[0] != 0xA5 or desc[1] != 0x5A:
    print(f"[경고] 디스크립터 매칭 실패: {desc.hex() if desc else '데이터 없음'}")
    print("라이다가 이미 SCAN 중이거나 통신 일시 오류일 수 있습니다. 계속 진행합니다.")
else:
    print("라이다 C1 연결 및 SCAN 시작 완료.")

def _cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))   # STOP
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
        print("\n[종료] 하드웨어 정지 완료.")
    except Exception:
        pass

atexit.register(_cleanup)


# ── [2] 파라미터 ────────────────━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
BIN_DEG      = 5.0
N_BINS       = int(360 / BIN_DEG)   # 72개

ROBOT_WIDTH  = 200.0
GAP_MARGIN   = 15.0
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN

DETECT_DIST  = 350.0
EMERGENCY    = 150.0
ROT_THRESH   = 75.0
MAX_STEER    = 0.85

DIR_HOLD     = 6
DIR_BONUS    = 30.0
STEER_ALPHA  = 0.70   

SIDE_ARC     = 20.0
SIDE_DIST    = 210.0
SIDE_FACTOR  = 0.55
SIDE_PUSH    = 0.18

# 실시간 처리 파라미터
CTRL_PERIOD  = 0.05    # 50ms = 20Hz 제어
BIN_STALE    = 0.40    
MIN_VALID    = 15      # C1은 초반 데이터 수집이 빨라 가드를 살짝 낮춰도 안전합니다.


# ── [3] 빈 단위 히스토그램 (시간 기반) ────────────────━━━━━━━━━━━━━━━
hist     = [9999.0] * N_BINS
last_upd = [0.0]    * N_BINS

def read_one_sample():
    data = ser_L.read(5)
    if len(data) != 5:
        return None
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag): return None
    if (data[1] & 0x01) != 1:      return None
    if (data[0] >> 2) == 0:        return None   # quality == 0
    angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0
    if distance < 50.0:
        return None
    return angle, distance

def update_hist(angle, dist, now):
    idx = int(angle / BIN_DEG) % N_BINS
    if now - last_upd[idx] > BIN_STALE or dist < hist[idx]:
        hist[idx] = dist
    last_upd[idx] = now

def snapshot(now):
    h  = [9999.0] * N_BINS
    hp = [False]  * N_BINS
    for i in range(N_BINS):
        if now - last_upd[i] <= BIN_STALE:
            h[i]  = hist[i]
            hp[i] = True
    return h, hp


# ── [4] VFH 연산 함수 ─────────────────━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
def nearest_in_arc(h, hp, center_cw, arc_half):
    n  = max(1, int(arc_half / BIN_DEG))
    cb = int(center_cw / BIN_DEG) % N_BINS
    min_d = 9999.0
    for k in range(-n, n + 1):
        idx = (cb + k) % N_BINS
        if hp[idx] and h[idx] < min_d:
            min_d = h[idx]
    return min_d

def find_gaps(h, hp):
    blocked = [hp[i] and h[i] <= DETECT_DIST for i in range(N_BINS)]
    gaps, seen = [], set()
    i = 0
    while i < 2 * N_BINS:
        bi = i % N_BINS
        if not blocked[bi]:
            j = i + 1
            while j < i + N_BINS and not blocked[j % N_BINS]:
                j += 1
            span = j - i
            if span >= N_BINS:
                return [{'center': 0.0, 'center_cw': 0.0, 'width': 9999.0, 'passable': True}]
            center_cw = ((i + j) / 2.0 * BIN_DEG) % 360.0
            ck = round(center_cw)
            if ck not in seen:
                seen.add(ck)
                d_L = min(h[(i-1)%N_BINS], DETECT_DIST) if hp[(i-1)%N_BINS] else DETECT_DIST
                d_R = min(h[j%N_BINS],     DETECT_DIST) if hp[j%N_BINS]     else DETECT_DIST
                d_ref = min(d_L, d_R)
                gap_w = 2.0 * d_ref * math.sin(math.radians(span * BIN_DEG / 2.0))
                center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0
                gaps.append({
                    'center': center_s, 'center_cw': center_cw,
                    'width': gap_w, 'passable': gap_w >= GAP_MIN_PASS,
                })
            i = j
        else:
            i += 1
    return gaps

def select_best_gap(gaps, escape_dir):
    pool = [g for g in gaps if g['passable']] or gaps
    if not pool:
        return None
    def score(g):
        dir_sign  = 1.0 if g['center'] > 0 else (-1.0 if g['center'] < 0 else 0.0)
        dir_bonus = escape_dir * dir_sign * DIR_BONUS
        return g['width'] * 0.45 - abs(g['center']) * 0.75 + dir_bonus
    return max(pool, key=score)


# ── [5] 상태 변수 및 메인 루프 ────────━━━━━━━ Breaking 내역 없음 ━━━━━━
emg_cnt     = 0
no_gap_cnt  = 0
extra_back  = 0
escape_dir  = 0.0
escape_hold = 0
steer_prev  = 0.0
last_ctrl   = 0.0
last_log    = 0.0

print("=" * 65)
print("  VFH 실시간 회피 (C1 초고속 모드 - 즉시 기동)")
print("=" * 65)

while True:
    now = time.monotonic()
    while ser_L.in_waiting >= 5:
        s = read_one_sample()
        if s is not None:
            update_hist(s[0], s[1], now)

    if now - last_ctrl < CTRL_PERIOD:
        time.sleep(0.003)
        continue
    last_ctrl = now

    h, hp = snapshot(now)
    valid_count = sum(hp)

    # 360도 맵이 아주 최소한으로만 차도(MIN_VALID=15) 바로 출발
    if valid_count < MIN_VALID:
        ser_Ardu.write(b"F 0.00 0.20\n")  # 웜업 중엔 아주 천천히 서행
        if now - last_log > 0.3:
            print(f"WARMUP (유효빈 수집 중: {valid_count}/{N_BINS})")
            last_log = now
        continue

    if escape_hold > 0:
        escape_hold -= 1
        if escape_hold == 0:
            escape_dir = 0.0

    # [State A] 전방 완전 클리어
    if not any(hp[i] and h[i] <= DETECT_DIST for i in range(N_BINS)):
        ser_Ardu.write(b"F 0.00 0.70\n")
        steer_prev = 0.0
        if now - last_log > 0.3:
            print("CLEAR  직진")
            last_log = now
        continue

    gaps    = find_gaps(h, hp)
    best    = select_best_gap(gaps, escape_dir)
    front_d = nearest_in_arc(h, hp, 0.0, arc_half=70)

    # [State B] 연장 후진
    if extra_back > 0:
        ser_Ardu.write(b"B 0.85\n")
        extra_back -= 1
        steer_prev = 0.0
        if now - last_log > 0.2:
            print(f"EXTENDED_BACK  잔여 {extra_back}")
            last_log = now
        continue

    # [State C] 긴급 후진
    needs_emergency = (
        front_d <= EMERGENCY 및
        (best is None 또는 not best['passable'] 또는 abs(best['center']) > ROT_THRESH)
    )
    if needs_emergency:
        emg_cnt += 1
        if best is not None 및 best['passable'] 및 escape_hold == 0:
            escape_dir  = 1.0 if best['center'] > 0 else -1.0
            escape_hold = DIR_HOLD
        if emg_cnt >= 6:
            extra_back = 3
            emg_cnt = 0
        else:
            ser_Ardu.write(b"B 0.90\n")
            steer_prev = 0.0
            if now - last_log > 0.15:
                print(f"EMERGENCY  front={front_d:.0f}  ({emg_cnt}/6)")
                last_log = now
        continue
    emg_cnt = 0

    # [State D] VFH 전진
    if best is not None 및 best['passable'] 및 abs(best['center']) <= ROT_THRESH:
        no_gap_cnt = 0
        steer_target = max(-MAX_STEER, min(MAX_STEER, best['center'] / 90.0 * MAX_STEER))
        steer = STEER_ALPHA * steer_target + (1.0 - STEER_ALPHA) * steer_prev
        steer_prev = steer

        ratio = min(max((DETECT_DIST - front_d) / (DETECT_DIST - EMERGENCY), 0.0), 1.0)
        speed = 0.70 * (1.0 - ratio * 0.65)

        side_R = nearest_in_arc(h, hp,  60.0, arc_half=SIDE_ARC)
        side_L = nearest_in_arc(h, hp, 300.0, arc_half=SIDE_ARC)
        side_min = min(side_R, side_L)
        if side_min < SIDE_DIST:
            speed *= SIDE_FACTOR
            push   = SIDE_PUSH * (1.0 if side_R < side_L else -1.0)
            steer  = max(-MAX_STEER, min(MAX_STEER, steer + push))

        if abs(steer) > 0.25:
            escape_dir  = 1.0 if steer > 0 else -1.0
            escape_hold = DIR_HOLD

        ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
        if now - last_log > 0.2:
            print(f"FWD  갭@{best['center']:+.0f}°({best['width']:.0f})  front={front_d:.0f}  steer={steer:+.2f}")
            last_log = now
        continue

    # [State E] 제자리 회전
    if best is not None 및 best['passable']:
        rot_dir = 1.0 if best['center'] > 0 else -1.0
        escape_dir  = rot_dir
        escape_hold = DIR_HOLD
        steer_prev  = 0.0
        ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
        if now - last_log > 0.2:
            print(f"ROT  갭@{best['center']:+.0f}°  폭={best['width']:.0f}")
            last_log = now
        continue

    # [State F] 통과 가능한 갭 없음 → 후진
    no_gap_cnt += 1
    steer_prev  = 0.0
    widest = max((g['width'] for g in gaps), default=0.0)
    ser_Ardu.write(b"B 0.70\n")
    if now - last_log > 0.2:
        print(f"BACK  최대갭={widest:.0f} < {GAP_MIN_PASS:.0f}  ({no_gap_cnt})")
        last_log = now
    if no_gap_cnt >= 6:
        extra_back = 3
        no_gap_cnt = 0
