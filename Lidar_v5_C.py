import serial
import time
import math
import sys
import atexit

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  플랜 C: 우측 벽면 추종 주행 (RPLidar C1 최적화)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# ── [1] 하드웨어 즉시 초기화 ────────────────────────────────────────
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=0.05)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=0.1)

print("[Plan C] 라이다 SCAN 모드 즉시 진입 중...")
ser_L.reset_input_buffer() 

ser_L.write(bytes([0xA5, 0x20]))   
desc = ser_L.read(7)               

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
# 아두이노 모터 특성에 맞춘 제어 주기 설정
N_BINS       = int(360 / BIN_DEG)   # 72개

DETECT_DIST  = 350.0   # 전방 감지 거리
EMERGENCY    = 150.0   # 전방 긴급 제동 거리
MAX_STEER    = 0.85    # 최대 조향각 제한

# [벽 추종 핵심 튜닝 파라미터]
TARGET_WALL_DIST = 230.0  # 우측 벽과 유지할 목표 거리 (23cm)
KP               = 0.005  # P-Control 비례 게인 (로봇 반응이 둔하면 0.007로 상향, 털면 낮춤)

# 실시간 처리 파라미터
CTRL_PERIOD  = 0.05    
BIN_STALE    = 0.40    
MIN_VALID    = 15      


# ── [3] 빈 단위 히스토그램 및 호(Arc) 연산 ────────────────━━━━━━━━━━━━━
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
    if (data[0] >> 2) == 0:        return None   
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

def nearest_in_arc(h, hp, center_cw, arc_half):
    n  = max(1, int(arc_half / BIN_DEG))
    cb = int(center_cw / BIN_DEG) % N_BINS
    min_d = 9999.0
    for k in range(-n, n + 1):
        idx = (cb + k) % N_BINS
        if hp[idx] and h[idx] < min_d:
            min_d = h[idx]
    return min_d


# ── [4] 메인 루프 ─────────────────━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
last_ctrl      = 0.0
last_log       = 0.0
sync_error_cnt = 0

print("=" * 65)
print("  [Plan C] 우측 벽면 추종(Wall-Following) 주행 시작")
print("=" * 65)

while True:
    now = time.monotonic()
    
    while ser_L.in_waiting >= 5:
        s = read_one_sample()
        if s is not None:
            update_hist(s[0], s[1], now)
            sync_error_cnt = 0
        else:
            sync_error_cnt += 1
            if sync_error_cnt > 10:
                ser_L.reset_input_buffer()
                sync_error_cnt = 0
                break

    if now - last_ctrl < CTRL_PERIOD:
        time.sleep(0.003)
        continue
    last_ctrl = now

    h, hp = snapshot(now)
    valid_count = sum(hp)

    if valid_count < MIN_VALID:
        ser_Ardu.write(b"F 0.00 0.20\n")
        continue

    # 1. 센서 영역 감지 (호 범위 추출)
    # 정면 범위 (충돌 제동용 시야 확보)
    dist_front = nearest_in_arc(h, hp, 0.0, arc_half=30)
    # 우측 벽 범위 (라이다 기준 우측 90도 방향인 270도를 타겟으로 넓게 서치)
    dist_wall  = nearest_in_arc(h, hp, 270.0, arc_half=25)

    # 2. 제어 로직 (State Machine)
    # [Emergency Step] 전방에 박스가 갑자기 다가와 충돌 각이면 우선 회피 제자리 회전
    if dist_front <= EMERGENCY:
        ser_Ardu.write(b"T -0.60\n")  # 우측 벽과의 거리를 벌리기 위해 반시계(좌) 제자리 회전
        if now - last_log > 0.2:
            print(f"[Plan C] 전방 벽 발생! 회전 복귀 중... (Front: {dist_front:.0f}mm)")
            last_log = now
        continue

    # [Wall Tracking Step] 우측 벽과의 거리 오차를 계산하여 주행 조향 제어
    if dist_wall < 1000.0:  # 벽이 1미터 이내 유효거리에 찍히는 경우
        error = dist_wall - TARGET_WALL_DIST
        
        # 오차가 음수(가까움) -> 좌측(-) 조향하여 벽에서 떨어짐
        # 오차가 양수(머나먼) -> 우측(+) 조향하여 벽으로 붙음
        steer_target = error * KP
        steer = max(-MAX_STEER, min(MAX_STEER, steer_target))
        
        # 전방 거리 안전도에 따라 속도를 다이나믹 감속 제어
        speed_ratio = min(max((dist_front - EMERGENCY) / (DETECT_DIST - EMERGENCY), 0.0), 1.0)
        speed = 0.35 + 0.30 * speed_ratio
        state_str = "벽 추종 주행"
    else:
        # [Wall Search Step] 우측 벽이 유실된 경우 (코너 구간 등) 벽을 잡기 위해 완만한 우회전
        steer = 0.40
        speed = 0.40
        state_str = "우측 벽 탐색 유도"

    # 아두이노 명령 전송
    ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
    if now - last_log > 0.2:
        print(f"[Plan C] {state_str} | Wall:{dist_wall:.0f}mm F:{dist_front:.0f}mm | Steer:{steer:+.2f} Speed:{speed:.2f}")
        last_log = now