import serial
import time
import math
import sys
import atexit

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  플랜 B: 3분할 섹터 기반 직관적 회피 (RPLidar C1 최적화)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# ── [1] 하드웨어 즉시 초기화 ────────────────────────────────────────
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=0.05)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=0.1)

print("[Plan B] 라이다 SCAN 모드 즉시 진입 중...")
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
N_BINS       = int(360 / BIN_DEG)   # 72개

DETECT_DIST  = 350.0   # 장애물 감지 기준 거리 (35cm)
EMERGENCY    = 150.0   # 긴급 후진 기준 거리 (15cm)
MAX_STEER    = 0.85    # 최대 조향각 제한

SIDE_DIST    = 210.0   # 측면 벽 기피 거리
SIDE_PUSH    = 0.18    # 측면 벽 밀어내기 조향 가중치

# 실시간 처리 파라미터
CTRL_PERIOD  = 0.05    # 50ms = 20Hz 제어 주기
BIN_STALE    = 0.40    # 데이터 유효 시간 (0.4초)
MIN_VALID    = 15      # 최소 유효 데이터 빈 수


# ── [3] 빈 단위 히스토그램 및 데이터 파서 ────────────────━━━━━━━━━━━━━
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


# ── [4] 메인 루프 ─────────────────━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
last_ctrl      = 0.0
last_log       = 0.0
sync_error_cnt = 0

print("=" * 65)
# 리더 보정 각도 부호 검증 안내 (라이다 방향에 따라 조향 방향이 반대면 steer 기호 반전 필요)
print("  [Plan B] 3분할 섹터 기반 실시간 회피 주행 시작")
print("=" * 65)

while True:
    now = time.monotonic()
    
    # 시리얼 버퍼 비우기 및 데이터 업데이트 (싱크 에러 복구 포함)
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

    # 초기 데이터 수집 가드
    if valid_count < MIN_VALID:
        ser_Ardu.write(b"F 0.00 0.20\n")
        if now - last_log > 0.3:
            print(f"WARMUP (유효데이터 수집 중: {valid_count}/{N_BINS})")
            last_log = now
        continue

    # 1. 전방 영역 3분할 섹터 최소 거리 연산
    dist_F = 9999.0  # 정면 (-25도 ~ +25도)
    dist_L = 9999.0  # 좌측 (+25도 ~ +65도)
    dist_R = 9999.0  # 우측 (-65도 ~ -25도)

    for i in range(N_BINS):
        if not hp[i]: continue
        angle = i * BIN_DEG
        angle_s = angle if angle <= 180.0 else angle - 360.0

        if -25.0 <= angle_s <= 25.0:
            if h[i] < dist_F: dist_F = h[i]
        elif 25.0 < angle_s <= 65.0:
            if h[i] < dist_L: dist_L = h[i]
        elif -65.0 <= angle_s < -25.0:
            if h[i] < dist_R: dist_R = h[i]

    # 2. 제어 상태 머신 (State Machine)
    # [Emergency Step] 정면에 박스가 너무 가까우면 즉시 후진
    if dist_F <= EMERGENCY:
        ser_Ardu.write(b"B 0.85\n")
        if now - last_log > 0.2:
            print(f"[Plan B] EMERGENCY! 후진 (Front: {dist_F:.0f}mm)")
            last_log = now
        continue

    # [Obstacle Ahead Step] 감지거리 내에 장애물이 있을 때 트인 곳으로 조향
    if dist_F <= DETECT_DIST:
        if dist_L > dist_R:
            steer = -MAX_STEER  # 좌회전 회피
            state_str = "좌회전 회피"
        else:
            steer = MAX_STEER   # 우회전 회피
            state_str = "우회전 회피"
        
        # 장애물 거리에 비례하여 속도 감속 (최소 0.35)
        speed = 0.35 + 0.25 * ((dist_F - EMERGENCY) / (DETECT_DIST - EMERGENCY))
    else:
        # [Clear Step] 정면이 뚫려있을 때 직진 및 측면 벽 원격 유지
        steer = 0.0
        if dist_L < SIDE_DIST: steer += SIDE_PUSH    # 좌측 벽 근접 -> 우측 밀어내기
        if dist_R < SIDE_DIST: steer -= SIDE_PUSH    # 우측 벽 근접 -> 좌측 밀어내기
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        speed = 0.70
        state_str = "직진 주행"

    # 아두이노 명령 전송
    ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
    if now - last_log > 0.2:
        print(f"[Plan B] {state_str} | F:{dist_F:.0f} L:{dist_L:.0f} R:{dist_R:.0f} | Steer:{steer:+.2f} Speed:{speed:.2f}")
        last_log = now