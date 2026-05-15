import serial
import time
import atexit

# ─── 파라미터 ─────────────────────────────────────────────────────────────
PORT_L           = "/dev/ttyUSB0"
PORT_ARDU        = "/dev/ttyS0"
EMERGENCY        = 140.0   # 즉시 후진 거리 (mm)
DETECT           = 350.0   # 일반 장애물 감지 거리 (mm)
CORNER_DIST      = 200.0   # 구석 판별 기준 거리 (mm) ← 여기서 조정
MIN_BLOCKED      = 4       # 구석 판별 최소 막힌 섹터 수
NUM_SECTORS      = 8       # 360° 분할 수
SECTOR_SIZE      = 360.0 / NUM_SECTORS   # 45°
MIN_ESCAPE_ANGLE = 45.0    # 최소 탈출 회전 각도 (°)
DEG_PER_CYCLE    = 15.0    # 한 사이클당 회전 각도 (°) — 로봇 속도에 맞게 조정
MAX_STEER        = 0.85
MIN_COUNT        = 4       # 구역 유효 포인트 최소 수
MIN_DIST_NOISE   = 80.0    # 이 거리 미만은 노이즈로 제거 (mm)


# ═══════════════════════════════════════════════════════════════════════════
#  함수 정의
# ═══════════════════════════════════════════════════════════════════════════

def parse_packet(data: bytes):
    """
    5바이트 RPLIDAR 패킷 파싱.
    Returns: (valid: bool, s_flag: int, angle: float, distance: float, quality: int)
    """
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        return False, 0, 0.0, 0.0, 0
    if (data[1] & 0x01) != 1:
        return False, 0, 0.0, 0.0, 0
    quality  = data[0] >> 2
    angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0
    return True, s_flag, angle, distance, quality


def accumulate_sector(angle: float, distance: float,
                      sector_sum: list, sector_cnt_buf: list):
    """포인트를 해당 섹터 버퍼에 누적 (50~8000mm 유효 범위)."""
    if distance <= 8000:
        idx = int(angle / SECTOR_SIZE) % NUM_SECTORS
        sector_sum[idx]     += distance
        sector_cnt_buf[idx] += 1


def compute_sector_avg(sector_sum: list, sector_cnt_buf: list) -> list:
    """
    섹터별 평균 거리 계산.
    포인트가 없는 섹터(개방 공간) = 8000mm 로 처리.
    """
    return [
        sector_sum[i] / sector_cnt_buf[i] if sector_cnt_buf[i] > 0 else 8000.0
        for i in range(NUM_SECTORS)
    ]


def detect_corner(front_min: float, sector_avg: list) -> bool:
    """
    구석 판별 함수.
    조건: 전방 장애물 감지(front_min <= DETECT)
          + CORNER_DIST(200mm) 이내 섹터가 MIN_BLOCKED개 이상
    Returns: True if cornered
    """
    if front_min > DETECT:
        return False
    blocked = sum(1 for avg in sector_avg if avg <= CORNER_DIST)
    return blocked >= MIN_BLOCKED


def find_escape_direction(sector_avg: list) -> tuple:
    """
    가장 열린 섹터를 찾아 탈출 방향과 각도 계산.
    Returns: (escape_steer, pending_angle, best_idx, best_center, best_avg)
      - escape_steer : -1.0(오른쪽 회전) / +1.0(왼쪽 회전)
      - pending_angle: 실제 회전 필요 각도 (°)
      - best_idx     : 가장 열린 섹터 인덱스
      - best_center  : 해당 섹터 중심각 (°)
      - best_avg     : 해당 섹터 평균 거리 (mm)
    """
    best_idx    = max(range(NUM_SECTORS), key=lambda i: sector_avg[i])
    best_avg    = sector_avg[best_idx]
    best_center = best_idx * SECTOR_SIZE + SECTOR_SIZE / 2

    # CW 기준: 0~180° → 오른쪽 영역, 180~360° → 왼쪽 영역
    if best_center <= 180:
        escape_steer  = -1.0
        pending_angle = best_center
    else:
        escape_steer  = +1.0
        pending_angle = 360.0 - best_center

    return escape_steer, pending_angle, best_idx, best_center, best_avg


def calc_escape_cycles(pending_angle: float) -> int:
    """
    탈출 회전 사이클 수 계산.
    MIN_ESCAPE_ANGLE 보장: 계산 각도가 작아도 최소한 MIN_ESCAPE_ANGLE 만큼은 회전.
    """
    eff_angle = max(MIN_ESCAPE_ANGLE, pending_angle)
    return max(1, int(eff_angle / DEG_PER_CYCLE))


def compute_normal_avoidance(front_min: float,
                             left_min: float,
                             right_min: float) -> tuple | None:
    """
    일반 장애물 회피 steer·speed 계산.
    Returns: (steer, speed, zone_label) or None(장애물 없음 = 직진)
    """
    if front_min <= DETECT:
        ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
        speed = 0.70 * (1 - ratio * 0.7)
        steer = -(ratio * MAX_STEER) if left_min > right_min else (ratio * MAX_STEER)
        return steer, speed, "FRONT"

    if left_min <= DETECT:
        ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
        steer = ratio * 0.75
        speed = 0.70 * (1 - ratio * 0.6)
        return steer, speed, "LEFT"

    if right_min <= DETECT:
        ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
        steer = -(ratio * 0.75)
        speed = 0.70 * (1 - ratio * 0.6)
        return steer, speed, "RIGHT"

    return None   # 장애물 없음


def send_cmd(ser: serial.Serial, cmd: bytes):
    """아두이노 시리얼로 명령 전송."""
    ser.write(cmd)


# ═══════════════════════════════════════════════════════════════════════════
#  초기화
# ═══════════════════════════════════════════════════════════════════════════

ser_L    = serial.Serial(PORT_L,    460800, timeout=1)
ser_Ardu = serial.Serial(PORT_ARDU, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

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

# 스캔 버퍼 및 구역 변수
scan_buf       = []
sector_sum     = [0.0] * NUM_SECTORS
sector_cnt_buf = [0]   * NUM_SECTORS
front_min = left_min = right_min = 9999.0
front_cnt = left_cnt = right_cnt = 0

# 상태 변수
back_cnt       = 0
extra_back     = 0
escape_left    = 0
escape_steer   = 0.0
pending_escape = False
pending_angle  = 0.0

print("=" * 62)
print("  장애물 회피 [함수형 / 구석 탈출 200mm 기준] 시작  (Ctrl+C 종료)")
print(f"  일반감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
print(f"  구석 판별: {int(CORNER_DIST)}mm 이내 섹터 {MIN_BLOCKED}개 이상 막힘")
print(f"  최소탈출회전: {MIN_ESCAPE_ANGLE:.0f}°  /  사이클당: {DEG_PER_CYCLE:.0f}°")
print("  ※ 아두이노에서 'T {steer}\\n' 제자리 회전 명령 지원 필요")
print("=" * 62)


# ═══════════════════════════════════════════════════════════════════════════
#  메인 루프
# ═══════════════════════════════════════════════════════════════════════════

while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    valid, s_flag, angle, distance, quality = parse_packet(data)
    if not valid or quality == 0 or distance < MIN_DIST_NOISE:
        continue

    # 섹터 및 구역 데이터 누적
    accumulate_sector(angle, distance, sector_sum, sector_cnt_buf)

    if (angle <= 20 or angle >= 340) and distance <= 345:
        front_min = min(front_min, distance);  front_cnt += 1
    elif (angle > 20  and angle < 50)  and distance <= 350:
        right_min = min(right_min, distance);  right_cnt += 1
    elif (angle > 310 and angle < 340) and distance <= 350:
        left_min  = min(left_min,  distance);  left_cnt  += 1

    scan_buf.append((angle, distance))

    # ── 1회전 완료 시 판단 ─────────────────────────────────────────────
    if s_flag == 1 and len(scan_buf) > 15:

        if front_cnt < MIN_COUNT: front_min = 9999.0
        if left_cnt  < MIN_COUNT: left_min  = 9999.0
        if right_cnt < MIN_COUNT: right_min = 9999.0

        # 매 사이클 섹터 평균 및 구석 여부 계산
        sector_avg = compute_sector_avg(sector_sum, sector_cnt_buf)
        cornered   = detect_corner(front_min, sector_avg)

        # ── P1: 탈출 회전 진행 중 ─────────────────────────────────────
        if escape_left > 0:
            send_cmd(ser_Ardu, f"T {escape_steer:.2f}\n".encode())
            escape_left -= 1
            print(f"ESCAPE_ROTATE  steer={escape_steer:+.2f}  잔여={escape_left}사이클")

        # ── P2: 후진 진행 중 ──────────────────────────────────────────
        elif extra_back > 0:
            send_cmd(ser_Ardu, b"B 0.90\n")
            extra_back -= 1
            if extra_back == 0 and pending_escape:
                escape_left    = calc_escape_cycles(pending_angle)
                pending_escape = False
                eff = max(MIN_ESCAPE_ANGLE, pending_angle)
                print(f"ESCAPE  후진완료 → 회전 {escape_left}사이클 ({eff:.0f}°) steer={escape_steer:+.2f}")
            else:
                print(f"ESCAPE  후진 잔여={extra_back}사이클")

        # ── P3: 구석 감지 → 탈출 루틴 (200mm 기준) ───────────────────
        elif cornered:
            escape_steer, pending_angle, b_idx, b_center, b_avg = \
                find_escape_direction(sector_avg)
            pending_escape = True
            extra_back     = 2
            send_cmd(ser_Ardu, b"B 0.90\n")

            blocked_cnt = sum(1 for avg in sector_avg if avg <= CORNER_DIST)
            eff         = max(MIN_ESCAPE_ANGLE, pending_angle)
            print(f"CORNER!  {blocked_cnt}/{NUM_SECTORS}섹터가 {CORNER_DIST:.0f}mm 이내")
            print(f"  → 최적섹터={b_idx}({b_center:.0f}°)  avg={b_avg:.0f}mm  "
                  f"회전={'R' if escape_steer<0 else 'L'}  "
                  f"각도={eff:.0f}°  사이클={calc_escape_cycles(pending_angle)}")
            print(f"  섹터avg(mm): {[f'{v:.0f}' for v in sector_avg]}")

        # ── P4: 긴급 후진 ─────────────────────────────────────────────
        elif front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:
            back_cnt += 1
            if back_cnt >= 6:
                send_cmd(ser_Ardu, b"B 0.90\n")
                extra_back = 2
                back_cnt   = 0
                print("EXTENDED_BACK 시작! (3x)")
            else:
                send_cmd(ser_Ardu, b"B 0.90\n")
                print(f"EMERGENCY!  F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm  ({back_cnt}/6)")

        # ── P5: 일반 회피 / 직진 ──────────────────────────────────────
        else:
            result = compute_normal_avoidance(front_min, left_min, right_min)
            if result:
                steer, speed, zone = result
                send_cmd(ser_Ardu, f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"{zone}_OBS  F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm  "
                      f"→ {'R' if steer<0 else 'L'}  steer={steer:.2f}  spd={speed:.2f}")
            else:
                send_cmd(ser_Ardu, b"F 0.00 0.70\n")
                back_cnt = 0

        # ── 사이클 말 초기화 ──────────────────────────────────────────
        scan_buf       = []
        sector_sum     = [0.0] * NUM_SECTORS
        sector_cnt_buf = [0]   * NUM_SECTORS
        front_min = left_min = right_min = 9999.0
        front_cnt = left_cnt = right_cnt = 0
