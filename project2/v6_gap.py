import serial
import time
import math

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

baudrate_L    = 460800
baudrate_Ardu = 460800

ser_L    = serial.Serial(port_L,    baudrate_L,    timeout=1)
ser_Ardu = serial.Serial(port_Ardu, baudrate_Ardu, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))   # RESET
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))   # SCAN 시작


# ═══════════════════════════════════════════════════════════════════════
#  갭(통로) 탐색 함수
#
#  [갭 폭 계산 원리]
#
#           로봇(0,0)
#               │
#         θ_L  │  θ_R
#          ↙   │   ↘
#      ●────────────────●   ← 장애물 엣지 포인트
#      d_L              d_R
#      │←──── gap_w ───→│
#
#  gap_w = 2 × min(d_L, d_R) × sin(Δθ / 2)
#  Δθ = θ_R - θ_L (갭 각도 폭)
#  (gap_w 계산 시, 가까운 엣지 기준으로 보수적 추정:
#   멀리 있는 엣지의 과대평가 방지 위해 min(d_L, d_R) 사용) 
#  (가까운 엣지 기준으로 보수적 추정)
# ═══════════════════════════════════════════════════════════════════════

def find_best_gap(scan_buf, detect_dist, min_pass_mm):
    """
    전방 ±90° 스캔에서 통과 가능한 최적 갭(통로)을 탐색.

    처리 순서:
      1) 스캔 포인트를 BIN_DEG(2°) 단위 빈에 집계 (빈별 최소 거리)
      2) detect_dist 초과인 연속 빈 구간 → '갭'
      3) 갭 물리 폭 = 2 × min(좌엣지 거리, 우엣지 거리) × sin(Δθ/2)
      4) min_pass_mm 이상이면 통과 가능

    Returns: dict 또는 None
      'center'    : 갭 중심각 (부호 °, CW 기준: +오른쪽 / -왼쪽)
      'width'     : 갭 추정 물리 폭 (mm)
      'passable'  : 통과 가능 여부
      'd_left'    : 갭 좌측 엣지 장애물 거리 (mm)
      'd_right'   : 갭 우측 엣지 장애물 거리 (mm)
      'delta_deg' : 갭 각도 폭 (°)
    """
    BIN_DEG = 2.0
    n_bins  = int(180 / BIN_DEG)       # 전방 ±90° = 90개 빈
    bin_min = [None] * n_bins           # None = 해당 각도에 포인트 없음

    # ── 스캔 포인트를 빈에 집계 ──────────────────────────────
    for a, d in scan_buf:
        # CW 각도(0°~360°) → 부호있는 각도(-90°~+90°)
        # +오른쪽 / -왼쪽
        a_s = a if a <= 180 else a - 360
        if not (-90.0 <= a_s < 90.0):
            continue
        idx = min(int((a_s + 90.0) / BIN_DEG), n_bins - 1)
        if bin_min[idx] is None or d < bin_min[idx]:
            bin_min[idx] = d

    def is_open(i):
        """빈 i가 개방인지 판별 (포인트 없음 또는 detect_dist 초과)"""
        return bin_min[i] is None or bin_min[i] > detect_dist

    def get_dist(i):
        return bin_min[i] if bin_min[i] is not None else 9999.0

    # ── 연속 개방 구간 → 갭 수집 ────────────────────────────
    gaps = []
    i = 0
    while i < n_bins:
        if is_open(i):
            j = i
            while j < n_bins and is_open(j):
                j += 1
            # 갭 각도 범위: 빈 i ~ j-1
            center_ang = -90.0 + (i + j) / 2.0 * BIN_DEG
            delta_deg  = (j - i) * BIN_DEG

            # 갭 양쪽 장애물 거리 (구간 경계 밖이면 detect_dist 사용)
            d_L = min(get_dist(i - 1), detect_dist) if i > 0      else detect_dist
            d_R = min(get_dist(j),     detect_dist) if j < n_bins  else detect_dist

            # 갭 물리 폭: 가까운 엣지 기준으로 현(chord) 계산
            d_ref = min(d_L, d_R)
            gap_w = 2.0 * d_ref * math.sin(math.radians(delta_deg / 2.0))

            gaps.append({
                'center'   : center_ang,
                'width'    : gap_w,
                'passable' : gap_w >= min_pass_mm,
                'd_left'   : d_L,
                'd_right'  : d_R,
                'delta_deg': delta_deg,
            })
            i = j
        else:
            i += 1

    if not gaps:
        return None

    # ── 최적 갭 선택 ────────────────────────────────────────
    # 우선순위: 통과 가능한 갭 > 가장 넓은 갭
    # 동점 시: 중앙(0°)에 가까운 갭 선호 (패널티: 각도 × 1.5)
    passable = [g for g in gaps if g['passable']]
    pool     = passable if passable else gaps
    return max(pool, key=lambda g: g['width'] - abs(g['center']) * 1.5)


# ── 메인 루프 ──────────────────────────────────────────────────────────
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

    quality     = data[0] >> 2
    angle       = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance    = (data[3] | (data[4] << 8)) / 4.0
    if distance < 80:
        continue

    # ── 1회성 초기화 ────────────────────────────────────────
    try:
        _ready
    except NameError:
        import atexit
        _ready    = True
        scan_buf  = []
        front_min = 9999.0;  front_cnt = 0
        left_min  = 9999.0;  left_cnt  = 0
        right_min = 9999.0;  right_cnt = 0
        MIN_COUNT  = 4
        back_cnt   = 0
        extra_back = 0
        EMERGENCY  = 140.0
        DETECT     = 360.0

        # ── 차량 치수 및 갭 통과 파라미터 ──────────────────
        ROBOT_WIDTH  = 200.0             # 차량 폭 (mm)
        ROBOT_LENGTH = 200.0             # 차량 길이 (mm)
        GAP_MARGIN   = 30.0              # 통과 안전 마진 (mm)
        GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 최소 통과 폭: 230mm
        GAP_SPD_BASE = 0.50              # 갭 통과 기본 속도 (느리게)
        MAX_STEER    = 0.85              # 최대 조향값

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

        print("=" * 62)
        print("  장애물 회피 + 갭 통과 판단  (Ctrl+C 로 종료)")
        print(f"  감지:{int(DETECT)}mm  긴급:{int(EMERGENCY)}mm")
        print(f"  차량 크기:{int(ROBOT_WIDTH)}×{int(ROBOT_LENGTH)}mm  "
              f"최소 통과 폭:{int(GAP_MIN_PASS)}mm")
        print("  steer 부호: +오른쪽 / -왼쪽")
        print("=" * 62)

    if quality == 0:
        continue

    # ── 전방/좌/우 narrow zone 누적 (긴급후진 판단용) ──────
    if (angle <= 20 or angle >= 340) and distance <= 355:
        front_min = min(front_min, distance);  front_cnt += 1
    elif (angle > 20 and angle < 55) and distance <= 360:
        right_min = min(right_min, distance);  right_cnt += 1
    elif (angle > 305 and angle < 340) and distance <= 360:
        left_min  = min(left_min,  distance);  left_cnt  += 1

    scan_buf.append((angle, distance))

    # ── 1회전 완료 → 판단 ─────────────────────────────────
    if s_flag == 1 and len(scan_buf) > 15:

        if front_cnt < MIN_COUNT: front_min = 9999.0
        if left_cnt  < MIN_COUNT: left_min  = 9999.0
        if right_cnt < MIN_COUNT: right_min = 9999.0

        # ── P1: 확장 후진 진행 중 ─────────────────────────
        if extra_back > 0:
            ser_Ardu.write(b"B 0.80\n")
            extra_back -= 1
            print(f"EXTENDED_BACK 잔여 {extra_back}사이클")

        # ── P2: 긴급 후진 (EMERGENCY 이내) ────────────────
        elif front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:
            back_cnt += 1
            if back_cnt >= 6:
                ser_Ardu.write(b"B 0.90\n")
                extra_back = 3
                back_cnt   = 0
                print("EXTENDED_BACK 시작! (3x)")
            else:
                ser_Ardu.write(b"B 0.90\n")
                print(f"EMERGENCY! F:{front_min:.0f} L:{left_min:.0f} "
                      f"R:{right_min:.0f}mm ({back_cnt}/6)")

        # ── P3: 전방 장애물 → 갭 통과 가능 여부 판단 ──────
        elif front_min <= DETECT:
            gap   = find_best_gap(scan_buf, DETECT, GAP_MIN_PASS)
            ratio = min(max((DETECT - front_min) / (DETECT - EMERGENCY), 0.0), 1.0)

            if gap and gap['passable']:
                # ── 통과 가능한 갭 발견 → 갭 방향으로 조향 전진 ─
                # gap['center']: + = 오른쪽, - = 왼쪽
                # steer = center / 90 × MAX_STEER
                #   오른쪽 갭(+center) → 양수 steer (오른쪽)
                #   왼쪽 갭(-center)   → 음수 steer (왼쪽)
                steer = max(-MAX_STEER, min(MAX_STEER,
                            gap['center'] / 90.0 * MAX_STEER))
                speed = GAP_SPD_BASE * (1.0 - ratio * 0.4)  # 접근할수록 감속
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"GAP_PASS  폭={gap['width']:.0f}mm  "
                      f"중심={gap['center']:+.1f}°  "
                      f"(L:{gap['d_left']:.0f}/R:{gap['d_right']:.0f}mm  "
                      f"Δ{gap['delta_deg']:.0f}°)  "
                      f"steer={steer:+.2f}  spd={speed:.2f}")

            else:
                # ── 통과 불가 or 갭 없음 → 기존 방식 회피 ───
                speed = 0.70 * (1 - ratio * 0.7)
                steer = -(ratio * MAX_STEER) if left_min > right_min \
                        else (ratio * MAX_STEER)
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                if gap:
                    print(f"F_OBS  {front_min:.0f}mm  "
                          f"갭폭={gap['width']:.0f}mm < {GAP_MIN_PASS:.0f}mm (통과불가)  "
                          f"steer={steer:+.2f}")
                else:
                    print(f"F_OBS  {front_min:.0f}mm  갭 없음  steer={steer:+.2f}")

        # ── P4: 좌측 장애물 회피 ──────────────────────────
        elif left_min <= DETECT:
            ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
            steer = (ratio * 0.85)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"L_OBS  {left_min:.0f}mm (pts:{left_cnt})")

        # ── P5: 우측 장애물 회피 ──────────────────────────
        elif right_min <= DETECT:
            ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
            steer = -(ratio * 0.85)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"R_OBS  {right_min:.0f}mm (pts:{right_cnt})")

        # ── P6: 장애물 없음 → 가장 개방된 갭 방향으로 유도 ─
        # 갭이 중앙(0°)에서 10° 이상 벗어난 경우 부드럽게 조향
        # 중앙에 가까우면 직진 유지
        else:
            gap = find_best_gap(scan_buf, DETECT, GAP_MIN_PASS)
            if gap and abs(gap['center']) > 10.0:
                # 최대 ±0.25 의 완만한 조향 (급조향 방지)
                gentle = max(-0.25, min(0.25,
                             gap['center'] / 90.0 * 0.25))
                ser_Ardu.write(f"F {gentle:.2f} 0.70\n".encode())
                print(f"OPEN  최적갭={gap['width']:.0f}mm@{gap['center']:+.1f}°  "
                      f"gentle_steer={gentle:+.2f}")
            else:
                ser_Ardu.write(b"F 0.00 0.70\n")

        # ── 버퍼 초기화 ────────────────────────────────────
        scan_buf  = []
        front_min = 9999.0;  front_cnt = 0
        left_min  = 9999.0;  left_cnt  = 0
        right_min = 9999.0;  right_cnt = 0
