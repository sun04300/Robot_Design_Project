import serial
import time

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L     = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu  = serial.Serial(port_Ardu, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

NUM_SECTORS  = 8
SECTOR_SIZE  = 360.0 / NUM_SECTORS   # 45° per sector

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

    quality    = data[0] >> 2
    angle      = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance   = (data[3] | (data[4] << 8)) / 4.0
    if distance < 50:
        continue

    try:
        _ready
    except NameError:
        import atexit
        _ready          = True
        scan_buf        = []
        front_min       = 9999.0
        left_min        = 9999.0
        right_min       = 9999.0
        front_cnt       = 0
        left_cnt        = 0
        right_cnt       = 0
        MIN_COUNT       = 4
        EMERGENCY       = 140.0
        DETECT          = 330.0
        back_cnt        = 0
        extra_back      = 0
        # 탈출 관련 변수
        escape_left     = 0        # 회전 남은 사이클
        escape_steer    = 0.0      # 회전 방향 steer 값
        pending_escape  = False    # 후진 완료 후 회전 대기 플래그
        pending_angle   = 0.0      # 계산된 회전 각도(도)
        # 전방위 섹터 누적 버퍼
        sector_sum      = [0.0] * NUM_SECTORS
        sector_cnt_buf  = [0]   * NUM_SECTORS

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
        print("=" * 55)
        print("  장애물 회피 [방법 A: 전방위 최적방향 탐색] 시작")
        print(f"  감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
        print(f"  섹터 수: {NUM_SECTORS}  ({int(SECTOR_SIZE)}° / 섹터)")
        print("  ※ 아두이노에서 'T {{steer}}\\n' 제자리 회전 명령 지원 필요")
        print("=" * 55)

    if quality == 0:
        continue

    # 전방위 섹터 누적 (50 ~ 8000mm 유효 거리만)
    if distance <= 8000:
        idx = int(angle / SECTOR_SIZE) % NUM_SECTORS
        sector_sum[idx]     += distance
        sector_cnt_buf[idx] += 1

    # 구역별 최솟값/카운트 누적
    if (angle <= 20 or angle >= 340) and distance <= 320:
        front_min = min(front_min, distance)
        front_cnt += 1
    elif (angle > 20 and angle < 50) and distance <= 330:
        right_min = min(right_min, distance)
        right_cnt += 1
    elif (angle > 310 and angle < 340) and distance <= 330:
        left_min = min(left_min, distance)
        left_cnt += 1

    scan_buf.append((angle, distance))

    if s_flag == 1 and len(scan_buf) > 15:

        if front_cnt < MIN_COUNT: front_min = 9999.0
        if left_cnt  < MIN_COUNT: left_min  = 9999.0
        if right_cnt < MIN_COUNT: right_min = 9999.0

        # 전방위 섹터 평균 거리 계산 (포인트 없는 섹터 = 개방 공간으로 간주)
        sector_avg = [
            (sector_sum[i] / sector_cnt_buf[i]) if sector_cnt_buf[i] > 0 else 8000.0
            for i in range(NUM_SECTORS)
        ]
        # 막힌 섹터 수로 구석 판단 (좁은 left/right 구역 변수에 의존하지 않음)
        blocked     = sum(1 for avg in sector_avg if avg <= DETECT)
        is_cornered = (front_min <= DETECT and blocked >= 4)

        # ── 우선순위 1: 탈출 회전 진행 중 ────────────────────────────
        if escape_left > 0:
            ser_Ardu.write(f"T {escape_steer:.2f}\n".encode())
            escape_left -= 1
            print(f"ESCAPE_ROTATE  steer={escape_steer:+.2f}  잔여={escape_left}사이클")

        # ── 우선순위 2: 후진 진행 중 ──────────────────────────────────
        elif extra_back > 0:
            ser_Ardu.write(b"B 0.70\n")
            extra_back -= 1
            # 후진 완료 → 예약된 회전 시작
            if extra_back == 0 and pending_escape:
                escape_left    = max(4, int(pending_angle / 15)) 
                # 회전 각도에 비례한 회전 사이클 (최소 4사이클)
                pending_escape = False
                print(f"ESCAPE_A  후진완료 → 회전 시작  {escape_left}사이클  steer={escape_steer:+.2f}")
            else:
                print(f"ESCAPE_A  후진 잔여={extra_back}사이클")

        # ── 우선순위 3: 구석 감지 (전방 막힘 + 4개 이상 섹터 막힘) ────
        elif is_cornered:

            # 가장 열린 섹터 탐색 (sector_avg는 위에서 이미 계산됨)
            best_idx = sector_avg.index(max(sector_avg))
            best_avg = sector_avg[best_idx]
            best_center = best_idx * SECTOR_SIZE + SECTOR_SIZE / 2  # 섹터 중심각

            # 회전 방향 및 필요 각도 계산
            # CW 기준: 0~180 → 오른쪽 방향, 180~360 → 왼쪽 방향
            if best_center <= 180:
                escape_steer = -1.0                         # 오른쪽 회전
                pending_angle = best_center
            else:
                escape_steer = +1.0                         # 왼쪽 회전
                pending_angle = 360.0 - best_center

            pending_escape = True
            extra_back     = 2                              # 후진 먼저
            ser_Ardu.write(b"B 0.70\n")
            print(f"CORNER_A!  F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm")
            print(f"  → 최적섹터={best_idx}({best_center:.0f}°)  avg={best_avg:.0f}mm  "
                  f"회전={'R' if escape_steer<0 else 'L'}  각도={pending_angle:.0f}°  "
                  f"예상사이클={max(4, int(pending_angle/15))}")
            print(f"  섹터별 평균: {[f'{v:.0f}' for v in sector_avg]}")

        # ── 우선순위 4: 긴급 후진 ────────────────────────────────────
        elif (front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY):
            back_cnt += 1
            if back_cnt >= 6:
                ser_Ardu.write(b"B 0.70\n")
                extra_back = 2
                back_cnt   = 0
                print(f"EXTENDED_BACK 시작! (3x)  back_cnt 초기화")
            else:
                ser_Ardu.write(b"B 0.70\n")
                print(f"EMERGENCY!  F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm  ({back_cnt}/6)")

        # ── 일반 회피 ────────────────────────────────────────────────
        elif front_min <= DETECT:
            ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
            speed = 0.70 * (1 - ratio * 0.7)
            steer = -(ratio * 0.85) if left_min > right_min else (ratio * 0.85)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"F_OBS  {front_min:.0f}mm → {'R' if steer<0 else 'L'}  steer={steer:.2f}  spd={speed:.2f}")
            print(f"DBG  L:{left_min:.0f}mm({left_cnt}pts)  R:{right_min:.0f}mm({right_cnt}pts)")

        elif left_min <= DETECT:
            ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
            steer = (ratio * 0.75)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"L_OBS  {left_min:.0f}mm  (pts:{left_cnt})")

        elif right_min <= DETECT:
            ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
            steer = -(ratio * 0.75)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"R_OBS  {right_min:.0f}mm  (pts:{right_cnt})")

        else:
            ser_Ardu.write(b"F 0.00 0.70\n")

        # 사이클 말 초기화
        scan_buf       = []
        front_min      = 9999.0
        left_min       = 9999.0
        right_min      = 9999.0
        front_cnt      = 0
        left_cnt       = 0
        right_cnt      = 0
        sector_sum     = [0.0] * NUM_SECTORS
        sector_cnt_buf = [0]   * NUM_SECTORS
