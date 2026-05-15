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
SECTOR_SIZE  = 360.0 / NUM_SECTORS

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
        FRONT_CLEAR     = DETECT * 1.3
        # [v3a] 코너 감지 엄격 기준
        CORNER_FRONT    = EMERGENCY * 1.4   # ≈ 196mm
        CORNER_BLOCKED  = 6
        # [v3b 신규] 회전 직후 전진 확정 (오실레이션 차단)
        COMMIT_CYCLES   = 5                 # 회전 종료 후 직진 보장 사이클
        forward_commit  = 0
        back_cnt        = 0
        extra_back      = 0
        MIN_ESCAPE_ANGLE = 30.0
        DEG_PER_CYCLE    = 15.0
        BACKUP_CYCLES    = 3
        escape_left      = 0
        escape_steer     = 0.0
        pending_escape   = False
        pending_angle    = 0.0
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
        print("  장애물 회피 v3b: 코너기준 강화 + 전진확정")
        print(f"  감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
        print(f"  코너기준: 전방 ≤ {int(CORNER_FRONT)}mm AND {CORNER_BLOCKED}섹터 이상 막힘")
        print(f"  회전종료(폐루프): 전방 {int(FRONT_CLEAR)}mm 이상")
        print(f"  전진확정: {COMMIT_CYCLES}사이클 (코너 재트리거 차단)")
        print(f"  섹터 수: {NUM_SECTORS}  /  사이클당: {DEG_PER_CYCLE:.0f}°")
        print("=" * 55)

    if quality == 0:
        continue

    if distance <= 8000:
        idx = int(angle / SECTOR_SIZE) % NUM_SECTORS
        sector_sum[idx]     += distance
        sector_cnt_buf[idx] += 1

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

        sector_avg = [
            (sector_sum[i] / sector_cnt_buf[i]) if sector_cnt_buf[i] > 0 else 8000.0
            for i in range(NUM_SECTORS)
        ]
        blocked = sum(1 for avg in sector_avg if avg <= DETECT)
        is_cornered = (front_min <= CORNER_FRONT and blocked >= CORNER_BLOCKED)

        # ── 우선순위 1: 탈출 회전 (폐루프) ───────────────────────────
        if escape_left > 0:
            if front_cnt >= MIN_COUNT and front_min >= FRONT_CLEAR:
                escape_left    = 0
                forward_commit = COMMIT_CYCLES   # [v3b 신규] 전진 확정 시작
                ser_Ardu.write(b"F 0.00 0.70\n")
                print(f"ESCAPE_DONE  전방 {front_min:.0f}mm 확보 → 전진확정 {COMMIT_CYCLES}사이클")
            else:
                ser_Ardu.write(f"T {escape_steer:.2f}\n".encode())
                escape_left -= 1
                # [v3b 신규] 회전이 자연 만료로 끝나는 경우도 전진 확정 진입
                if escape_left == 0:
                    forward_commit = COMMIT_CYCLES
                    print(f"ESCAPE_EXPIRED  사이클 만료 → 전진확정 {COMMIT_CYCLES}사이클")
                else:
                    fm = f"{front_min:.0f}mm" if front_cnt >= MIN_COUNT else "N/A"
                    print(f"ESCAPE_ROTATE  steer={escape_steer:+.2f}  잔여={escape_left}  front={fm}")

        # ── 우선순위 2: 후진 진행 중 ──────────────────────────────────
        elif extra_back > 0:
            ser_Ardu.write(b"B 0.70\n")
            extra_back -= 1
            if extra_back == 0 and pending_escape:
                fresh_avg = [
                    (sector_sum[i] / sector_cnt_buf[i]) if sector_cnt_buf[i] > 0 else 8000.0
                    for i in range(NUM_SECTORS)
                ]
                best_idx    = fresh_avg.index(max(fresh_avg))
                best_avg    = fresh_avg[best_idx]
                best_center = best_idx * SECTOR_SIZE + SECTOR_SIZE / 2

                if best_center <= 180:
                    escape_steer  = -1.0
                    pending_angle = best_center
                else:
                    escape_steer  = +1.0
                    pending_angle = 360.0 - best_center

                escape_left = max(int(MIN_ESCAPE_ANGLE / DEG_PER_CYCLE),
                                  int(pending_angle    / DEG_PER_CYCLE))
                pending_escape = False
                eff_angle = max(MIN_ESCAPE_ANGLE, pending_angle)
                print(f"REPLAN  후진완료 → 재계산  best={best_idx}({best_center:.0f}°) "
                      f"avg={best_avg:.0f}mm  회전={'R' if escape_steer<0 else 'L'} "
                      f"{eff_angle:.0f}°  최대{escape_left}사이클")
            else:
                print(f"ESCAPE_A  후진 잔여={extra_back}사이클")

        # ── [v3b 신규] 우선순위 3: 전진 확정 (코너 재트리거 차단) ────
        elif forward_commit > 0:
            forward_commit -= 1
            # 진짜 위험할 때만 인터럽트 (긴급 후진)
            if front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:
                ser_Ardu.write(b"B 0.70\n")
                forward_commit = 0
                print(f"COMMIT_ABORT!  긴급상황(F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}) → 후진")
            else:
                ser_Ardu.write(b"F 0.00 0.70\n")
                print(f"COMMIT_FORWARD  잔여={forward_commit}  front={front_min:.0f}mm")

        # ── 우선순위 4: 구석 감지 (강화된 기준) ───────────────────────
        elif is_cornered:
            best_idx    = sector_avg.index(max(sector_avg))
            best_avg    = sector_avg[best_idx]
            best_center = best_idx * SECTOR_SIZE + SECTOR_SIZE / 2

            if best_center <= 180:
                escape_steer  = -1.0
                pending_angle = best_center
            else:
                escape_steer  = +1.0
                pending_angle = 360.0 - best_center

            pending_escape = True
            extra_back     = BACKUP_CYCLES
            ser_Ardu.write(b"B 0.70\n")
            print(f"CORNER!  F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm "
                  f"blocked={blocked}/{NUM_SECTORS}")
            eff_angle = max(MIN_ESCAPE_ANGLE, pending_angle)
            print(f"  → 초기추정: 섹터={best_idx}({best_center:.0f}°)  avg={best_avg:.0f}mm  "
                  f"회전={'R' if escape_steer<0 else 'L'}  각도={eff_angle:.0f}°")

        # ── 우선순위 5: 긴급 후진 ────────────────────────────────────
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

        # ── 우선순위 6: 일반 회피 ────────────────────────────────────
        elif front_min <= DETECT:
            ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
            speed = 0.70 * (1 - ratio * 0.7)
            steer = -(ratio * 0.85) if left_min > right_min else (ratio * 0.85)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"F_OBS  {front_min:.0f}mm → {'R' if steer<0 else 'L'}  steer={steer:.2f}  spd={speed:.2f}")

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

        scan_buf       = []
        front_min      = 9999.0
        left_min       = 9999.0
        right_min      = 9999.0
        front_cnt      = 0
        left_cnt       = 0
        right_cnt      = 0
        sector_sum     = [0.0] * NUM_SECTORS
        sector_cnt_buf = [0]   * NUM_SECTORS