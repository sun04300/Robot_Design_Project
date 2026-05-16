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

        # ── 코너 감지 파라미터 ──────────────────────────────────────
        CORNER_FRONT    = EMERGENCY * 1.5   # 약 210mm
        # [수정②] CORNER_BLOCKED = 6 유지 (4로 낮추면 정상 통로에서도 오작동)
        # is_narrow_trapped OR 조건이 놓치는 케이스를 이미 보완하므로 6이 안전
        CORNER_BLOCKED  = 6

        # [수정③] BACKUP_CYCLES 절충값 5 (3→8은 과도, 후방 충돌 위험)
        BACKUP_CYCLES    = 5

        # [수정⑤] 최소 회전 보장 사이클 (OR 조건으로 되돌리되 즉시 종료 방지)
        MIN_ROTATION    = 2

        COMMIT_CYCLES   = 5
        forward_commit  = 0
        back_cnt        = 0
        extra_back      = 0
        MIN_ESCAPE_ANGLE = 30.0
        DEG_PER_CYCLE    = 15.0
        escape_left      = 0
        escape_steer     = 0.0
        initial_escape_left = 0          # [수정⑤] 회전 시작 시 기록용
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
        print("=" * 60)
        print("  장애물 회피 v3d: 피드백 5항목 완전 반영 버전")
        print(f"  감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
        print(f"  코너기준: 전방 ≤ {int(CORNER_FRONT)}mm AND ({CORNER_BLOCKED}섹터 이상 OR 3면 차단)")
        print(f"  탈출종료: 협소 OR 광역 클리어 + 최소 {MIN_ROTATION}사이클 회전 보장")
        print(f"  전진확정: {COMMIT_CYCLES}사이클  /  후진사이클: {BACKUP_CYCLES}")
        print(f"  섹터수: {NUM_SECTORS}  /  사이클당: {DEG_PER_CYCLE:.0f}°")
        print("=" * 60)

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

        # [유지④] 3면 동시 차단: 가장 정확한 코너 판정 신호
        is_narrow_trapped = (front_min <= DETECT and left_min <= DETECT and right_min <= DETECT)
        # [수정②] CORNER_BLOCKED = 6 유지, is_narrow_trapped OR로 보완
        is_cornered = (front_min <= CORNER_FRONT and (blocked >= CORNER_BLOCKED or is_narrow_trapped))

        # 광역 전방 섹터 (섹터 0, 섹터 7) 중 더 짧은 쪽 기준
        front_sectors_min = min(sector_avg[0], sector_avg[NUM_SECTORS - 1])

        # [수정③] 후방 안전용: 섹터 3,4 (약 135°~225°) 평균
        rear_avg = (sector_avg[3] + sector_avg[4]) / 2

        # [수정⑤] 탈출 종료 조건 → OR로 되돌림 (AND는 과회전 유발)
        # narrow: 협소 zone이 명확히 막힌 게 아니면 pass
        narrow_clear = (front_cnt >= MIN_COUNT and front_min >= FRONT_CLEAR) or (front_cnt < MIN_COUNT)
        wide_clear   = (front_sectors_min >= FRONT_CLEAR)

        front_clear_signal = False
        signal_source = ""
        # [수정⑤] OR 조건 + 최소 회전 사이클 보장은 escape 분기 안에서 처리
        if narrow_clear or wide_clear:
            front_clear_signal = True
            signal_source = f"Clear(N:{front_min:.0f} W:{front_sectors_min:.0f})"

        # ── 우선순위 1: 탈출 회전 (폐루프) ───────────────────────────
        if escape_left > 0:
            rotated_cycles = initial_escape_left - escape_left   # [수정⑤] 경과 사이클

            # [유지⑥] 회전 중 ESCAPE_ABORT + [수정⑥] pending_escape = True 추가
            if front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:
                escape_left    = 0
                extra_back     = 4
                pending_escape = True      # ← 수정⑥: 후진 후 반드시 재계획 실행
                ser_Ardu.write(b"B 0.70\n")
                print("ESCAPE_ABORT! 회전 중 장애물 재근접 → 재후진 후 재계획")

            # [수정⑤] 최소 MIN_ROTATION 사이클 돌고나서만 종료 허용
            elif front_clear_signal and rotated_cycles >= MIN_ROTATION:
                escape_left    = 0
                forward_commit = COMMIT_CYCLES
                ser_Ardu.write(b"F 0.00 0.70\n")
                print(f"ESCAPE_DONE  {signal_source}  회전{rotated_cycles}사이클 → 전진확정 {COMMIT_CYCLES}사이클")

            else:
                ser_Ardu.write(f"T {escape_steer:.2f}\n".encode())
                escape_left -= 1
                if escape_left == 0:
                    forward_commit = COMMIT_CYCLES
                    print(f"ESCAPE_EXPIRED  사이클 만료 → 전진확정 {COMMIT_CYCLES}사이클")
                else:
                    fm = f"{front_min:.0f}mm" if front_cnt >= MIN_COUNT else "N/A"
                    min_rot_info = f"  최소회전잔여={max(0, MIN_ROTATION - rotated_cycles)}" if rotated_cycles < MIN_ROTATION else ""
                    print(f"ESCAPE_ROTATE  steer={escape_steer:+.2f}  잔여={escape_left}"
                          f"  front={fm}  wide={front_sectors_min:.0f}mm{min_rot_info}")

        # ── 우선순위 2: 후진 진행 중 ──────────────────────────────────
        elif extra_back > 0:
            # [수정③] 후방 섹터 안전 체크: 후방 20cm 이내면 강제 종료
            if rear_avg <= 200:
                print(f"BACKUP_STOPPED  후방 {rear_avg:.0f}mm 근접 → 후진 강제 종료")
                extra_back = 0
                if pending_escape:
                    # 후진 못 해도 열린 방향으로 회전 시도
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
                    escape_left         = max(int(MIN_ESCAPE_ANGLE / DEG_PER_CYCLE),
                                              int(pending_angle    / DEG_PER_CYCLE))
                    initial_escape_left = escape_left   # [수정⑤] 기록
                    pending_escape      = False
                    eff_angle = max(MIN_ESCAPE_ANGLE, pending_angle)
                    print(f"REPLAN(후방막힘)  best={best_idx}({best_center:.0f}°)  avg={best_avg:.0f}mm"
                          f"  회전={'R' if escape_steer<0 else 'L'}  {eff_angle:.0f}°  최대{escape_left}사이클")
            else:
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
                    escape_left         = max(int(MIN_ESCAPE_ANGLE / DEG_PER_CYCLE),
                                              int(pending_angle    / DEG_PER_CYCLE))
                    initial_escape_left = escape_left   # [수정⑤] 기록
                    pending_escape      = False
                    eff_angle = max(MIN_ESCAPE_ANGLE, pending_angle)
                    print(f"REPLAN  후진완료 → 재계산  best={best_idx}({best_center:.0f}°)"
                          f"  avg={best_avg:.0f}mm  회전={'R' if escape_steer<0 else 'L'}"
                          f"  {eff_angle:.0f}°  최대{escape_left}사이클")
                else:
                    print(f"ESCAPE_A  후진 잔여={extra_back}사이클  후방={rear_avg:.0f}mm")

        # ── 우선순위 3: 전진 확정 ─────────────────────────────────────
        elif forward_commit > 0:
            forward_commit -= 1
            if front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:
                ser_Ardu.write(b"B 0.70\n")
                forward_commit = 0
                print(f"COMMIT_ABORT!  긴급(F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}) → 후진")
            else:
                ser_Ardu.write(b"F 0.00 0.70\n")
                print(f"COMMIT_FORWARD  잔여={forward_commit}  front={front_min:.0f}mm")

        # ── 우선순위 4: 코너 감지 ─────────────────────────────────────
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
            print(f"CORNER!  F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm"
                  f"  blocked={blocked}/{NUM_SECTORS}  narrow_trapped={is_narrow_trapped}"
                  f"  후방={rear_avg:.0f}mm")
            eff_angle = max(MIN_ESCAPE_ANGLE, pending_angle)
            print(f"  → 섹터={best_idx}({best_center:.0f}°)  avg={best_avg:.0f}mm"
                  f"  회전={'R' if escape_steer<0 else 'L'}  {eff_angle:.0f}°")

        # ── 우선순위 5: 긴급 후진 ─────────────────────────────────────
        elif (front_min <= EMERGENCY 또는 left_min <= EMERGENCY 또는 right_min <= EMERGENCY):
            back_cnt += 1
            if back_cnt >= 6:
                ser_Ardu.write(b"B 0.70\n")
                extra_back = 2
                back_cnt   = 0
                print(f"EXTENDED_BACK 시작! (2사이클 추가)  back_cnt 초기화")
            else:
                ser_Ardu.write(b"B 0.70\n")
                print(f"EMERGENCY!  F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm  ({back_cnt}/6)")

        # ── 우선순위 6: 일반 회피 ─────────────────────────────────────
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
