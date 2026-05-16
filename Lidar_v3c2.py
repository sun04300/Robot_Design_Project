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
        
        # 주행 임계값
        MIN_COUNT       = 4
        EMERGENCY       = 140.0
        DETECT          = 330.0
        FRONT_CLEAR     = DETECT * 1.3
        CORNER_FRONT    = EMERGENCY * 1.4
        CORNER_BLOCKED  = 6
        
        # 제어 상태 변수
        COMMIT_CYCLES   = 5
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

        # [v3d 신규] 교착상태(Deadlock) 방지용 최상위 상태 변수들
        escape_streak   = 0     # 연속 코너 감지 횟수 (무한 진동 파악용)
        last_steer_dir  = 0.0   # 마지막 탈출 회피 방향 (+1.0 또는 -1.0)
        corner_cooldown = 0     # 코너 탈출 직후 즉각적인 재감지 무시 (스침 억제)

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
        print("  장애물 회피 v3d: 무한 교착상태(Deadlock) 해결 적용")
        print("  - 연속 코너 감지 시 기존 회전 방향 강제 유지 (오실레이션 방지)")
        print("  - 3연속 코너 감지 시 강제 180도 광역 회전 및 딥 후진")
        print("  - 쿨다운(Cooldown) 타이머 적용으로 미세한 재감지 패스")
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
        is_cornered = (front_min <= CORNER_FRONT and blocked >= CORNER_BLOCKED)
        front_sectors_min = min(sector_avg[0], sector_avg[NUM_SECTORS - 1])

        front_clear_signal = False
        signal_source = ""
        if front_cnt >= MIN_COUNT and front_min >= FRONT_CLEAR:
            front_clear_signal = True
            signal_source = f"narrow={front_min:.0f}mm"
        elif front_sectors_min >= FRONT_CLEAR:
            front_clear_signal = True
            signal_source = f"wide={front_sectors_min:.0f}mm"

        # ── 우선순위 1: 탈출 회전 (광역 폐루프) ──────────────────────
        if escape_left > 0:
            # 180도 도는 중이라면 전방 클리어 신호가 잡혀도 무시하고 끝까지 돔
            if front_clear_signal and escape_left < (180.0 / DEG_PER_CYCLE): 
                escape_left    = 0
                forward_commit = COMMIT_CYCLES
                corner_cooldown = 4  # 탈출 직후 4사이클 동안 코너 재감지 무시
                ser_Ardu.write(b"F 0.00 0.70\n")
                print(f"ESCAPE_DONE  {signal_source} → 전진확정 {COMMIT_CYCLES}사이클")
            else:
                ser_Ardu.write(f"T {escape_steer:.2f}\n".encode())
                escape_left -= 1
                if escape_left == 0:
                    forward_commit = COMMIT_CYCLES
                    corner_cooldown = 4
                    print(f"ESCAPE_EXPIRED  사이클 만료 → 전진확정 {COMMIT_CYCLES}사이클")
                else:
                    fm = f"{front_min:.0f}mm" if front_cnt >= MIN_COUNT else "N/A"
                    print(f"ESCAPE_ROTATE  steer={escape_steer:+.2f}  잔여={escape_left}  "
                          f"front={fm}  wide_front={front_sectors_min:.0f}mm")

        # ── 우선순위 2: 후진 진행 중 ──────────────────────────────────
        elif extra_back > 0:
            ser_Ardu.write(b"B 0.70\n")
            extra_back -= 1
            if extra_back == 0 and pending_escape:
                # 3번 이상 연속 탈출 시도면, 재계산 없이 바로 180도 강제 파워 턴
                if escape_streak >= 3:
                    escape_steer  = last_steer_dir if last_steer_dir != 0.0 else 1.0
                    pending_angle = 180.0
                    escape_left   = int(180.0 / DEG_PER_CYCLE)
                    pending_escape = False
                    print(f"DEADLOCK BREAK! 무조건 180도 회전 락온 (방향: {'R' if escape_steer<0 else 'L'})")
                else:
                    fresh_avg = [
                        (sector_sum[i] / sector_cnt_buf[i]) if sector_cnt_buf[i] > 0 else 8000.0
                        for i in range(NUM_SECTORS)
                    ]
                    best_idx    = fresh_avg.index(max(fresh_avg))
                    best_avg    = fresh_avg[best_idx]
                    best_center = best_idx * SECTOR_SIZE + SECTOR_SIZE / 2

                    new_steer = -1.0 if best_center <= 180 else +1.0
                    new_angle = best_center if best_center <= 180 else 360.0 - best_center

                    # 진동(Ping-Pong) 방지: 이전 방향과 달라지려 하면, 강제로 이전 방향 고정
                    if last_steer_dir != 0.0 and new_steer != last_steer_dir:
                        escape_steer  = last_steer_dir
                        pending_angle = max(90.0, new_angle) # 강제로 최소 90도 이상 돌림
                        print("OSCILLATION PREVENTED! (와이퍼 현상 방지: 이전 회전 방향 유지)")
                    else:
                        escape_steer  = new_steer
                        pending_angle = new_angle

                    escape_left = max(int(MIN_ESCAPE_ANGLE / DEG_PER_CYCLE),
                                      int(pending_angle    / DEG_PER_CYCLE))
                                      
                    last_steer_dir = escape_steer
                    pending_escape = False
                    
                    eff_angle = max(MIN_ESCAPE_ANGLE, pending_angle)
                    print(f"REPLAN  후진완료 → 재계산  best={best_idx}({best_center:.0f}°) "
                          f"avg={best_avg:.0f}mm  회전={'R' if escape_steer<0 else 'L'} "
                          f"{eff_angle:.0f}°  최대{escape_left}사이클")
            else:
                print(f"ESCAPE_A  후진 잔여={extra_back}사이클")

        # ── 우선순위 3: 전진 확정 ────────────────────────────────────
        elif forward_commit > 0:
            forward_commit -= 1
            # 전진 확정을 무사히 마쳤다면 무한루프 및 탈출 카운터 완전 초기화
            if forward_commit == 0:
                escape_streak = 0
                last_steer_dir = 0.0

            if front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:
                ser_Ardu.write(b"B 0.70\n")
                forward_commit = 0
                print(f"COMMIT_ABORT!  긴급상황(F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}) → 후진")
            else:
                ser_Ardu.write(b"F 0.00 0.70\n")
                print(f"COMMIT_FORWARD  잔여={forward_commit}  front={front_min:.0f}mm")

        # ── 우선순위 4: 구석 감지 ────────────────────────────────────
        # 쿨다운 중첩 방지: 코너 쿨다운이 없을 때만 구석으로 판별. 
        elif is_cornered and corner_cooldown == 0:
            escape_streak += 1
            pending_escape = True
            
            if escape_streak >= 3:
                # 심각한 데드락 상태: 뒤로 2배 깊게 뺌
                extra_back = BACKUP_CYCLES * 2
                print(f"DEADLOCK ALERT! ({escape_streak}연속 갇힘) → 딥(Deep) 후진 시작")
            else:
                extra_back = BACKUP_CYCLES
                
            ser_Ardu.write(b"B 0.70\n")
            print(f"CORNER!  F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm "
                  f"blocked={blocked}/{NUM_SECTORS}")

        # ── 코너 쿨다운 관리 ──────────────────────────────────────────
        elif corner_cooldown > 0:
            corner_cooldown -= 1
            ser_Ardu.write(b"F 0.00 0.70\n")

        # ── 우선순위 5: 긴급 후진 ────────────────────────────────────
        elif (front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY):
            back_cnt += 1
            if back_cnt >= 6:
                ser_Ardu.write(b"B 0.70\n")
                extra_back = 3
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
            escape_streak = 0  # 일반 회피를 할 정도면 충분히 빠져나온 것으로 간주
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
            escape_streak = 0
            last_steer_dir = 0.0
            back_cnt = 0

        scan_buf       = []
        front_min      = 9999.0
        left_min       = 9999.0
        right_min      = 9999.0
        front_cnt      = 0
        left_cnt       = 0
        right_cnt      = 0
        sector_sum     = [0.0] * NUM_SECTORS
        sector_cnt_buf = [0]   * NUM_SECTORS