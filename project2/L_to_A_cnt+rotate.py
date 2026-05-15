import serial
import time

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"
ser_L     = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu  = serial.Serial(port_Ardu, 460800, timeout=1)

ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

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

    quality  = data[0] >> 2
    angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0
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
        # 구석 탈출 관련 변수
        corner_cnt      = 0        # 연속 구석 감지 횟수
        CORNER_THRESH   = 3        # 탈출 루틴 진입 임계값
        ROTATE_CYCLES   = 8        # 회전 지속 사이클 수
        CORNER_BACK     = 3        # 구석 탈출 전 후진 사이클 수
        rotate_left     = 0        # 회전 남은 사이클
        rotate_steer    = 0.0      # 회전 방향 steer 값
        pending_rotate  = False    # 후진 완료 후 회전 대기 플래그
        pending_rot_dir = 0        # +1=좌회전, -1=우회전

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
        print("  장애물 회피 [방법 B+D: 막힘감지 + 구석탈출] 시작")
        print(f"  감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
        print(f"  구석판단: {CORNER_THRESH}연속  /  회전사이클: {ROTATE_CYCLES}")
        print("  ※ 아두이노에서 'T {{steer}}\\n' 제자리 회전 명령 지원 필요")
        print("=" * 55)

    if quality == 0:
        continue

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

        # 구석 판단 조건 (D): 세 구역 모두 DETECT 이내
        is_cornered = (front_min <= DETECT and
                       left_min  <= DETECT and
                       right_min <= DETECT)

        # ── 우선순위 1: 탈출 회전 진행 중 ────────────────────────────
        if rotate_left > 0:
            ser_Ardu.write(f"T {rotate_steer:.2f}\n".encode())
            rotate_left -= 1
            print(f"ESCAPE_ROTATE  steer={rotate_steer:+.2f}  잔여={rotate_left}사이클")

        # ── 우선순위 2: 후진 진행 중 ──────────────────────────────────
        elif extra_back > 0:
            ser_Ardu.write(b"B 0.70\n")
            extra_back -= 1
            # 구석 탈출 후진 완료 → 회전 시작
            if extra_back == 0 and pending_rotate:
                rotate_steer   = +1.0 if pending_rot_dir > 0 else -1.0
                rotate_left    = ROTATE_CYCLES
                pending_rotate = False
                dir_str = "좌회전(+1.00)" if rotate_steer > 0 else "우회전(-1.00)"
                print(f"ESCAPE_BD  후진완료 → {dir_str}  {ROTATE_CYCLES}사이클 시작")
            else:
                label = "구석탈출" if pending_rotate else "확장"
                print(f"{label}_BACK  잔여={extra_back}사이클")

        # ── 우선순위 3: 구석 감지 (B+D) ──────────────────────────────
        elif is_cornered:
            corner_cnt += 1
            ser_Ardu.write(b"B 0.70\n")
            print(f"CORNERED!  ({corner_cnt}/{CORNER_THRESH})  "
                  f"F:{front_min:.0f}  L:{left_min:.0f}  R:{right_min:.0f}mm  → 후진")

            if corner_cnt >= CORNER_THRESH:
                # 더 열린 쪽(left_min vs right_min)으로 회전 예약
                pending_rot_dir = +1 if left_min > right_min else -1
                pending_rotate  = True
                extra_back      = CORNER_BACK
                corner_cnt      = 0
                dir_str = "좌회전" if pending_rot_dir > 0 else "우회전"
                print(f"ESCAPE_BD  임계도달 → 후진 {CORNER_BACK}사이클 후 {dir_str} 예정")

        # ── 우선순위 4: 긴급 후진 (구석 아님) ───────────────────────
        elif (front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY):
            corner_cnt = 0
            back_cnt  += 1
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
            corner_cnt = 0
            ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
            speed = 0.70 * (1 - ratio * 0.7)
            steer = -(ratio * 0.85) if left_min > right_min else (ratio * 0.85)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"F_OBS  {front_min:.0f}mm → {'R' if steer<0 else 'L'}  steer={steer:.2f}  spd={speed:.2f}")
            print(f"DBG  L:{left_min:.0f}mm({left_cnt}pts)  R:{right_min:.0f}mm({right_cnt}pts)")

        elif left_min <= DETECT:
            corner_cnt = 0
            ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
            steer = (ratio * 0.75)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"L_OBS  {left_min:.0f}mm  (pts:{left_cnt})")

        elif right_min <= DETECT:
            corner_cnt = 0
            ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
            steer = -(ratio * 0.75)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"R_OBS  {right_min:.0f}mm  (pts:{right_cnt})")

        else:
            corner_cnt = 0
            ser_Ardu.write(b"F 0.00 0.70\n")

        # 사이클 말 초기화
        scan_buf  = []
        front_min = 9999.0
        left_min  = 9999.0
        right_min = 9999.0
        front_cnt = 0
        left_cnt  = 0
        right_cnt = 0
