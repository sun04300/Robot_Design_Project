import serial
import time

port_L = "/dev/ttyUSB0" # USB-시리얼 포트 - LiDAR 연결된 포트로 설정
port_Ardu = "/dev/ttyS0" # USB-시리얼 포트 - 아두이노 연결된 포트로 설정

baudrate_L = 460800 # 보드레이트 적용
baudrate_Ardu = 460800 # 보드레이트 적용

ser_L = serial.Serial(port_L, baudrate_L, timeout=1) # LiDAR 시리얼 포트와 보드레이트 설정 -> 460800 bps
ser_Ardu = serial.Serial(port_Ardu, baudrate_Ardu, timeout=1) # 라즈베리파이의 시리얼 포트와 보드레이트 설정 -> 115200 bps


# RESET 요청 패킷 전송 (0xA5 0x40)
scan_request = bytes([0xA5, 0x40])
ser_L.write(scan_request)
time.sleep(1) # 1초 동안 멈춤 (초기화 시간 확보)

# SCAN 요청 패킷 전송 (0xA5 0x20)
scan_request = bytes([0xA5, 0x20])
ser_L.write(scan_request)
        
# 응답 데이터 읽기
while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    # Start Flag와 Inversed Start Flag 검증
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        continue

    # Check Bit 검증
    check_bit = data[1] & 0x01
    if check_bit != 1:
        continue

    # 품질
    quality = data[0] >> 2

    # 각도 계산
    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    angle = angle_q6 / 64.0 #각도

    # 거리 계산
    distance_q2 = (data[3] | (data[4] << 8))
    distance = distance_q2 / 4.0  # 거리mm
    if distance < 80:  # 거리가 너무 짧은 경우 노이즈로 간주하고 무시
        continue  

        
    #  첫 루프 진입 시 1회 초기화
    try:
        _ready
    except NameError:
        import atexit
        _ready     = True
        scan_buf   = []        # 1회전 포인트 누적 버퍼
        front_min  = 9999.0    # 전방 최소 거리 (mm)
        left_min   = 9999.0    # 좌측 최소 거리 (mm)  [긴급후진 판단용]
        right_min  = 9999.0    # 우측 최소 거리 (mm)  [긴급후진 판단용]
        front_cnt  = 0         # 앞쪽 구역 유효 포인트 수
        left_cnt   = 0         # 왼쪽 구역 유효 포인트 수
        right_cnt  = 0         # 오른쪽 구역 유효 포인트 수
        MIN_COUNT  = 4         # 최소 포인트 수 (미만이면 노이즈로 무시)
        back_cnt   = 0         # 누적 후진 횟수
        extra_back = 0         # 확장 후진 남은 사이클 수
        EMERGENCY  = 140.0     # 즉시 후진 거리 (mm) — 여유 확보
        DETECT     = 340.0     # 장애물 감지 거리 (mm)

        # ── 좌/우 개방도 측정용 넓은 구역 ─────────────────────
        # 기존 left_min/right_min: 좁은 전방-사이드(±20°~50°) 만 감지
        # 아래 변수: 진짜 측면까지 포함한 넓은 호(arc)의 평균 거리를 측정
        #   평균 거리가 클수록 → 장애물이 멀다 → 더 개방된 공간
        # 우측 개방 구역: 20°~100° (전방 우측 ~ 진짜 우측)
        # 좌측 개방 구역: 260°~340° (진짜 좌측 ~ 전방 좌측)
        left_open_sum  = 0.0;  left_open_cnt  = 0
        right_open_sum = 0.0;  right_open_cnt = 0
        
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
        print("=" * 50)
        print("  장애물 회피 자동차 시작  (Ctrl+C 로 종료)")
        print(f"  감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
        print("=" * 50)

    # 품질 필터: quality==0 은 노이즈 포인트 → 스킵
    if quality == 0:
        continue

    # ── 포인트를 구역별 최솟값/합계/카운트에 반영 ────────────

    # 전방 구역: ±20° (긴급후진·전방회피 판단)
    if (angle <= 20 or angle >= 340) and distance <= 320:
        front_min = min(front_min, distance)
        front_cnt += 1

    # 오른쪽 narrow 구역: 20°~50° (긴급후진 판단용)
    elif (angle > 20 and angle < 50) and distance <= 330:
        right_min = min(right_min, distance)
        right_cnt += 1

    # 왼쪽 narrow 구역: 310°~340° (긴급후진 판단용)
    elif (angle > 310 and angle < 340) and distance <= 330:
        left_min = min(left_min, distance)
        left_cnt += 1

    # ── 넓은 좌/우 개방도 구역 (조향 방향 결정용) ────────────
    # 전방(±20°)과 후방(150°~210°)을 제외한 순수 측면 공간 측정
    # 평균 거리 = 공간이 넓을수록 값이 큼
    if 20 <= angle < 70:                   # 우측 넓은 호 (20°~70°)
        right_open_sum += distance
        right_open_cnt += 1
    elif 290 <= angle < 340:                # 좌측 넓은 호 (290°~340°)
        left_open_sum  += distance
        left_open_cnt  += 1

    scan_buf.append((angle, distance))

    # 새 회전 시작(s_flag==1) → 1회전치 데이터로 판단 및 명령 전송
    if s_flag == 1 and len(scan_buf) > 15: 
        # 충분한 포인트가 누적된 경우에만 처리

        # 최소 포인트 수 미만 구역은 노이즈로 간주해 무시
        if front_cnt < MIN_COUNT: front_min = 9999.0
        if left_cnt  < MIN_COUNT: left_min  = 9999.0
        if right_cnt < MIN_COUNT: right_min = 9999.0


        # 확장 후진 진행 중: 나머지 사이클 동안 후진 명령 유지
        if extra_back > 0:
            ser_Ardu.write(b"B 0.80\n")
            extra_back -= 1
            print(f"EXTENDED_BACK 잔여 {extra_back}사이클")

        # 긴급 후진: 어느 방향이든 EMERGENCY 이내
        # 우선순위 최상위 — 조건 순서 버그 방지를 위해 분리
        elif front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:
            back_cnt += 1
            if back_cnt >= 6:
                ser_Ardu.write(b"B 0.90\n")
                extra_back = 3          # 이번 포함 총 3사이클 = 기본의 3배
                back_cnt   = 0
                print(f"EXTENDED_BACK 시작! (3x) → back_cnt 초기화")
            else:
                ser_Ardu.write(b"B 0.90\n")
                print(f"EMERGENCY! F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm → BACKWARD ({back_cnt}/6)")
            

        # 전방 장애물 회피
        elif front_min <= DETECT:
            ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
            speed = 0.70 * (1 - ratio * 0.7)

            # ── 넓은 구역 평균 거리로 개방 방향 결정 ──────────
            # 평균이 클수록 = 장애물이 멀다 = 더 개방된 공간
            # 포인트가 없으면(0) 해당 방향 데이터 없음 → 반대 방향 선택
            left_avg  = left_open_sum  / left_open_cnt  if left_open_cnt  >= 2 else 0.0
            right_avg = right_open_sum / right_open_cnt if right_open_cnt >= 2 else 0.0

            if left_avg == 0.0 and right_avg == 0.0:
                # 양쪽 모두 데이터 없음 → 기존 narrow min 비교로 폴백
                steer = -(ratio * 0.85) if left_min > right_min else (ratio * 0.85)
                open_dbg = f"폴백(L:{left_min:.0f}/R:{right_min:.0f})"
            else:
                # left_avg > right_avg → 좌측이 더 개방 → 좌회전(양수 steer)
                # right_avg >= left_avg → 우측이 더 개방 → 우회전(음수 steer)
                steer = (ratio * 0.85) if left_avg > right_avg else -(ratio * 0.85)
                open_dbg = f"L_avg:{left_avg:.0f}mm({left_open_cnt}pts) R_avg:{right_avg:.0f}mm({right_open_cnt}pts)"
                # 기존 narrow min 비교보다 넓은 구역 평균으로 회전 방향 결정 → 더 안정적 회피 가능

            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"F_OBS  {front_min:.0f}mm → {'L' if steer>0 else 'R'} steer={steer:.2f} spd={speed:.2f}")
            print(f"  개방도: {open_dbg}")

        elif left_min <= DETECT:
            ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
            steer = (ratio * 0.75)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"L_OBS  {left_min:.0f}mm (pts:{left_cnt})")


        elif right_min <= DETECT:
            ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
            steer = -(ratio * 0.75)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"R_OBS  {right_min:.0f}mm (pts:{right_cnt})")
            

        # 장애물 없음 → 직진
        else:
            ser_Ardu.write(b"F 0.00 0.70\n")


        # 버퍼 및 구역 값 초기화
        scan_buf  = []
        front_min = 9999.0
        left_min  = 9999.0
        right_min = 9999.0
        front_cnt = 0
        left_cnt  = 0
        right_cnt = 0
