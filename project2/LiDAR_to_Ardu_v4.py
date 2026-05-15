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
    if distance < 50:  # 거리가 너무 짧은 경우 노이즈로 간주하고 무시
        continue  

        
    #  첫 루프 진입 시 1회 초기화
    try:
        _ready
    except NameError:
        import atexit
        _ready     = True
        scan_buf   = []        # 1회전 포인트 누적 버퍼
        front_min  = 9999.0    # 전방 최소 거리 (mm)
        left_min   = 9999.0    # 좌측 최소 거리 (mm)
        right_min  = 9999.0    # 우측 최소 거리 (mm)
        front_cnt  = 0         # 앞쪽 구역 유효 포인트 수
        left_cnt   = 0         # 왼쪽 구역 유효 포인트 수
        right_cnt  = 0         # 오른쪽 구역 유효 포인트 수
        MIN_COUNT  = 4         # 최소 포인트 수 (미만이면 노이즈로 무시)
        EMERGENCY  = 130.0     # 즉시 후진 거리 (mm) — 여유 확보
        DETECT     = 200.0     # 장애물 감지 거리 (mm)
        
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

    # 포인트를 구역별 최솟값 및 카운트에 반영
    
    # 전방 구역: ±20°
    if (angle <= 20 or angle >= 340) and distance <= 180:
        front_min = min(front_min, distance)
        front_cnt += 1

    # 오른쪽 구역 (CW 기준: 0°=앞, 50°=우 → 20~50°)
    elif (angle > 20 and angle < 50) and distance <= 200:
        right_min = min(right_min, distance)
        right_cnt += 1

    # 왼쪽 구역 (CW 기준: 310°=좌 → 310~340°)
    elif (angle > 310 and angle < 340) and distance <= 200:
        left_min = min(left_min, distance)
        left_cnt += 1

    scan_buf.append((angle, distance))

    # 새 회전 시작(s_flag==1) → 1회전치 데이터로 판단 및 명령 전송
    if s_flag == 1 and len(scan_buf) > 15: 
        # 충분한 포인트가 누적된 경우에만 처리

        # 최소 포인트 수 미만 구역은 노이즈로 간주해 무시
        if front_cnt < MIN_COUNT: front_min = 9999.0
        if left_cnt  < MIN_COUNT: left_min  = 9999.0
        if right_cnt < MIN_COUNT: right_min = 9999.0


        # 긴급 후진: 어느 방향이든 EMERGENCY 이내
        # 우선순위 최상위 — 조건 순서 버그 방지를 위해 분리
        if front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:
            ser_Ardu.write(b"B 0.50\n")
            print(f"EMERGENCY! F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm → BACKWARD")
            

        # 전방 장애물 회피 (우선순위 최상위)
        # 정면 벽이 좌/우 구역에도 동시에 잡히므로 front를 먼저 처리
        elif front_min <= DETECT:
            ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
            speed = 0.70 * (1 - ratio * 0.7)
            # left_min > right_min → 왼쪽에 공간 있음 → 좌회전(양수) / 오른쪽에 공간 있거나 동일 → 우회전(음수, 기본값)
            steer = -(ratio * 0.90) if left_min > right_min else (ratio * 0.90)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"F_OBS  {front_min:.0f}mm → {'R' if steer<0 else 'L'} steer={steer:.2f} spd={speed:.2f}")
            print(f"DBG L:{left_min:.0f}mm({left_cnt}pts) R:{right_min:.0f}mm({right_cnt}pts)")

        elif left_min <= DETECT:
            ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
            steer = (ratio * 0.90)
            speed = 0.70 * (1 - ratio * 0.6)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"L_OBS  {left_min:.0f}mm (pts:{left_cnt})")


        elif right_min <= DETECT:
            ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
            steer = -(ratio * 0.90)
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
