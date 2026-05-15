import serial
import time
import atexit

port_L = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

baudrate_L = 460800
baudrate_Ardu = 460800

ser_L = serial.Serial(port_L, baudrate_L, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, baudrate_Ardu, timeout=1)

# 차량 설정
CAR_WIDTH = 200.0
SAFETY_MARGIN = 100.0
MIN_PASS_WIDTH = CAR_WIDTH + SAFETY_MARGIN

# 거리 기준
EMERGENCY = 150.0
DETECT = 350.0

# smoothing
SMOOTH_ALPHA = 0.6
prev_front = 9999
prev_left = 9999
prev_right = 9999

scan_request = bytes([0xA5,0x40])
ser_L.write(scan_request)
time.sleep(1)

scan_request = bytes([0xA5,0x20])
ser_L.write(scan_request)

scan_buf = []

front_min = 9999
left_min = 9999
right_min = 9999
back_min = 9999

front_cnt = 0
left_cnt = 0
right_cnt = 0
back_cnt_pts = 0

MIN_COUNT = 5

back_cnt = 0
extra_back = 0

def cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5,0x25]))
        ser_L.close()
        ser_Ardu.close()
    except:
        pass

atexit.register(cleanup)

print("Autonomous Car Start")

while True:

    data = ser_L.read(5)
    if len(data) != 5:
        continue

    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        continue

    check_bit = data[1] & 0x01
    if check_bit != 1:
        continue

    quality = data[0] >> 2

    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    angle = angle_q6 / 64.0

    distance_q2 = (data[3] | (data[4] << 8))
    distance = distance_q2 / 4.0

    if distance < 80:
        continue

    if quality == 0:
        continue

    # 구역 분류

    if (angle <= 20 or angle >= 340) and distance <= DETECT:
        front_min = min(front_min, distance)
        front_cnt += 1

    elif (angle > 20 and angle < 60) and distance <= DETECT:
        right_min = min(right_min, distance)
        right_cnt += 1

    elif (angle > 300 and angle < 340) and distance <= DETECT:
        left_min = min(left_min, distance)
        left_cnt += 1

    elif (angle > 160 and angle < 200) and distance <= DETECT:
        back_min = min(back_min, distance)
        back_cnt_pts += 1

    scan_buf.append((angle,distance))

    if s_flag == 1 and len(scan_buf) > 15:

        if front_cnt < MIN_COUNT: front_min = 9999
        if left_cnt < MIN_COUNT: left_min = 9999
        if right_cnt < MIN_COUNT: right_min = 9999
        if back_cnt_pts < MIN_COUNT: back_min = 9999

        # smoothing
        front_min = SMOOTH_ALPHA * front_min + (1-SMOOTH_ALPHA)*prev_front
        left_min = SMOOTH_ALPHA * left_min + (1-SMOOTH_ALPHA)*prev_left
        right_min = SMOOTH_ALPHA * right_min + (1-SMOOTH_ALPHA)*prev_right

        prev_front = front_min
        prev_left = left_min
        prev_right = right_min

        gap_width = left_min + right_min

        # 후진 중 뒤 충돌 방지
        if extra_back > 0:

            if back_min < 200:
                ser_Ardu.write(b"S\n")
                print("BACK BLOCKED → STOP")
                extra_back = 0

            else:
                ser_Ardu.write(b"B 0.80\n")
                extra_back -= 1
                print("EXTENDED BACK")

        # 긴급 충돌
        elif front_min <= EMERGENCY or left_min <= EMERGENCY or right_min <= EMERGENCY:

            back_cnt += 1

            if back_min > 200:
                ser_Ardu.write(b"B 0.80\n")
                ser_Ardu.write(b"B 0.90\n")

                if back_cnt >= 5:
                    extra_back = 3
                    back_cnt = 0

                print("EMERGENCY BACK")

        # 통로 폭 판단
        elif gap_width < MIN_PASS_WIDTH:

            if left_min > right_min:
                ser_Ardu.write(b"L 0.60\n")
                print("NARROW GAP → LEFT")

            else:
                ser_Ardu.write(b"R 0.60\n")
                print("NARROW GAP → RIGHT")

        # 코너 감속
        elif front_min < 420:

            if left_min > right_min:
                ser_Ardu.write(b"L 0.50\n")
                print("CORNER LEFT SLOW")

            else:
                ser_Ardu.write(b"R 0.50\n")
                print("CORNER RIGHT SLOW")

        # 일반 주행
        else:

            if left_min > right_min + 120:
                ser_Ardu.write(b"L 0.70\n")

            elif right_min > left_min + 120:
                ser_Ardu.write(b"R 0.70\n")

            else:
                ser_Ardu.write(b"F 0.80\n")

        # 초기화
        scan_buf.clear()

        front_min = 9999
        left_min = 9999
        right_min = 9999
        back_min = 9999

        front_cnt = 0
        left_cnt = 0
        right_cnt = 0
        back_cnt_pts = 0