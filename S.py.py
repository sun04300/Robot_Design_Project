import serial
import time
import math
import atexit

# ============================================================
# 1. 통신 설정
# ============================================================

port_L = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

baudrate_L = 460800
baudrate_Ardu = 460800

ser_L = serial.Serial(port_L, baudrate_L, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, baudrate_Ardu, timeout=1)


# ============================================================
# 2. 완주 우선 파라미터
# ============================================================

MIN_DIST = 80.0
MIN_COUNT = 4

DETECT = 350.0
EMERGENCY = 140.0

# 차폭 기준 현재 진행 방향 검사
CAR_HALF_WIDTH = 100.0
SAFETY_MARGIN = 40.0
PATH_HALF_WIDTH = CAR_HALF_WIDTH + SAFETY_MARGIN

PATH_CHECK_DIST = 500.0
PATH_DANGER_DIST = 320.0
PATH_BLOCK_POINTS = 5

# 코너/막힘 최후 대응
CORNER_FRONT = 230.0
CORNER_SIDE = 260.0

# 속도
NORMAL_SPEED = 0.42
AVOID_SPEED = 0.35
CAUTION_SPEED = 0.22
BACK_SPEED = 0.38

# 조향
FRONT_STEER_GAIN = 0.45
SIDE_STEER_GAIN = 0.35
PATH_AVOID_STEER = 0.20

# 조향 smoothing
SMOOTH = 0.82
prev_steer = 0.0

# 후진은 최후 수단
back_phase = 0
back_cnt = 0

# path_blocked 회피 유지
path_avoid_turn = 0
path_avoid_dir = 0


# ============================================================
# 3. 종료 처리
# ============================================================

def cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
    except Exception:
        pass


atexit.register(cleanup)


# ============================================================
# 4. 보조 함수
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def send_forward(steer, speed):
    global prev_steer

    steer = clamp(steer, -0.60, 0.60)
    speed = clamp(speed, 0.0, 1.0)

    steer = SMOOTH * prev_steer + (1.0 - SMOOTH) * steer
    steer = clamp(steer, -0.60, 0.60)

    prev_steer = steer

    ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())


def send_backward(speed=BACK_SPEED):
    global prev_steer

    prev_steer = 0.0
    speed = clamp(speed, 0.0, 1.0)
    ser_Ardu.write(f"B {speed:.2f}\n".encode())


def normalize_angle(angle):
    if angle > 180:
        angle -= 360
    return angle


def polar_to_xy(angle, distance):
    """
    x > 0 : 현재 RC카가 바라보는 앞
    y > 0 : 오른쪽
    y < 0 : 왼쪽
    """
    angle = normalize_angle(angle)
    theta = math.radians(angle)

    x = distance * math.cos(theta)
    y = distance * math.sin(theta)

    return x, y


def choose_wider_dir(left_min, right_min, left_cnt, right_cnt):
    """
    더 넓어 보이는 방향 선택.

    거리만 보면 '멀지만 막힌 방향'을 고를 수 있으므로,
    포인트 개수가 많은 방향은 막힌 방향으로 보고 감점한다.

    기존 코드 부호 기준:
    왼쪽 선택  -> -1
    오른쪽 선택 ->  1
    """

    L = min(left_min, 800.0)
    R = min(right_min, 800.0)

    left_score = L - left_cnt * 20
    right_score = R - right_cnt * 20

    if left_score > right_score:
        return -1
    else:
        return 1


# ============================================================
# 5. LiDAR 시작
# ============================================================

ser_L.write(bytes([0xA5, 0x40]))  # RESET
time.sleep(1.0)

ser_L.write(bytes([0xA5, 0x20]))  # SCAN
time.sleep(0.05)

try:
    ser_L.read(7)
except Exception:
    pass

print("=" * 60)
print("완주 우선 LiDAR 장애물 회피 시작")
print("핵심: path_blocked 발생 시 더 넓은 쪽으로 약하게 회피")
print(f"PATH_HALF_WIDTH={PATH_HALF_WIDTH:.0f}mm")
print(f"PATH_CHECK_DIST={PATH_CHECK_DIST:.0f}mm")
print("=" * 60)


# ============================================================
# 6. 스캔 누적 변수
# ============================================================

scan_buf = []

front_min = 9999.0
left_min = 9999.0
right_min = 9999.0

front_cnt = 0
left_cnt = 0
right_cnt = 0

path_min = 9999.0
path_cnt = 0


# ============================================================
# 7. 메인 루프
# ============================================================

while True:
    data = ser_L.read(5)

    if len(data) != 5:
        continue

    # 패킷 검증
    s_flag = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1

    if s_inv_flag != (1 - s_flag):
        continue

    check_bit = data[1] & 0x01
    if check_bit != 1:
        continue

    quality = data[0] >> 2
    if quality == 0:
        continue

    # 각도, 거리 계산
    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    angle = angle_q6 / 64.0

    distance_q2 = data[3] | (data[4] << 8)
    distance = distance_q2 / 4.0

    if distance < MIN_DIST:
        continue

    scan_buf.append((angle, distance))

    # 기존 전방/좌/우 구역 최소거리
    if (angle <= 20 or angle >= 340) and distance <= DETECT:
        front_min = min(front_min, distance)
        front_cnt += 1

    elif 20 < angle < 50 and distance <= DETECT:
        right_min = min(right_min, distance)
        right_cnt += 1

    elif 310 < angle < 340 and distance <= DETECT:
        left_min = min(left_min, distance)
        left_cnt += 1

    # 차폭 기준 현재 진행 방향 검사
    x, y = polar_to_xy(angle, distance)

    if 0 < x < PATH_CHECK_DIST and abs(y) < PATH_HALF_WIDTH:
        path_min = min(path_min, x)
        path_cnt += 1

    # 한 바퀴 스캔 완료 시 판단
    if s_flag == 1 and len(scan_buf) > 15:

        # 포인트 수 부족하면 노이즈로 무시
        if front_cnt < MIN_COUNT:
            front_min = 9999.0
        if left_cnt < MIN_COUNT:
            left_min = 9999.0
        if right_cnt < MIN_COUNT:
            right_min = 9999.0

        path_blocked = (
            path_cnt >= PATH_BLOCK_POINTS and
            path_min < PATH_DANGER_DIST
        )

        # ====================================================
        # 판단 우선순위
        # ====================================================

        # 1. 최후 후진 유지
        if back_phase > 0:
            send_backward(BACK_SPEED)
            back_phase -= 1

            print(f"BACK_PHASE remain={back_phase}")

        # 2. path_blocked 회피를 짧게 유지
        elif path_avoid_turn > 0:
            send_forward(PATH_AVOID_STEER * path_avoid_dir, CAUTION_SPEED)
            path_avoid_turn -= 1

            print(
                f"PATH_AVOID_HOLD dir={path_avoid_dir} "
                f"remain={path_avoid_turn}"
            )

        # 3. 핵심: 차폭 기준으로 현재 진행 방향이 막힘
        elif path_blocked:
            path_avoid_dir = choose_wider_dir(
                left_min, right_min,
                left_cnt, right_cnt
            )

            send_forward(PATH_AVOID_STEER * path_avoid_dir, CAUTION_SPEED)

            # 바로 직진으로 튀지 않도록 1사이클 유지
            path_avoid_turn = 1

            print(
                f"PATH_BLOCKED path_min={path_min:.0f} path_cnt={path_cnt} "
                f"F:{front_min:.0f}({front_cnt}) "
                f"L:{left_min:.0f}({left_cnt}) "
                f"R:{right_min:.0f}({right_cnt}) "
                f"→ wider_dir={path_avoid_dir}"
            )

        # 4. 이미 막힌 공간에 들어간 경우만 최후 후진
        elif (
            front_min < CORNER_FRONT and
            left_min < CORNER_SIDE and
            right_min < CORNER_SIDE
        ):
            send_backward(BACK_SPEED)
            back_phase = 2

            print(
                f"CORNER_LAST_RESORT "
                f"F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f} "
                f"→ BACK"
            )

        # 5. 충돌 직전 긴급 후진
        elif (
            front_min <= EMERGENCY or
            left_min <= EMERGENCY or
            right_min <= EMERGENCY
        ):
            send_backward(BACK_SPEED)
            back_cnt += 1

            if back_cnt >= 4:
                back_phase = 2
                back_cnt = 0

            print(
                f"EMERGENCY "
                f"F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f} "
                f"→ BACK"
            )

        # 6. 일반 전방 장애물 회피
        elif front_min <= DETECT:
            ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
            ratio = clamp(ratio, 0.0, 1.0)

            avoid_dir = choose_wider_dir(
                left_min, right_min,
                left_cnt, right_cnt
            )

            steer = FRONT_STEER_GAIN * ratio * avoid_dir
            speed = AVOID_SPEED * (1.0 - ratio * 0.25)
            speed = clamp(speed, 0.28, AVOID_SPEED)

            send_forward(steer, speed)

            print(
                f"F_OBS "
                f"F:{front_min:.0f}({front_cnt}) "
                f"L:{left_min:.0f}({left_cnt}) "
                f"R:{right_min:.0f}({right_cnt}) "
                f"dir={avoid_dir} steer={steer:.2f}, speed={speed:.2f}"
            )

        # 7. 왼쪽 장애물 회피
        elif left_min <= DETECT:
            ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
            ratio = clamp(ratio, 0.0, 1.0)

            steer = SIDE_STEER_GAIN * ratio
            speed = AVOID_SPEED * (1.0 - ratio * 0.20)
            speed = clamp(speed, 0.30, AVOID_SPEED)

            send_forward(steer, speed)

            print(
                f"L_OBS L:{left_min:.0f}({left_cnt}) "
                f"steer={steer:.2f}, speed={speed:.2f}"
            )

        # 8. 오른쪽 장애물 회피
        elif right_min <= DETECT:
            ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
            ratio = clamp(ratio, 0.0, 1.0)

            steer = -SIDE_STEER_GAIN * ratio
            speed = AVOID_SPEED * (1.0 - ratio * 0.20)
            speed = clamp(speed, 0.30, AVOID_SPEED)

            send_forward(steer, speed)

            print(
                f"R_OBS R:{right_min:.0f}({right_cnt}) "
                f"steer={steer:.2f}, speed={speed:.2f}"
            )

        # 9. 안전하면 직진
        else:
            back_cnt = 0
            send_forward(0.0, NORMAL_SPEED)

            print("CLEAR → FORWARD")

        # 다음 스캔 초기화
        scan_buf = []

        front_min = 9999.0
        left_min = 9999.0
        right_min = 9999.0

        front_cnt = 0
        left_cnt = 0
        right_cnt = 0

        path_min = 9999.0
        path_cnt = 0