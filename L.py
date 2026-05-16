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
# 2. 파라미터
# ============================================================

# 라이다 유효 거리
MIN_DIST = 80.0

# RC카 크기: 20cm x 20cm, 라이다 정중앙
CAR_HALF_WIDTH = 100.0       # mm
SAFETY_MARGIN = 30.0         # mm
PATH_HALF_WIDTH = CAR_HALF_WIDTH + SAFETY_MARGIN  # 150mm

# 현재 진행 경로 검사
PATH_CHECK_DIST = 350.0      # 앞쪽 45cm까지 검사
PATH_DANGER_DIST = 300.0     # 30cm 이내면 회피 시작
PATH_BLOCK_POINTS = 4

# 좌우 여유공간 검사
SIDE_CHECK_DIST = 550.0
SIDE_MAX_SCORE_DIST = 650.0

# 진짜 막힘 판단
STUCK_FRONT_DIST = 170.0
STUCK_POINTS = 6

# 속도
NORMAL_SPEED = 0.42
AVOID_SPEED = 0.30
BACK_SPEED = 0.35
ESCAPE_SPEED = 0.30

# 조향
AVOID_STEER = 0.52=--
ESCAPE_STEER = 0.50

# 조향 smoothing
SMOOTH = 0.45
prev_steer = 0.0

# 상태
MODE_NORMAL = 0
MODE_BACK = 1
MODE_ESCAPE = 2

mode = MODE_NORMAL

back_count = 0
escape_count = 0
escape_dir = 0

BACK_CYCLES = 2
ESCAPE_CYCLES = 6


# ============================================================
# 3. 종료 처리
# ============================================================

def cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))  # STOP
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
    x > 0 : RC카가 현재 바라보는 앞쪽
    y > 0 : 오른쪽
    y < 0 : 왼쪽
    """
    a = normalize_angle(angle)
    theta = math.radians(a)

    x = distance * math.cos(theta)
    y = distance * math.sin(theta)

    return x, y


def choose_wider_dir(left_score, right_score):
    """
    왼쪽 선택  -> -1
    오른쪽 선택 ->  1
    """
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
    ser_L.read(7)  # response descriptor
except Exception:
    pass

print("=" * 60)
print("단순 완주 우선 LiDAR 장애물 회피 시작")
print("RC카 크기: 20cm x 20cm")
print("라이다 위치: 정중앙")
print("구조: NORMAL → BACK → ESCAPE")
print(f"PATH_HALF_WIDTH = {PATH_HALF_WIDTH:.0f} mm")
print(f"PATH_CHECK_DIST = {PATH_CHECK_DIST:.0f} mm")
print(f"PATH_DANGER_DIST = {PATH_DANGER_DIST:.0f} mm")
print("=" * 60)


# ============================================================
# 6. 스캔 누적 변수
# ============================================================

scan_buf = []

path_cnt = 0
path_min = 9999.0

front_close_cnt = 0
front_min = 9999.0

left_score = 0.0
right_score = 0.0
left_cnt = 0
right_cnt = 0


# ============================================================
# 7. 메인 루프
# ============================================================

while True:
    data = ser_L.read(5)

    if len(data) != 5:
        continue

    # --------------------------------------------------------
    # 패킷 검증
    # --------------------------------------------------------
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

    # --------------------------------------------------------
    # 각도, 거리 계산
    # --------------------------------------------------------
    angle_q6 = ((data[1] >> 1) | (data[2] << 7))
    angle = angle_q6 / 64.0

    distance_q2 = data[3] | (data[4] << 8)
    distance = distance_q2 / 4.0

    if distance < MIN_DIST:
        continue

    scan_buf.append((angle, distance))

    x, y = polar_to_xy(angle, distance)

    # --------------------------------------------------------
    # 1) 차폭 안 현재 진행 경로 검사
    # --------------------------------------------------------
    if 0 < x < PATH_CHECK_DIST 및 abs(y) < PATH_HALF_WIDTH:
        path_cnt += 1
        path_min = min(path_min, x)

    # --------------------------------------------------------
    # 2) 진짜 막힘 판단용: 매우 가까운 정면 점
    # --------------------------------------------------------
    if 0 < x < STUCK_FRONT_DIST 및 abs(y) < PATH_HALF_WIDTH:
        front_close_cnt += 1
        front_min = min(front_min, x)

    # --------------------------------------------------------
    # 3) 좌우 여유공간 점수 계산
    # --------------------------------------------------------
    if 0 < x < SIDE_CHECK_DIST:
        score = min(distance, SIDE_MAX_SCORE_DIST)

        if y < -PATH_HALF_WIDTH:
            left_score += score
            left_cnt += 1

        elif y > PATH_HALF_WIDTH:
            right_score += score
            right_cnt += 1

    # --------------------------------------------------------
    # 한 바퀴 스캔 완료 시 판단
    # --------------------------------------------------------
    if s_flag == 1 및 len(scan_buf) > 15:

        path_blocked = (
            path_cnt >= PATH_BLOCK_POINTS 및
            path_min < PATH_DANGER_DIST
        )

        stuck = (
            front_close_cnt >= STUCK_POINTS 및
            front_min < STUCK_FRONT_DIST
        )

        wider_dir = choose_wider_dir(left_score, right_score)

        # ====================================================
        # 상태 기반 판단
        # ====================================================

        if mode == MODE_BACK:
            send_backward(BACK_SPEED)
            back_count -= 1

            print(
                f"BACK remain={back_count} "
                f"escape_dir={escape_dir}"
            )

            if back_count <= 0:
                mode = MODE_ESCAPE
                escape_count = ESCAPE_CYCLES

        elif mode == MODE_ESCAPE:
            send_forward(ESCAPE_STEER * escape_dir, ESCAPE_SPEED)
            escape_count -= 1

            print(
                f"ESCAPE dir={escape_dir} "
                f"remain={escape_count}"
            )

            if escape_count <= 0:
                mode = MODE_NORMAL

        else:
            # ------------------------------------------------
            # NORMAL 상태
            # ------------------------------------------------

            if stuck:
                escape_dir = wider_dir
                mode = MODE_BACK
                back_count = BACK_CYCLES

                send_backward(BACK_SPEED)

                print(
                    f"STUCK → BACK "
                    f"front_min={front_min:.0f} "
                    f"front_cnt={front_close_cnt} "
                    f"Lscore={left_score:.0f} "
                    f"Rscore={right_score:.0f} "
                    f"escape_dir={escape_dir}"
                )

            elif path_blocked:
                steer = AVOID_STEER * wider_dir
                send_forward(steer, AVOID_SPEED)

                print(
                    f"PATH_BLOCKED → AVOID "
                    f"path_min={path_min:.0f} "
                    f"path_cnt={path_cnt} "
                    f"Lscore={left_score:.0f} "
                    f"Rscore={right_score:.0f} "
                    f"dir={wider_dir} "
                    f"steer={steer:.2f}"
                )

            else:
                send_forward(0.0, NORMAL_SPEED)

                print(
                    f"CLEAR → FORWARD "
                    f"Lscore={left_score:.0f} "
                    f"Rscore={right_score:.0f}"
                )

        # ----------------------------------------------------
        # 다음 스캔 초기화
        # ----------------------------------------------------
        scan_buf = []

        path_cnt = 0
        path_min = 9999.0

        front_close_cnt = 0
        front_min = 9999.0

        left_score = 0.0
        right_score = 0.0
        left_cnt = 0
        right_cnt = 0
