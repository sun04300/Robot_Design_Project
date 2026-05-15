import serial
import time

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

# ── 시리얼 포트 초기화 ─────────────────────────────────────────────────────
# RPLIDAR: 460800bps, 아두이노: 460800bps (양쪽 모두 고속 전송)
ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

# RPLIDAR 초기화 시퀀스
# 0xA5 0x40 = RESET 명령 (드라이버 재시작)
# 0xA5 0x20 = SCAN 명령  (연속 360° 스캔 시작)
ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

# ═══════════════════════════════════════════════════════════════
#  전방 240° 호(arc)  :  120° ──(반시계)──► 0° ──(시계)──► 240°
#
#            0° (전방)
#             │
#  270°(좌) ──┼── 90°(우)
#             │
#            180° (후방)
#
#  [5개 섹터 분할 — 각 48°]
#
#   S0  240°~288°    좌측        steer = +0.90  (하드 좌회전)
#   S1  288°~336°    좌전방      steer = +0.50  (완만 좌회전)
#   S2  336°~ 24°    정면 ★0°관통 steer =  0.00  (직진)
#   S3   24°~ 72°    우전방      steer = -0.50  (완만 우회전)
#   S4   72°~120°    우측        steer = -0.90  (하드 우회전)
#
#  ┌────────────────────────────────────────────────────────┐
#  │  구석 탈출 상태머신                                     │
#  │                                                        │
#  │  NORMAL(0) ──[구석 N회 연속]──► CORNER_BACK(1)         │
#  │                                      │                 │
#  │                        후진하며 S0~S4 거리 누적         │
#  │                                      │[사이클 완료]     │
#  │                                      ▼                 │
#  │                               CORNER_ESC(2)            │
#  │                                      │                 │
#  │                      최대거리 섹터 방향으로 전진 탈출   │
#  │                                      │[사이클 완료]     │
#  │                                      ▼                 │
#  │                               NORMAL(0) 복귀           │
#  └────────────────────────────────────────────────────────┘
# ═══════════════════════════════════════════════════════════════

# ── 섹터 정의 테이블 ────────────────────────────────────────────
# (이름, 시작각, 끝각, 탈출 steer값, 0°관통 여부)
# - steer 부호: 음수 = 우회전, 양수 = 좌회전
# - wrap=True : S2처럼 0°를 관통하는 섹터 (336°~360° + 0°~24°)
SECTORS = [
    ("S0_LEFT",       240, 288,  0.90, False),
    ("S1_FRONTLEFT",  288, 336,  0.50, False),
    ("S2_FRONT",      336,  24,  0.00, True ),   # ★ 0° 관통
    ("S3_FRONTRIGHT",  24,  72, -0.50, False),
    ("S4_RIGHT",       72, 120, -0.90, False),
]
N_SEC = len(SECTORS)


def in_sector(angle, s_start, s_end, wrap):
    """
    주어진 angle이 섹터 범위 안에 있는지 판별.

    wrap=True (0° 관통 섹터) :
        336°~360° 또는 0°~24° 범위 → OR 조건으로 판별
    wrap=False (일반 섹터) :
        s_start <= angle < s_end 범위 판별
    """
    if wrap:                          # 336°~24° (0° 관통)
        return angle >= s_start or angle <= s_end
    return s_start <= angle < s_end


while True:
    # ── RPLIDAR 5바이트 패킷 수신 ──────────────────────────────
    # RPLIDAR 표준 출력 포맷: 5바이트/포인트
    # [0] quality(6b) | s_inv_flag(1b) | s_flag(1b)
    # [1] angle_q6[6:0](7b) | check_bit(1b=1)
    # [2] angle_q6[13:7] (8b)
    # [3] distance_q2[7:0] (8b)
    # [4] distance_q2[15:8] (8b)
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    # ── 패킷 유효성 검사 ──────────────────────────────────────
    # s_flag: 새 스캔 시작을 알리는 플래그 (1회전 = 한 번 1이 됨)
    # s_inv_flag: s_flag의 논리 반전값이어야 함 (비트 오류 검출)
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):   # 반전 불일치 → 패킷 손상
        continue
    if (data[1] & 0x01) != 1:        # check_bit 불일치 → 패킷 손상
        continue

    # ── 각도 / 거리 디코딩 ────────────────────────────────────
    # angle : Q6 고정소수점 → 64로 나눠서 도(°) 단위
    # distance : Q2 고정소수점 → 4로 나눠서 mm 단위
    quality     = data[0] >> 2
    angle_q6    = (data[1] >> 1) | (data[2] << 7)
    angle       = angle_q6 / 64.0
    distance_q2 = data[3] | (data[4] << 8)
    distance    = distance_q2 / 4.0

    # 80mm 미만은 센서 노이즈 또는 자기 몸체 반사로 제거
    if distance < 80:
        continue

    # ── 1회성 초기화 (첫 유효 패킷 수신 시 한 번만 실행) ─────
    # 'try: _ready' 패턴: _ready 변수가 없으면 NameError 발생 → 초기화 실행
    # 이후에는 _ready = True 이므로 except 블록 진입 안 함
    try:
        _ready
    except NameError:
        import atexit
        _ready = True

        # ── [△ 개선 후보] scan_buf (리스트) ─────────────────
        # 현재 scan_buf는 (angle, distance) 튜플을 누적하지만,
        # 실제로는 'len(scan_buf) > 15' 체크에만 사용됨.
        # 튜플 리스트 자체는 결정 로직에서 전혀 읽지 않으므로
        # 아래처럼 정수 카운터로 대체하면 메모리 절약 가능:
        #   scan_cnt = 0
        #   (루프 내) scan_cnt += 1
        #   (판단부) if s_flag == 1 and scan_cnt > 15:
        #   (초기화) scan_cnt = 0
        scan_buf  = []   # ← 리스트 자체는 불필요; 카운터로 대체 가능

        # ── 전방/좌/우 narrow zone 변수 ─────────────────────
        # front: ±10°, right: 10°~50°, left: 310°~350°
        # 사용 목적: 긴급후진·일반회피의 EMERGENCY/DETECT 판단
        # ※ is_corner 조건에서도 사용되나 신뢰도 낮음 (아래 참조)
        front_min = 9999.0;  front_cnt = 0
        left_min  = 9999.0;  left_cnt  = 0
        right_min = 9999.0;  right_cnt = 0
        MIN_COUNT  = 4        # 이 수 미만 포인트 → 해당 구역 무효화
        back_cnt   = 0        # 연속 긴급후진 횟수 (6 이상 → EXTENDED_BACK)
        extra_back = 0        # EXTENDED_BACK 잔여 사이클 수
        EMERGENCY  = 140.0    # 즉시 후진 거리 기준 (mm)
        DETECT     = 350.0    # 일반 장애물 감지 거리 기준 (mm)

        # ── 전방 240° 섹터 변수 (S0~S4) ────────────────────
        # CORNER_BACK 상태에서 후진하며 각 섹터별 최소 거리 누적
        # → 후진 완료 후 가장 먼 섹터(장애물 없는 방향)로 탈출
        sec_min = [9999.0] * N_SEC   # 섹터별 최솟값 (9999 = 포인트 없음)
        sec_cnt = [0]      * N_SEC   # 섹터별 유효 포인트 수

        # ── 구석 탈출 상태머신 변수 ──────────────────────────
        # corner_state: 0=NORMAL / 1=CORNER_BACK / 2=CORNER_ESC
        corner_state     = 0
        corner_back_left = 0    # CORNER_BACK 잔여 사이클
        corner_esc_left  = 0    # CORNER_ESC 잔여 사이클
        corner_esc_cmd   = b""  # 탈출 방향 명령 (후진 완료 후 결정)
        corner_det_n     = 0    # 연속 구석 감지 횟수 (오감지 방지용)

        # ── 구석 탈출 파라미터 ──────────────────────────────
        CORNER_DET_N    = 3     # N회 연속 is_corner=True 시 구석 확정
        CORNER_BACK_CYC = 5     # 구석 탈출 전 후진 사이클 수
        CORNER_ESC_CYC  = 7     # 탈출 전진 사이클 수
        CORNER_BACK_SPD = 0.80  # 구석 탈출 후진 속도
        CORNER_ESC_SPD  = 0.55  # 구석 탈출 전진 속도 (벽 접근 방지용으로 낮게)

        # 프로그램 종료 시 아두이노 정지 + RPLIDAR 스캔 중단
        def _cleanup():
            try:
                ser_Ardu.write(b"S\n")                # 아두이노: 정지
                ser_L.write(bytes([0xA5, 0x25]))      # RPLIDAR: STOP
                time.sleep(0.1)
                ser_L.close()
                ser_Ardu.close()
            except Exception:
                pass
        atexit.register(_cleanup)

        print("=" * 60)
        print("  장애물 회피 + 구석 탈출  (전방 240° 5섹터 판별)")
        print(f"  감지:{int(DETECT)}mm  긴급:{int(EMERGENCY)}mm")
        print(f"  구석감지:{CORNER_DET_N}회  후진:{CORNER_BACK_CYC}c  탈출:{CORNER_ESC_CYC}c")
        print("  섹터: S0(좌240°) S1(좌전288°) S2(정면336°~24°) "
              "S3(우전24°) S4(우72°)")
        print("=" * 60)

    # quality == 0 : 센서 신뢰도 없음 → 스킵
    if quality == 0:
        continue

    # ── 전방/좌/우 narrow zone 포인트 누적 ──────────────────
    # 긴급후진(EMERGENCY)·일반회피(DETECT) 판단에 사용
    # 전방: ±10° 이내 + 345mm 이하
    # 우측: 10°~50° + 350mm 이하  (CW 기준 → 오른쪽)
    # 좌측: 310°~350° + 350mm 이하 (CW 기준 → 왼쪽)
    if (angle <= 10 or angle >= 350) and distance <= 345:
        front_min = min(front_min, distance); front_cnt += 1
    elif 10 < angle < 50 and distance <= 350:
        right_min = min(right_min, distance); right_cnt += 1
    elif 310 < angle < 350 and distance <= 350:
        left_min  = min(left_min,  distance); left_cnt  += 1

    # ── 전방 240° 섹터 포인트 누적 ───────────────────────────
    # 포인트가 해당하는 첫 번째 섹터에만 누적 (break)
    # CORNER_BACK 상태에서 방향 결정에 사용
    for i, (name, s_start, s_end, steer, wrap) in enumerate(SECTORS):
        if in_sector(angle, s_start, s_end, wrap):
            sec_min[i] = min(sec_min[i], distance)
            sec_cnt[i] += 1
            break

    # scan_buf: 실제 결정 로직에서 데이터를 읽지 않음
    # len() 체크에만 사용 → 아래 정수 카운터 대체 가능
    scan_buf.append((angle, distance))

    # ── 1회전 완료 판단 ──────────────────────────────────────
    # s_flag == 1 : 새 스캔 시작 신호 = 직전 회전 완료
    # len(scan_buf) > 15 : 최소 포인트 수 확보 여부 (노이즈 스캔 방지)
    if s_flag == 1 and len(scan_buf) > 15:

        # ── 최소 포인트 미달 구역 무효화 ─────────────────────
        # MIN_COUNT(4개) 미만 포인트 구역은 신뢰도 낮음 → 9999로 리셋
        if front_cnt < MIN_COUNT: front_min = 9999.0
        if left_cnt  < MIN_COUNT: left_min  = 9999.0
        if right_cnt < MIN_COUNT: right_min = 9999.0
        for i in range(N_SEC):
            if sec_cnt[i] < MIN_COUNT:
                sec_min[i] = 9999.0

        # ── [⚠ 잠재적 문제] is_corner 구석 감지 조건 ─────────
        # 현재 조건: 전방 + 좌 + 우 모두 DETECT 이내 = 구석
        #
        # 문제: left_min은 310°~350°, right_min은 10°~50° 범위만 감지.
        # 즉, 전방 사이드 영역만 커버함.
        # 진짜 측면 벽(90°, 270°)이 있어도 이 변수들은 9999 유지 → is_corner=False.
        #
        # 개선안: sec_min을 활용해 "막힌 섹터 수" 기준으로 판별
        #   예) blocked_sec = sum(1 for d in sec_min if d <= DETECT)
        #       is_corner = (front_min <= DETECT and blocked_sec >= 3)
        is_corner = (front_min <= DETECT and
                     left_min  <= DETECT and
                     right_min <= DETECT)

        # ════════════════════════════════════════════════════
        #  상태머신
        # ════════════════════════════════════════════════════

        # ── 상태 1: CORNER_BACK (후진 + 섹터 누적) ───────────
        # 매 사이클 후진 명령을 내리며 sec_min/sec_cnt를 계속 쌓음.
        # 후진하는 동안 각 방향의 거리 정보가 갱신됨 →
        # 후진 완료 후 가장 먼 섹터(열린 방향)를 선택해 탈출.
        if corner_state == 1:
            ser_Ardu.write(f"B {CORNER_BACK_SPD:.2f}\n".encode())
            corner_back_left -= 1

            # 매 사이클 섹터 상황 출력 (디버깅용)
            sec_dbg = "  ".join(
                f"{SECTORS[i][0]}:{sec_min[i]:.0f}" for i in range(N_SEC)
            )
            print(f"[BACK {corner_back_left:02d}] {sec_dbg}  (mm)")

            # ── 후진 완료 → 탈출 방향 결정 ──────────────────
            if corner_back_left <= 0:

                # 유효 섹터(포인트 있는 섹터) 중 최대 거리(가장 열린 방향) 선택
                # sec_min == 9999.0 인 섹터는 포인트 없음(MIN_COUNT 미달) → 제외
                best_idx  = -1
                best_dist = -1.0
                for i in range(N_SEC):
                    if sec_min[i] < 9999.0 and sec_min[i] > best_dist:
                        best_dist = sec_min[i]
                        best_idx  = i

                if best_idx == -1:
                    # 전 섹터가 유효 포인트 없음 → 방향 판단 불가 → 계속 후진
                    # (매우 드문 케이스: 밝은 환경 등 센서 불량 시 발생 가능)
                    corner_esc_cmd = f"B {CORNER_BACK_SPD:.2f}\n".encode()
                    esc_label = "전섹터 무효→직진후진"
                else:
                    # 가장 열린 섹터의 steer값으로 전진 탈출 명령 구성
                    # steer: S0=+0.90(좌) / S1=+0.50(좌) / S2=0.00(직) / S3=-0.50(우) / S4=-0.90(우)
                    esc_name, _, _, esc_steer, _ = SECTORS[best_idx]
                    corner_esc_cmd = (
                        f"F {esc_steer:.2f} {CORNER_ESC_SPD:.2f}\n".encode()
                    )
                    esc_label = f"{esc_name}({best_dist:.0f}mm) steer={esc_steer:+.2f}"

                print(f"[BACK→ESC] 결정: {esc_label}")
                corner_esc_left = CORNER_ESC_CYC
                corner_state    = 2   # CORNER_ESC 상태로 전환

        # ── 상태 2: CORNER_ESC (탈출 전진) ───────────────────
        # 후진 완료 시 결정한 corner_esc_cmd를 반복 전송.
        # CORNER_ESC_CYC 사이클 완료 후 NORMAL로 복귀.
        elif corner_state == 2:
            ser_Ardu.write(corner_esc_cmd)
            corner_esc_left -= 1
            print(f"[ESC {corner_esc_left:02d}] {corner_esc_cmd.decode().strip()}")

            if corner_esc_left <= 0:
                corner_state = 0
                # [△ 중복 초기화] corner_det_n은 CORNER_BACK 진입 시 이미 0으로
                # 설정됨. 여기서의 초기화는 안전장치지만 기능상 중복.
                corner_det_n = 0
                print("[CORNER] 탈출 완료 → NORMAL 복귀")

        # ── 상태 0: NORMAL ────────────────────────────────────
        # 일반 주행 로직. 우선순위 순서:
        #   1) 구석 N회 연속 감지 → CORNER_BACK 진입
        #   2) EXTENDED_BACK 잔여 사이클 소화
        #   3) 긴급 후진 (EMERGENCY 이내)
        #   4) 전방 장애물 회피
        #   5) 좌측 장애물 회피
        #   6) 우측 장애물 회피
        #   7) 장애물 없음 → 직진
        else:
            # ── 구석 연속 감지 카운터 ─────────────────────────
            # 오감지 방지: CORNER_DET_N회 연속 is_corner=True 시 구석 확정
            if is_corner:
                corner_det_n += 1
                print(f"[CORNER?] {corner_det_n}/{CORNER_DET_N}  "
                      f"F:{front_min:.0f} L:{left_min:.0f} R:{right_min:.0f}mm")
            else:
                corner_det_n = 0   # 한 번이라도 False면 카운터 리셋

            # ── CORNER_BACK 진입 ─────────────────────────────
            if corner_det_n >= CORNER_DET_N:
                corner_state     = 1
                corner_back_left = CORNER_BACK_CYC
                corner_det_n     = 0   # 상태 전환 시 카운터 리셋
                print(f"[CORNER] 구석 확정! CORNER_BACK 진입 "
                      f"({CORNER_BACK_CYC}사이클 후진 + 섹터 분석)")

            # ── [△ 주의] extra_back 처리 우선순위 ────────────
            # corner_det_n >= CORNER_DET_N 조건이 먼저 체크되므로,
            # 구석 확정과 extra_back이 동시에 발생하면 구석 탈출이 우선됨.
            # CORNER_BACK/CORNER_ESC 상태에서는 extra_back이 체크되지 않으므로,
            # 해당 상태 진입 후 extra_back > 0이 남아있으면 NORMAL 복귀 후 소화됨.
            elif extra_back > 0:
                ser_Ardu.write(b"B 0.80\n")
                extra_back -= 1
                print(f"EXTENDED_BACK 잔여 {extra_back}사이클")

            # ── 긴급 후진 ────────────────────────────────────
            # EMERGENCY(140mm) 이내 장애물 → 즉시 후진
            # back_cnt 6회 이상 → EXTENDED_BACK(extra_back=3사이클) 돌입
            elif (front_min <= EMERGENCY or
                  left_min  <= EMERGENCY or
                  right_min <= EMERGENCY):
                back_cnt += 1
                ser_Ardu.write(b"B 0.90\n")
                if back_cnt >= 6:
                    extra_back = 3; back_cnt = 0
                    print("EXTENDED_BACK 시작! (3x)")
                else:
                    print(f"EMERGENCY! F:{front_min:.0f} L:{left_min:.0f} "
                          f"R:{right_min:.0f}mm ({back_cnt}/6)")

            # ── 전방 장애물 회피 ─────────────────────────────
            # DETECT(350mm) 이내 전방 장애물 감지 시
            # ratio: 0=350mm(멀다) → 1=140mm(EMERGENCY 바로 앞)
            # steer 방향: left_min > right_min → 오른쪽(음수), 아니면 왼쪽(양수)
            elif front_min <= DETECT:
                ratio = (DETECT - front_min) / (DETECT - EMERGENCY)
                speed = 0.70 * (1 - ratio * 0.7)   # 가까울수록 감속
                steer = -(ratio * 0.85) if left_min > right_min else (ratio * 0.85)
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"F_OBS {front_min:.0f}mm → {'R' if steer<0 else 'L'} "
                      f"steer={steer:.2f} spd={speed:.2f}")

            # ── 좌측 장애물 회피 → 우회전 ────────────────────
            # steer 양수(+) = 좌회전 / steer가 ratio*0.75 → 오른쪽으로 밀어냄
            elif left_min <= DETECT:
                ratio = (DETECT - left_min) / (DETECT - EMERGENCY)
                ser_Ardu.write(
                    f"F {ratio*0.75:.2f} {0.70*(1-ratio*0.6):.2f}\n".encode())
                print(f"L_OBS {left_min:.0f}mm")

            # ── 우측 장애물 회피 → 좌회전 ────────────────────
            # steer 음수(-) = 우회전 / -ratio*0.75 → 왼쪽으로 밀어냄
            elif right_min <= DETECT:
                ratio = (DETECT - right_min) / (DETECT - EMERGENCY)
                ser_Ardu.write(
                    f"F {-ratio*0.75:.2f} {0.70*(1-ratio*0.6):.2f}\n".encode())
                print(f"R_OBS {right_min:.0f}mm")

            # ── 장애물 없음 → 직진 ───────────────────────────
            else:
                ser_Ardu.write(b"F 0.00 0.70\n")

        # ── 버퍼 초기화 (매 사이클 말) ───────────────────────
        # 다음 회전 데이터를 위해 누적값 전부 리셋
        scan_buf  = []
        front_min = 9999.0;  front_cnt = 0
        left_min  = 9999.0;  left_cnt  = 0
        right_min = 9999.0;  right_cnt = 0
        sec_min   = [9999.0] * N_SEC
        sec_cnt   = [0]      * N_SEC
