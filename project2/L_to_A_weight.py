import serial
import time
import math

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

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
    if distance < 80:
        continue

    try:
        _ready
    except NameError:
        import atexit
        _ready        = True
        scan_buf      = []
        EMERGENCY     = 140.0   # 즉시 후진 거리 (mm)
        DETECT        = 350.0   # 장애물 감지 거리 (mm)
        MAX_STEER     = 0.85    # 최대 조향값
        REAR_MIN      = 150.0   # 후방 제외 시작 각도 (°)
        REAR_MAX      = 210.0   # 후방 제외 끝 각도 (°)
        FRONT_DEAD    = 0.08    # 이 미만이면 정면 장애물로 판단 (sin 기준)
        WEIGHT_POWER  = 2.0     # 거리 가중치 지수 (1=선형, 2=제곱) ← 여기서 조정
        back_cnt      = 0
        extra_back    = 0

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
        print("  장애물 회피 [거리 가중치 기반] 시작  (Ctrl+C 종료)")
        print(f"  감지: {int(DETECT)}mm  /  긴급후진: {int(EMERGENCY)}mm")
        print(f"  후방제외: {int(REAR_MIN)}°~{int(REAR_MAX)}°  /  최대조향: {MAX_STEER}")
        print(f"  가중치 지수: {WEIGHT_POWER}  (1=선형, 2=제곱)")
        print("=" * 60)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    if s_flag == 1 and len(scan_buf) > 15:

        # 후방 제외 후 DETECT 이내 포인트 필터링
        near_pts = [
            (a, d) for a, d in scan_buf
            if d <= DETECT and not (REAR_MIN <= a <= REAR_MAX)
        ]

        # ── 우선순위 1: 확장 후진 진행 중 ───────────────────────────
        if extra_back > 0:
            ser_Ardu.write(b"B 0.90\n")
            extra_back -= 1
            print(f"EXTENDED_BACK 잔여={extra_back}사이클")

        # ── 우선순위 2: 장애물 없음 → 직진 ─────────────────────────
        elif not near_pts:
            ser_Ardu.write(b"F 0.00 0.70\n")
            back_cnt = 0

        # ── 우선순위 3: 긴급 후진 ────────────────────────────────────
        elif min(d for _, d in near_pts) <= EMERGENCY:
            back_cnt += 1
            near_angle, near_dist = min(near_pts, key=lambda x: x[1])
            if back_cnt >= 6:
                ser_Ardu.write(b"B 0.90\n")
                extra_back = 2
                back_cnt   = 0
                print("EXTENDED_BACK 시작! (3x)")
            else:
                ser_Ardu.write(b"B 0.90\n")
                print(f"EMERGENCY!  최근접={near_dist:.0f}mm@{near_angle:.0f}°  ({back_cnt}/6)")

        # ── 우선순위 4: 거리 가중치 기반 회피 ───────────────────────
        else:
            # ── 가중 합산으로 반발력 벡터 계산 ──────────────────────
            steer_sum = 0.0   # 가중 steer 누적
            w_sum     = 0.0   # 가중치 누적 (정규화용)
            min_dist  = min(d for _, d in near_pts)

            for a, d in near_pts:
                # 거리 가중치: 가까울수록 지수적으로 증가
                # WEIGHT_POWER=1 → 선형  /  WEIGHT_POWER=2 → 제곱(가까운 것이 훨씬 지배적)
                w = (DETECT - d) ** WEIGHT_POWER

                # 각도를 부호 있는 값으로 변환 (양수=오른쪽, 음수=왼쪽)
                a_s = a if a <= 180 else a - 360

                # sin 반발력: 장애물 반대 방향으로 밀어내는 성분
                push = -math.sin(math.radians(a_s))

                steer_sum += push * w
                w_sum     += w

            # 가중 평균 steer (-1 ~ +1 범위)
            steer_raw = steer_sum / w_sum

            # 속도: 가장 가까운 장애물 거리 기준
            ratio = (DETECT - min_dist) / (DETECT - EMERGENCY)
            ratio = min(max(ratio, 0.0), 1.0)

            # ── 정면 장애물 판단: 합산 sin≈0 이면 좌우 여유로 결정 ──
            if abs(steer_raw) < FRONT_DEAD:
                left_dists  = [d for a, d in near_pts if a > 180]
                right_dists = [d for a, d in near_pts if 0 < a <= 180]
                l_min = min(left_dists)  if left_dists  else DETECT
                r_min = min(right_dists) if right_dists else DETECT
                steer = -(ratio * MAX_STEER) if l_min > r_min else (ratio * MAX_STEER)
                print(f"FRONT(weighted)  최근접={min_dist:.0f}mm  "
                      f"L_min={l_min:.0f}  R_min={r_min:.0f}  → {'R' if steer<0 else 'L'}")
            else:
                steer = max(-MAX_STEER, min(MAX_STEER, steer_raw * MAX_STEER))
                near_a, near_d = min(near_pts, key=lambda x: x[1])
                print(f"WEIGHTED  최근접={near_d:.0f}mm@{near_a:.0f}°  "
                      f"pts={len(near_pts)}  raw={steer_raw:+.3f}  "
                      f"→ {'R' if steer<0 else 'L'}  steer={steer:.2f}")

            speed = 0.70 * (1 - ratio * 0.7)
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())

        # ── 사이클 말 초기화 ──────────────────────────────────────────
        scan_buf = []
