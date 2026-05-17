import serial
import time
import atexit
from typing import Optional

# 포트 및 시리얼 설정
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L     = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu  = serial.Serial(port_Ardu, 460800, timeout=1)

# LiDAR 구동 명령
ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

# --- [VFH 설정 파라미터] ---
NUM_SECTORS       = 72
SECTOR_SIZE       = 360.0 / NUM_SECTORS
MAX_DETECT_DIST   = 450.0
VALLEY_THRESHOLD  = 180.0
MAX_FORWARD_ANGLE = 90.0
MAX_BUF_SIZE      = 400
FWD_OPEN_DIST     = MAX_DETECT_DIST - VALLEY_THRESHOLD  # 270mm
FWD_CONE_DEG      = 35.0

# --- [데드락 탈출 설정] ---
DEADLOCK_LIMIT   = 3
ESCAPE_REVERSE_T = 0.6
ESCAPE_TURN_T    = 0.5

# 상태 변수
scan_buf       = []
deadlock_count = 0
last_steer: Optional[float] = None  # None = 아직 정상 주행 이력 없음

def _cleanup():
    try:
        ser_Ardu.write(b"S\n")
        ser_L.write(bytes([0xA5, 0x25]))
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
        print("\n[시스템 종료] 하드웨어 안전 정지 완료.")
    except Exception:
        pass

atexit.register(_cleanup)

def escape_maneuver():
    """
    데드락 탈출 기동: 직진 후진 → 조향 후진
    - last_steer가 None(최초 데드락)이면 우회전으로 탈출
    - last_steer가 있으면 마지막 조향의 반대 방향으로 탈출
    """
    if last_steer is None:
        escape_dir = 1.0   # 주행 이력 없음 → 임의로 우회전
    else:
        escape_dir = -1.0 if last_steer >= 0 else 1.0

    side = '우' if escape_dir > 0 else '좌'
    print(f"[탈출 기동] 직진 후진 {ESCAPE_REVERSE_T}s → {side}회전 후진 {ESCAPE_TURN_T}s")

    ser_Ardu.write(b"B 0.55\n")
    time.sleep(ESCAPE_REVERSE_T)

    ser_Ardu.write(f"B {escape_dir:.2f} 0.50\n".encode())
    time.sleep(ESCAPE_TURN_T)

    ser_Ardu.write(b"S\n")
    time.sleep(0.1)

print("=" * 60)
print("  VFH(Vector Field Histogram) 자율주행 엔진 v1.4")
print(f"  - 섹터 수: {NUM_SECTORS}개 (해상도: {SECTOR_SIZE}도)")
print(f"  - 감지 반경: {MAX_DETECT_DIST}mm | 빈길 기준치: {VALLEY_THRESHOLD}")
print(f"  - 전방 개방 기준: {FWD_OPEN_DIST}mm | 데드락 탈출: {DEADLOCK_LIMIT}회")
print("=" * 60)

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

    if distance < 100 or quality == 0:
        continue

    scan_buf.append((angle, distance))

    if len(scan_buf) > MAX_BUF_SIZE:
        print("[경고] 버퍼 상한 초과 - 비정상 스캔, 강제 리셋")
        scan_buf = []
        continue

    if s_flag == 1 and len(scan_buf) > 30:

        # 1. 히스토그램 생성
        hist            = [0.0] * NUM_SECTORS
        min_d_in_sector = [float('inf')] * NUM_SECTORS

        for a, d in scan_buf:
            if d <= MAX_DETECT_DIST:
                idx = int(a / SECTOR_SIZE) % NUM_SECTORS
                if d < min_d_in_sector[idx]:
                    min_d_in_sector[idx] = d

        for i in range(NUM_SECTORS):
            if min_d_in_sector[i] < float('inf'):
                hist[i] = MAX_DETECT_DIST - min_d_in_sector[i]

        # 2. Dilation 평활화 (±1 섹터)
        smoothed_hist = [0.0] * NUM_SECTORS
        for i in range(NUM_SECTORS):
            smoothed_hist[i] = max(
                hist[(i - 1) % NUM_SECTORS],
                hist[i],
                hist[(i + 1) % NUM_SECTORS],
            )

        # 3. Valley 필터링
        free_sectors = [i for i, val in enumerate(smoothed_hist) if val < VALLEY_THRESHOLD]

        # 4. 최적 섹터 탐색
        best_sector = None
        min_delta   = 9999

        for sector in free_sectors:
            delta = min(sector, NUM_SECTORS - sector)
            if delta > (MAX_FORWARD_ANGLE / SECTOR_SIZE):
                continue
            if delta < min_delta:
                min_delta   = delta
                best_sector = sector

        # 5. 제어 명령
        if best_sector is not None:
            deadlock_count = 0

            target_angle = best_sector * SECTOR_SIZE
            if target_angle > 180.0:
                target_angle -= 360.0

            steer      = (target_angle / 90.0) * 0.85
            steer      = max(-1.0, min(1.0, steer))
            last_steer = steer

            speed = 0.65 * (1.0 - abs(steer) * 0.55)

            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"[VFH 주행] 각도: {target_angle:+.1f}° | 조향: {steer:+.2f} | 속도: {speed:.2f}")

        else:
            fwd_half  = int(FWD_CONE_DEG / SECTOR_SIZE)
            fwd_vals  = [
                min_d_in_sector[k % NUM_SECTORS]
                for k in range(-fwd_half, fwd_half + 1)
                if min_d_in_sector[k % NUM_SECTORS] < float('inf')
            ]
            fwd_min_d = min(fwd_vals) if fwd_vals else float('inf')

            if fwd_min_d > FWD_OPEN_DIST:
                deadlock_count = 0
                ser_Ardu.write(b"F 0.00 0.25\n")
                print(f"[오판 방지] 전방 실측 {fwd_min_d:.0f}mm 여유 → 저속 직진")
            else:
                deadlock_count += 1
                print(f"[VFH 데드락] 전방 실측 {fwd_min_d:.0f}mm 폐쇄 ({deadlock_count}/{DEADLOCK_LIMIT}회)")

                if deadlock_count >= DEADLOCK_LIMIT:
                    escape_maneuver()
                    deadlock_count = 0
                else:
                    ser_Ardu.write(b"B 0.60\n")

        scan_buf = []
