import serial
import time
import atexit

# 포트 및 시리얼 설정
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L     = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu  = serial.Serial(port_Ardu, 460800, timeout=1)

# LiDAR 구동 명령
ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

# --- [VFH 초정밀 설정 파라미터] ---
NUM_SECTORS      = 72                    # 5도 단위 (360 / 72 = 5)
SECTOR_SIZE      = 360.0 / NUM_SECTORS
MAX_DETECT_DIST  = 450.0                 # 550 → 450: 서킷 폭 1100mm 절반 기준, 맞은편 벽 상시 감지 방지
VALLEY_THRESHOLD = 180.0                 # 80 → 180: 최단거리 270mm 이상인 섹터를 빈 공간으로 인정 (기존 80은 470mm 이상만 허용해 데드락 빈발)
MAX_FORWARD_ANGLE = 75.0                 # 100 → 75: 전방 탐색 범위 축소, 급격한 방향 전환 억제

# 데이터 저장용 버퍼 및 변수 초기화
scan_buf = []

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

print("=" * 60)
print("  VFH(Vector Field Histogram) 자율주행 엔진 v1.1")
print(f"  - 섹터 수: {NUM_SECTORS}개 (해상도: {SECTOR_SIZE}도)")
print(f"  - 감지 반경: {MAX_DETECT_DIST}mm | 빈길 기준치: {VALLEY_THRESHOLD}")
print("  - 목적지: 전방 정면(0도) 방향 최우선 탐색")
print("=" * 60)

while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    # LiDAR 데이터 유효성 검사
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        continue
    if (data[1] & 0x01) != 1:
        continue

    quality  = data[0] >> 2
    angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0

    if distance < 100 or quality == 0:   # 50 → 100: 근거리 LiDAR 노이즈 제거 범위 확대
        continue

    # 스캔 데이터 버퍼 생성
    scan_buf.append((angle, distance))

    # LiDAR가 한 바퀴(s_flag == 1) 돌았고 데이터가 충분히 쌓였을 때 VFH 연산 가동
    if s_flag == 1 and len(scan_buf) > 60:   # 15 → 60: 포인트 부족 시 다수 섹터 공백 발생 방지


        # 1. 장애물 밀도 히스토그램 생성 (최근접 거리 매핑)
        hist = [0.0] * NUM_SECTORS
        min_d_in_sector = [9999.0] * NUM_SECTORS

        for a, d in scan_buf:
            if d <= MAX_DETECT_DIST:
                idx = int(a / SECTOR_SIZE) % NUM_SECTORS
                if d < min_d_in_sector[idx]:
                    min_d_in_sector[idx] = d

        # 최단 거리를 기반으로 밀도 환산
        for i in range(NUM_SECTORS):
            if min_d_in_sector[i] < 9999.0:
                hist[i] = MAX_DETECT_DIST - min_d_in_sector[i]


        # 2. 히스토그램 평활화 (Dilation 필터)
        # ±2 → ±1 섹터로 축소: 기존 25° 팽창은 좁은 통로에서 모든 섹터를 막아 데드락 유발
        smoothed_hist = [0.0] * NUM_SECTORS
        for i in range(NUM_SECTORS):
            v = max([
                hist[(i-1) % NUM_SECTORS],
                hist[i],
                hist[(i+1) % NUM_SECTORS],
            ])
            smoothed_hist[i] = v

        # 3. 주행 가능한 빈 공간(Valley) 섹터 필터링
        free_sectors = [i for i, val in enumerate(smoothed_hist) if val < VALLEY_THRESHOLD]

        # 4. 목적지(정면 = 0도 = 인덱스 0)와 가장 가까운 최적 탈출 섹터 찾기
        best_sector = None
        min_delta = 9999

        for sector in free_sectors:
            delta = min(sector, NUM_SECTORS - sector)

            if delta > (MAX_FORWARD_ANGLE / SECTOR_SIZE):
                continue

            if delta < min_delta:
                min_delta = delta
                best_sector = sector

        # 5. 아두이노 차량 제어 명령 전송
        if best_sector is not None:
            target_angle = best_sector * SECTOR_SIZE
            if target_angle > 180.0:
                target_angle -= 360.0

            steer = (target_angle / 90.0) * 0.85
            steer = max(-1.0, min(1.0, steer))

            speed = 0.65 * (1.0 - abs(steer) * 0.55)

            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"[VFH 주행] 각도: {target_angle:+.1f}° | 조향: {steer:+.2f} | 속도: {speed:.2f}")
        else:
            ser_Ardu.write(b"B 0.60\n")
            print("[VFH 데드락] 전방 완전 폐쇄! 비상 후진 진행")

        # 다음 사이클을 위한 버퍼 초기화
        scan_buf = []
