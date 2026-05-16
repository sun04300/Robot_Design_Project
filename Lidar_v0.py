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

# --- [VFH 초정밀 설정 패라미터] ---
NUM_SECTORS      = 72                   # 5도 단위로 촘촘하게 분할 (360 / 72 = 5)
SECTOR_SIZE      = 360.0 / NUM_SECTORS
MAX_DETECT_DIST  = 1200.0               # 1.1m x 3.1m 서킷 맞춤형 (1.2m 이내만 집중 감지)
VALLEY_THRESHOLD = 150.0                # 빈 길(계곡)로 인정할 최대 장애물 밀도 커트라인

# 데이터 저장용 버퍼 및 변수 초기화
scan_buf = []
sector_sum = [0.0] * NUM_SECTORS

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
print("  VFH(Vector Field Histogram) 자율주행 엔진 v1.0")
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
    
    if distance < 50 or quality == 0:
        continue

    # 스캔 데이터 버퍼 생성
    scan_buf.append((angle, distance))

    # LiDAR가 한 바퀴(s_flag == 1) 돌았고 데이터가 충분히 쌓색됐을 때 VFH 연산 가동
    if s_flag == 1 and len(scan_buf) > 15:
        
        # 1. 장애물 밀도 히스토그램(Polar Histogram) 생성
        hist = [0.0] * NUM_SECTORS
        for a, d in scan_buf:
            if d <= MAX_DETECT_DIST:
                idx = int(a / SECTOR_SIZE) % NUM_SECTORS
                # 가깝고 확실한 장애물일수록 가중치(밀도 점수)를 높게 부여
                density = MAX_DETECT_DIST - d
                hist[idx] += density

        # 2. 히스토그램 평활화 (Smoothing)
        # 로봇의 물리적 부피를 감안하여, 장애물 감지 점수를 좌우 2칸(총 25도 범위)씩 흐리게 펴줌
        smoothed_hist = [0.0] * NUM_SECTORS
        for i in range(NUM_SECTORS):
            v = (hist[(i-2)%NUM_SECTORS] + hist[(i-1)%NUM_SECTORS] + 
                 hist[i] + hist[(i+1)%NUM_SECTORS] + hist[(i+2)%NUM_SECTORS]) / 5.0
            smoothed_hist[i] = v

        # 3. 주행 가능한 빈 공간(Valley) 섹터 필터링
        # 안전 점수가 커트라인(VALLEY_THRESHOLD)보다 낮은 깨끗한 섹터들의 인덱스만 추출
        free_sectors = [i for i, val in enumerate(smoothed_hist) if val < VALLEY_THRESHOLD]

        # 4. 목적지(정면 = 0도 = 인덱스 0)와 가장 가까운 최적 탈출 섹터 찾기
        best_sector = None
        min_delta = 9999

        for sector in free_sectors:
            # 0도(인덱스 0 또는 NUM_SECTORS)와의 최소 섹터 차이 계산
            delta = min(sector, NUM_SECTORS - sector)
            
            # 주행 방향 기준 뒤쪽 역주행 방향(좌우 90도 이상 꺾어야 하는 곳)은 전진 목적에 맞지 않으므로 차단
            if delta > (90.0 / SECTOR_SIZE): 
                continue
                
            if delta < min_delta:
                min_delta = delta
                best_sector = sector

        # 5. 아두이노 차량 제어 명령 전송
        if best_sector is not None:
            # 최적 섹터를 실제 조향 각도로 환산 (-180도 ~ +180도 스케일링)
            target_angle = best_sector * SECTOR_SIZE
            if target_angle > 180.0:
                target_angle -= 360.0 # 좌회전 (+), 우회전 (-)
            
            # 조향값(Steer) 매핑 (최대 회전 각도 45도를 제어 가중치 1.0/-1.0으로 바운딩)
            steer = target_angle / 45.0
            steer = max(-1.0, min(1.0, steer))
            
            # 사잇길을 빠져나가기 위해 조향을 크게 틀 때는 속도를 줄이고, 정면이 열리면 고속 질주
            speed = 0.70 * (1.0 - abs(steer) * 0.4)
            
            ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
            print(f"[VFH 주행] 각도: {target_angle:+.1f}° | 조향: {steer:+.2f} | 속도: {speed:.2f}")
        else:
            # 좌/우/정면 모든 전방 통로가 박스로 빽빽하게 완전히 막힌 극단적인 데드락 상황
            ser_Ardu.write(b"B 0.60\n")
            print("[VFH 데드락] 전방 완전 폐쇄! 비상 후진 진행")

        # 다음 사이클을 위한 버퍼 초기화
        scan_buf = []
