import serial
import time
import math
import atexit
from typing import Optional

# ── 1. 하드웨어 및 포트 설정 ───────────────────────────────────────
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

# LiDAR 구동 명령 (RPLIDAR 고속 모드 활성화)
ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

# ── 2. VFH 및 차량 설정 파라미터 ───────────────────────────────────
BIN_DEG          = 5.0                # 히스토그램 빈 해상도 (도)
N_BINS           = int(360 / BIN_DEG) # 72개 빈
ROBOT_WIDTH      = 140.0              # 차량 자체 실제 폭 (mm)
GAP_MARGIN       = 10.0               # 통과 안전 마진 (mm)
GAP_MIN_PASS     = ROBOT_WIDTH + GAP_MARGIN # 최소 통과 가능 폭 (150mm)

DETECT           = 500.0              # 장애물 감지 거리 (mm)
EMERGENCY        = 150.0              # 감속 및 선회 전환 기준 거리 (mm)
P4_DIST          = 200.0              # 측후방 갭 선환 기준 장애물 근접 거리 (mm)
MAX_STEER        = 0.85               # 최대 조향비 (0.0 ~ 1.0)
ROT_THRESH       = 110.0              # 이 각도 초과 시 제자리 회전 사용 (도)
FRONT_ARC        = 60.0               # 좁은길 개척 시 탐색할 전방 원호 반경 (도)

# 주행 제어 변수
scan_buf = []

# ── 3. 시스템 종료 및 안전 관리 ────────────────────────────────────
def _cleanup():
    try:
        ser_Ardu.write(b"S\n")           # 아두이노 모터 정지
        ser_L.write(bytes([0xA5, 0x25])) # LiDAR 레이저 정지
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
        print("\n[시스템 종료] 하드웨어 안전 정지 완료.")
    except Exception:
        pass

atexit.register(_cleanup)

# ── 4. VFH 핵심 헬퍼 함수 ──────────────────────────────────────────
def build_polar_hist(scan_data):
    """360도 극좌표 히스토그램 생성 (각 방향의 최근접 장애물 거리 매핑)"""
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_data:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt

def find_vfh_gaps(hist, has_pt):
    """
    히스토그램에서 단일 빈 노이즈를 제거하고,
    양쪽 장애물 엣지 사이의 실제 물리 폭(mm)을 계산하여 통과 가능한 갭 탐색
    """
    blocked = [has_pt[i] and hist[i] <= DETECT for i in range(N_BINS)]

    # [장점 통합] 단일 노이즈 빈 필터링 (허상 장애물 제거)
    smoothed = blocked[:]
    for i in range(N_BINS):
        if blocked[i] and not blocked[(i - 1) % N_BINS] and not blocked[(i + 1) % N_BINS]:
            smoothed[i] = False
    blocked = smoothed

    gaps = []
    seen = set()
    i = 0
    while i < 2 * N_BINS:
        bi = i % N_BINS
        if not blocked[bi]:
            j = i + 1
            while j < i + N_BINS and not blocked[j % N_BINS]:
                j += 1
            span = j - i

            # 완전히 개방된 평지는 루프 바깥의 예외 처리(F 0.0 0.70)로 유도하기 위해 제외
            if span < N_BINS:
                center_cw = ((i + j) / 2.0 * BIN_DEG) % 360.0
                ck = round(center_cw)
                if ck not in seen:
                    seen.add(ck)
                    delta_deg = span * BIN_DEG

                    # 갭 경계 장애물의 실제 거리 추출
                    d_L = min(hist[(i - 1) % N_BINS] if has_pt[(i - 1) % N_BINS] else DETECT, DETECT)
                    d_R = min(hist[j % N_BINS]        if has_pt[j % N_BINS]        else DETECT, DETECT)

                    # [장점 통합] 정밀 현(Chord) 계산식 적용 (실제 너비 산출)
                    gap_w = (d_L + d_R) * math.sin(math.radians(delta_deg / 2.0))

                    # 부호 있는 각도로 변환 (+: 우측 조향, -: 좌측 조향)
                    center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0

                    gaps.append({
                        'center'   : center_s,
                        'center_cw': center_cw,
                        'width'    : gap_w,
                        'passable' : gap_w >= GAP_MIN_PASS,
                        'delta_deg': delta_deg,
                        'd_L'      : d_L,
                        'd_R'      : d_R,
                    })
            i = j
        else:
            i += 1
    return gaps

def select_best_gap(gaps):
    """안전 통과폭을 만족하는 갭 중, 넓고 전방(0도)에 가까운 최적 경로 선정"""
    if not gaps:
        return None
    passable = [g for g in gaps if g['passable']]
    pool     = passable if passable else gaps
    return max(pool, key=lambda g: g['width'] * 0.25 - abs(g['center']) * 1.9)

def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    """특정 각도 원호 범위 내의 최근접 장애물 거리 확인"""
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d

# ── 5. 메인 제어 루프 ────────────────────────────────────────────────
print("=" * 65)
print("  통합 VFH+ 고속 자율주행 엔진 v2.5 (No-Reverse / Continuous)")
print(f"  - 감지 반경: {DETECT}mm | 긴급선회: {ROT_THRESH}° | 탐색원호: ±{FRONT_ARC}°")
print(f"  - 최소요구 통과폭: {GAP_MIN_PASS}mm (차폭 {ROBOT_WIDTH}mm + 마진 {GAP_MARGIN}mm)")
print("=" * 65)

while True:
    # 💡 [버퍼 초과 차단] 시리얼 포트에 쌓인 바이트 수 파악
    waiting_bytes = ser_L.in_waiting
    if waiting_bytes < 5:
        time.sleep(0.001)
        continue

    # 밀려 있는 패킷 데이터를 대량으로 수집하여 가공 지연 현상 제거
    packets_to_read = min(waiting_bytes // 5, 80)
    
    for _ in range(packets_to_read):
        data = ser_L.read(5)
        if len(data) != 5:
            continue

        # RPLIDAR 데이터 프레임 무결성 검증
        s_flag     = data[0] & 0x01
        s_inv_flag = (data[0] & 0x02) >> 1
        if s_inv_flag != (1 - s_flag):
            continue
        if (data[1] & 0x01) != 1:
            continue

        quality  = data[0] >> 2
        angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
        distance = (data[3] | (data[4] << 8)) / 4.0

        if distance < 80 or quality == 0:
            continue

        scan_buf.append((angle, distance))

        # 360도 1회전 스캔 데이터가 완비된 시점
        if s_flag == 1 and len(scan_buf) > 30:
            hist, has_pt = build_polar_hist(scan_buf)
            
            # 전방 시야 장애물 유무 체크
            emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=70)

            # ── [상태 1] 전방에 아무것도 없는 완전 개방 지대 -> 고속 직진
            if not any(has_pt):
                ser_Ardu.write(b"F 0.00 0.70\n")
            
            else:
                gaps = find_vfh_gaps(hist, has_pt)
                best = select_best_gap(gaps)

                # ── [상태 2] 주행 가능한 안전 갭이 존재하는 경우
                if best is not None and best['passable']:
                    
                    # ── (선택 A) 최적 경로가 조향각 범위 내인 경우 -> 동적 감속 주행
                    if abs(best['center']) <= ROT_THRESH:
                        # [장점 통합] 양쪽 장애물 거리 불균형에 따른 조향 타깃 보정 (Bias)
                        d_L, d_R  = best['d_L'], best['d_R']
                        imbalance = (d_R - d_L) / (d_L + d_R + 1e-9)
                        bias      = imbalance * (best['delta_deg'] / 3.5)
                        target    = best['center'] + bias
                        
                        steer = max(-MAX_STEER, min(MAX_STEER, target / 90.0 * MAX_STEER))
                        
                        # 갭 중심 기준 주행 차선 내부 장애물 최단 거리 확인
                        near_d = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=30)
                        
                        # 선형 감속비 계산 (EMERGENCY 거리 근접 시 점진적 감속)
                        ratio  = min(max((DETECT - near_d) / (DETECT - EMERGENCY + 5), 0.0), 1.0)
                        speed  = 0.65 * (1.0 - ratio * 0.55)
                        
                        ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                        print(f"[VFH 주행] 갭: {best['width']:.0f}mm @ {best['center']:+.0f}° | 편향: {bias:+.1f}° | 조향: {steer:+.2f} | 속도: {speed:.2f}")
                    
                    # ── (선택 B) 최적 경로가 후방에 존재하며, 장애물이 가까운 경우 -> 제자리 선회(Turn)
                    elif emg_near <= P4_DIST:
                        rot_dir = 1.0 if best['center'] > 0 else -1.0
                        ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
                        print(f"[VFH 선회] 갭 후방 위치({best['center']:+.0f}°) & 근접장애물({emg_near:.0f}mm) -> 제자리 회전")
                    
                    # ── (선택 C) 후방에 갭이 있지만 긴급 거리가 아니라면 저속 조향 복귀 탐색
                    else:
                        steer = max(-MAX_STEER, min(MAX_STEER, best['center'] / 90.0 * MAX_STEER * 0.4))
                        ser_Ardu.write(f"F {steer:.2f} 0.25\n".encode())

                # ── [상태 3] 차 폭보다 넓은 갭이 없는 상황 -> [후진 제거 차선책] 좁은길 탐색 개척
                else:
                    if gaps:
                        # 전방 원호(±FRONT_ARC) 내부의 갭들만 필터링
                        front_gaps = [g for g in gaps if abs(g['center']) <= FRONT_ARC]
                        
                        if front_gaps:
                            # 전방 시야 내에서 그나마 제일 탈출 가능성 높은(가장 넓은) 갭 선택
                            open_g     = max(front_gaps, key=lambda g: g['width'])
                            target_dir = open_g['center']
                        else:
                            # 전방에 갭이 아예 없다면 전체 영역의 최대 갭 방향을 바라보도록 꺾기 제한
                            open_g     = max(gaps, key=lambda g: g['width'])
                            target_dir = max(-FRONT_ARC, min(FRONT_ARC, open_g['center']))
                        widest = open_g['width']
                    else:
                        target_dir = 0.0
                        widest     = 0.0
                    
                    # 후진을 박아넣는 대신, 가장 유망한 샛길 방향으로 조향을 틀고 초저속 기어 주행
                    steer = max(-MAX_STEER, min(MAX_STEER, target_dir / 90.0 * MAX_STEER * 0.5))
                    ser_Ardu.write(f"F {steer:.2f} 0.20\n".encode())
                    print(f"[좁은길 개척] 통과 가능 갭 없음 -> 최선책 방향 {target_dir:+.0f}° 조향 탐색 (최대폭: {widest:.0f}mm)")

            # 주행 판단 연산이 끝났으므로 버퍼를 비우고 배치 루프 조기 탈출
            scan_buf = []
            break
