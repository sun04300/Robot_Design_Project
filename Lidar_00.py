import serial
import time
import math
import atexit

# 포트 설정
port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

ser_L    = serial.Serial(port_L,    460800, timeout=1)
ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)

# RPLIDAR 구동 신호
ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

# ── 전역 파라미터 (1.1m x 3.1m 서킷 최적화) ───────────────────────
BIN_DEG      = 5.0                 # 히스토그램 빈 해상도 (도)
N_BINS       = int(360 / BIN_DEG)  # 72개 빈
ROBOT_WIDTH  = 180.0               # 차량 폭 (mm)
GAP_MARGIN   = 40.0                # 실전 주행 안전 마진 (박스 충돌 방지용 확대)
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN   # 최종 최소 통과 폭 (220mm)

DETECT       = 650.0               # 감지 거리 (mm) - 박스 사잇길을 미리 인지하도록 소폭 확대
EMERGENCY    = 140.0               # 즉시 대응 거리 (mm)
MAX_STEER    = 0.85                # 최대 조향 가중치
ROT_THRESH   = 75.0                # 이 각도 초과 시 제자리 회전 모드 진입 (도)

# ── VFH 핵심 연산 함수군 ───────────────────────────────────────────

def build_polar_hist(scan_buf):
    """ 각 빈에 해당 방향의 최근접 장애물 거리를 마킹 """
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt


def find_vfh_gaps(hist, has_pt, detect_dist, min_pass_mm):
    """ 2배 언롤링 배열을 이용해 0-360도 경계를 허물고 물리적 갭 계산 """
    blocked = [has_pt[i] and hist[i] <= detect_dist for i in range(N_BINS)]
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

            if span < N_BINS:
                center_cw = ((i + j) / 2.0 * BIN_DEG) % 360.0
                ck = round(center_cw)
                if ck not in seen:
                    seen.add(ck)
                    delta_deg = span * BIN_DEG

                    # 갭 양 끝단의 장애물 거리 산출
                    d_L = hist[(i - 1) % N_BINS] if has_pt[(i - 1) % N_BINS] else detect_dist
                    d_R = hist[j % N_BINS]        if has_pt[j % N_BINS]        else detect_dist
                    d_L = min(d_L, detect_dist)
                    d_R = min(d_R, detect_dist)

                    # [좋은 부분] 현(Chord) 공식을 이용한 정밀한 물리 폭 추정
                    # $$gap\_w = 2 \cdot \min(d_L, d_R) \cdot \sin\left(\frac{\Delta\theta}{2}\right)$$
                    d_ref = min(d_L, d_R)
                    gap_w = 2.0 * d_ref * math.sin(math.radians(delta_deg / 2.0))

                    # 0~360도 좌표계를 부호 있는 좌표계(-180 ~ +180)로 변환
                    center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0

                    gaps.append({
                        'center'   : center_s,
                        'center_cw': center_cw,
                        'width'    : gap_w,
                        'passable' : gap_w >= min_pass_mm,
                        'delta_deg': delta_deg,
                    })
            i = j
        else:
            i += 1
    return gaps


def select_best_gap(gaps):
    """ 통과 가능 갭 중 정면(0도) 선호 가중치 부여하여 최적 선택 """
    if not gaps:
        return None
    passable = [g for g in gaps if g['passable']]
    pool     = passable if passable else gaps
    # 전방 각도 패널티를 기존 1.5에서 1.2로 미세 조정하여 사잇길 유도 밸런스 매칭
    return max(pool, key=lambda g: g['width'] * 0.3 - abs(g['center']) * 1.2)


def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    """ 특정 조향각 주행 경로상의 최단 장애물 거리 계산 (감속용) """
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d


# ── 메인 주행 루프 ──────────────────────────────────────────────────
while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    # 패킷 유효성 1차 검사
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        continue
    if (data[1] & 0x01) != 1:
        continue

    quality  = data[0] >> 2
    angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0
    if distance < 70:
        continue

    # 시스템 초기화 및 자원 정리 선언
    try:
        _ready
    except NameError:
        _ready     = True
        scan_buf   = []
        emg_cnt    = 0   
        extra_back = 0

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

        print("=" * 65)
        print("  하이브리드 VFH 자율주행 엔진 가동 (Dynamic Gap 계산 빌드)")
        print(f"  - 센싱 반경: {int(DETECT)}mm | 비상정지 락다운: {int(EMERGENCY)}mm")
        print(f"  - 합격 갭 폭 제한: {int(GAP_MIN_PASS)}mm (안전마진 포함)")
        print("=" * 65)

    if quality == 0:
        continue

    scan_buf.append((angle, distance))

    # ── 1회전 스캔 완료 시점에 데이터 처리 ──────────────────────────
    if s_flag == 1 and len(scan_buf) > 15:
        hist, has_pt = build_polar_hist(scan_buf)
        
        # 전방 메인 시야(+-70도 넓은 반구)의 긴급 장애물 모니터링
        emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=70)

        # 완전히 뚫린 평지 공간인 경우
        if not any(has_pt):
            ser_Ardu.write(b"F 0.00 0.70\n")
            emg_cnt = 0
        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps)

            # ── [P1] 이스케이프 후진 예약 사이클 수행 중 ─────────────────
            if extra_back > 0:
                ser_Ardu.write(b"B 0.75\n")
                extra_back -= 1
                print(f"EXTENDED_BACK 주행 중... 잔여 {extra_back}사이클")

            # ── [P2] 전방 돌발 상황 및 탈출 통로 부재 -> 비상 후진 ─────────
            elif (emg_near <= EMERGENCY and 
                  (best is None or not best['passable'] or abs(best['center']) > ROT_THRESH)):
                emg_cnt += 1
                if emg_cnt >= 5: # 연속 트리거 제한을 6에서 5로 줄여 신속 탈출하도록 보정
                    ser_Ardu.write(b"B 0.75\n")
                    extra_back = 3
                    emg_cnt = 0
                    print("!! DEADLOCK BREAK !! 대피공간 확보를 위한 딥 후진 시동")
                else:
                    ser_Ardu.write(b"B 0.75\n")
                    print(f"EMERGENCY LOCK! 정면 급근접={emg_near:.0f}mm ({emg_cnt}/5)")

            # ── [P3] VFH 조향 전진 (타겟 갭이 전방 가용 조향각 내에 존재) ────
            elif best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
                emg_cnt = 0
                # 목표 방향 조향값 정규화 매핑
                steer = max(-MAX_STEER, min(MAX_STEER, best['center'] / 90.0 * MAX_STEER))
                
                # 선택한 사잇길(Valley) 내부 장애물과의 거리를 정밀 체크하여 스마트 융합 감속
                near_d = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=20)
                ratio  = min(max((DETECT - near_d) / (DETECT - EMERGENCY), 0.0), 1.0)
                speed  = 0.70 * (1.0 - ratio * 0.50) # 통로가 좁아지면 속도를 줄이고 정면 통과 시 가속
                
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"VFH_FORWARD -> 타겟갭:{best['width']:.0f}mm @ {best['center']:+.0f}° | 조향:{steer:+.2f} | 속도:{speed:.2f}")

            # ── [P4] VFH 제자리 피벗 회전 (안전 통로가 측후방에 잡힌 경우) ──
            elif best is not None and best['passable']:
                emg_cnt = 0
                rot_dir = 1.0 if best['center'] > 0 else -1.0
                ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
                print(f"VFH_PIVOT_TURN -> 후방 사잇길 발견({best['center']:+.0f}°) | 제자리회전 방향: {rot_dir:+.0f}")

            # ── [P5] 통과 가능한 정상 갭이 없는 경우 -> 가장 널널한 틈새 조준 ──
            else:
                open_g = max(gaps, key=lambda g: g['width']) if gaps else None
                widest = open_g['width'] if open_g else 0.0
                
                # 전방 크리티컬 존까지 막혔다면 전진하지 않고 제자리에서 각을 찾도록 변환
                if emg_near < DETECT * 0.6:
                    ser_Ardu.write(b"B 0.60\n")
                    print(f"NO_PASSABLE_GAP -> 탈출 통로 폐쇄 및 전방 장애물 근접. 탈출 공간 생성 후진")
                else:
                    steer = max(-MAX_STEER, min(MAX_STEER, open_g['center'] / 90.0 * MAX_STEER * 0.5)) if open_g else 0.0
                    ser_Ardu.write(f"F {steer:.2f} 0.25\n".encode())
                    print(f"NUDGING -> 합격점 미달 통로 진입 유도 | 최대폭:{widest:.0f}mm | 조향:{steer:+.2f}")

        # [버그 수정 완료] 데이터 버퍼 비우기는 반드시 이 조건문 안쪽 최하단에서 이루어져야 함!
        scan_buf = []
