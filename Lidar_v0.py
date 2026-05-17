import serial
import time
import math
import atexit

port_L    = "/dev/ttyUSB0"
port_Ardu = "/dev/ttyS0"

# 통신 연결
try:
    ser_L    = serial.Serial(port_L,    460800, timeout=1)
    ser_Ardu = serial.Serial(port_Ardu, 460800, timeout=1)
except Exception as e:
    print(f"시리얼 연결 실패: {e}")
    exit(1)

# 라이다 모터 구동
ser_L.write(bytes([0xA5, 0x40]))
time.sleep(1)
ser_L.write(bytes([0xA5, 0x20]))

# ═══════════════════════════════════════════════════════════════════
#  VFH (Vector Field Histogram) 기반 장애물 회피 (No-Reverse & 안정성 극대화)
# ═══════════════════════════════════════════════════════════════════

# ── 파라미터 최적화 (실내 안정성 튜닝) ─────────────────────────
BIN_DEG      = 5.0                # 히스토그램 빈 해상도 (도)
N_BINS       = int(360 / BIN_DEG) # 72개 빈

ROBOT_WIDTH  = 140.0              # 차량 폭 (mm)
GAP_MARGIN   = 10.0               # 좌우 통과 안전 마진 (mm)
GAP_MIN_PASS = ROBOT_WIDTH + GAP_MARGIN  # 160.0mm (좁은 길 통과 능력 유지)

DETECT       = 500.0             # 감지 거리: 500(너무 멈춤)과 350(너무 늦음)의 최적 타협점
EMERGENCY    = 130.0              # 긴급 제어 거리
P4_DIST      = 180.0              # 제자리 회전(T) 트리거 거리
MAX_STEER    = 0.85               # 최대 조향값 limit
ROT_THRESH   = 110.0             # 갭이 이 각도를 넘어가면 제자리 회전으로 탈출 시도 (도)


# ── VFH 알고리즘 함수군 ──────────────────────────────────────────

def build_polar_hist(scan_buf):
    hist   = [9999.0] * N_BINS
    has_pt = [False]  * N_BINS
    for a, d in scan_buf:
        idx = int(a / BIN_DEG) % N_BINS
        if d < hist[idx]:
            hist[idx] = d
            has_pt[idx] = True
    return hist, has_pt

def find_vfh_gaps(hist, has_pt, detect_dist, min_pass_mm):
    blocked = [has_pt[i] and hist[i] <= detect_dist for i in range(N_BINS)]

    # 단일 빈 노이즈 제거 (양쪽 열려있으면 장애물 무시)
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

            if span < N_BINS: # 360도 전체 개방이 아닌 경우
                center_cw = ((i + j) / 2.0 * BIN_DEG) % 360.0
                ck = round(center_cw)
                if ck not in seen:
                    seen.add(ck)
                    delta_deg = span * BIN_DEG
                    
                    d_L = hist[(i - 1) % N_BINS] if has_pt[(i - 1) % N_BINS] else detect_dist
                    d_R = hist[j % N_BINS]        if has_pt[j % N_BINS]        else detect_dist
                    d_L = min(d_L, detect_dist)
                    d_R = min(d_R, detect_dist)

                    gap_w = (d_L + d_R) * math.sin(math.radians(delta_deg / 2.0))
                    center_s = center_cw if center_cw <= 180.0 else center_cw - 360.0

                    gaps.append({
                        'center'   : center_s,
                        'center_cw': center_cw,
                        'width'    : gap_w,
                        'passable' : gap_w >= min_pass_mm,
                        'delta_deg': delta_deg,
                        'd_L'      : d_L,
                        'd_R'      : d_R,
                    })
            i = j
        else:
            i += 1
    return gaps

def select_best_gap(gaps, min_pass_mm=GAP_MIN_PASS):
    if not gaps:
        return None
    passable = [g for g in gaps if g['width'] >= min_pass_mm]
    pool     = passable if passable else gaps
    # 전방 중심(0도)을 아주 강하게 선호하도록 가중치 부여
    return max(pool, key=lambda g: g['width'] * 0.25 - abs(g['center']) * 1.9)

def nearest_in_arc(hist, has_pt, center_cw, arc_half=25):
    center_bin = int(center_cw / BIN_DEG) % N_BINS
    n_check    = max(1, int(arc_half / BIN_DEG))
    min_d = 9999.0
    for k in range(-n_check, n_check + 1):
        idx = (center_bin + k) % N_BINS
        if has_pt[idx] and hist[idx] < min_d:
            min_d = hist[idx]
    return min_d

# ── 안전 종료 트리거 ─────────────────────────────────────────────
def cleanup():
    print("\n[시스템 종료] 하드웨어 안전 정지 중...")
    try:
        ser_Ardu.write(b"S\n")  # 로봇 정지
        ser_L.write(bytes([0xA5, 0x25])) # 라이다 모터 정지
        time.sleep(0.1)
        ser_L.close()
        ser_Ardu.close()
    except Exception:
        pass
atexit.register(cleanup)


print("=" * 65)
print("  🚀 VFH 자율 주행 통합본 (후진 배제 & 안정성 강화)")
print(f"  제원 감지: {DETECT}mm | 긴급거리: {EMERGENCY}mm | 통과폭: {GAP_MIN_PASS}mm")
print("=" * 65)

scan_buf = []

# ── 메인 제어 루프 ───────────────────────────────────────────────
while True:
    data = ser_L.read(5)
    if len(data) != 5:
        continue

    # ── RPLIDAR 패킷 유효성 검사 
    s_flag     = data[0] & 0x01
    s_inv_flag = (data[0] & 0x02) >> 1
    if s_inv_flag != (1 - s_flag):
        continue
    if (data[1] & 0x01) != 1:
        continue

    quality  = data[0] >> 2
    angle    = ((data[1] >> 1) | (data[2] << 7)) / 64.0
    distance = (data[3] | (data[4] << 8)) / 4.0
    
    # 🛡️ [하드웨어 필터]: 내 몸체 크기 이내의 반사파 및 품질 0 노이즈 무시
    if distance < ROBOT_WIDTH or quality == 0:
        continue

    scan_buf.append((angle, distance))

    # ── 1회전 스캔이 끝났을 때 1회만 계산 및 명령 하달 ─────────
    if s_flag == 1:
        if len(scan_buf) < 10:  # 노이즈성 텅 빈 스캔 방지
            scan_buf = []
            continue

        hist, has_pt = build_polar_hist(scan_buf)
        emg_near = nearest_in_arc(hist, has_pt, 0.0, arc_half=70) # 전방 140도 영역

        # 케이스 0: 완전한 개방 공간 (초기 구동 등)
        if not any(has_pt):
            ser_Ardu.write(b"F 0.00 0.70\n")
            print("CLEAR_OPEN  주변 장애물 없음 -> 전진")
        else:
            gaps = find_vfh_gaps(hist, has_pt, DETECT, GAP_MIN_PASS)
            best = select_best_gap(gaps, GAP_MIN_PASS)

            # 케이스 1: VFH 주행 (통과 가능한 갭이 전방 반경 이내에 있음)
            if best is not None and best['passable'] and abs(best['center']) <= ROT_THRESH:
                d_L, d_R  = best['d_L'], best['d_R']
                imbalance = (d_R - d_L) / (d_L + d_R + 1e-9)
                bias      = imbalance * (best['delta_deg'] / 3.5) # 더 열린 구역으로 살짝 유도
                
                target    = best['center'] + bias
                steer     = max(-MAX_STEER, min(MAX_STEER, target / 90.0 * MAX_STEER))
                
                near_d = nearest_in_arc(hist, has_pt, best['center_cw'], arc_half=30)
                
                # 거리에 따른 속도 동적 조절 (긴급거리 근접 시 0.29까지 늦춤, 여유로울시 0.65)
                ratio  = min(max((DETECT - near_d) / (DETECT - EMERGENCY + 5), 0.0), 1.0)
                speed  = 0.65 * (1.0 - ratio * 0.55)
                
                ser_Ardu.write(f"F {steer:.2f} {speed:.2f}\n".encode())
                print(f"VFH_FWD   갭:{best['width']:.0f}mm @ {best['center']:+.0f}° | "
                      f"조향:{steer:+.2f} | 속도:{speed:.2f} | 장애물:{near_d:.0f}mm")

            # 케이스 2: 갭이 너무 측방/후방에 있고 정면에 장애물이 근접 (제자리 회전 탈출)
            elif best is not None and best['passable'] and emg_near <= P4_DIST:
                rot_dir = 1.0 if best['center'] > 0 else -1.0
                ser_Ardu.write(f"T {rot_dir:.2f}\n".encode())
                print(f"VFH_ROT   전방 막힘({emg_near:.0f}mm) -> 타겟 갭 {best['center']:+.0f}° 방향으로 제자리 회전")

            # 케이스 3: 완전히 갇혀있는 상황 (비상 저속 돌파 시도)
            else:
                FRONT_ARC = 60.0 # 넓어진 전방 감시망
                if gaps:
                    front_gaps = [g for g in gaps if abs(g['center']) <= FRONT_ARC]
                    if front_gaps:
                        open_g = max(front_gaps, key=lambda g: g['width'])
                        target_dir = open_g['center']
                    else:
                        open_g = max(gaps, key=lambda g: g['width'])
                        target_dir = max(-FRONT_ARC, min(FRONT_ARC, open_g['center']))
                    widest = open_g['width']
                else:
                    target_dir = 0.0
                    widest = 0.0
                
                steer = max(-MAX_STEER, min(MAX_STEER, target_dir / 90.0 * MAX_STEER * 0.5))
                # 0.20이라는 매우 느린 속도로 그나마 제일 넓은 곳을 향해 살살 비비며 빠져나가기
                ser_Ardu.write(f"F {steer:.2f} 0.20\n".encode())
                print(f"NO_GAP    안전 갭 없음 (최대:{widest:.0f}mm) -> 비상 저속 탈출 조향:{steer:+.2f}")

        # 다음 바퀴 계산을 위해 버퍼 초기화
        scan_buf = []

    else:
        # 데이터 수신 중 (1회전 미달)
        # 매우 중요: 여기에 F 0.00 명령을 지속 전송하면 시리얼 트래픽 병목으로 시스템이 마비됩니다. 
        # 아두이노는 이전 스캔에서 받은 구동 파라미터를 유지하므로 여기서는 패스합니다.
        pass
