import cv2
import numpy as np

# 1. 3D 공간 상의 객체 좌표 정의 (정사각형 300mm x 300mm)
# 점의 순서: 좌상(TL) -> 우상(TR) -> 우하(BR) -> 좌하(BL)
square_length = 300.0  # 단위: mm

obj_points = np.array([
    [-square_length / 2,  square_length / 2, 0.0], # 좌상
    [ square_length / 2,  square_length / 2, 0.0], # 우상
    [ square_length / 2, -square_length / 2, 0.0], # 우하
    [-square_length / 2, -square_length / 2, 0.0]  # 좌하
], dtype=np.float32)

# 2. 카메라 내부 파라미터 가상 설정 (본인의 카메라에 맞게 캘리브레이션 값 입력 필수)
# 임의의 웹캠 해상도(640x480) 기준 가상 행렬입니다.
focal_length = 700.0
center_x, center_y = 320.0, 240.0
camera_matrix = np.array([
    [focal_length, 0, center_x],
    [0, focal_length, center_y],
    [0, 0, 1]
], dtype=np.float32)

# 왜곡 계수 (기본 0 설정)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

# 3. 비디오 캡처 시작 (기본 웹캠 0번)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 이미지 크기 및 결과 출력용 복사
    h, w, _ = frame.shape
    display_frame = frame.copy()

    # 4. 색상 영역 검출 (HSV 색 공간 변환)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # BGR to HSV
    
    # [예시] 파란색 종이를 찾기 위한 HSV 범위 범위 설정 (종이 색상에 맞게 수정 필요)
    # 빨강
    red_lower_color = np.array([0, 100, 100])
    red_upper_color = np.array([10, 255, 255])
    # 노랑
    yellow_lower_color = np.array([17, 30, 160])
    yellow_upper_color = np.array([25, 255, 255])
    # 파랑
    blue_lower_color = np.array([100, 100, 50])
    blue_upper_color = np.array([140, 255, 255])
    
    mask_red = cv2.inRange(hsv, red_lower_color, red_upper_color)
    mask_yellow = cv2.inRange(hsv, yellow_lower_color, yellow_upper_color)
    mask_blue = cv2.inRange(hsv, blue_lower_color, blue_upper_color)

    # 노이즈 제거 (모폴로지 연산)
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_CLOSE, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

    # 5. 윤곽선 검출
    contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    

    for contour in contours:
        # 노이즈를 피하기 위해 일정 면적 이상만 처리
        if cv2.contourArea(contour) < 2000:
            continue

        # 윤곽선 근사화를 통해 꼭짓점 추출
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # 정사각형 타겟이므로 꼭짓점이 정확히 4개여야 함
        if len(approx) == 4:
            # 2D 이미지 좌표 정렬 (PnP 입력을 위해 3D 좌표 순서와 일치시킴)
            # approx의 형태는 (4, 1, 2)이므로 (4, 2)로 변경
            pts = approx.reshape(4, 2)
            
            # y값 기준 정렬 후 상단 2개, 하단 2개 분리하여 좌상/우상/우하/좌하 순서 매칭
            pts = pts[np.argsort(pts[:, 1])] # y 기준 정렬
            top_pts = pts[:2]
            bottom_pts = pts[2:]
            
            # x값 기준으로 좌우 구분
            tl = top_pts[np.argmin(top_pts[:, 0])]
            tr = top_pts[np.argmax(top_pts[:, 0])]
            br = bottom_pts[np.argmax(bottom_pts[:, 0])]
            bl = bottom_pts[np.argmin(bottom_pts[:, 0])]
            
            img_points = np.array([tl, tr, br, bl], dtype=np.float32)

            # 화면에 검출된 꼭짓점과 외곽선 그리기
            for pt in img_points:
                cv2.circle(display_frame, tuple(pt.astype(int)), 7, (0, 0, 255), -1)
            cv2.drawContours(display_frame, [approx], -1, (0, 255, 0), 2)

            # 6. solvePnP 실행 (정사각형 특화 알고리즘 사용)
            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, camera_matrix, dist_coeffs, 
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            ) #

            if success:
                # tvec에서 거리 성분(Z축) 추출 (단위: mm)
                distance_mm = tvec[2][0]
                x_pos = tvec[0][0]
                y_pos = tvec[1][0]

                # 3D 축 그리기 (시각화)
                axis = np.float32([[150,0,0], [0,150,0], [0,0,-150]]).reshape(-1,3)
                imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
                
                origin = tuple(tl.astype(int))
                display_frame = cv2.line(display_frame, origin, tuple(imgpts[0].ravel().astype(int)), (0,0,255), 3) # X축 (적색)
                display_frame = cv2.line(display_frame, origin, tuple(imgpts[1].ravel().astype(int)), (0,255,0), 3) # Y축 (녹색)
                display_frame = cv2.line(display_frame, origin, tuple(imgpts[2].ravel().astype(int)), (255,0,0), 3) # Z축 (청색)

                # 화면에 거리 정보 출력
                text = f"Distance: {distance_mm:.1f}mm (X:{x_pos:.1f}, Y:{y_pos:.1f})"
                cv2.putText(display_frame, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 결과 화면 출력
    cv2.imshow("Color Square SolvePnP", display_frame)
    cv2.imshow("Color Mask", mask_red | mask_yellow | mask_blue)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
