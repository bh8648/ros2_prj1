"""바구니 4코너의 픽셀 좌표를 클릭으로 측정하는 캘리브레이션 도우미.

카메라 영상을 띄우고 바구니 네 모서리를 아래 순서로 클릭하면 픽셀 좌표를 출력한다.
이 픽셀값을 robot_motion.py의 코너 로봇 좌표와 매칭해 호모그래피를 만든다.

클릭 순서 (robot_motion.py의 상수와 동일 순서):
  1. top_left   (로봇 [292, 292])
  2. top_right  (로봇 [610, 255])
  3. bot_left   (로봇 [283, -64])
  4. bot_right  (로봇 [611, -63])

사용:
  python3 calibrate_corners.py            # 카메라 인덱스 2
  python3 calibrate_corners.py 0          # 카메라 인덱스 0

조작:
  - 모서리를 좌클릭해서 4점 찍기 (찍을 때마다 화면/콘솔에 표시)
  - r : 초기화(다시 찍기)
  - s : 4점 찍었으면 결과 출력
  - q : 종료
"""
import sys

import cv2

LABELS = ['top_left', 'top_right', 'bot_left', 'bot_right']
points = []


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append((x, y))
        print(f'  {LABELS[len(points) - 1]} 픽셀 = ({x}, {y})')


def print_result():
    if len(points) < 4:
        print(f'아직 {4 - len(points)}점 남았습니다.')
        return
    print('\n===== 픽셀 좌표 (이 값을 알려주세요) =====')
    for label, (x, y) in zip(LABELS, points):
        print(f'{label}_px = [{x}, {y}]')
    print('==========================================\n')


def main():
    cam_index = int(sys.argv[1]) if len(sys.argv) > 1 else 2
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f'카메라(index={cam_index})를 열 수 없습니다.')
        return

    win = 'corner calibration (click 4 corners: TL, TR, BL, BR / r:reset s:show q:quit)'
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    print('바구니 네 모서리를 순서대로 클릭하세요: TL → TR → BL → BR')

    while True:
        ret, frame = cap.read()
        if not ret:
            print('프레임을 읽지 못했습니다.')
            break

        for i, (x, y) in enumerate(points):
            cv2.circle(frame, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(frame, f'{i+1}:{LABELS[i]}', (x + 8, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            points.clear()
            print('초기화됨. 다시 클릭하세요.')
        elif key == ord('s'):
            print_result()

    cap.release()
    cv2.destroyAllWindows()
    print_result()


if __name__ == '__main__':
    main()
