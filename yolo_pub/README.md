### 패키지 생성

$ ros2 pkg create --build-type ament_python yolo_pub --dependencies rclpy std_msgs
sensor_msgs cv_bridge

### 노드 구조

[Webcam Node]
camera_publisher
        │
        ▼
/camera/image_raw
        │
        ▼
[YOLO Node]
yolo_detector
        │
        ▼
/detection/result
        │
        ▼
[Subscriber Node]
result_subscriber

### 실행 순서

패키지 안에서

ros2 run yolo_pub cam

다른 터미널

ros2 run yolo_pub yolo

다른 터미널

ros2 run yolo_pub sub



##  추가 프로그램 설치

pip install ultralytics

wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

sudo apt update
sudo apt install python3-opencv

### 경로에 모델파일 추가

보통 패키지 안에 models 폴더를 만듭니다.

예시 ROS2 패키지 구조

yolo_pub
 ├── yolo_pub
 │   ├── camera_publisher.py
 │   ├── yolo_detector.py
 │   ├── result_subscriber.py
 │
 ├── models
 │   └── yolov8n.pt
 │
 ├── package.xml
 ├── setup.py

그리고 코드에서 경로 지정

from ultralytics import YOLO

self.model = YOLO("models/yolov8n.pt")

### yolo 와 NumPy버전이 맞지 않아 오류가 생길때 해결법

1️⃣ 현재 NumPy 버전 확인
python -c "import numpy; print(numpy.__version__)"

2️⃣ NumPy 삭제
pip uninstall numpy

여러 번 설치된 경우가 있으니 2~3번 반복 실행해서 완전히 삭제합니다.

pip uninstall numpy
pip uninstall numpy

3️⃣ 원하는 버전 설치
예: YOLOv8에서 안정적인 버전

pip install numpy==1.26.4

또는
pip install "numpy<2.0"

4️⃣ 설치 확인
python -c "import numpy; print(numpy.__version__)"

5️⃣ YOLOv8 재설치 (필요한 경우)
NumPy 문제로 깨졌다면 **Ultralytics YOLO 패키지도 다시 설치하는 것이 좋습니다.

pip uninstall ultralytics
pip install ultralytics

추천 조합 (안정적인 환경)

pip install numpy==1.26.4
pip install ultralytics