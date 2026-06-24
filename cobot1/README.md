<<<<<<< HEAD
# ros2_prj1
(A-4조) ROS2를 활용한 로봇 자동화 공정 시스템 구현 프로젝트
=======
<h1 align="center"> 협동로봇 1 프로젝트 기본 연습파일 </h1>

<p align="center">두산 협동로봇 m0609 파이썬 제어 코드</p>

## :wrench: 설정
1. 홈폴더에 workspace 생성와 src 폴더 생성

```bash
$ cd
$ mkdir -p 워크스페이스명/src
```

2. 해당 폴더에 파일들을 넣고 빌드와 소스 작업
```bash
$ cd 워크스페이스명
$ colcon build --packages-select cobot1 --symlink-install
$ source install/setup.bash
``` 
3. 원하는 작업 실행
``` bash
$ ros2 run cobot1 실행파일
```





## move.py
- movej(관절 이동)와 movel(직선 이동) 사용 예제 
- 사용 명령어 : **ros2 run cobot1 move_basic** 

## move_periodic.py
- amove_periodic(비동기식 주기적 모션) 사용 예제
- 사용 명령어 : **ros2 run cobot1 move_periodic**
  
## grip_test.py
-  set_digital_output과 get_digital_input을 활용하여 그리퍼 조종 예제
-  사용 명령어 : **ros2 run cobot1 grip_test**

## force_test.py
- task_compliance_ctrl(순응 제어), set_desire_force(힘제어) 사용 예제
- 사용 명령어 : **ros2 run cobot1 force_test**

##  block1_1.py 
- 위의 예제들을 활용한 블록 집기 실습 코드.
- 사용 명령어 : **ros2 run cobot1 block_moving**
<h3 align="center">초기 세팅 방법</h4>
<p align="center"> 나중에 사진 추가 </p>


<p align="center">- 블록이 꽂혀있는 첫번째 블록(맨위, 맨 왼쪽 블록)의 좌표를 System_first_block에 입력 -> z축 좌표는 고정</p>

<p align="center"> - 목표 블록의 첫번째 블록(맨위, 맨 왼쪽 블록)좌표를 System_second_block에 입력 -> 마찬가지로 z축 좌표는 고정 </p>




>>>>>>> 727de1a (DRL 파이썬 프로그래밍 패키지 파일)
