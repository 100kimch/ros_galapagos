# `Galapagos_v2` 패키지 구성

## 파일 시스템

- `images`
  - 표지판, 테스트 이미지 등을 저장
- `launch`
  - 맨 처음 패키지가 시작될 때 실행하는 코드
  - 반드시 `launch`라는 이름에 `.launch` 파일들이 저장되어야 함
  - `runner.launch`는 모터를 작동시킴
  - `viewer.launch`는 디버깅용 파일로서 모터를 작동시키지 않음.
- `libs`
  - 카메라 영상처리 알고리즘, 라이다 동작 코드 등을 모듈별로 분리
  - `lib_*.py` 형식으로 이름이 지정
- `runners`
  - 구간별로 디버깅하기 편하게 하기 위해 여러 모드로 동작을 수행
  - `run_*.py` 형식으로 이름지 지정
- `scripts`
  - 스케줄러, 터틀봇 동작 함수 등 패키지를 실행하는 데 핵심적인 함수들이 저장
- `CMakeLists.txt`
- `package.xml`

## 코드 분석 순서

- `.launch` 파일 보기 -> `scripts/launcher.py` 보기 -> `runners/run_viewer.py` 보기 -> `turtlebot.py`, `constants.py`, `processor.py`를 동시에 같이보기
- 상단에 `import`가 뭐가 되어있는지 확인하면서 보라는 의미
- 최대한 함수명, 변수명을 토대로 이해하면서 넘어가고, 그래도 이해가 안되면 직접 해당 함수를 열어서 보기
- `system_architecture` Wiki 파일이 깃허브에 올라와있으니 보면서 같이 코드 분석할 것

### `runners/run_full.py`

- `roslaunch galapagos_v2 runner.launch`를 실행하면 실행되는 모듈
- `PATH_RASPICAM`, 즉 라즈베리파이 카메라 이미지를 받으면 `process_fishcam()`가 실행됨
- `PATH_USBCAM`, 즉 USB 카메라 이미지를 받으면 `process_frontcam()`이 실행됨
- `PATH_LIDAR`, 즉 라이다 센서값을 받으면 `process_lidar()`가 실행됨
- `runners/run_viewer.py`를 비롯한 모든 `run_*.py` 파일들이 이와 비슷하게 구성

### `turtlebot.py`

- 터틀봇 모터를 실행시키기 위한 함수들이 정의되어 있음
- TURTLE.set_speed('fast')

  - == 터틀봇의 스피드를 "빠르게" 해주세요
  - 'fast', 'normal', 'slow' 등의 속도 조절 값은 'constants.py'에 저장되어있음

- TURTLE.set_speed_smooth('fast')

  - == 터틀봇의 스피드를 서서히 "빠르게" 해주세요
  - 'fast', 'normal', 'slow' 등의 속도 조절 값은 'constants.py'에 저장되어있음

- TURTLE.turn(0.13, -1.0)

  - 터틀봇을 0.13초만큼 -1.0 각도로 틀어주세요

- 나머지 함수들은 '굳이' 볼 필요 없지만, `processor.py`에서 실행하는 `TURTLE` 함수가 있으면 그때마다 볼 것

### `scripts/constants.py`

- 설정해야할 수치 상수들이 이곳에 모두 저장되어 있음.
- 사용 시, 다른 소스코드들은 손대지 않고 본 파일의 값만 조정함으로써 미세 조정을 하기 위함

### `lib/lib_eye.py`

- 라즈베리파이 카메라, USB 카메라 등 카메라 류의 영상처리 함수들이 EYE 클래스 안에 정의되어 있음
- `lib/lib_frontcam.py`, `lib/lib_fishcam.py` will be deprecated
- 여기에 OpenCV 영상처리 코드들이 다 정의되어 있음

### `scripts/processor.py`

- 실제 맵을 주행하기 위한 알고리즘 코드
- 추후 설명하겠지만, 우선, [ROS Tutorial - Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)를 이햬할 필요가 있음.

## TODO

- `galapagos_v2` 뜯어보기
- ROS Tutorial 1, 2, 3, 4, 5, 6, 7, 8, 11 읽고 이해하기
- (할 수 있으면) ROS Tutorial 15 읽고 라이다 값을 읽을 수 있는 패키지 만들어보기

