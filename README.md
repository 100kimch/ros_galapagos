# Galapagos, a ROS Package for autoracing

This `Galapagos` project is for autoracing Turtlebot 3 burger in the steamcup 2020 map, which includes 6 missions: `Traffic Signal`, `Intersection`, `In construction`, `Stop Signal`, and `Tunnel`. This package at least needs a lidar sensor and two cameras. For any questions or suggestions, feel free to contact [Kim Jihyeong](kjhricky@gmail.com)

본 프로젝트는 ROS 기반 터틀봇3 버거로 스팀컵 2020 자율주행 맵을 주행하기 위한 `갈라파고스` 패키지입니다. `신호등`, `교차로`, `공사구간`, `차단바`, `터널` 등의 구간으로 이루어져 있는데, 1대의 라이다 센서와 2대의 카메라 모듈을 기반으로 작동합니다. 문의사항은 언제든지 [다음 이메일](kjhricky@gmail.com)에 남겨 주세요.

## File Systems

This project consists three packages:

- `galapagos`: initial package but deprecated. Just use it for reference only.
- `galapagos_v2`: main package to run. Designed for wide scalability of various types of sensors and debugging modes.
- `galapagos_embedded`: a package porked from `galapagos_v2`. Image Processing is processed in Raspberry Pi board.

본 프로젝트는 3개의 패키지를 가지고 있습니다:

- `galapagos`: 초창기 패키지이나 지금은 사용되지 않습니다. 참고용으로만 사용하세요.
- `galapagos_v2`: 주 패키지이며, 디버깅 모드와 다양한 센서들을 부착하기 위한 확장성을 가지고 있습니다.
- `galapagos_embedded`: 라즈베리파이 보드에서 영상처리를 하기 위해 별도의 패키지로 구성하였습니다. `galapagos_v2` 코드에서부터 시작되었습니다.

## Specifications

### Hardware

- Raspberry Pi 3 Model B+
- Logitech C270 (1EA for default, extra 1EA can be installed for stability)
- Raspicam 1EA
- LDS-01 Lidar
- OpenCR
- DYNAMIXEL Motor 2EA

### Software

터틀봇 버거 및 노트북 환경 설정을 다음과 같이 진행합니다.

#### for Turtlebot3 Burger

- Raspbian OS
  - Python 3.6 or above
  - ROS (Kinetic Kame)
  - packages on catkin_ws/src/
    - raspicam_node
    - hls_lfcd_lds_driver
    - usb_cam
    - common_msgs
    - turtlebot3
    - turtlebot3_autorace
    - turtlebot3_msgs

#### for Laptop PC (Remote PC)

- Ubuntu 18.04
  - Python 3.6 or above
  - ROS (Melodic Morenia)
  - OpenCV 3.4.0
  - packages on catkin_ws/src/
    - turtlebot3
    - turtlebot3_msgs
    - turtlebot3_simulations
    - turtlebot3_autorace
    - common_msgs

## Other Information or manuals

are uploaded on [Wiki Page](https://github.com/100kimch/ros_galapagos/wiki)
