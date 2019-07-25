---
id: ros-setting-environment
title: Steamcup 프로젝트 진행을 위한 환경설정
sidebar_label: Steamcup 프로젝트 진행을 위한 환경설정
---

- 작성 기준일: 2019. 07. 25.
- 작성자: 김지형 (100kimch@naver.com)

## 변경 사항

- 본 문서를 수정한 이력이 있다면 다음 표에 작성해주시기 바랍니다.

| 순번 | 일자         | 수정자 | 수정자 이메일      | 수정 내용             | 비고 |
| ---- | ------------ | ------ | ------------------ | --------------------- | ---- |
| 1    | 2019. 07. 25 | 김지형 | 100kimch@naver.com | Initial Documentation |      |

## 개요

본 문서는 2020 Steamcup 프로젝트 진행하기 위한 환경 설정을 위한 문서입니다.

## 개발 환경

본 공모전은 다음과 같이 개발 환경을 구성하였습니다.

### Hardware

- 노트북 4대

  - OS
    - 듀얼부팅 체제
    - Ubuntu 16.04 LTS (Xenial Xerus)
    - Windows 10 Education
  - Storage
    - 60GB for Ubuntu
    - 120GB for Windows

- TurtleBot 2대
  - TurtleBot #1
  - TurtleBot #2

### Software

- Framework
  - openCV2
  - numpy

## 환경 설정

## OpenCV 설치

OpenCV를 설치하는 방법입니다.

```bash
cd ~
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
git clone https://github.com/opencv/opencv_extra.git
cd ~/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j7
sudo make install
```
