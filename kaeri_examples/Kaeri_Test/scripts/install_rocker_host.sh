#!/bin/bash
set -e

echo "=== 로컬 시스템에 Rocker 설치 스크립트 ==="

# ROS 저장소가 설정되어 있는지 확인
if [ ! -f /etc/apt/sources.list.d/ros-latest.list ]; then
    echo "ROS 저장소 설정 중..."
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi

# 패키지 목록 업데이트
echo "패키지 목록 업데이트 중..."
sudo apt update

# Rocker 설치
echo "Rocker 설치 중..."
sudo apt install -y python3-rocker

echo "=== 설치 완료! ==="
echo "이제 로컬 시스템에서 rocker 명령을 사용하여 ROS Docker 컨테이너를 실행할 수 있습니다."
echo "예시: rocker --nvidia --x11 --user --volume /path/to/your/workspace osrf/ros:noetic-desktop-full"
