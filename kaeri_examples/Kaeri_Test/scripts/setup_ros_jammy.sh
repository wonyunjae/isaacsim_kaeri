#!/bin/bash
set -e

echo "=== Ubuntu 22.04 (Jammy)에서 ROS Noetic 실행을 위한 설정 스크립트 ==="

# 필수 패키지 설치
sudo apt update
sudo apt install -y git python3-pip python3-rosdep python3-rosinstall-generator

# turtlebot3_msgs 저장소 클론
echo "turtlebot3_msgs 저장소 클론 및 빌드 중..."
cd ~/ros1_ws/src
if [ ! -d "turtlebot3_msgs" ]; then
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
fi

# navigation 저장소 클론 (move_base 포함)
echo "navigation 저장소 클론 중..."
if [ ! -d "navigation" ]; then
    git clone -b noetic-devel https://github.com/ros-planning/navigation.git
fi

# custom rosdep 규칙 추가
echo "커스텀 rosdep 규칙 추가 중..."
sudo mkdir -p /etc/ros/rosdep/sources.list.d
cat << EOF | sudo tee /etc/ros/rosdep/sources.list.d/20-custom-jammy.list
yaml file:///etc/ros/rosdep/custom-jammy.yaml
EOF

cat << EOF | sudo tee /etc/ros/rosdep/custom-jammy.yaml
turtlebot3_msgs:
  ubuntu:
    jammy: [ros-noetic-turtlebot3-msgs]
move_base:
  ubuntu:
    jammy: [ros-noetic-move-base]
EOF

# rosdep 업데이트
echo "rosdep 업데이트 중..."
rosdep update

cat << EOF

=== 설정 완료 ===

다음 단계로 워크스페이스를 빌드하세요:
cd ~/ros1_ws
catkin_make

그런 다음 소스를 설정하고 다시 rosdep 설치를 실행하세요:
source devel/setup.bash
rosdep install -i --from-path src --rosdistro noetic -y

문제가 계속되면 수동으로 필요한 패키지를 설치하세요:
sudo apt install ros-noetic-move-base
EOF
