#!/bin/bash

# /isaac-sim/python.sh shell을 참조함

set -e

error_exit()
{
    echo "There was an error running python"
    exit 1
}

# ISAAC_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ISAAC_DIR="/isaac-sim"
PACKAGE_DIR="/isaac-sim/isaac_sim"

# 환경변수 세팅 
export CARB_APP_PATH=$ISAAC_DIR/kit
export ISAAC_PATH=$ISAAC_DIR
export EXP_PATH=$ISAAC_DIR/apps
export PACKAGE_PATH=$PACKAGE_DIR

# 파이썬 환경 세팅 shell 스크립트 실행
# (default)
# source ${ISAAC_DIR}/setup_python_env.sh
# (custom)
source ${PACKAGE_DIR}/example/setup_python_env.sh



# docker 환경 관련하여 체크할 사항 체크
if [ -f /.dockerenv ]; then
  # Check for vulkan in docker container
  if [[ -f "${ISAAC_DIR}/vulkan_check.sh" ]]; then
    ${ISAAC_DIR}/vulkan_check.sh
  fi
fi

# Show icon if not running headless
export RESOURCE_NAME="IsaacSim"
# WAR for missing libcarb.so
export LD_PRELOAD=$ISAAC_DIR/kit/libcarb.so

# make softlink from "/isaac-sim/isaac_sim/extensions/omni.test.extension" to "/isaac-sim/isaac_sim/isaac_sim/extensions/omni.test.extension"
# if already exist, delete it.
if [ -L /isaac-sim/exts/omni.kaeri.ros_bridge ]; then
  rm /isaac-sim/exts/omni.kaeri.ros_bridge
fi
ln -s /isaac-sim/isaac_sim/extensions/omni.kaeri.ros_bridge    /isaac-sim/exts/omni.kaeri.ros_bridge

if [ -L /isaac-sim/exts/omni.isaac.terrain_generator ]; then
  rm /isaac-sim/exts/omni.isaac.terrain_generator
fi
ln -s /isaac-sim/isaac_sim/extensions/omni.isaac.terrain_generator    /isaac-sim/exts/omni.isaac.terrain_generator

# 사용할 python interpretor
python_exe=${PYTHONEXE:-"${ISAAC_DIR}/kit/python/bin/python3"}

# Run
$python_exe "$@" $args || error_exit
