# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# # NOTE: Import here your extension examples to be propagated to ISAAC SIM Extensions startup
# from Kaeri_Test_python.go1_mainRL.test_mainRL import MainRL
# from Kaeri_Test_python.go1_mainRL.test_mainRL_extension import MainRLExtension
import sys
sys.path.append("/home/smarthc/isaacsim/IsaacLab/source")  # isaaclab_tasks 경로 추가
# IsaacLab 경로 추가
sys.path.insert(0, "/home/smarthc/isaacsim/IsaacLab/source")
sys.path.insert(0, "/home/smarthc/isaacsim/IsaacLab/source/isaaclab_tasks")
sys.path.insert(0, "/home/smarthc/.local/lib/python3.10/site-packages")
sys.path.append("/home/smarthc/isaacsim/exts/isaacsim.ros2.bridge/humble")
import isaaclab_tasks

# MainRL 클래스 임포트
from Kaeri_Test_python.go1_mainRL.test_mainRL import MainRL