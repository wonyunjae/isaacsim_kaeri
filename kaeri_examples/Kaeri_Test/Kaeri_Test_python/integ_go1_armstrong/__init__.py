# Copyright (c) 2018-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# NOTE: Import here your extension examples to be propagated to ISAAC SIM Extensions startup
from Kaeri_Test_python.integ_go1_armstrong.go1_armstrong_main import Main
from Kaeri_Test_python.integ_go1_armstrong.go1_armstrong_extension import MainExtension
import sys
sys.path.append("/home/smarthc/isaacsim/exts/isaacsim.ros2.bridge/humble")
sys.path.append('/usr/lib/python3/dist-packages')

