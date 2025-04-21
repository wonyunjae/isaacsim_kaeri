# python package
import os
import sys
# 직접 지정해줘야 함.
PACKAGE_PATH = os.environ["PACKAGE_PATH"] = "/isaac-sim/isaac_sim"
sys.path.append("/isaac-sim")

# ROS2 브릿지 로드 방지를 위한 환경 변수 설정
os.environ["DISABLE_ROS2_BRIDGE"] = "1"

# ROS_MASTER_URI를 명시적으로 설정 - Docker 내부에서 사용
# 호스트 시스템의 hostname이 "smarthc"라면 아래 줄을 대신 사용
os.environ["ROS_MASTER_URI"] = "http://smarthc:11311"


import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
from isaacsim.core.api.world import World
from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, UsdGeom

# enable ROS bridge extension
from omni.isaac.kit import SimulationApp

# ROS1 브릿지 활성화
enable_extension("isaacsim.ros1.bridge")
if "simulation_app" in globals():
    simulation_app.update()

import rospy
# from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import yaml

# custom scripts
import numpy as np

from Kaeri_Test_python.kaeri_base_sample import BaseSample

class Main(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/talon/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]
        self.control_freq = config["control_freq"]
        # self.cfg_og = config["omnigraph"]

        self._world_settings = {"physics_dt": 1.0/self.physics_freq, "stage_units_in_meters": 1.0, "rendering_dt": 1.0/self.render_freq}
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = self.get_world()

        # clock for control freq
        self._clock_control_iter = 0

        # 이 부분을 추가해 주세요
        self._clock_publisher_iter = 0  # _clock_publisher_iter 초기화

        # self._base_command = [0., 0., 0., 0]
        self.desired_vel = [0.0, 0.0, 0.0]
        self.yaw = 0

        return

    def setup_scene(self):
        if not rospy.core.is_initialized():
            rospy.init_node("isaac", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
        rospy.set_param("use_sim_time", True)
        
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/moon_surface/MoonSurface_v1128.usdc"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Moon")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [robot] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        usd_path = "/isaac-sim/isaac_sim/model/rover/ROVER_v1211_colored.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")

        # robot prim
        talon_prim = self._world.scene.stage.GetPrimAtPath("/World/Robot")
        xform = UsdGeom.Xformable(talon_prim)
        self.talon_transform = xform.AddTransformOp()

        # Set the new location & rotation
        new_location = Gf.Vec3d(0.0, 3.0, 1.5)  # 내리막길에서 시작하려면 -12.0, 2.0, 3.5 # 구덩이 안은 0.0, 3.0, 1.5
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)  # Replace with your desired rotation

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.talon_transform.Set(mat4d)
        
        return

    async def setup_pre_reset(self):
        # # Set the new location & rotation
        # new_location = Gf.Vec3d(0.0, 0.0, 0.65)  # Replace with your desired location
        # new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)  # Replace with your desired rotation

        # # Create a new transform
        # mat4d = Gf.Matrix4d()
        # mat4d.SetTranslateOnly(new_location)
        # mat4d.SetRotateOnly(new_rotation)

        # # Apply the new transform
        # self.talon_transform.Set(mat4d)
        # await self._world.pause()   # 리셋 이후 멈추도록 하기 위해.

        return


    async def setup_post_reset(self):
        # await self._world.pause()

        return

    def world_cleanup(self):
        # rospy.signal_shutdown("armstrong complete")
        return