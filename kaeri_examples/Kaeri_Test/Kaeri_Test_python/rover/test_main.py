# python package
import os
import sys
# 직접 지정해줘야 함.
sys.path.append("/home/smarthc")
sys.path.append("/home/smarthc/isaacsim/exts/isaacsim.ros2.bridge/humble")

PACKAGE_PATH = os.environ["PACKAGE_PATH"] = "/home/smarthc/isaacsim/isaac_sim"

# ROS1 브릿지 로드 방지를 위한 환경 변수 설정
os.environ["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH", "") + ":/home/smarthc/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
os.environ["DISABLE_ROS1_BRIDGE"] = "1"
os.environ["ROS_DISTRO"] = "humble"
os.environ["ENABLE_ROS2_BRIDGE"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
from isaacsim.core.api.world import World
from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, UsdGeom

# ROS2 브릿지 활성화
enable_extension("isaacsim.ros2.bridge")
if "simulation_app" in globals():
    simulation_app.update()

# custom scripts
import yaml
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

        self._world_settings = {"physics_dt": 1.0/self.physics_freq, "stage_units_in_meters": 1.0, "rendering_dt": 1.0/self.render_freq}
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = self.get_world()

        # clock for control freq
        self._clock_control_iter = 0
        self._clock_publisher_iter = 0  # _clock_publisher_iter 초기화

        self.desired_vel = [0.0, 0.0, 0.0]
        self.yaw = 0

        return

    def setup_scene(self):
        # ROS2 노드 초기화 필요 없음 - 호스트 시스템과 통신하는 브릿지만 사용
        carb.log_info("호스트 ROS2 Humble과 통신을 시작합니다")
        
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/moon_surface/MoonSurface_v1128.usdc"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Moon")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [robot] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        usd_path = PACKAGE_PATH + "/model/rover/ROVER_v1211_colored.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")

        # robot prim
        talon_prim = self._world.scene.stage.GetPrimAtPath("/World/Robot")
        xform = UsdGeom.Xformable(talon_prim)
        self.talon_transform = xform.AddTransformOp()

        # Set the new location & rotation
        new_location = Gf.Vec3d(0.0, 3.0, 1.5)  # 내리막길에서 시작하려면 -12.0, 2.0, 3.5 # 구덩이 안은 0.0, 3.0, 1.5
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.talon_transform.Set(mat4d)
        
        return

    async def setup_pre_reset(self):
        return

    async def setup_post_reset(self):
        return

    def world_cleanup(self):
        # ROS2 브릿지 정리 작업
        carb.log_info("ROS2 브릿지 연결 종료")
        return