# python package
import os
import sys
# 직접 지정해줘야 함.
PACKAGE_PATH = os.environ["PACKAGE_PATH"] = "/home/smarthc/isaacsim/isaac_sim"
sys.path.append("/home/smarthc")
sys.path.insert(0, "/home/smarthc/isaacsim/exts/isaacsim.ros2.bridge/humble")


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

# enable ROS bridge extension
enable_extension("isaacsim.ros2.bridge")
# 존재하지 않는 확장 모듈 호출 제거
# enable_extension("omni.kaeri.ros_bridge")

# Try to enable ROS2 bridge
try:
    enable_extension("isaacsim.ros2.bridge")
    if "simulation_app" in globals():
        simulation_app.update()
    carb.log_info("ROS2 bridge enabled successfully.")
except Exception as e:
    carb.log_warn(f"Failed to enable ROS2 bridge: {e}")

import rclpy
# from isaacsim.ros2.bridge.humble.rclpy.parameter import Parameter
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray
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
    if not rclpy.ok():  # 4칸 들여쓰기(표준)
        rclpy.init()    # 8칸 들여쓰기
        self.node = rclpy.create_node("talon_sim")  # 노드 생성 추가
        self.node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/environment/Warehouse.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [robot] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        usd_path = PACKAGE_PATH + "/model/talon/TALON_v1123.usd"
        add_reference_to_stage(usd_path=usd_path, prim_path="/World/Robot")

        # robot prim
        talon_prim = self._world.scene.stage.GetPrimAtPath("/World/Robot")
        xform = UsdGeom.Xformable(talon_prim)
        self.talon_transform = xform.AddTransformOp()

        # Set the new location & rotation
        new_location = Gf.Vec3d(0.0, 0.0, 0.7)
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.talon_transform.Set(mat4d)
    
    return  # 함수와 같은 레벨의 들여쓰기

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