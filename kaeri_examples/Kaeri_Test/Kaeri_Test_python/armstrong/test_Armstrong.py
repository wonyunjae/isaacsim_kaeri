# python package
import os
import sys
import carb
import numpy as np
import yaml
from isaacsim.core.utils.extensions import enable_extension
from rclpy.parameter import Parameter
# Import other dependencies that should be available
from Kaeri_Test_python.kaeri_base_sample import BaseSample
# Use ROS2 publishers instead of ROS1
from Kaeri_Test_python.ros2_publishers.ros2_publisher_armstrong import *

# 직접 지정해줘야 함.
PACKAGE_PATH = os.environ["PACKAGE_PATH"] = "/home/smarthc/isaacsim/isaac_sim"
# sys.path.append("/isaac-sim")
sys.path.append("/home/smarthc/isaacsim/exts/isaacsim.ros2.bridge/humble")


# ROS1 브릿지 로드 방지를 위한 환경 변수 설정
os.environ["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH", "") + ":/home/smarthc/isaacsim/exts/isaacsim.ros2.bridge/humble/lib"
os.environ["DISABLE_ROS1_BRIDGE"] = "1"
os.environ["ROS_DISTRO"] = "humble"
os.environ["ENABLE_ROS2_BRIDGE"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"

# Try to enable ROS2 bridge
try:
    enable_extension("isaacsim.ros2.bridge")
    if "simulation_app" in globals():
        simulation_app.update()
except Exception as e:
    carb.log_warn(f"Failed to enable ROS2 bridge: {e}")

# Try to import ROS2 packages, but provide fallbacks
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
    from geometry_msgs.msg import Twist
except Exception as e:
    carb.log_warn(f"Failed to enable ROS2 bridge: {e}")


class Main(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/armstrong/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]
        self.control_freq = config["control_freq"]
        self.cfg_og = config["omnigraph"]

        self._world_settings = {"physics_dt": 1.0/self.physics_freq, "stage_units_in_meters": 1.0, "rendering_dt": 1.0/self.render_freq}

        # clock for control freq
        self._clock_control_iter = 0
        # self._base_command = [0., 0., 0., 0]
        self.desired_vel = [0.0, 0.0, 0.0]
        self.yaw = 0

        return

    def setup_scene(self):
        if ROS2_AVAILABLE:
            rclpy.init(args=None)
            self.node = rclpy.create_node("isaac")
            self.node.declare_parameter("use_sim_time", True)
            self.subscription = self.node.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        else:
            if not rclpy.core.is_initialized():
                rclpy.init_node("isaac", anonymous=False, disable_signals=True, log_level=rclpy.ERROR)
            node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            rclpy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = self.get_world()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/environment/Warehouse.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [robot] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # [1] isaac style
        # load from URDF : https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html
        #                  https://docs.omniverse.nvidia.com/kit/docs/omniverse-mjcf-importer/latest/source/extensions/omni.importer.mjcf/docs/index.html
        # URDF extension interface
        urdf_interface = _urdf.acquire_urdf_interface()
        # Set the settings in the import config
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.fix_base = False
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.create_physics_scene = True
        import_config.import_inertia_tensor = True
        
        # Get the urdf file path
        extension_path = get_extension_path_from_name("omni.importer.urdf")
        root_path = PACKAGE_PATH + "/model/armstrong/full_test/urdf"
        file_name = "full_test2.urdf"
        # Finally import the robot
        result, _armstrong_prim = omni.kit.commands.execute("URDFParseAndImportFile", 
                                                            urdf_path="{}/{}".format(root_path, file_name),
                                                      import_config=import_config,)

        # robot prim
        prim_path = "/full_test"
        armstrong_prim = self._world.scene.stage.GetPrimAtPath(prim_path)
        # print("=-=-=-=-=")
        # print("=-=-=-=-=")
        # print("=-=-=-=-=")
        # print(armstrong_prim)
        xform = UsdGeom.Xformable(armstrong_prim)
        self.armstrong_transform = xform.AddTransformOp()

        # Set the new location & rotation
        new_location = Gf.Vec3d(0.0, 0.0, 0.25)  # Replace with your desired location
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)  # Replace with your desired rotation

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.armstrong_transform.Set(mat4d)

        # # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [omnigraph] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # try:
        #     if self.cfg_og["ros_clock_publisher"]["enable"]:
        #         self._clock_graph = ros_clock_publisher()
        #         self._clock_publisher_iter = 0
            
        # except Exception as e:
        #     print(e)
        #     # simulation_app.close()
        #     exit()
        
        return
    
    def cmd_vel_cb(self, msg):
        self.desired_vel = [msg.linear.x, msg.linear.y, msg.angular.z]

    async def setup_post_load(self):
        self._world = self.get_world()
        self._world.add_physics_callback("advance", callback_fn=self.on_physics_step)
        return
    
    def set_position(self, dt):
        # get current location
        transform = self.armstrong_transform.Get()

        pos = transform.ExtractTranslation() # Gf.Vec3d
        rot = transform.ExtractRotation() # Gf.Rotation 
        
        # Set the new location
        new_location = pos
        # consider yaw
        new_location[0] = pos[0] + self.desired_vel[0] * dt * np.cos(self.yaw) - self.desired_vel[1] * dt * np.sin(self.yaw)
        new_location[1] = pos[1] + self.desired_vel[1] * dt * np.cos(self.yaw) + self.desired_vel[0] * dt * np.sin(self.yaw)
        # new_location[2] = 0.25

        # Set the new rotation
        self.yaw += self.desired_vel[2] * dt
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), self.yaw * 180.0 / np.pi)

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.armstrong_transform.Set(mat4d)
    
    def on_physics_step(self, step_size) -> None:

        # move
        self.set_position(1.0/self.physics_freq)

        # Tick omnigraph
        if self.cfg_og["ros_clock_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_clock_publisher"]["freq"] == self._clock_publisher_iter:
            og.Controller.evaluate_sync(self._clock_graph)
            self._clock_publisher_iter = 0
        else:
            self._clock_publisher_iter += self.cfg_og["ros_clock_publisher"]["freq"]

    async def setup_pre_reset(self):
        # Set the new location & rotation
        new_location = Gf.Vec3d(0.0, 0.0, 0.25)  # Replace with your desired location
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)  # Replace with your desired rotation

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.armstrong_transform.Set(mat4d)
        await self._world.pause()   # 리셋 이후 멈추도록 하기 위해.

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [omnigraph] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        try:
            if self.cfg_og["ros_clock_publisher"]["enable"]:
                self._clock_graph = ros_clock_publisher()
                self._clock_publisher_iter = 0
            
        except Exception as e:
            print(e)
            # simulation_app.close()
            exit()
        return

    async def setup_post_reset(self):
        # await self._world.pause()

        return

    def world_cleanup(self):
        # rospy.signal_shutdown("armstrong complete")
        return