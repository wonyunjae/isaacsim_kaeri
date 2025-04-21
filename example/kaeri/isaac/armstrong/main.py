# ===================================================================================
# last update : 2024.07.22
# updated by Woo-Cheol Lee (wclee6314@gmail.com / wclee@kaist.ac.kr / wclee@kaeri.re.kr)
# ===================================================================================

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
from isaacsim.core.api.world import World
from isaacsim.core.utils.extensions import enable_extension
# from omni.isaac.quadruped.robots import Unitree
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.asset.importer.urdf import _urdf
from pxr import Gf, UsdGeom

# enable ROS 
enable_extension("isaacsim.ros1.bridge")
simulation_app.update()

# ROS
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist



# python package
import os
import yaml
PACKAGE_PATH = os.environ["PACKAGE_PATH"]

print(PACKAGE_PATH)

# custom scripts
import sys
import numpy as np
sys.path.append(PACKAGE_PATH)
from example.kaeri.isaac.armstrong.articulation.isaac_articulation import IsaacArmstrongArticulation
from example.kaeri.isaac.armstrong.ros_publisher import *

class Armstrong_runner(object):
    def __init__(self, physics_dt, render_dt) -> None:
        """
        [Summary]

        create simulation world with preset physics_dt and render_dt and create a Armstrong robot

        Argument:
        physics_freq {float} -- Physics frequency of the scene.
        render_freq {float} -- Render frequency of the scene.
        control_freq {float} -- Control frequency of the scene.
        """

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/armstrong/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]
        self.control_freq = config["control_freq"]
        self.cfg_og = config["omnigraph"]

        # clock for control freq
        self._clock_control_iter = 0
        self._base_command = [0., 0., 0., 0]

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = World(stage_units_in_meters=1.0, physics_dt=1.0/self.physics_freq, rendering_dt=1.0/self.render_freq)

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
        print("=-=-=-=-=")
        print("=-=-=-=-=")
        print("=-=-=-=-=")
        print(armstrong_prim)
        xform = UsdGeom.Xformable(armstrong_prim)
        self.armstrong_transform = xform.AddTransformOp()

        # Set the new location & rotation
        new_location = Gf.Vec3d(0.0, 0.0, 0.5)  # Replace with your desired location
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)  # Replace with your desired rotation

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.armstrong_transform.Set(mat4d)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [reset] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world.reset()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [omnigraph] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        try:
            if self.cfg_og["ros_clock_publisher"]["enable"]:
                self._clock_graph = ros_clock_publisher()
                self._clock_publisher_iter = 0
            
        except Exception as e:
            print(e)
            simulation_app.close()
            exit()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [subscriber] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        self.desired_vel = [0.0, 0.0, 0.0]
        self.yaw = 0

        return

    def cmd_vel_cb(self, msg):
        self.desired_vel = [msg.linear.x, msg.linear.y, msg.angular.z]

        # print(self.desired_vel)

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("advance", callback_fn=self.on_physics_step)

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
        new_location[2] = 0.5

        # Set the new rotation
        self.yaw += self.desired_vel[2] * dt 
        new_rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), 90.0) * Gf.Rotation(Gf.Vec3d(0, 0, 1), self.yaw * 180.0 / np.pi)

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

    def run(self) -> None:
        while simulation_app.is_running():
            self._world.step(render=True)
        return

def main() -> None:
    rospy.init_node("armstrong_standalone", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    physics_downtime = 1 / 400.0
    runner = Armstrong_runner(physics_dt=physics_downtime, render_dt=16 * physics_downtime)

    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()
    runner._world.reset()
    runner.run()
    rospy.signal_shutdown("armstrong complete")
    simulation_app.close()

if __name__ == "__main__":
    main()
