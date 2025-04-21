# This script is a standalone script to run the go1 robot with sensors in omniverse
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import os
import yaml

import carb
import omni.appwindow  # Contains handle to keyboard
from isaacsim.core.api.world import World
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, UsdGeom

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")
enable_extension("omni.isaac.terrain_generator")

# update the simulation
simulation_app.update()

# ROS
# check if rosmaster is online
import rosgraph
if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()
import rospy

# $PACKAGE_PATH to python path
import sys
PACKAGE_PATH = os.environ["PACKAGE_PATH"]
os.environ["ROS_MASTER_URI"] = "http://localhost:11311"
sys.path.append(PACKAGE_PATH)

import omni.graph.core as og

class Sensor_runner(object):
    def __init__(self) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a unitree go1 robot
        """

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/environment/terrain/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_freq = config["control_freq"]
        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]
        self.cfg_og       = config["omnigraph"]

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = World(stage_units_in_meters=1.0, physics_dt=1.0/self.physics_freq, rendering_dt=1.0/self.render_freq)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # groundplane
        prim = get_prim_at_path("/World")
        if not prim.IsValid():
            prim = define_prim("/World", "Xform")
        asset_path = PACKAGE_PATH + "/model/environment/defaultGroundPlane.usd"
        prim.GetReferences().AddReference(asset_path)

        groundplane_prim = self._world.scene.stage.GetPrimAtPath("/World/defaultGroundPlane")
        xform = UsdGeom.Xformable(groundplane_prim)
        transform = xform.AddTransformOp()

        # Set the new location
        new_location = Gf.Vec3d(0.0, 0.0, -5)  # Replace with your desired location

        # Set the new rotation
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 1, 0), 0)  # Replace with your desired rotation

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        transform.Set(mat4d)
        

        # reset world
        self._world.reset()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [ROS] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        
        return

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("lidar", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        pass

    def run(self) -> None:
        """
        [Summary]

        Step simulation based on rendering downtime

        """
        # change to sim running
        while simulation_app.is_running():
            self._world.step(render=True)
        return

def main() -> None:
    rospy.init_node("terrain", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    runner = Sensor_runner()
    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()
    runner._world.reset()
    runner.run()
    rospy.signal_shutdown("terrain world complete")
    simulation_app.close()


if __name__ == "__main__":
    main()
