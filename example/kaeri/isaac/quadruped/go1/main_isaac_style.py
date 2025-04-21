import numpy as np
import argparse

from omni.isaac.orbit.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""
put ./orbit.sh down below code

# =-=-=-=-=-=-=-=-=-=-=-
ISAAC_DIR="/isaac-sim"
PACKAGE_DIR="/isaac-sim/isaac_sim"
export PACKAGE_PATH=$PACKAGE_DIR

export ROS_ENABLED=1 # for ros1 noetic
# export ROS_ENABLED=2 # for ros2 foxy
"""

import carb
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.api.world import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim import create_prim
from omni.isaac.quadruped.robots import Unitree
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.utils.extensions import get_extension_path_from_name

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")
enable_extension("omni.kaeri.ros_bridge")

# update the simulation
simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

import rospy
from geometry_msgs.msg import Twist

import os
import yaml

# get $PACKAGE_PATH
PACKAGE_PATH = os.environ["PACKAGE_PATH"]

# add $PACKAGE_PATH to python path
import sys
sys.path.append(PACKAGE_PATH)

# custom scripts
from example.kaeri.isaac.quadruped.go1.articulation.isaac_articulation import IsaacArticulation
from example.kaeri.isaac.quadruped.go1.ros_publisher import *

class Go1_runner(object):
    def __init__(self) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a unitree go1 robot
        """

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/quadruped/go1/config_isaac_style.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_freq = config["control_freq"]
        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]

        self.cfg_og = config["omnigraph"]

        # clock for control freq
        self._clock_control_iter = 0
        self._base_command = [0., 0., 0., 0]

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = World(stage_units_in_meters=1.0, physics_dt=1.0/self.physics_freq, rendering_dt=1.0/self.render_freq)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
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
        root_path = PACKAGE_PATH + "/model/go1/go1_description/urdf"
        file_name = "go1.urdf"
        # Finally import the robot
        result, go1_prim = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path="{}/{}".format(root_path, file_name),
                                                      import_config=import_config,)

        # articulation
        self._go1 = self._world.scene.add(
            IsaacArticulation(
                prim_path  = go1_prim,
                name       = "Go1", 
                position   = np.array([0, 0, 1.0]), 
                physics_dt = 1.0/self.physics_freq, 
                model      = "Go1",
            )
        )

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
        
        self.cmd_vel = [0.0, 0.0, 0.0]
        
        return
    
    def cmd_vel_cb(self, msg):
        self.cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        self._base_command = [*self.cmd_vel, 1]

    def setup(self) -> None:
        """
        [Summary]

        Set unitree robot's default stance, set up keyboard listener and add physics callback

        """
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("go1_advance", callback_fn=self.on_physics_step)
        # self._world.add_timeline_callback("go1_advance", callback_fn=self.on_timeline_step)

    def on_physics_step(self, step_size) -> None:
        """
        [Summary]

        Physics call back, switch robot mode and call robot advance function to compute and apply joint torque

        """
        # if self._event_flag:
        #     self._go1._qp_controller.switch_mode()
        #     self._event_flag = False

        if self.physics_freq - self.control_freq == self._clock_control_iter:
            step_size = 1.0 / self.control_freq

            self._go1.advance(step_size, self._base_command)
            
            self._clock_control_iter = 0
        else:
            self._clock_control_iter += self.control_freq

        # Tick omnigraph
        if self.cfg_og["ros_clock_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_clock_publisher"]["freq"] == self._clock_publisher_iter:
            og.Controller.evaluate_sync(self._clock_graph)
            self._clock_publisher_iter = 0
        else:
            self._clock_publisher_iter += self.cfg_og["ros_clock_publisher"]["freq"]

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
    rospy.init_node("go1_standalone", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    runner = Go1_runner()
    runner._world.reset()

    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()
    runner._world.reset()
    runner.run()
    rospy.signal_shutdown("go1 complete")
    simulation_app.close()


if __name__ == "__main__":
    main()
