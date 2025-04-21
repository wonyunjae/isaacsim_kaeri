from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
from isaacsim.core.api.world import World
from isaacsim.core.utils.extensions import enable_extension

from omni.isaac.quadruped.robots import Unitree
from isaacsim.core.utils.prims import create_prim, get_prim_at_path
import omni.replicator.core as rep
from omni.isaac.core.objects import DynamicCuboid, DynamicCylinder
from pxr import Gf, UsdGeom, UsdPhysics
from omni.isaac.core.utils.transformations import pose_from_tf_matrix
from omni.isaac.core.utils.rotations import rot_matrix_to_quat, quat_to_euler_angles, euler_to_rot_matrix, matrix_to_euler_angles, euler_angles_to_quat

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")
enable_extension("omni.kaeri.ros_bridge")
enable_extension("omni.isaac.terrain_generator")

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
import tf
from std_msgs.msg import Float32MultiArray

import os
import yaml

# get $PACKAGE_PATH
PACKAGE_PATH = os.environ["PACKAGE_PATH"]

# add $PACKAGE_PATH to python path
import sys
sys.path.append(PACKAGE_PATH)

# custom omnigraph
from example.kaeri.go1.omnigraph.ros_publisher import *

class Go1_runner(object):
    def __init__(self) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a unitree go1 robot
        """

        # =-=-=-=-=-=-= config =-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/go1/config/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_freq = config["control_freq"]
        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]

        self.cfg_og = config["omnigraph"]

        # clock for control freq
        self._clock_control_iter = 0

        # =-=-=-=-=-=-= World =-=-=-=-=-=-=
        self._world = World(stage_units_in_meters=1.0, physics_dt=1.0/self.physics_freq, rendering_dt=1.0/self.render_freq)

        # robot
        self._go1 = self._world.scene.add(
            Unitree(
                prim_path="/World/Robot/Go1", name="Go1", position=np.array([0, 0, 1.0]), physics_dt=1.0/self.physics_freq, usd_path = PACKAGE_PATH + "/model/go1/go1.usd", model="Go1"
            )
        )

        # groundplane
        prim = get_prim_at_path("/World")
        if not prim.IsValid():
            prim = define_prim("/World", "Xform")
        asset_path = PACKAGE_PATH + "/model/go1/defaultGroundPlane.usd"
        prim.GetReferences().AddReference(asset_path)

        # physics materials
        if not prim.IsValid():
            prim = define_prim("/World", "Xform")
        asset_path = PACKAGE_PATH + "/model/go1/Physics_Materials.usd"
        prim.GetReferences().AddReference(asset_path)


        # Get the prim you want to move
        stage = omni.usd.get_context().get_stage()

        # Get go1
        go1_prim = stage.GetPrimAtPath("/World/Go1")
        self.xform_go1 = UsdGeom.Xformable(go1_prim)

        # Get base of go1
        go1_base_prim = stage.GetPrimAtPath("/World/Go1/base")
        self.xform_go1_base = UsdGeom.Xformable(go1_base_prim)
        
        # 
        self._world.reset()

        # 
        self._enter_toggled = 0
        self._base_command = [0.0, 0.0, 0.0, 0]
        self._event_flag = False
        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [1.8*2, 0.0, 0.0],
            "UP": [1.8*2, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-1.8*2, 0.0, 0.0],
            "DOWN": [-1.8*2, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -1.8*2, 0.0],
            "RIGHT": [0.0, -1.8*2, 0.0],
            # right command
            "NUMPAD_4": [0.0, 1.8*2, 0.0],
            "LEFT": [0.0, 1.8*2, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 1.0*2],
            "N": [0.0, 0.0, 1.0*2],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -1.0*2],
            "M": [0.0, 0.0, -1.0*2],
        }

        # =-=-=-=-=-=-= ROS =-=-=-=-=-=-=-=
        try:
            if self.cfg_og["ros_clock_publisher"]["enable"]:
                self._clock_graph = ros_clock_publisher()
                self._clock_publisher_iter = 0

            if self.cfg_og["ros_joints_tf_publisher"]["enable"]:
                self._joint_tf_graph = ros_joints_tf_publisher()
                self._tf_publisher_iter = 0

            if self.cfg_og["ros_joint_state_publisher"]["enable"]:
                self._joint_state_graph = ros_joint_state_publisher()
                self._joint_state_publisher_iter = 0

            if self.cfg_og["ros_imu_publisher"]["enable"]:
                self._imu_graph   = ros_imu_publisher()
                self._imu_publisher_iter = 0

            if self.cfg_og["ros_foot_contact_publisher"]["enable"]:
                self._foot_force_graph = ros_foot_contact_publisher()
                self._foot_force_publisher_iter = 0

        except Exception as e:
            print(e)
            simulation_app.close()
            exit()

        self._pub = rospy.Publisher("/isaac_a1/output", Float32MultiArray, queue_size=10)
        return

    def setup(self) -> None:
        """
        [Summary]

        Set unitree robot's default stance, set up keyboard listener and add physics callback

        """
        self._go1.set_state(self._go1._default_a1_state)
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        self._world.add_physics_callback("a1_advance", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        """
        [Summary]

        Physics call back, switch robot mode and call robot advance function to compute and apply joint torque

        """
        if self._event_flag:
            self._go1._qp_controller.switch_mode()
            self._event_flag = False

        if self.physics_freq - self.control_freq == self._clock_control_iter:
            step_size = 1.0 / self.control_freq
            self._go1.advance(step_size, self._base_command)
            self._clock_control_iter = 0
        else:
            self._clock_control_iter += self.control_freq

        # Tick omnigraph
        if self.physics_freq - self.cfg_og["ros_clock_publisher"]["freq"] == self._clock_publisher_iter:
            og.Controller.evaluate_sync(self._clock_graph)
            self._clock_publisher_iter = 0
        else:
            self._clock_publisher_iter += self.cfg_og["ros_clock_publisher"]["freq"]
            
        if self.physics_freq - self.cfg_og["ros_joints_tf_publisher"]["freq"] == self._tf_publisher_iter:
            og.Controller.evaluate_sync(self._joint_tf_graph)
            self._tf_publisher_iter = 0
        else:
            self._tf_publisher_iter += self.cfg_og["ros_joints_tf_publisher"]["freq"]

        if self.physics_freq - self.cfg_og["ros_joint_state_publisher"]["freq"] == self._joint_state_publisher_iter:
            og.Controller.evaluate_sync(self._joint_state_graph)
            self._joint_state_publisher_iter = 0
        else:
            self._joint_state_publisher_iter += self.cfg_og["ros_joint_state_publisher"]["freq"]

        if self.physics_freq - self.cfg_og["ros_imu_publisher"]["freq"] == self._imu_publisher_iter:
            og.Controller.evaluate_sync(self._imu_graph)
            self._imu_publisher_iter = 0
        else:
            self._imu_publisher_iter += self.cfg_og["ros_imu_publisher"]["freq"]

        if self.physics_freq - self.cfg_og["ros_foot_contact_publisher"]["freq"] == self._foot_force_publisher_iter:
            og.Controller.evaluate_sync(self._foot_force_graph)
            self._foot_force_publisher_iter = 0
        else:
            self._foot_force_publisher_iter += self.cfg_og["ros_foot_contact_publisher"]["freq"]

        self._pub.publish(Float32MultiArray(data=self.get_footforce_data()))

    def get_footforce_data(self) -> np.array:
        """
        [Summary]

        get foot force and position data
        """
        data = np.concatenate((self._go1.foot_force, self._go1._qp_controller._ctrl_states._foot_pos_abs[:, 2]))
        return data

    def run(self) -> None:
        """
        [Summary]

        Step simulation based on rendering downtime

        """
        # change to sim running
        while simulation_app.is_running():
            self._world.step(render=True)
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> None:
        """
        [Summary]

        Subscriber callback to when kit is updated.

        """
        # reset event
        self._event_flag = False
        # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
        print(" -- keyboard input detected ")
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            print(" -- keyboard input detected ")

            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] += np.array(self._input_keyboard_mapping[event.input.name])
                self._event_flag = True

            # enter, toggle the last command
            if event.input.name == "ENTER" and self._enter_toggled is False:
                self._enter_toggled = True
                if self._base_command[3] == 0:
                    self._base_command[3] = 1
                else:
                    self._base_command[3] = 0
                self._event_flag = True

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            print(" -- keyboard release detected ")

            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] -= np.array(self._input_keyboard_mapping[event.input.name])
                self._event_flag = True
            # enter, toggle the last command
            if event.input.name == "ENTER":
                self._enter_toggled = False
        # since no error, we are fine :)
        return True


def main() -> None:
    rospy.init_node("go1_standalone", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    runner = Go1_runner()
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
