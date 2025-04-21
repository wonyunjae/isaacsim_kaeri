from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
from isaacsim.core.api.world import World
from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.utils.rotations import euler_angles_to_quat

import math

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
import tf

import os
import yaml

# custom manipulator related
from tasks.follow_target import FollowTarget
from tasks.ik_solver import KinematicsSolver

# get $PACKAGE_PATH
PACKAGE_PATH = os.environ["PACKAGE_PATH"]

# add $PACKAGE_PATH to python path
import sys
sys.path.append(PACKAGE_PATH)

class Openmanipulator_runner(object):
    def __init__(self) -> None:
        # =-=-=-=-=-=-= config =-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/manipulator/config/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_freq = config["control_freq"]
        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]

        self.cfg_og = config["omnigraph"]

        # clock for control freq
        self._clock_control_iter = 0

        # =-=-=-=-=-=-= World =-=-=-=-=-=-=
        self._world = World(stage_units_in_meters=1.0, physics_dt=1.0/self.physics_freq, rendering_dt=1.0/self.render_freq)
        
        # =-=-=-=-=-=-= Tasks =-=-=-=-=-=-=
        self.offset_1 = np.array([0.0, 0.0, 0.0])
        task_1 = FollowTarget(name="openmanipulator_p_1", 
                            target_position=np.array([0.4, 0, 0.1]), 
                            target_orientation=euler_angles_to_quat(np.array([0, np.pi/2.0, 0]), degrees=False),
                            target_prim_path="/World/TargetCube_1",
                            target_name="target_1",
                            offset=self.offset_1)
        self._world.add_task(task_1)

        self.offset_2 = np.array([0.0, 1.0, 0.0])
        task_2 = FollowTarget(name="openmanipulator_p_2", 
                            target_position=np.array([0.4, 0, 0.1]), 
                            target_orientation=euler_angles_to_quat(np.array([0, np.pi/2.0, 0]), degrees=False),
                            target_prim_path="/World/TargetCube_2",
                            target_name="target_2",
                            offset=self.offset_2)
        self._world.add_task(task_2)


        # Get the prim you want to move
        stage = omni.usd.get_context().get_stage()
        
        # =-=-=-=-=-=-= Reset =-=-=-=-=-=-=
        self._world.reset()

        # =-=-=-=-=-=-= task related =-=-=-=-=-=-=
        task_params_1 = self._world.get_task("openmanipulator_p_1").get_params()
        self.target_name_1 = task_params_1["target_name"]["value"]
        self.robot_name_1 = task_params_1["robot_name"]["value"]
        my_robot_1 = self._world.scene.get_object(self.robot_name_1)

        self.my_controller_1 = KinematicsSolver(robot_articulation = my_robot_1, 
                                              end_effector_frame_name = "end_link", 
                                              robot_description_path = PACKAGE_PATH + "/model/manipulator/openmanipulator_p/openmanipulator_p_descriptor.yaml",
                                              urdf_path = PACKAGE_PATH + "/model/manipulator/openmanipulator_p/openmanipulator_p.urdf"
        )
        self.articulation_controller_1 = my_robot_1.get_articulation_controller()

        task_params_2 = self._world.get_task("openmanipulator_p_2").get_params()
        self.target_name_2 = task_params_2["target_name"]["value"]
        self.robot_name_2 = task_params_2["robot_name"]["value"]
        my_robot_2 = self._world.scene.get_object(self.robot_name_2)

        self.my_controller_2 = KinematicsSolver(robot_articulation = my_robot_2,
                                                end_effector_frame_name = "end_link", 
                                                robot_description_path = PACKAGE_PATH + "/model/manipulator/openmanipulator_p/openmanipulator_p_descriptor.yaml",
                                                urdf_path = PACKAGE_PATH + "/model/manipulator/openmanipulator_p/openmanipulator_p.urdf"
        )
        self.articulation_controller_2 = my_robot_2.get_articulation_controller()

        # =-=-=-=-=-=-= ROS =-=-=-=-=-=-=-=
        try:
            pass

        except Exception as e:
            print(e)
            simulation_app.close()
            exit()

        return

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("openmanipulator", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        observations = self._world.get_observations()
        
        # robot 1 control
        actions, succ = self.my_controller_1.compute_inverse_kinematics(
            target_position=observations[self.target_name_1]["position"] - self.offset_1,
            target_orientation=observations[self.target_name_1]["orientation"],
            position_tolerance=0.001,
            orientation_tolerance=0.001 # pi
        )

        if succ:
            self.articulation_controller_1.apply_action(actions)
        else:
            # self.articulation_controller_1.apply_action(actions)
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

        # robot 2 control
        actions, succ = self.my_controller_2.compute_inverse_kinematics(
            target_position=observations[self.target_name_2]["position"] - self.offset_2,
            target_orientation=observations[self.target_name_2]["orientation"],
            position_tolerance=0.001,
            orientation_tolerance=0.001 # pi
        )

        if succ:
            self.articulation_controller_2.apply_action(actions)
        else:
            # self.articulation_controller_2.apply_action(actions)
            carb.log_warn("IK did not converge to a solution.  No action is being taken.")

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
    rospy.init_node("openmanipulator", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    runner = Openmanipulator_runner()
    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()
    runner._world.reset()
    runner.run()
    rospy.signal_shutdown("openmanipulator complete")
    simulation_app.close()

if __name__ == "__main__":
    main()
