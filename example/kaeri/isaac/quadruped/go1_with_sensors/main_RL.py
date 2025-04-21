import argparse

# orbit
from omni.isaac.orbit.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Flat-Unitree-Go1-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# [isaac sim]
import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.api.world import World
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim import create_prim
from omni.isaac.quadruped.robots import Unitree
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.utils.extensions import get_extension_path_from_name

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")
enable_extension("omni.kaeri.ros_bridge")

# [orbit]
import gymnasium as gym
import os
import torch
import traceback

import carb
# from rsl_rl.runners import OnPolicyRunner

import omni.isaac.contrib_tasks  # noqa: F401
import omni.isaac.orbit_tasks  # noqa: F401
from omni.isaac.orbit_tasks.utils import get_checkpoint_path, parse_env_cfg
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_onnx,
)

# from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import 

# [ROS]
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

import rospy
import tf
from geometry_msgs.msg import Twist

# ros use simulation time
rospy.set_param("use_sim_time", True)

import os
import yaml

# get $PACKAGE_PATH
PACKAGE_PATH = os.environ["PACKAGE_PATH"]

# add $PACKAGE_PATH to python path
import sys
sys.path.append(PACKAGE_PATH)

# [custom]
from example.kaeri.isaac.quadruped.go1_with_sensors.orbit2isaac.go1_cfg import GO1_CFG
from example.kaeri.isaac.quadruped.go1_with_sensors.orbit2isaac.articulation import Articulation
from example.kaeri.isaac.quadruped.go1_with_sensors.orbit2isaac.rsl_rl_bridge import RslRLBridge
from example.kaeri.isaac.quadruped.go1_with_sensors.orbit2isaac.on_policy_runner import OnPolicyRunner
from example.kaeri.isaac.quadruped.go1_with_sensors.ros_publisher import *

class Go1_runner(object):
    def __init__(self) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a unitree go1 robot
        """

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/quadruped/go1_with_sensors/config.yaml", "r") as f:
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

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [unitree go1] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        create_prim("/World/Robot", "Xform", translation=[0.0, 0.0, 0.05])
        self._go1 = Articulation(GO1_CFG.replace(prim_path="/World/Robot/Go1"))

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [rsl-rl] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # parse configuration
        env_cfg = parse_env_cfg(args_cli.task, use_gpu=not args_cli.cpu, num_envs=args_cli.num_envs)
        agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)

        # load previously trained model
        ppo_runner = OnPolicyRunner(agent_cfg.to_dict(), 
                                    num_obs=48, 
                                    num_critic_obs=48, 
                                    num_actions=12, 
                                    num_envs=1, 
                                    device=agent_cfg.device)
        ppo_runner.load(PACKAGE_PATH + "/model/go1/rl_model/2024-06-26_11-28-32/model_299.pt")
        
        # obtain the trained policy for inference
        self.policy = ppo_runner.get_inference_policy(device="cpu") # or "cuda:0"

        # previous actions
        self.prev_action = torch.zeros(12) # 초기 행동 벡터 설정

        # =-=-=-=-=-=-=-=-=-= [rsl_rl_bridge] =-=-=-=-=-=-=-=-=-=-=-=-=-
        self.rsl_rl_bridge = RslRLBridge()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [reset] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world.reset()

        # =-=-=-=-=-=-=-=-==-= [ sensors ] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=    
        # lidar
        asset_path = PACKAGE_PATH + "/model/sensors/lidar_wo_vis.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Sensors")

        # d455 front
        asset_path = PACKAGE_PATH + "/model/sensors/Realsense_D455_1.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Sensors")

        # d455 rear
        asset_path = PACKAGE_PATH + "/model/sensors/Realsense_D455_2.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Sensors")

        # d455 left
        asset_path = PACKAGE_PATH + "/model/sensors/Realsense_D455_3.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Sensors")

        # d455 right
        asset_path = PACKAGE_PATH + "/model/sensors/Realsense_D455_4.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Sensors")


        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [joints] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # go1 trunk
        prim_go1_trunk = self._world.scene.stage.GetPrimAtPath("/World/Robot/Go1/trunk")

        # [set fixed joint for lidar]
        lidar_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/lidar")

        fixed_joint_go1_to_lidar = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_lidar")
        fixed_joint_go1_to_lidar.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
        fixed_joint_go1_to_lidar.CreateBody1Rel().SetTargets([lidar_prim.GetPrimPath()])

        fixed_joint_go1_to_lidar.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.2))
        fixed_joint_go1_to_lidar.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_lidar.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))
        fixed_joint_go1_to_lidar.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)


        # [set fixed joint for d455 front]
        d455_front_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_1/RSD455/Camera_Pseudo_Depth")

        fixed_joint_go1_to_D455_front = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_front")
        fixed_joint_go1_to_D455_front.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
        fixed_joint_go1_to_D455_front.CreateBody1Rel().SetTargets([d455_front_mount_prim.GetPrimPath()])

        fixed_joint_go1_to_D455_front.CreateLocalPos0Attr().Set(Gf.Vec3f(0.3, 0, 0.1))
        fixed_joint_go1_to_D455_front.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_D455_front.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([80, 0, -90]), degrees=True)))
        fixed_joint_go1_to_D455_front.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        # [set fixed joint for d455 rear]
        d455_rear_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_2/RSD455/Camera_Pseudo_Depth")

        fixed_joint_go1_to_D455_rear = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_rear")
        fixed_joint_go1_to_D455_rear.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
        fixed_joint_go1_to_D455_rear.CreateBody1Rel().SetTargets([d455_rear_mount_prim.GetPrimPath()])

        fixed_joint_go1_to_D455_rear.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.2, 0.0, 0.1))
        fixed_joint_go1_to_D455_rear.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_D455_rear.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([80, 0, 90]), degrees=True)))
        fixed_joint_go1_to_D455_rear.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        # [set fixed joint for d455 left]
        d455_left_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_3/RSD455/Camera_Pseudo_Depth")

        fixed_joint_go1_to_D455_left = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_left")
        fixed_joint_go1_to_D455_left.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
        fixed_joint_go1_to_D455_left.CreateBody1Rel().SetTargets([d455_left_mount_prim.GetPrimPath()])

        fixed_joint_go1_to_D455_left.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.1, 0.1))
        fixed_joint_go1_to_D455_left.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_D455_left.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([80, 0, 0]), degrees=True)))
        fixed_joint_go1_to_D455_left.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        # [set fixed joint for d455 right]
        d455_right_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_4/RSD455/Camera_Pseudo_Depth")

        fixed_joint_go1_to_D455_right = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_right")
        fixed_joint_go1_to_D455_right.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
        fixed_joint_go1_to_D455_right.CreateBody1Rel().SetTargets([d455_right_mount_prim.GetPrimPath()])

        fixed_joint_go1_to_D455_right.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, -0.1, 0.1))
        fixed_joint_go1_to_D455_right.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_D455_right.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([80, 0, 180]), degrees=True)))
        fixed_joint_go1_to_D455_right.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [omnigraph] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        try:
            if self.cfg_og["ros_clock_publisher"]["enable"]:
                self._clock_graph = ros_clock_publisher()                
                self._clock_publisher_iter = 0

            if self.cfg_og["ros_joints_tf_publisher"]["enable"]:
                bodies = ["/World/Robot/Go1/trunk"]
                self._joint_tf_graph = ros_joints_tf_publisher(bodies)
                self._tf_publisher_iter = 0

            if self.cfg_og["ros_joint_state_publisher"]["enable"]:
                self._joint_state_graph = ros_joint_state_publisher()
                self._joint_state_publisher_iter = 0

            if self.cfg_og["ros_sensors_tf_publisher"]["enable"]:
                self._sensors_tf_graph = ros_sensors_tf_publisher()
                self._sensors_tf_publisher_iter = 0

            if self.cfg_og["ros_imu_publisher"]["enable"]:
                self._imu_graph = ros_imu_publisher()
                self._imu_publisher_iter = 0

            if self.cfg_og["ros_odom_publisher"]["enable"]:
                self._odom_graph = ros_odom_publisher()
                self._odom_publisher_iter = 0

            if self.cfg_og["ros_foot_contact_publisher"]["enable"]:
                self._foot_force_graph = ros_foot_contact_publisher()
                self._foot_force_publisher_iter = 0

            if self.cfg_og["ros_lidar_publisher"]["enable"]:
                self._lidar_graph = ros_lidar_publisher(frame_id = "lidar", 
                                                        topic_name = self.cfg_og["ros_lidar_publisher"]["topic"], 
                                                        lidar_prim = "/World/Sensors/lidar")
                self._lidar_publisher_iter = 0

            if self.cfg_og["ros_d455_publisher"]["enable"]:
                self._d455_graph_front = ros_d455_publisher(frame_id = "d455_front" , 
                                                            namespace="d455_front", 
                                                            topic_name = self.cfg_og["ros_d455_publisher"]["topic"], 
                                                            depth_prim="/World/Sensors/Realsense_D455_1/RSD455/Camera_Pseudo_Depth",
                                                            rgb_prim="/World/Sensors/Realsense_D455_1/RSD455/Camera_OmniVision_OV9782_Color")
                self._d455_publisher_iter = 0

            if self.cfg_og["ros_d455_publisher"]["enable"]:
                self._d455_graph_rear = ros_d455_publisher(frame_id = "d455_rear" , 
                                                           namespace="d455_rear", 
                                                           topic_name = self.cfg_og["ros_d455_publisher"]["topic"],
                                                           depth_prim="/World/Sensors/Realsense_D455_2/RSD455/Camera_Pseudo_Depth",
                                                           rgb_prim="/World/Sensors/Realsense_D455_2/RSD455/Camera_OmniVision_OV9782_Color")

                self._d455_publisher_iter = 0

            if self.cfg_og["ros_d455_publisher"]["enable"]:
                self._d455_graph_left = ros_d455_publisher(frame_id = "d455_left" , 
                                                           namespace = "d455_left", 
                                                           topic_name = self.cfg_og["ros_d455_publisher"]["topic"],
                                                           depth_prim = "/World/Sensors/Realsense_D455_3/RSD455/Camera_Pseudo_Depth",
                                                           rgb_prim = "/World/Sensors/Realsense_D455_3/RSD455/Camera_OmniVision_OV9782_Color")

                self._d455_publisher_iter = 0

            if self.cfg_og["ros_d455_publisher"]["enable"]:
                self._d455_graph_right = ros_d455_publisher(frame_id = "d455_right" ,
                                                            namespace = "d455_right", 
                                                            topic_name = self.cfg_og["ros_d455_publisher"]["topic"],
                                                            depth_prim="/World/Sensors/Realsense_D455_4/RSD455/Camera_Pseudo_Depth",
                                                            rgb_prim="/World/Sensors/Realsense_D455_4/RSD455/Camera_OmniVision_OV9782_Color")

                self._d455_publisher_iter = 0

            
        except Exception as e:
            print(e)
            simulation_app.close()
            exit()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [publisher] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # tf broadcaster
        self._tf_trunk_to_d455_front = tf.TransformBroadcaster()
        self._tf_trunk_to_d455_rear = tf.TransformBroadcaster()
        self._tf_trunk_to_d455_left = tf.TransformBroadcaster()
        self._tf_trunk_to_d455_right = tf.TransformBroadcaster()
        self._tf_base_link_to_trunk = tf.TransformBroadcaster()

        
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [subscriber] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        self.cmd_vel = [0.0, 0.0, 0.0]

        self.i= 0
        return

    def cmd_vel_cb(self, msg):
        self.cmd_vel = [msg.linear.x, msg.linear.y, msg.angular.z]
        # self.cmd_vel = [self.cmd_vel[0]*2, self.cmd_vel[1]*2, self.cmd_vel[2]*2]
        self._base_command = [*self.cmd_vel, 1]

    def setup(self) -> None:
        """
        [Summary]

        Set unitree robot's default stance, set up keyboard listener and add physics callback

        """
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("go1_advance", callback_fn=self.on_physics_step)
        self._world.add_timeline_callback

    def on_physics_step(self, step_size) -> None:
        # broadcast tf from base_link to trunk
        self._tf_base_link_to_trunk.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "trunk", "base_link")

        if self.physics_freq - self.control_freq == self._clock_control_iter:
            step_size = 1.0 / self.control_freq

            # 1. observation
            obs = self.rsl_rl_bridge.gather_observation(self._go1, self.cmd_vel, self.prev_action)

            # 2. inference
            action = self.policy(obs)
            self.prev_action = action
            action = action / 4.0

            action += torch.tensor([0.1, -0.1,  0.1, -0.1, 
                                    0.8,  0.8,  1.0,  1.0, 
                                    -1.5, -1.5, -1.5, -1.5])
            
            action.detach().numpy()

            # 3. action
            self._go1.set_joint_position_target(torch.tensor(action))
            self._go1.write_data_to_sim()
            self._go1.update(step_size)
            
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
        
        if self.cfg_og["ros_joints_tf_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_joints_tf_publisher"]["freq"] == self._tf_publisher_iter:
            og.Controller.evaluate_sync(self._joint_tf_graph)

            # ros clock time
            time = rospy.Time.now()
            
            # [-120, 0, -90] to quaternion
            self._tf_trunk_to_d455_front.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-1.5707, 0, 1.5707), rospy.Time.now(), "d455_front", "RSD455")
            # [-120, 0.0, 90] to quaternion
            self._tf_trunk_to_d455_rear.sendTransform((0, 0.0, 0), tf.transformations.quaternion_from_euler(-1.5707, 0, 1.5707), rospy.Time.now(), "d455_rear", "World_Sensors_Realsense_D455_2_RSD455")
            # [-120, 0, 0] to quaternion
            self._tf_trunk_to_d455_left.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-1.5707, 0, 1.5707), rospy.Time.now(), "d455_left", "World_Sensors_Realsense_D455_3_RSD455")
            # [-120, 0.0, 180] to quaternion
            self._tf_trunk_to_d455_right.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-1.5707, 0, 1.5707), rospy.Time.now(), "d455_right", "World_Sensors_Realsense_D455_4_RSD455")

            self._tf_publisher_iter = 0
        else:
            self._tf_publisher_iter += self.cfg_og["ros_joints_tf_publisher"]["freq"]

        if self.cfg_og["ros_sensors_tf_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_sensors_tf_publisher"]["freq"] == self._sensors_tf_publisher_iter:
            og.Controller.evaluate_sync(self._sensors_tf_graph)
            self._sensors_tf_publisher_iter = 0
        else:
            self._sensors_tf_publisher_iter += self.cfg_og["ros_sensors_tf_publisher"]["freq"]

        if self.cfg_og["ros_joint_state_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_joint_state_publisher"]["freq"] == self._joint_state_publisher_iter:
            og.Controller.evaluate_sync(self._joint_state_graph)
            self._joint_state_publisher_iter = 0
        else:
            self._joint_state_publisher_iter += self.cfg_og["ros_joint_state_publisher"]["freq"]

        if self.cfg_og["ros_imu_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_imu_publisher"]["freq"] == self._imu_publisher_iter:
            og.Controller.evaluate_sync(self._imu_graph)
            self._imu_publisher_iter = 0
        else:
            self._imu_publisher_iter += self.cfg_og["ros_imu_publisher"]["freq"]

        if self.cfg_og["ros_odom_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_odom_publisher"]["freq"] == self._odom_publisher_iter:
            og.Controller.evaluate_sync(self._odom_graph)
            self._odom_publisher_iter = 0
        else:
            self._odom_publisher_iter += self.cfg_og["ros_odom_publisher"]["freq"]

        if self.cfg_og["ros_foot_contact_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_foot_contact_publisher"]["freq"] == self._foot_force_publisher_iter:
            og.Controller.evaluate_sync(self._foot_force_graph)
            self._foot_force_publisher_iter = 0
        else:
            self._foot_force_publisher_iter += self.cfg_og["ros_foot_contact_publisher"]["freq"]
        
        if self.cfg_og["ros_lidar_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_lidar_publisher"]["freq"] == self._lidar_publisher_iter:
            og.Controller.evaluate_sync(self._lidar_graph)
            self._lidar_publisher_iter = 0
        else:
            self._lidar_publisher_iter += self.cfg_og["ros_lidar_publisher"]["freq"]

        if self.cfg_og["ros_d455_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_d455_publisher"]["freq"] == self._d455_publisher_iter:
            og.Controller.evaluate_sync(self._d455_graph_front)
            og.Controller.evaluate_sync(self._d455_graph_rear)
            og.Controller.evaluate_sync(self._d455_graph_left)
            og.Controller.evaluate_sync(self._d455_graph_right)
            self._d455_publisher_iter = 0
        else:
            self._d455_publisher_iter += self.cfg_og["ros_d455_publisher"]["freq"]



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
    rospy.set_param("use_sim_time", False)
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
