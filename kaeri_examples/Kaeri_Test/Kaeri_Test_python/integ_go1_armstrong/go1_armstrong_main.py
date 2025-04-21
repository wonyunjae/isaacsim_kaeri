import os
import sys

# 환경 변수 지정해줘야 함.
PACKAGE_PATH = os.environ["PACKAGE_PATH"] = "/isaac-sim/isaac_sim"
sys.path.append("/isaac-sim")

import argparse
from isaac_sim.example.kaeri.isaac.quadruped.go1_with_sensors import cli_args

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Flat-Unitree-Go1-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")

# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
# 기존 코드: 
# args_cli = parser.parse_args()
args_cli, unknown = parser.parse_known_args()

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim 
# from omni.isaac.quadruped.robots import Unitree
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.utils.extensions import get_extension_path_from_name
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.api.robots import Robot
import yaml

# [orbit]
import gymnasium as gym
import torch
import traceback

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import get_checkpoint_path, parse_env_cfg
from isaaclab_rl.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper,
    export_policy_as_onnx
)
# enable ROS bridge extensions
enable_extension("isaacsim.ros1.bridge")
enable_extension("omni.kaeri.ros_bridge")

# [ROS]
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    # simulation_app.close()
    exit()

import rospy
import tf
from geometry_msgs.msg import Twist

# [Custom]
from isaaclab_assets.robots.unitree import UNITREE_GO1_CFG  # Go1용
from isaaclab.assets import Articulation 
from rsl_rl.runners.on_policy_runner import OnPolicyRunner
# from isaaclab.bridges.ros import RosPublisher 
from isaac_sim.example.kaeri.isaac.integ_go1_armstrong.ros_publisher_go1 import *
from isaac_sim.example.kaeri.isaac.integ_go1_armstrong.ros_publisher_armstrong import *

from isaacsim.core.api.world import World
from Kaeri_Test_python.kaeri_base_sample import BaseSample

# Custom bridge implementation to provide the necessary functionality
class RslRlBridge:
    def gather_observation(self, articulation, cmd_vel, prev_action):
        # Base observation states from the articulation
        base_state = articulation.get_world_pose()
        base_lin_vel = articulation.get_linear_velocity()
        base_ang_vel = articulation.get_angular_velocity()
        joint_pos = articulation.get_joint_positions()
        joint_vel = articulation.get_joint_velocities()
        
        # Convert command velocities to tensor if needed
        command = torch.tensor([cmd_vel[0], cmd_vel[1], cmd_vel[2]], dtype=torch.float)
        
        # Combine all observations into a single tensor
        obs = torch.cat([
            base_lin_vel, base_ang_vel, 
            joint_pos, joint_vel, 
            command, prev_action
        ], dim=0)
        
        return obs

class Main(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        with open(PACKAGE_PATH + "/example/kaeri/isaac/integ_go1_armstrong/config_master.yaml", "r") as f:
            config_master = yaml.load(f, Loader=yaml.FullLoader)

        self.control_freq = config_master["control_freq"]
        self.physics_freq = config_master["physics_freq"]
        self.render_freq  = config_master["render_freq"]

        # go1- load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/integ_go1_armstrong/config_go1.yaml", "r") as f:
            config_go1 = yaml.load(f, Loader=yaml.FullLoader)
        self.cfg_og_go1 = config_go1["omnigraph"]

        # armstrong- load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/integ_go1_armstrong/config_armstrong.yaml", "r") as f:
            config_armstrong = yaml.load(f, Loader=yaml.FullLoader)
        self.cfg_og_armstrong = config_armstrong["omnigraph"]

        self._world_settings = {"physics_dt": 1.0/self.physics_freq, "stage_units_in_meters": 1.0, "rendering_dt": 1.0/self.render_freq}

        # clock for control freq
        self._clock_control_iter_go1 = 0

        self.cmd_vel_go1 = [0.0, 0.0, 0.0]
        self.cmd_vel_armstrong = [0.0, 0.0, 0.0]
        self.armstrong_yaw = 0

        # previous actions
        self.prev_action = torch.zeros(12) # 초기 행동 벡터 설정

        # In the initialization:
        self.rsl_rl_bridge = RslRlBridge()
        return  

    def setup_scene(self):
        if not rospy.core.is_initialized():
            rospy.init_node("isaac", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
        rospy.set_param("use_sim_time", True)
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [subscriber] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        rospy.Subscriber("/go1/cmd_vel", Twist, self.cmd_vel_go1_cb)
        rospy.Subscriber("/armstrong/cmd_vel", Twist, self.cmd_vel_armstrong_cb)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = self.get_world()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/environment/Warehouse.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [unitree go1] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        create_prim("/World/Robot", "Xform", translation=[0.0, 0.0, 0.05])
        self._go1 = Articulation(GO1_CFG.replace(prim_path="/World/Robot/Go1"))

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [armstrong] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
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
        xform = UsdGeom.Xformable(armstrong_prim)   # 옴니버스에 있는 물체들의 위치를 설명하는 요소들
        self.armstrong_transform = xform.AddTransformOp()

        # Set the new location & rotation
        new_location = Gf.Vec3d(0.0, -3.0, 0.25)  # Replace with your desired location
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0)  # Replace with your desired rotation

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.armstrong_transform.Set(mat4d)

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

        # =-=-=-=-=-=-=-=-=-= [rsl_rl_bridge] =-=-=-=-=-=-=-=-=-=-=-=-=-
        self.rsl_rl_bridge = RslRLBridge()

        # =-=-=-=-=-=-=-=-==-= [ sensors - go1 ] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=    
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

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [sensors - armstrong] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # d455 
        asset_path = PACKAGE_PATH + "/model/sensors/Realsense_D455_5.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Sensors")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [publisher] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # tf broadcaster
        self._tf_trunk_to_d455_front = tf.TransformBroadcaster()
        self._tf_trunk_to_d455_rear = tf.TransformBroadcaster()
        self._tf_trunk_to_d455_left = tf.TransformBroadcaster()
        self._tf_trunk_to_d455_right = tf.TransformBroadcaster()
        self._tf_base_link_to_trunk = tf.TransformBroadcaster()

        # tf broadcaster - armstrong
        self._tf_trunk_to_d455_front_armstrong = tf.TransformBroadcaster()
        self._tf_base_link_to_armstrong = tf.TransformBroadcaster()

        return

    def cmd_vel_go1_cb(self, msg):
        self.cmd_vel_go1 = [msg.linear.x, msg.linear.y, msg.angular.z]

    def cmd_vel_armstrong_cb(self, msg):
        self.cmd_vel_armstrong = [msg.linear.x, msg.linear.y, msg.angular.z]

    async def setup_post_load(self):
        self._world = self.get_world()
        self._world.add_physics_callback("go1_advance", callback_fn=self.on_physics_step)
        return
    
    def set_position_armstrong(self, dt):
        # get current location
        transform = self.armstrong_transform.Get()

        pos = transform.ExtractTranslation() # Gf.Vec3d
        rot = transform.ExtractRotation() # Gf.Rotation

        # Set the new rotation
        self.armstrong_yaw+= self.cmd_vel_armstrong[2] * dt 
        
        # Set the new location
        new_location = pos
        # consider yaw
        new_location[0] = pos[0] + self.cmd_vel_armstrong[0] * dt * np.cos(self.armstrong_yaw) - self.cmd_vel_armstrong[1] * dt * np.sin(self.armstrong_yaw)
        new_location[1] = pos[1] + self.cmd_vel_armstrong[1] * dt * np.cos(self.armstrong_yaw) + self.cmd_vel_armstrong[0] * dt * np.sin(self.armstrong_yaw)
        # new_location[2] = 0.5
        
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), self.armstrong_yaw* 180.0 / np.pi)

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.armstrong_transform.Set(mat4d)

    def on_physics_step(self, step_size) -> None:
        # ------- armstrong -
        self.set_position_armstrong(1.0/self.physics_freq)

        if self.physics_freq - self.control_freq == self._clock_control_iter_go1:
            step_size = 1.0 / self.control_freq
            
            # ----- go1-
            # 1. observation
            obs = self.rsl_rl_bridge.gather_observation(self._go1, self.cmd_vel_go1, self.prev_action)

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


            
            self._clock_control_iter_go1 = 0
        else:
            self._clock_control_iter_go1 += self.control_freq


        # ========= Tick omnigraph - go1
        if self.cfg_og_go1["ros_clock_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_clock_publisher"]["freq"] == self._clock_publisher_iter:
            og.Controller.evaluate_sync(self._clock_graph)
            self._clock_publisher_iter = 0
        else:
            self._clock_publisher_iter += self.cfg_og_go1["ros_clock_publisher"]["freq"]
        
        if self.cfg_og_go1["ros_joints_tf_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_joints_tf_publisher"]["freq"] == self._tf_publisher_iter:
            og.Controller.evaluate_sync(self._joint_tf_graph)

            # ros clock time
            time = rospy.Time.now()
            
            # [-120, 0, -90] to quaternion
            self._tf_trunk_to_d455_front.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-1.5708, 0, 1.5708), rospy.Time.now(), "go1/d455_front", "go1/RSD455")
            # [-120, 0.0, 90] to quaternion
            self._tf_trunk_to_d455_rear.sendTransform((0, 0.0, 0), tf.transformations.quaternion_from_euler(-1.5708, 0, 1.5708), rospy.Time.now(), "go1/d455_rear", "go1/World_Sensors_Realsense_D455_2_RSD455")
            # [-120, 0, 0] to quaternion
            self._tf_trunk_to_d455_left.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-1.5708, 0, 1.5708), rospy.Time.now(), "go1/d455_left", "go1/World_Sensors_Realsense_D455_3_RSD455")
            # [-120, 0.0, 180] to quaternion
            self._tf_trunk_to_d455_right.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-1.5708, 0, 1.5708), rospy.Time.now(), "go1/d455_right", "go1/World_Sensors_Realsense_D455_4_RSD455")
            # broadcast tf from base_link to trunk
            self._tf_base_link_to_trunk.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "go1/trunk", "go1/base_link")
            self._tf_publisher_iter = 0
        else:
            self._tf_publisher_iter += self.cfg_og_go1["ros_joints_tf_publisher"]["freq"]

        if self.cfg_og_go1["ros_sensors_tf_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_sensors_tf_publisher"]["freq"] == self._sensors_tf_publisher_iter:
            og.Controller.evaluate_sync(self._sensors_tf_graph)
            self._sensors_tf_publisher_iter = 0
        else:
            self._sensors_tf_publisher_iter += self.cfg_og_go1["ros_sensors_tf_publisher"]["freq"]

        if self.cfg_og_go1["ros_joint_state_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_joint_state_publisher"]["freq"] == self._joint_state_publisher_iter:
            og.Controller.evaluate_sync(self._joint_state_graph)
            self._joint_state_publisher_iter = 0
        else:
            self._joint_state_publisher_iter += self.cfg_og_go1["ros_joint_state_publisher"]["freq"]

        if self.cfg_og_go1["ros_imu_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_imu_publisher"]["freq"] == self._imu_publisher_iter:
            og.Controller.evaluate_sync(self._imu_graph)
            self._imu_publisher_iter = 0
        else:
            self._imu_publisher_iter += self.cfg_og_go1["ros_imu_publisher"]["freq"]

        if self.cfg_og_go1["ros_odom_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_odom_publisher"]["freq"] == self._odom_publisher_iter:
            og.Controller.evaluate_sync(self._odom_graph)
            self._odom_publisher_iter = 0
        else:
            self._odom_publisher_iter += self.cfg_og_go1["ros_odom_publisher"]["freq"]

        if self.cfg_og_go1["ros_foot_contact_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_foot_contact_publisher"]["freq"] == self._foot_force_publisher_iter:
            og.Controller.evaluate_sync(self._foot_force_graph)
            self._foot_force_publisher_iter = 0
        else:
            self._foot_force_publisher_iter += self.cfg_og_go1["ros_foot_contact_publisher"]["freq"]
        
        if self.cfg_og_go1["ros_lidar_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_lidar_publisher"]["freq"] == self._lidar_publisher_iter:
            og.Controller.evaluate_sync(self._lidar_graph)
            self._lidar_publisher_iter = 0
        else:
            self._lidar_publisher_iter += self.cfg_og_go1["ros_lidar_publisher"]["freq"]

        if self.cfg_og_go1["ros_d455_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_go1["ros_d455_publisher"]["freq"] == self._d455_publisher_iter:
            og.Controller.evaluate_sync(self._d455_graph_front)
            og.Controller.evaluate_sync(self._d455_graph_rear)
            og.Controller.evaluate_sync(self._d455_graph_left)
            og.Controller.evaluate_sync(self._d455_graph_right)
            self._d455_publisher_iter = 0
        else:
            self._d455_publisher_iter += self.cfg_og_go1["ros_d455_publisher"]["freq"]

        # ========= Tick omnigraph - armstrong
        if self.cfg_og_armstrong["ros_joints_tf_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_armstrong["ros_joints_tf_publisher"]["freq"] == self._joint_state_publisher_iter_armstrong:
            og.Controller.evaluate_sync(self._joint_state_graph_armstrong)

            # [-120, 0, -90] to quaternion
            self._tf_trunk_to_d455_front_armstrong.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-1.5708, 0, 1.5708), rospy.Time.now(), "armstrong/d455_front", "armstrong/RSD455")
            
            self._tf_base_link_to_armstrong.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(1.5708, 0, 0), rospy.Time.now(), "armstrong/full_test", "armstrong/base_link")

            self._joint_state_publisher_iter_armstrong = 0
        else:
            self._joint_state_publisher_iter_armstrong += self.cfg_og_armstrong["ros_joints_tf_publisher"]["freq"]

        if self.cfg_og_armstrong["ros_d455_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og_armstrong["ros_d455_publisher"]["freq"] == self._d455_publisher_iter_armstrong:
            og.Controller.evaluate_sync(self._d455_graph_armstrong)
            self._d455_publisher_iter_armstrong = 0
        else:
            self._d455_publisher_iter_armstrong += self.cfg_og_armstrong["ros_d455_publisher"]["freq"]

    # Articulation 이후에 한 번의 리셋이 필요하기 때문에 여기에 joints와 omnigraph가 있어야 함.
    async def setup_pre_reset(self):
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [joints] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # go1 trunk
        prim_go1_trunk = self._world.stage.GetPrimAtPath("/World/Robot/Go1/trunk")

        # [set fixed joint for lidar]
        lidar_prim = self._world.stage.GetPrimAtPath("/World/Sensors/lidar")
        joint_path = "/World/FixedJoint/go1_to_lidar"

        fixed_joint_go1_to_lidar = self._world.stage.GetPrimAtPath(joint_path)
        if not fixed_joint_go1_to_lidar.IsValid():
            fixed_joint_go1_to_lidar = UsdPhysics.FixedJoint.Define(self._world.scene.stage, joint_path)
            fixed_joint_go1_to_lidar.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
            fixed_joint_go1_to_lidar.CreateBody1Rel().SetTargets([lidar_prim.GetPrimPath()])

            fixed_joint_go1_to_lidar.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.2))
            fixed_joint_go1_to_lidar.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

            fixed_joint_go1_to_lidar.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))
            fixed_joint_go1_to_lidar.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))
        # await self._world.reset_async()

        # [set fixed joint for d455 front]
        d455_front_mount_prim = self._world.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_1/RSD455/Camera_Pseudo_Depth")
        joint_path_front = "/World/FixedJoint/go1_to_D455_front"

        fixed_joint_go1_to_D455_front = self._world.stage.GetPrimAtPath(joint_path_front)
        if not fixed_joint_go1_to_D455_front.IsValid():
            fixed_joint_go1_to_D455_front = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_front")
            fixed_joint_go1_to_D455_front.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
            fixed_joint_go1_to_D455_front.CreateBody1Rel().SetTargets([d455_front_mount_prim.GetPrimPath()])

            fixed_joint_go1_to_D455_front.CreateLocalPos0Attr().Set(Gf.Vec3f(0.3, 0, 0.1))
            fixed_joint_go1_to_D455_front.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

            fixed_joint_go1_to_D455_front.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([80, 0, -90]), degrees=True)))
            fixed_joint_go1_to_D455_front.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # [set fixed joint for d455 rear]
        d455_rear_mount_prim = self._world.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_2/RSD455/Camera_Pseudo_Depth")
        joint_path_rear = "/World/FixedJoint/go1_to_D455_rear"

        fixed_joint_go1_to_D455_rear = self._world.stage.GetPrimAtPath(joint_path_rear)
        if not fixed_joint_go1_to_D455_rear.IsValid():
            fixed_joint_go1_to_D455_rear = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_rear")
            fixed_joint_go1_to_D455_rear.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
            fixed_joint_go1_to_D455_rear.CreateBody1Rel().SetTargets([d455_rear_mount_prim.GetPrimPath()])

            fixed_joint_go1_to_D455_rear.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.2, 0.0, 0.1))
            fixed_joint_go1_to_D455_rear.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

            fixed_joint_go1_to_D455_rear.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([80, 0, 90]), degrees=True)))
            fixed_joint_go1_to_D455_rear.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # [set fixed joint for d455 left]
        d455_left_mount_prim = self._world.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_3/RSD455/Camera_Pseudo_Depth")
        joint_path_left = "/World/FixedJoint/go1_to_D455_left"

        fixed_joint_go1_to_D455_left = self._world.stage.GetPrimAtPath(joint_path_left)
        if not fixed_joint_go1_to_D455_left.IsValid():
            fixed_joint_go1_to_D455_left = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_left")
            fixed_joint_go1_to_D455_left.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
            fixed_joint_go1_to_D455_left.CreateBody1Rel().SetTargets([d455_left_mount_prim.GetPrimPath()])

            fixed_joint_go1_to_D455_left.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.1, 0.1))
            fixed_joint_go1_to_D455_left.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

            fixed_joint_go1_to_D455_left.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([80, 0, 0]), degrees=True)))
            fixed_joint_go1_to_D455_left.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # [set fixed joint for d455 right]
        d455_right_mount_prim = self._world.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_4/RSD455/Camera_Pseudo_Depth")
        joint_path_right = "/World/FixedJoint/go1_to_D455_right"

        fixed_joint_go1_to_D455_right = self._world.stage.GetPrimAtPath(joint_path_right)
        if not fixed_joint_go1_to_D455_right.IsValid():
            fixed_joint_go1_to_D455_right = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_right")
            fixed_joint_go1_to_D455_right.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
            fixed_joint_go1_to_D455_right.CreateBody1Rel().SetTargets([d455_right_mount_prim.GetPrimPath()])

            fixed_joint_go1_to_D455_right.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, -0.1, 0.1))
            fixed_joint_go1_to_D455_right.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

            fixed_joint_go1_to_D455_right.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([80, 0, 180]), degrees=True)))
            fixed_joint_go1_to_D455_right.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [joints - armstrong] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # armstrong trunk
        prim_armstrong_base = self._world.scene.stage.GetPrimAtPath("/full_test/base_link_R")

        # [set fixed joint for d455]
        d455_armstrong_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_5/RSD455/Camera_Pseudo_Depth")
        joint_path_armstrong = "/World/FixedJoint/armstrong_to_d455"

        fixed_joint_armstrong_to_d455 = self._world.stage.GetPrimAtPath(joint_path_armstrong)
        if not fixed_joint_armstrong_to_d455.IsValid():
            fixed_joint_armstrong_to_d455 = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/armstrong_to_d455")
            fixed_joint_armstrong_to_d455.CreateBody0Rel().SetTargets([prim_armstrong_base.GetPrimPath()])
            fixed_joint_armstrong_to_d455.CreateBody1Rel().SetTargets([d455_armstrong_prim.GetPrimPath()])

            fixed_joint_armstrong_to_d455.CreateLocalPos0Attr().Set(Gf.Vec3f(0.1016179, 0, 0.4988096))
            fixed_joint_armstrong_to_d455.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

            fixed_joint_armstrong_to_d455.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([50, 0, -90]), degrees=True)))
            fixed_joint_armstrong_to_d455.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        await self._world.reset_async() # 센서 부착 후 리셋

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [omnigraph - go1] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        try:
            if self.cfg_og_go1["ros_clock_publisher"]["enable"]:
                self._clock_graph = ros_clock_publisher_go1()                
                self._clock_publisher_iter = 0

            if self.cfg_og_go1["ros_joints_tf_publisher"]["enable"]:
                bodies = ["/World/Robot/Go1/trunk"]
                self._joint_tf_graph = ros_joints_tf_publisher_go1(bodies)
                self._tf_publisher_iter = 0

            if self.cfg_og_go1["ros_joint_state_publisher"]["enable"]:
                self._joint_state_graph = ros_joint_state_publisher_go1()
                self._joint_state_publisher_iter = 0

            if self.cfg_og_go1["ros_sensors_tf_publisher"]["enable"]:
                self._sensors_tf_graph = ros_sensors_tf_publisher_go1()
                self._sensors_tf_publisher_iter = 0

            if self.cfg_og_go1["ros_imu_publisher"]["enable"]:
                self._imu_graph = ros_imu_publisher_go1()
                self._imu_publisher_iter = 0

            if self.cfg_og_go1["ros_odom_publisher"]["enable"]:
                self._odom_graph = ros_odom_publisher_go1()
                self._odom_publisher_iter = 0

            if self.cfg_og_go1["ros_foot_contact_publisher"]["enable"]:
                self._foot_force_graph = ros_foot_contact_publisher_go1()
                self._foot_force_publisher_iter = 0

            if self.cfg_og_go1["ros_lidar_publisher"]["enable"]:
                self._lidar_graph = ros_lidar_publisher_go1(frame_id = "lidar", 
                                                        topic_name = self.cfg_og_go1["ros_lidar_publisher"]["topic"], 
                                                        lidar_prim = "/World/Sensors/lidar")
                self._lidar_publisher_iter = 0

            if self.cfg_og_go1["ros_d455_publisher"]["enable"]:
                self._d455_graph_front = ros_d455_publisher_go1(frame_id = "d455_front" , 
                                                            namespace="d455_front", 
                                                            topic_name = self.cfg_og_go1["ros_d455_publisher"]["topic"], 
                                                            depth_prim="/World/Sensors/Realsense_D455_1/RSD455/Camera_Pseudo_Depth",
                                                            rgb_prim="/World/Sensors/Realsense_D455_1/RSD455/Camera_OmniVision_OV9782_Color")
                self._d455_publisher_iter = 0

            if self.cfg_og_go1["ros_d455_publisher"]["enable"]:
                self._d455_graph_rear = ros_d455_publisher_go1(frame_id = "d455_rear" , 
                                                                namespace="d455_rear", 
                                                                topic_name = self.cfg_og_go1["ros_d455_publisher"]["topic"],
                                                                depth_prim="/World/Sensors/Realsense_D455_2/RSD455/Camera_Pseudo_Depth",
                                                                rgb_prim="/World/Sensors/Realsense_D455_2/RSD455/Camera_OmniVision_OV9782_Color")

                self._d455_publisher_iter = 0

            if self.cfg_og_go1["ros_d455_publisher"]["enable"]:
                self._d455_graph_left = ros_d455_publisher_go1(frame_id = "d455_left" , 
                                                                namespace = "d455_left", 
                                                                topic_name = self.cfg_og_go1["ros_d455_publisher"]["topic"],
                                                                depth_prim = "/World/Sensors/Realsense_D455_3/RSD455/Camera_Pseudo_Depth",
                                                                rgb_prim = "/World/Sensors/Realsense_D455_3/RSD455/Camera_OmniVision_OV9782_Color")

                self._d455_publisher_iter = 0

            if self.cfg_og_go1["ros_d455_publisher"]["enable"]:
                self._d455_graph_right = ros_d455_publisher_go1(frame_id = "d455_right" ,
                                                                namespace = "d455_right", 
                                                                topic_name = self.cfg_og_go1["ros_d455_publisher"]["topic"],
                                                                depth_prim="/World/Sensors/Realsense_D455_4/RSD455/Camera_Pseudo_Depth",
                                                                rgb_prim="/World/Sensors/Realsense_D455_4/RSD455/Camera_OmniVision_OV9782_Color")

                self._d455_publisher_iter = 0
        except Exception as e:
            print(e)
            # simulation_app.close()
            exit()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [omnigraph - armstrong] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        try:
            if self.cfg_og_armstrong["ros_joints_tf_publisher"]["enable"]:
                bodies = ["/full_test"]
                self._joint_state_graph_armstrong = ros_joint_tf_publisher_armstrong(bodies)
                self._joint_state_publisher_iter_armstrong = 0

            if self.cfg_og_armstrong["ros_d455_publisher"]["enable"]:
                self._d455_graph_armstrong = ros_d455_publisher_armstrong(frame_id = "d455_front", 
                                                                        namespace="d455", 
                                                                        topic_name = self.cfg_og_armstrong["ros_d455_publisher"]["topic"], 
                                                                        depth_prim="/World/Sensors/Realsense_D455_5/RSD455/Camera_Pseudo_Depth",
                                                                        rgb_prim="/World/Sensors/Realsense_D455_5/RSD455/Camera_OmniVision_OV9782_Color")
                self._d455_publisher_iter_armstrong = 0

        except Exception as e:
            print(e)
            exit()

        return

    async def setup_post_reset(self):
        
        return

    def world_cleanup(self):
        # srospy.signal_shutdown("go1_armstrong integration complete")
        return