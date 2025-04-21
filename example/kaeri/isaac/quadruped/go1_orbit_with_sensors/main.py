

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
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import omni.graph.core as og
from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.api.world import World
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.prims import create_prim import create_prim

from omni.isaac.orbit_assets.unitree import UNITREE_GO1_CFG
from orbit2isaac.articulation4sim import Articulation4SIM

import torch
from orbit2isaac.rsl_rl_controller import RslRLController

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
from std_msgs.msg import Float32MultiArray

import os
import yaml

# get $PACKAGE_PATH
PACKAGE_PATH = os.environ["PACKAGE_PATH"]

# add $PACKAGE_PATH to python path
import sys
sys.path.append(PACKAGE_PATH)

# custom omnigraph
from example.kaeri.isaac.quadruped.go1_orbit_with_sensors.ros_publisher import *

class Go1_runner(object):
    def __init__(self) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a unitree go1 robot
        """

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/quadruped/go1_orbit_with_sensors/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_freq = config["control_freq"]
        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]

        self.cfg_og = config["omnigraph"]

        # iter for control freq
        self._control_iter = 0

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = World(stage_units_in_meters=1.0, physics_dt=1.0/self.physics_freq, rendering_dt=1.0/self.render_freq)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/environment/Warehouse.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [robot] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        create_prim("/World/Robot", "Xform", translation=[0.0, 0.0, 0.05])
        self.unitree_go1_orbit = Articulation4SIM(UNITREE_GO1_CFG.replace(prim_path="/World/Robot/Go1"))

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

        fixed_joint_go1_to_lidar.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.4))
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

        fixed_joint_go1_to_D455_front.CreateLocalPos0Attr().Set(Gf.Vec3f(0.3, 0, 0.2))
        fixed_joint_go1_to_D455_front.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_D455_front.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([60, 0, -90]), degrees=True)))
        fixed_joint_go1_to_D455_front.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        # [set fixed joint for d455 rear]
        d455_rear_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_2/RSD455/Camera_Pseudo_Depth")

        fixed_joint_go1_to_D455_rear = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_rear")
        fixed_joint_go1_to_D455_rear.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
        fixed_joint_go1_to_D455_rear.CreateBody1Rel().SetTargets([d455_rear_mount_prim.GetPrimPath()])

        fixed_joint_go1_to_D455_rear.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.2, 0.0, 0.2))
        fixed_joint_go1_to_D455_rear.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_D455_rear.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([60, 0, 90]), degrees=True)))
        fixed_joint_go1_to_D455_rear.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        # [set fixed joint for d455 left]
        d455_left_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_3/RSD455/Camera_Pseudo_Depth")

        fixed_joint_go1_to_D455_left = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_left")
        fixed_joint_go1_to_D455_left.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
        fixed_joint_go1_to_D455_left.CreateBody1Rel().SetTargets([d455_left_mount_prim.GetPrimPath()])

        fixed_joint_go1_to_D455_left.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.1, 0.2))
        fixed_joint_go1_to_D455_left.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_D455_left.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([60, 0, 0]), degrees=True)))
        fixed_joint_go1_to_D455_left.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        # [set fixed joint for d455 right]
        d455_right_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_4/RSD455/Camera_Pseudo_Depth")

        fixed_joint_go1_to_D455_right = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/go1_to_D455_right")
        fixed_joint_go1_to_D455_right.CreateBody0Rel().SetTargets([prim_go1_trunk.GetPrimPath()])
        fixed_joint_go1_to_D455_right.CreateBody1Rel().SetTargets([d455_right_mount_prim.GetPrimPath()])

        fixed_joint_go1_to_D455_right.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, -0.1, 0.2))
        fixed_joint_go1_to_D455_right.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_go1_to_D455_right.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([60, 0, 180]), degrees=True)))
        fixed_joint_go1_to_D455_right.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        # 
        self._enter_toggled = 0

        # =-=-=-=-=-=-= ROS =-=-=-=-=-=-=-=
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

        self._pub = rospy.Publisher("/isaac_a1/output", Float32MultiArray, queue_size=10)
        self.tmp_val_pub = rospy.Publisher("/isaac_go1/tmp_val_pub", Float32MultiArray, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        
        self.rl_ctrl_helper = RslRLController(self.physics_freq)
        self.rl_ctrl_helper.load_pre_trained_model(PACKAGE_PATH)
        self.disier_velo = [0.0, 0.0, 0.0]
        
        return
    
    def cmd_vel_cb(self, msg):
        self.disier_velo = [msg.linear.x, msg.linear.y, msg.angular.z]

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("go1", callback_fn=self.on_physics_step)
        self._world.add_timeline_callback

    def on_physics_step(self, step_size) -> None:
        if self.physics_freq - self.control_freq == self._control_iter:            
            self.rl_ctrl_helper.move(self.unitree_go1_orbit, self.disier_velo, 1.0/self.control_freq)
            self._control_iter = 0
        else:
            self._control_iter += self.control_freq


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
        while simulation_app.is_running():
            self._world.step(render=True)
        return


def main() -> None:
    rospy.init_node("go1_with_sensors_sim", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
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
