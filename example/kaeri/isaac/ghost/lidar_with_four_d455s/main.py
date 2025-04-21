# This script is a standalone script to run the go1 robot with sensors in omniverse
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import os
import yaml

import carb
import omni.appwindow  # Contains handle to keyboard
from isaacsim.core.api.world import World
from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
import omni.replicator.core as rep
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.prims import create_prim

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")

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
from geometry_msgs.msg import Twist

# $PACKAGE_PATH to python path
import sys
PACKAGE_PATH = os.environ["PACKAGE_PATH"]
sys.path.append(PACKAGE_PATH)

from example.kaeri.isaac.ghost.lidar_with_four_d455s.ros_publisher import *

import omni.graph.core as og



class Sensor_runner(object):
    def __init__(self) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a unitree go1 robot
        """

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/ghost/lidar_rtx/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_freq = config["control_freq"]
        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]
        self.cfg_og       = config["omnigraph"]

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = World(stage_units_in_meters=1.0, physics_dt=1.0/self.physics_freq, rendering_dt=1.0/self.render_freq)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/environment/Warehouse.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")

        # # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [dummy cube] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # cube = self._world.scene.add(
        #     VisualCuboid(
        #         name="cube",
        #         prim_path="/World/cube",
        #         position=np.array([0.0, 0.0, 1.5]),
        #         orientation=euler_angles_to_quat(np.array([0, 0, 0]), degrees=False),
        #         color=np.array([1.0, 0.0, 0.0]),
        #         size=0.05,
        #         scale=np.array([1, 1, 1]),
        #     )
        # )

        # prim_path_cube = "/World/cube"
        # prim_cube = self._world.scene.stage.GetPrimAtPath(prim_path_cube)

        # # (optional) give rigidbody property to lidar
        # omni.kit.commands.execute('AddPhysicsComponent',
        #                           usd_prim=self._world.scene.stage.GetPrimAtPath("/World/cube"),
        #                           component='PhysicsRigidBodyAPI')
        # prim_cube.GetAttribute("physxRigidBody:disableGravity").Set(True) # 무중력

        # xform = UsdGeom.Xformable(prim_cube)
        # self.cube_transform = xform.AddTransformOp()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [dummy xform] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # Create an Xform
        xform = define_prim('/World/center', 'Xform')
        prim_center = self._world.scene.stage.GetPrimAtPath("/World/center")

        xform = UsdGeom.Xformable(prim_center)
        self.cube_transform = xform.AddTransformOp()
        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [sensor] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
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
        # [set fixed joint for lidar]
        lidar_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/lidar")

        fixed_joint_cube_to_lidar = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/cube_to_lidar")
        fixed_joint_cube_to_lidar.CreateBody0Rel().SetTargets([prim_center.GetPrimPath()])
        fixed_joint_cube_to_lidar.CreateBody1Rel().SetTargets([lidar_prim.GetPrimPath()])

        fixed_joint_cube_to_lidar.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.4))
        fixed_joint_cube_to_lidar.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_cube_to_lidar.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))
        fixed_joint_cube_to_lidar.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # sync
        for _ in range(5):
            self._world.step(render=True)

        # [set fixed joint for d455 front]
        d455_front_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_1/RSD455/Camera_Pseudo_Depth")

        fixed_joint_cube_to_D455_front = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/cube_to_D455_front")
        fixed_joint_cube_to_D455_front.CreateBody0Rel().SetTargets([prim_center.GetPrimPath()])
        fixed_joint_cube_to_D455_front.CreateBody1Rel().SetTargets([d455_front_mount_prim.GetPrimPath()])

        fixed_joint_cube_to_D455_front.CreateLocalPos0Attr().Set(Gf.Vec3f(0.3, 0, 0.2))
        fixed_joint_cube_to_D455_front.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        fixed_joint_cube_to_D455_front.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([60, 0, -90]), degrees=True)))
        fixed_joint_cube_to_D455_front.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # # sync
        # for _ in range(5):
        #     self._world.step(render=True)

        # # [set fixed joint for d455 rear]
        # d455_rear_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_2/RSD455/Camera_Pseudo_Depth")

        # fixed_joint_cube_to_D455_rear = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/cube_to_D455_rear")
        # fixed_joint_cube_to_D455_rear.CreateBody0Rel().SetTargets([prim_center.GetPrimPath()])
        # fixed_joint_cube_to_D455_rear.CreateBody1Rel().SetTargets([d455_rear_mount_prim.GetPrimPath()])

        # fixed_joint_cube_to_D455_rear.CreateLocalPos0Attr().Set(Gf.Vec3f(-0.3, 0.0, 0.2))
        # fixed_joint_cube_to_D455_rear.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        # fixed_joint_cube_to_D455_rear.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([60, 0, 90]), degrees=True)))
        # fixed_joint_cube_to_D455_rear.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # # sync
        # for _ in range(5):
        #     self._world.step(render=True)

        # # [set fixed joint for d455 left]
        # d455_left_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_3/RSD455/Camera_Pseudo_Depth")

        # fixed_joint_cube_to_D455_left = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/cube_to_D455_left")
        # fixed_joint_cube_to_D455_left.CreateBody0Rel().SetTargets([prim_center.GetPrimPath()])
        # fixed_joint_cube_to_D455_left.CreateBody1Rel().SetTargets([d455_left_mount_prim.GetPrimPath()])

        # fixed_joint_cube_to_D455_left.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.1, 0.2))
        # fixed_joint_cube_to_D455_left.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        # fixed_joint_cube_to_D455_left.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([60, 0, 0]), degrees=True)))
        # fixed_joint_cube_to_D455_left.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # # sync
        # for _ in range(5):
        #     self._world.step(render=True)

        # # [set fixed joint for d455 right]
        # d455_right_mount_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_4/RSD455/Camera_Pseudo_Depth")

        # fixed_joint_cube_to_D455_right = UsdPhysics.FixedJoint.Define(self._world.scene.stage, "/World/FixedJoint/cube_to_D455_right")
        # fixed_joint_cube_to_D455_right.CreateBody0Rel().SetTargets([prim_cube.GetPrimPath()])
        # fixed_joint_cube_to_D455_right.CreateBody1Rel().SetTargets([d455_right_mount_prim.GetPrimPath()])

        # fixed_joint_cube_to_D455_right.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, -0.1, 0.2))
        # fixed_joint_cube_to_D455_right.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        # fixed_joint_cube_to_D455_right.CreateLocalRot0Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([60, 0, 180]), degrees=True)))
        # fixed_joint_cube_to_D455_right.CreateLocalRot1Attr().Set(Gf.Quatf(*euler_angles_to_quat(np.array([0, 0, 0]), degrees=True)))

        # # sync
        # for _ in range(5):
        #     self._world.step(render=True)

        # reset world
        self._world.reset()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [ROS] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        self.desired_vel = [0.0, 0.0, 0.0]

        if self.cfg_og["ros_clock_publisher"]["enable"]:
            self._clock_graph = ros_clock_publisher()
            self._clock_publisher_iter = 0
        
        return

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("lidar", callback_fn=self.on_physics_step)

    def cmd_vel_cb(self, msg):
        self.desired_vel = [msg.linear.x, msg.linear.y, msg.angular.z]

        print(self.desired_vel)

    def set_position(self, dt):
        pass
        # get current location
        # transform = self.cube_transform.Get()

        # pos = transform.ExtractTranslation() # Gf.Vec3d
        # rot = transform.ExtractRotation() # Gf.Rotation
        
        # # Set the new location
        # new_location = pos
        # new_location[0] = pos[0] + self.desired_vel[0] * dt
        # new_location[1] = pos[1] + self.desired_vel[1] * dt

        # # Set the new rotation
        # new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot.GetAngle() + self.desired_vel[2] * dt)
        # # print("=-=-=-=-=-=-=")
        # # print(pos, rot)
        # # print(self.desired_vel)
        # # print(new_location, new_rotation.GetAngle())

        # # Create a new transform
        # mat4d = Gf.Matrix4d()
        # mat4d.SetTranslateOnly(new_location)
        # mat4d.SetRotateOnly(new_rotation)

        # # Apply the new transform
        # self.cube_transform.Set(mat4d)


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
        """
        [Summary]

        Step simulation based on rendering downtime

        """
        # change to sim running
        while simulation_app.is_running():
            self._world.step(render=True)
        return

def main() -> None:
    rospy.init_node("lidar_isaac", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    runner = Sensor_runner()
    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()
    runner._world.reset()
    runner.run()
    rospy.signal_shutdown("lidar ghost complete")
    simulation_app.close()

if __name__ == "__main__":
    main()
