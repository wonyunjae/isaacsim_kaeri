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
from pxr import Gf, UsdGeom
import omni.replicator.core as rep




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

from example.kaeri.isaac.ghost.lidar_rtx.ros_publisher import *

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

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/environment/Warehouse.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [sensor] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # URL : https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/isaac_sim_sensors_rtx_based_lidar.html
        # 1. Create The Camera
        _, sensor = omni.kit.commands.execute(
                        "IsaacSensorCreateRtxLidar",
                        path="/World/Sensors/lidar",
                        parent=None,
                        config="OS0_128ch10hz512res",
                        translation=(0, 0, 1.0),
                        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
                    )
        
        # 2. Create and Attach a render product to the camera
        render_product = rep.create.render_product(sensor.GetPath(), [1, 1])

        # 3. Create a Replicator Writer that "writes" points into the scene for debug viewing
        writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
        writer.attach(render_product)

        # 4. Create Annotator to read the data from with annotator.get_data()
        annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
        annotator.attach(render_product)

        # (optional) give rigidbody property to lidar
        # omni.kit.commands.execute('AddPhysicsComponent',
        #                           usd_prim=self._world.scene.stage.GetPrimAtPath("/World/Sensors/lidar"),
        #                           component='PhysicsRigidBodyAPI')

        # self._lidar_sensor_interface = _range_sensor.acquire_lidar_sensor_interface()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [set position & orientation] =-=-=-=-=-=-=-=-=-=-=-=-=-=
        prim_path = "/World/Sensors/lidar"
        lidar_prim = self._world.scene.stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(lidar_prim)
        self.lidar_transform = xform.AddTransformOp()

        # Set the new location
        new_location = Gf.Vec3d(0.0, 0.0, 1.0)  # Replace with your desired location

        # Set the new rotation
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), 1)  # Replace with your desired rotation

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.lidar_transform.Set(mat4d)

        # reset world
        self._world.reset()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [ROS] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        self.desired_vel = [0.0, 0.0, 0.0]

        if self.cfg_og["ros_clock_publisher"]["enable"]:
            self._clock_graph = ros_clock_publisher()
            self._clock_publisher_iter = 0

        if self.cfg_og["ros_lidar_publisher"]["enable"]:
            self._lidar_graph = ros_lidar_publisher(frame_id   = "lidar", 
                                                    topic_name = self.cfg_og["ros_lidar_publisher"]["topic"], 
                                                    lidar_prim = "/World/Sensors/lidar")
            self._lidar_publisher_iter = 0
        
        return

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("lidar", callback_fn=self.on_physics_step)

    def cmd_vel_cb(self, msg):
        self.desired_vel = [msg.linear.x, msg.linear.y, msg.angular.z]

        print(self.desired_vel)

    def set_position(self, dt):
        # get current location
        transform = self.lidar_transform.Get()

        pos = transform.ExtractTranslation() # Gf.Vec3d
        rot = transform.ExtractRotation() # Gf.Rotation
        
        # Set the new location
        new_location = pos
        new_location[0] = pos[0] + self.desired_vel[0] * dt
        new_location[1] = pos[1] + self.desired_vel[1] * dt

        # Set the new rotation
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot.GetAngle() + self.desired_vel[2] * dt)

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        self.lidar_transform.Set(mat4d)

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


        if self.cfg_og["ros_lidar_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_lidar_publisher"]["freq"] == self._lidar_publisher_iter:
            og.Controller.evaluate_sync(self._lidar_graph)
            self._lidar_publisher_iter = 0
        else:
            self._lidar_publisher_iter += self.cfg_og["ros_lidar_publisher"]["freq"]

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
