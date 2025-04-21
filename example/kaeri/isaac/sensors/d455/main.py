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
from omni.isaac.dynamic_control import _dynamic_control
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

# $PACKAGE_PATH to python path
import sys
PACKAGE_PATH = os.environ["PACKAGE_PATH"]
sys.path.append(PACKAGE_PATH)

import omni.graph.core as og

def ros_d455_publisher(frame_id, namespace, topic_name, depth_prim, rgb_prim):
    keys = og.Controller.Keys
    
    (d455_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_" + frame_id + "_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),

                ("createRenderDepth", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("createRenderRGB", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),

                ("publishD455_camera_info_depth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("publishD455_camera_info_rgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),

                ("publishD455_depth", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("publishD455_depth_pcl", "isaacsim.ros1.bridge.ROS1CameraHelper"),
                ("publishD455_rgb", "isaacsim.ros1.bridge.ROS1CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createRenderDepth.inputs:execIn"),
                ("OnTick.outputs:tick", "createRenderRGB.inputs:execIn"),

                ("createRenderDepth.outputs:renderProductPath", "publishD455_camera_info_depth.inputs:renderProductPath"),
                ("createRenderDepth.outputs:renderProductPath", "publishD455_depth.inputs:renderProductPath"),
                ("createRenderDepth.outputs:renderProductPath", "publishD455_depth_pcl.inputs:renderProductPath"),

                ("createRenderRGB.outputs:renderProductPath", "publishD455_camera_info_rgb.inputs:renderProductPath"),
                ("createRenderRGB.outputs:renderProductPath", "publishD455_rgb.inputs:renderProductPath"),

                ("createRenderDepth.outputs:execOut", "publishD455_camera_info_depth.inputs:execIn"),
                ("createRenderDepth.outputs:execOut", "publishD455_depth.inputs:execIn"),
                ("createRenderDepth.outputs:execOut", "publishD455_depth_pcl.inputs:execIn"),

                ("createRenderRGB.outputs:execOut", "publishD455_camera_info_rgb.inputs:execIn"),
                ("createRenderRGB.outputs:execOut", "publishD455_rgb.inputs:execIn"),
            ],
            keys.SET_VALUES: [
                ("createRenderDepth.inputs:cameraPrim", depth_prim),
                ("createRenderDepth.inputs:height", 45),
                ("createRenderDepth.inputs:width", 80),

                ("createRenderRGB.inputs:cameraPrim", rgb_prim),
                # ("createRenderRGB.inputs:height", 45),
                # ("createRenderRGB.inputs:width", 80),
                ("createRenderRGB.inputs:height", 180),
                ("createRenderRGB.inputs:width", 320),


                ("publishD455_camera_info_depth.inputs:frameId",  frame_id),
                ("publishD455_camera_info_rgb.inputs:frameId",  frame_id),
                ("publishD455_depth.inputs:frameId",  frame_id),
                ("publishD455_depth_pcl.inputs:frameId",  frame_id),
                ("publishD455_rgb.inputs:frameId",  frame_id),

                ("publishD455_camera_info_depth.inputs:topicName",  namespace + "/" + topic_name["camera_info"]),
                ("publishD455_camera_info_rgb.inputs:topicName",  namespace + "/" + topic_name["camera_info"]),
                ("publishD455_depth.inputs:topicName",  namespace + "/" + topic_name["depth"]),
                ("publishD455_depth_pcl.inputs:topicName",  namespace + "/" + topic_name["depth_pcl"]),
                ("publishD455_rgb.inputs:topicName",  namespace + "/" + topic_name["rgb"]),

                ("publishD455_camera_info_depth.inputs:type",  "camera_info"),
                ("publishD455_camera_info_rgb.inputs:type",  "camera_info"),
                ("publishD455_depth.inputs:type",  "depth"),
                ("publishD455_depth_pcl.inputs:type",  "depth_pcl"),
                ("publishD455_rgb.inputs:type",  "rgb"),
            ]
        }
    )

    return d455_graph

class Sensor_runner(object):
    def __init__(self) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a unitree go1 robot
        """

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/sensors/d455/config.yaml", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)

        self.control_freq = config["control_freq"]
        self.physics_freq = config["physics_freq"]
        self.render_freq  = config["render_freq"]
        self.cfg_og       = config["omnigraph"]

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [world] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        self._world = World(stage_units_in_meters=1.0, physics_dt=1.0/self.physics_freq, rendering_dt=1.0/self.render_freq)

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [environment] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        asset_path = PACKAGE_PATH + "/model/environment/Warehouse.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Warehouse")

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [sensor] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        asset_path = PACKAGE_PATH + "/model/sensors/Realsense_D455_1.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/Sensors")
        prim_path = "/World/Sensors/Realsense_D455_1"


        # # (optional) give rigidbody property to sensor
        # d455_prim = self._world.scene.stage.GetPrimAtPath("/World/Sensors/Realsense_D455_1")
        # omni.kit.commands.execute('AddPhysicsComponent',
        #                           usd_prim=d455_prim,
        #                           component='PhysicsRigidBodyAPI')
        # d455_prim.GetAttribute("physxRigidBody:disableGravity").Set(True) # 무중력

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [set position & orientation] =-=-=-=-=-=-=-=-=-=-=-=-=-=
        d455_prim = self._world.scene.stage.GetPrimAtPath(prim_path)
        xform = UsdGeom.Xformable(d455_prim)
        transform = xform.AddTransformOp()

        # Set the new location
        new_location = Gf.Vec3d(0.0, 0.0, 1.0)  # Replace with your desired location

        # Set the new rotation
        new_rotation = Gf.Rotation(Gf.Vec3d(0, 1, 0), -30)  # Replace with your desired rotation

        # Create a new transform
        mat4d = Gf.Matrix4d()
        mat4d.SetTranslateOnly(new_location)
        mat4d.SetRotateOnly(new_rotation)

        # Apply the new transform
        transform.Set(mat4d)

        # reset world
        self._world.reset()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [ROS] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        if self.cfg_og["ros_d455_publisher"]["enable"]:
            self._d455_graph_front = ros_d455_publisher(frame_id = "d455_front" , 
                                                        namespace="d455_front", 
                                                        topic_name = self.cfg_og["ros_d455_publisher"]["topic"], 
                                                        depth_prim="/World/Sensors/Realsense_D455_1/RSD455/Camera_Pseudo_Depth",
                                                        rgb_prim="/World/Sensors/Realsense_D455_1/RSD455/Camera_OmniVision_OV9782_Color")
            self._d455_publisher_iter = 0

        return

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("d455_world", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        # which data i can get? : https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.range_sensor/docs/index.html
        if self.cfg_og["ros_d455_publisher"]["enable"] == False:
            pass
        elif self.physics_freq - self.cfg_og["ros_d455_publisher"]["freq"] == self._d455_publisher_iter:
            og.Controller.evaluate_sync(self._d455_graph_front)
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
    rospy.init_node("lidar_isaac", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    runner = Sensor_runner()
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
