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
from omni.isaac.range_sensor import _range_sensor

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
def ros_lidar_publisher(frame_id, topic_name, lidar_prim):
    keys = og.Controller.Keys
    (lidar_graph, _, _, _) = og.Controller.edit(
        {
            "graph_path": "/ROS/ROS_lidar_publisher",
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("readLidar", "omni.isaac.range_sensor.IsaacReadLidarPointCloud"),
                ("publishLidar", "isaacsim.ros1.bridge.ROS1PublishPointCloud"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "readLidar.inputs:execIn"),

                ("readSimTime.outputs:simulationTime", "publishLidar.inputs:timeStamp"),
                
                ("readLidar.outputs:execOut", "publishLidar.inputs:execIn"),
                ("readLidar.outputs:data", "publishLidar.inputs:data"),
            ],
            keys.SET_VALUES: [
                ("readLidar.inputs:lidarPrim", lidar_prim),

                ("publishLidar.inputs:frameId",  frame_id),
                ("publishLidar.inputs:topicName",  topic_name),
            ]
        }
    )

    return lidar_graph

class Sensor_runner(object):
    def __init__(self) -> None:
        """
        [Summary]

        creates the simulation world with preset physics_dt and render_dt and creates a unitree go1 robot
        """

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [config] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # load yaml file
        with open(PACKAGE_PATH + "/example/kaeri/isaac/sensors/lidar/config.yaml", "r") as f:
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

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [sensor] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        # URL : https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.range_sensor/docs/index.html
        result, center_lidar = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path = "/World/Sensors/lidar",
            parent = None,
            min_range = 0.4,
            max_range = 30.0,
            draw_points = True,
            draw_lines = True,
            horizontal_fov = 360.0,
            vertical_fov = 30.0,
            horizontal_resolution = 0.4,
            vertical_resolution = 2.0,
            rotation_rate = 0,
            high_lod = True,
            yaw_offset = 0.0,
            enable_semantics = False,
        )

        # (optional) give rigidbody property to lidar
        # omni.kit.commands.execute('AddPhysicsComponent',
        #                           usd_prim=self._world.scene.stage.GetPrimAtPath("/World/Sensors/lidar"),
        #                           component='PhysicsRigidBodyAPI')
        

        self._lidar_sensor_interface = _range_sensor.acquire_lidar_sensor_interface()

        # lidar prim
        self.prim_center_lidar = "/World/Sensors/lidar"

        # reset world
        self._world.reset()

        # =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-= [ROS] =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
        if self.cfg_og["ros_lidar_publisher"]["enable"]:
            self._lidar_graph = ros_lidar_publisher(frame_id = "lidar", 
                                                    topic_name = self.cfg_og["ros_lidar_publisher"]["topic"], 
                                                    lidar_prim = "/World/Sensors/lidar")
            self._lidar_publisher_iter = 0
        
        return

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._world.add_physics_callback("lidar", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        # which data i can get? : https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.range_sensor/docs/index.html
        # number of rows
        print(self._lidar_sensor_interface.get_num_rows(self.prim_center_lidar))

        # pointcloud
        print(self._lidar_sensor_interface.get_point_cloud_data(self.prim_center_lidar))

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
    rospy.signal_shutdown("go1 complete")
    simulation_app.close()


if __name__ == "__main__":
    main()
