from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
from isaacsim.core.api.world import World

from isaacsim.core.utils.extensions import enable_extension

from isaacsim.core.utils.prims import create_prim, get_prim_at_path
import numpy as np
import omni.isaac.core.utils.numpy.rotations as rot_utils
from omni.isaac.core.objects import DynamicCuboid             
from omni.isaac.sensor import Camera
from omni.isaac.sensor import LidarRtx
import omni.kit.commands
import omni.replicator.core as rep
from omni.isaac.dynamic_control import _dynamic_control
from isaacsim.core.utils.prims import create_prim import get_all_matching_child_prims
from omni.isaac.core.utils.rotations import rot_matrix_to_quat, quat_to_euler_angles
from pxr import Gf, UsdGeom
import os

# get $PACKAGE_PATH
PACKAGE_PATH = os.environ["PACKAGE_PATH"]

# add $PACKAGE_PATH to python path
import sys
sys.path.append(PACKAGE_PATH)

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")
enable_extension("omni.kaeri.ros_bridge")

# update the simulation
simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rospy
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

class camera_runner(object):
    def __init__(self):
        self._world = World(stage_units_in_meters=1, physics_dt=1.0/400, rendering_dt=1.0/25)
        
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

        cube_2 = self._world.scene.add(
            DynamicCuboid(
                prim_path = "/cube_1",
                name ="cube_1",
                position = np.array([5, 0, 3]),
                scale = np.array([1, 1, 1]),
                size = 1.0,
                color = np.array([255, 0, 0]),
            )
        )

        cube_3 = self._world.scene.add(
            DynamicCuboid(
                prim_path = "/cube_2",
                name = "cube_2",
                position = np.array([-5, 1, 3.0]),
                scale = np.array([0.1, 0.1, 0.1]),
                size = 1.0,
                color = np.array([0, 0, 255]),
                linear_velocity = np.array([0, 0, 0.4]),
            )
        )

        cube_base = self._world.scene.add(
            DynamicCuboid(
                prim_path = "/cube_base",
                name = "cube_base",
                position = np.array([0, 0, 3]),
                scale = np.array([1, 1, 1]),
                size = 0.3,
                color = np.array([255, 0, 0]),
            )
        )

        result, sensor_front = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/sensor_front",
            parent=None,
            min_range=0.4,
            max_range=5.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=85.0,
            vertical_fov=40.0,
            horizontal_resolution=0.4,
            vertical_resolution=1.0,
            rotation_rate=0.0,
            high_lod=True,
            yaw_offset=0.0,
            enable_semantics=False,
        )

        result, sensor_rear = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/sensor_rear",
            parent=None,
            min_range=0.4,
            max_range=5.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=85.0,
            vertical_fov=40.0,
            horizontal_resolution=0.4,
            vertical_resolution=1.0,
            rotation_rate=0.0,
            high_lod=True,
            yaw_offset=0.0,
            enable_semantics=False,
        )

        result, sensor_left = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/sensor_left",
            parent=None,
            min_range=0.4,
            max_range=5.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=85.0,
            vertical_fov=40.0,
            horizontal_resolution=0.4,
            vertical_resolution=1.0,
            rotation_rate=0.0,
            high_lod=True,
            yaw_offset=0.0,
            enable_semantics=False,
        )

        result, sensor_right = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/sensor_right",
            parent=None,
            min_range=0.4,
            max_range=5.0,
            draw_points=True,
            draw_lines=True,
            horizontal_fov=85.0,
            vertical_fov=40.0,
            horizontal_resolution=0.4,
            vertical_resolution=1.0,
            rotation_rate=0.0,
            high_lod=True,
            yaw_offset=0.0,
            enable_semantics=False,
        )
        
        # Get the prim you want to move
        stage = omni.usd.get_context().get_stage()

        sensor_front_prim = stage.GetPrimAtPath("/sensor_front")
        xform = UsdGeom.Xformable(sensor_front_prim)
        self.sensor_transfom_front = xform.AddTransformOp()
        
        sensor_rear_prim = stage.GetPrimAtPath("/sensor_rear")
        xform = UsdGeom.Xformable(sensor_rear_prim)
        self.sensor_transfom_rear = xform.AddTransformOp()

        sensor_left_prim = stage.GetPrimAtPath("/sensor_left")
        xform = UsdGeom.Xformable(sensor_left_prim)
        self.sensor_transfom_left = xform.AddTransformOp()

        sensor_right_prim = stage.GetPrimAtPath("/sensor_right")
        xform = UsdGeom.Xformable(sensor_right_prim)
        self.sensor_transfom_right = xform.AddTransformOp()

        # Get base
        cube_base_prim = stage.GetPrimAtPath("/cube_base")
        self.xform_base_cube = UsdGeom.Xformable(cube_base_prim)
        
        

        # # 2. Create and Attach a render product to the camera
        # render_product = rep.create.render_product(sensor_front.GetPath(), [1, 1])

        # # 3. Create a Replicator Writer that "writes" points into the scene for debug viewing
        # writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
        # writer.attach(render_product)

        # # 4. Create Annotator to read the data from with annotator.get_data()
        # annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
        # annotator.attach(render_product)

        # 
        self._world.reset()

        return

    def setup(self) -> None:
        self._world.add_physics_callback("camera_advance", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        tf_matrix = np.array(self.xform_base_cube.GetLocalTransformation())
        quat_base_cube = rot_matrix_to_quat(tf_matrix[0:3, 0:3]) # w, x, y, z
        euler_base_cube = quat_to_euler_angles(quat_base_cube, degrees=True)
        pos_base_cube = tf_matrix[3, 0:3]

        print(tf_matrix)

        # Set front sensor
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(pos_base_cube[0] + 0.3, pos_base_cube[1] + 0.0, pos_base_cube[2] + 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0,1,0), 30))
        self.sensor_transfom_front.Set(mat)

        # Set rear sensor
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(pos_base_cube[0] - 0.3, pos_base_cube[1] + 0.0, pos_base_cube[2] + 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0,1,0), 30) * Gf.Rotation(Gf.Vec3d(0,0,1), 180))

        self.sensor_transfom_rear.Set(mat)

        # Set left sensor
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(pos_base_cube[0] + 0.0, pos_base_cube[1] + 0.3, pos_base_cube[2] + 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0,1,0), 30) * Gf.Rotation(Gf.Vec3d(0,0,1), 90))
        self.sensor_transfom_left.Set(mat)

        # Set right sensor
        mat = Gf.Matrix4d()
        mat.SetTranslateOnly(Gf.Vec3d(pos_base_cube[0] + 0.0, pos_base_cube[1] - 0.3, pos_base_cube[2] + 1.0))
        mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0,1,0), 30) * Gf.Rotation(Gf.Vec3d(0,0,1), -90))
        self.sensor_transfom_right.Set(mat)


        pass

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
    rospy.init_node("camera_test", anonymous=False, disable_signals=True, log_level=rospy.ERROR)
    rospy.set_param("use_sim_time", True)
    runner = camera_runner()
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
