import os
import sys

from omni.isaac.core.scenes.scene import Scene

# 환경 변수 지정해줘야 함.
PACKAGE_PATH = os.environ["PACKAGE_PATH"] = "/isaac-sim/isaac_sim"
sys.path.append("/isaac-sim/isaac_sim")


from Kaeri_Test_python.kaeri_base_sample import BaseSample
#########################################################################
from isaacsim.core.api.tasks.pick_place import PickPlace
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.storage.native import get_assets_root_path
from isaacsim.robot.wheeled_robots.controllers.wheel_base_pose_controller import WheelBasePoseController
from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from isaacsim.core.api.tasks.base_task import BaseTask
from isaacsim.core.utils.types import ArticulationAction
# find a unique string name, to use it for prim aths and scene names
from isaacsim.core.utils.string import find_unique_string_name   # unique prim path 생성
from isaacsim.core.utils.prims import create_prim import is_prim_path_valid          # prim path 유효한지 체크
import numpy as np


########### Task #############################
class RobotsPlaying(BaseTask):
    # offset을 추가한다
    # ==> 왜? task 오브젝트와 target을 이동시키기 위해서
    def __init__(self, name, offset=None):
        super().__init__(name=name, offset=offset)
        # task 활성화 여부 알라기 위한 플래그 역할
        self._task_event = 0
        # jebot은 목표 이동지점. franka는 task 설정.
        self._jetbot_goal_position = np.array([1.3, 0.3, 0]) + self._offset
        self._pick_place_task = PickPlace(cube_initial_position=np.array([0.1, 0.3, 0.05]),
                                          target_position=np.array([0.7, -0.3, 0.0515 / 2.0]), 
                                          offset=offset)
        return
    
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._pick_place_task.set_up_scene(scene)
        
        # unique scene name 찾기
        jetbot_name = find_unique_string_name(
            initial_name="fancy_jetbot", is_unique_fn=lambda x: not self.scene.object_exists(x)
        )
        # unique prim path 찾기
        jetbot_prim_path = find_unique_string_name(
            initial_name="/World/Fancy_Jetbot", is_unique_fn=lambda x: not is_prim_path_valid(x)
        )
        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        self._jetbot = scene.add(
            WheeledRobot(
                prim_path=jetbot_prim_path,
                name=jetbot_name,
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_asset_path,
                position=np.array([0, 0.3, 0]),
            )
        )

        # Task 객체에 Jetbot 추가
        self._task_objects[self._jetbot.name] = self._jetbot
        pick_place_params = self._pick_place_task.get_params()
        self._franka = scene.get_object(pick_place_params["robot_name"]["value"])
        # franka x축으로 100이동?
        current_position, _ = self._franka.get_world_pose()
        self._franka.set_world_pose(position=current_position + np.array([1.0,0,0]))
        self._franka.set_default_state(position=current_position + np.array([1.0,0,0]))
        self._move_task_objects_to_their_frame()
        
        return
    
    def get_observations(self):
        # 현재 jetbot 위치 받아오기
        current_jetbot_position, current_jetbot_orientation = self._jetbot.get_world_pose()
        # observation은 jetbot이 큐브를 밀기 위해서 필요함
        observations = {
            "task_event": self._task_event,
            self._jetbot.name:{
                "position": current_jetbot_position,
                "orientation": current_jetbot_orientation,
                "goal_position": self._jetbot_goal_position
            }  
        }
        observations.update(self._pick_place_task.get_observations())
        return observations
    
    def get_params(self):
        pick_place_params = self._pick_place_task.get_params()
        params_representation = pick_place_params
        params_representation["jetbot_name"] = {"value":self._jetbot.name, "modifiable":False}
        params_representation["franka_name"] = pick_place_params["robot_name"]
        return params_representation
    
    def pre_step(self, control_index, simulation_time):
        if self._task_event==0:
            current_jetbot_position, _ = self._jetbot.get_world_pose()
            # jetbot이 특정 위치로 큐브를 옮기면.
            if np.mean(np.abs(current_jetbot_position[:2] - self._jetbot_goal_position[:2])) < 0.04:
                self._task_event += 1
                self._cube_arrive_step_index = control_index

        # jetbot 후진.
        elif self._task_event == 1:
            # jetbot은 200 time step 동안 뒤로?
            if control_index - self._cube_arrive_step_index == 200:
                self._task_event += 1
        return
    
    def post_reset(self):
        self._franka.gripper.set_joint_positions(self._franka.gripper.joint_opened_positions)
        self._task_event = 0
        return
        
class Main(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        return  

    def setup_scene(self):
        world = self.get_world()
        # offset 추가
        world.add_task(RobotsPlaying(name="awesome_task", offset=np.array[(0, -1.0, 0)]))

        
        return

    async def setup_post_load(self):
        self._world = self.get_world()
        task_params = self._world.get_task("awesome_task").get_params()
        # franka action
        self._franka = self._world.scene.get_object(task_params["franka_name"]["value"])
        self._jetbot = self._world.scene.get_object(task_params["jetbot_name"]["value"])
        self._cube_name = task_params["cube_name"]["value"]
        self._franka_controller = PickPlaceController(name="pick_place_controller",
                                                      gripper=self._franka.gripper,
                                                      robot_articulation=self._franka)
        self._jetbot_controller = WheelBasePoseController(name="cool_controller",
                                                          open_loop_wheel_controller=DifferentialController(name="simple_controller",
                                                                                                            wheel_radius=0.03, wheel_base=0.1125))
        self._world.add_physics_callback("sim_step", callback_fn=self.physics_step)
        await self._world.play_async()
        return
    
    def physics_step(self, step_size):
        current_observations = self._world.get_observations()
        if(current_observations["task_event"]==0):
            self._jetbot.apply_wheel_actions(
                self._jetbot_controller.forward(
                    start_position=current_observations[self._jetbot.name]["position"],
                    start_orientation=current_observations[self._jetbot.name]["orientation"],
                    goal_position=current_observations[self._jetbot.name]["goal_position"]
                )
            )
        elif(current_observations["task_event"]==1):
            # 뒤로 움직임
            self._jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[-8, -8]))
        elif(current_observations["task_event"]==2):
            self._jetbot.apply_wheel_actions(ArticulationAction(joint_velocities=[0.0, 0.0]))
            # franka 작업 시작
            actions = self._franka_controller.forward(
                picking_position=current_observations[self._cube_name]["position"],
                placing_position=current_observations[self._cube_name]["target_position"],
                current_joint_positions=current_observations[self._franka.name]["joint_positions"]
            )
            self._franka.apply_action(actions)

        if self._franka_controller.is_done():
            self._world.pause()
        return

    # Articulation 이후에 한 번의 리셋이 필요하기 때문에 여기에 joints와 omnigraph가 있어야 함.
    async def setup_pre_reset(self):

        return

    async def setup_post_reset(self):
        self._franka_controller.reset()
        self._jetbot_controller.reset()
        await self._world.play_async()
        return

    def world_cleanup(self):
        return