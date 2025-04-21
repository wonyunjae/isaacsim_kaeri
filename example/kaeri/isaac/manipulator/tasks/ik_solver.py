from omni.isaac.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from omni.isaac.core.articulations import Articulation
from typing import Optional


class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] , robot_description_path: Optional[str], urdf_path: Optional[str]) -> None:
        #TODO: change the config path
        self._kinematics = LulaKinematicsSolver(robot_description_path=robot_description_path,
                                                urdf_path=urdf_path)

        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)

        return