import torch
import numpy as np
from rsl_rl.modules import ActorCritic

class RslRLController:
    def __init__(self, physics_freq):
        self.physics_freq = physics_freq
        self.pre_actions = torch.zeros(12) # 초기 행동 벡터 설정
        self.actor_model = None # 모델은 load_pre_trained_model을 통해 로드됩니다.

    def lets_go_unitree(self, unitree_go1_orbit, disier_velo):
        obs = self.gather_observation(unitree_go1_orbit, disier_velo)
        action = self.inference_actor_model(obs)

        # apply action to the robot
        unitree_go1_orbit.set_joint_position_target(torch.tensor(action))
        # write data to sim
        unitree_go1_orbit.write_data_to_sim()
        unitree_go1_orbit.update(1.0/self.physics_freq)

    def load_pre_trained_model(self, package_path):
        # TODO: 이거 config 파일로 옮기면 이쁠것 같다.
        # num_actor_obs, num_critic_obs, num_actions 등은 실제 환경에 맞게 설정해야 합니다.
        num_actor_obs = 48
        num_critic_obs = 48
        num_actions = 12
        actor_critic_model = ActorCritic(
            num_actor_obs=num_actor_obs,
            num_critic_obs=num_critic_obs,
            num_actions=num_actions,
            actor_hidden_dims=[128, 128, 128],
            critic_hidden_dims=[128, 128, 128],
            activation="elu",
            init_noise_std=1.0
        )

        print("\n=-=-=-=-=-=-= Model =-=-=-=-=-=-=-=")
        # 'your_model_path.pt'는 실제 모델 파일 경로로 대체해야 합니다.
        model_path = package_path + "/model/go1/go1_flat.pt"  # 모델 파일 경로
        model_state_dict = torch.load(model_path, map_location='cuda:0')

        actor_critic_model.load_state_dict(model_state_dict['model_state_dict'])
        actor_critic_model.eval()
        self.actor_model = actor_critic_model
        print("=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n")
        return

    def gather_observation(self, unitree_go1_orbit, disier_velo):
        # 각 관측치의 크기에 따라 빈 PyTorch tensor 생성
        base_lin_vel = unitree_go1_orbit._data.root_lin_vel_b[0]
        base_ang_vel = unitree_go1_orbit._data.root_ang_vel_b[0]
        projected_gravity = unitree_go1_orbit._data.projected_gravity_b[0]
        velocity_commands = torch.tensor(disier_velo, dtype=torch.float32)
        joint_pos_correction = torch.tensor([0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5], dtype=torch.float32)
        joint_pos = unitree_go1_orbit._data.joint_pos[0] - joint_pos_correction
        joint_vel = unitree_go1_orbit._data.joint_vel[0]
        actions = self.pre_actions

        observation_tensor = torch.cat([
            base_lin_vel,
            base_ang_vel,
            projected_gravity,
            velocity_commands,
            joint_pos,
            joint_vel,
            actions
        ])
        
        return observation_tensor

    def inference_actor_model(self, obs: torch.tensor) -> np.ndarray:
        action = self.actor_model.act_inference(obs)
        self.pre_actions = action
        action = action / 4
        # TODO: 이것도 config로 가는건 어떨까요?
        """
        add defalt position
        [FL_hip,  FR_hip,   RL_hip,   RR_hip,
        FL_thigh, FR_thigh, RL_thigh, RR_thigh,
        FL_calf,  FR_calf,  RL_calf,  RR_calf,]
        """
        action += torch.tensor([0.1, -0.1,  0.1, -0.1, 
                                0.8,  0.8,  1.0,  1.0, 
                               -1.5, -1.5, -1.5, -1.5])
        return action.detach().numpy()
