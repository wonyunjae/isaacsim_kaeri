import torch
import numpy as np
from rsl_rl.modules import ActorCritic

class RslRLBridge:
    def __init__(self):
        self.pre_actions = torch.zeros(12) # 초기 행동 벡터 설정
        self.actor_model = None # 모델은 load_pre_trained_model을 통해 로드됩니다.

    def move(self, unitree_go1_orbit, desired_vel, dt):
        # 1. observation
        obs = self.gather_observation(unitree_go1_orbit, desired_vel)

        # 2. inference
        action = self.inference_actor_model(obs)

        # 3. action
        unitree_go1_orbit.set_joint_position_target(torch.tensor(action))

        # 4. update
        unitree_go1_orbit.write_data_to_sim()
        unitree_go1_orbit.update(dt)

    def load_pre_trained_model(self, package_path):
        # model parameter
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

        # trained model load
        model_path = package_path + "/model/go1/go1_flat.pt"  # 모델 파일 경로
        model_state_dict = torch.load(model_path, map_location='cuda:0')

        actor_critic_model.load_state_dict(model_state_dict['model_state_dict'])
        actor_critic_model.eval()
        self.actor_model = actor_critic_model
        return

    def gather_observation(self, unitree_go1_orbit, desired_vel, prev_action):
        # 각 관측치의 크기에 따라 빈 PyTorch tensor 생성
        base_lin_vel = unitree_go1_orbit._data.root_lin_vel_b[0] # 3
        base_ang_vel = unitree_go1_orbit._data.root_ang_vel_b[0] # 3
        projected_gravity = unitree_go1_orbit._data.projected_gravity_b[0]# 3
        velocity_commands = torch.tensor(desired_vel, dtype=torch.float32) # 3
        joint_pos_correction = torch.tensor([0.1, -0.1, 0.1, -0.1, 0.8, 0.8, 1.0, 1.0, -1.5, -1.5, -1.5, -1.5], dtype=torch.float32) 
        joint_pos = unitree_go1_orbit._data.joint_pos[0] - joint_pos_correction # 12
        joint_vel = unitree_go1_orbit._data.joint_vel[0] # 12
        
        observation_tensor = torch.cat([
            base_lin_vel,
            base_ang_vel,
            projected_gravity,
            velocity_commands,
            joint_pos,
            joint_vel,
            prev_action
        ])
        
        return observation_tensor

    def inference_actor_model(self, obs: torch.tensor) -> np.ndarray:
        action = self.actor_model.act_inference(obs)
        
        self.pre_actions = action
        action = action * 0.25

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
        print("-- infer : ")
        print(action)
        return action.detach().numpy()
