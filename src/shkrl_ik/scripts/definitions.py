#!/usr/bin/env python3

#   CS22B1090
#   Shubh Khandelwal

from ament_index_python import get_package_share_directory
from collections import deque, namedtuple
import gymnasium as gym
from shkrl_msgs.srv import GetTCP
import numpy as np
import os
import random
import rclpy
from rclpy.node import Node
import torch
import torch.nn as nn
import torch.optim as optim

class ManipulatorEnv(gym.Env, Node):

    def __init__(self):

        Node.__init__(self, "manipulator_node")
        gym.Env.__init__(self)

        self.lower_limits = np.array([-3.142, 0, 0, 0], dtype = np.float32)
        self.upper_limits = np.array([3.142, 1.571, 1.571, 1.571], dtype = np.float32)

        self.observation_space = gym.spaces.Box(low = -np.inf, high = np.inf, shape = (6, ), dtype = np.float32)
        self.action_space = gym.spaces.Discrete(81)

        self.joints = np.zeros(4, dtype = np.float32)
        self.gripper = np.zeros(3, dtype = np.float32)
        self.goal = np.zeros(3, dtype = np.float32)
        self.initial_d = 0
        self.steps = 0

        self.gripper_client = self.create_client(GetTCP, '/get_tcp')
        self.request = GetTCP.Request()
    
    def _get_gripper(self):
        self.request.joints = self.joints
        future = self.gripper_client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        while future.result() is None or not future.result().success:
            future = self.gripper_client.call_async(self.request)
            rclpy.spin_until_future_complete(self, future)
        self.gripper = future.result().gripper
        
    def reset(self, seed = None, options = None):
        super().reset(seed = seed)
        self.joints = np.random.uniform(low = self.lower_limits, high = self.upper_limits, size = (4, )).astype(np.float32)
        self.goal = np.random.uniform(low = [-0.4, -0.4, 0.4], high = [0.4, 0.4, 0.85], size = (3, )).astype(np.float32)
        self.steps = 0
        self._get_gripper()
        self.initial_d = np.sqrt(np.sum((self.gripper - self.goal) ** 2))
        state = np.concatenate((self.gripper, self.goal), axis = 0)
        return state, {}
    
    def step(self, action):

        step_size = 0.01
        action1 = action % 3
        action = (action - action1) / 3
        action2 = action % 3
        action = (action - action2) / 3
        action3 = action % 3
        action = (action - action3) / 3
        action4 = action % 3
        delta = np.array([action1, action2, action3, action4], dtype = np.float32) - 1
        self.joints += delta * step_size
        self.joints = np.clip(self.joints, self.lower_limits, self.upper_limits)
        self.steps += 1

        self._get_gripper()
        state = np.concatenate((self.gripper, self.goal), axis = 0)

        terminated = False
        truncated = False
        reward = 0
        for i in range(4):
            reward -= (i + 1) * (abs(delta[3 - i]) + 1)
        distance = np.sqrt(np.sum((self.gripper - self.goal) ** 2))
        reward -= distance * 10
        reward += (distance - self.initial_d) * 50
        if distance <= 0.1:
            reward = 100000
            terminated = True
        elif self.steps >= 1000:
            truncated = True
        self.initial_d = distance
        
        return state, reward, terminated, truncated, {}

class DeepQNetwork(nn.Module):

    def __init__(self, observation_size, action_size):
        super(DeepQNetwork, self).__init__()
        self.layers = nn.Sequential(
            nn.Linear(observation_size, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, action_size)
        )
    
    def forward(self, x):
        return self.layers(x)

class ReplayBuffer:

    def __init__(self, buffer_size, batch_size):
        self.memory = deque(maxlen = buffer_size)
        self.batch_size = batch_size
        self.experience = namedtuple("Experience", field_names = ["state", "action", "reward", "next_state", "done"])

    def __len__(self):
        return len(self.memory)

    def add(self, state, action, reward, next_state, done):
        self.memory.append(self.experience(state, action, reward, next_state, done))

    def sample(self):
        experiences = random.sample(self.memory, k = self.batch_size)
        states = torch.stack([torch.from_numpy(e.state) for e in experiences if e is not None]).float()
        actions = torch.stack([torch.from_numpy(np.array([e.action], dtype = np.int32)) for e in experiences if e is not None]).long()
        rewards = torch.stack([torch.from_numpy(np.array([e.reward], dtype = np.float32)) for e in experiences if e is not None]).float()
        next_states = torch.stack([torch.from_numpy(e.next_state) for e in experiences if e is not None]).float()
        dones = torch.stack([torch.from_numpy(np.array([float(e.done)], dtype = np.float32)) for e in experiences if e is not None]).float()
        return (states, actions, rewards, next_states, dones)

class DQNAgent:

    def __init__(self, state_size, action_size, buffer_size = int(1e5), batch_size = 32, gamma = 0.99, tau = 1e-3, lr = 5e-4, update_after_steps = 4):

        self.state_size = state_size
        self.action_size = action_size
        self.batch_size = batch_size
        self.gamma = gamma
        self.tau = tau
        self.lr = lr
        self.update_after_steps = update_after_steps

        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.memory = ReplayBuffer(buffer_size, batch_size)
        self.network_local = DeepQNetwork(state_size, action_size).to(self.device)
        self.network_target = DeepQNetwork(state_size, action_size).to(self.device)
        self.optimizer = optim.Adam(self.network_local.parameters(), lr = self.lr)
        self.steps = 0

    def act(self, state, epsilon):
        state = torch.from_numpy(state).float()
        state = state.unsqueeze(0).to(self.device)
        self.network_local.eval()
        with torch.no_grad():
            action_values = self.network_local(state)
        self.network_local.train()
        if random.random() > epsilon:
            return torch.argmax(action_values).item()
        else:
            return random.randrange(self.action_size)
        
    def soft_update(self):
        for target_parameters, local_parameters in zip(self.network_target.parameters(), self.network_local.parameters()):
            target_parameters.data.copy_(self.tau * local_parameters.data + (1.0 - self.tau) * target_parameters.data)
    
    def learn(self, experiences):

        states, actions, rewards, next_states, dones = experiences
        states = states.to(self.device)
        actions = actions.to(self.device)
        rewards = rewards.to(self.device)
        next_states = next_states.to(self.device)
        dones = dones.to(self.device)

        targets_next = self.network_target(next_states).detach().max(1)[0].unsqueeze(1)
        target_values = rewards + (self.gamma * targets_next * (1 - dones))
        predicted_values = self.network_local(states).gather(1, actions)

        loss = nn.functional.mse_loss(predicted_values, target_values)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        self.soft_update()

    def step(self, state, action, reward, next_state, done):
        self.memory.add(state, action, reward, next_state, done)
        self.steps = (self.steps + 1) % self.update_after_steps
        if self.steps == 0:
            if len(self.memory) > self.batch_size:
                experiences = self.memory.sample()
                self.learn(experiences)
    
    def save(
        self,
        path = os.path.abspath(os.path.join(get_package_share_directory("control"), "..", "..", "..", "..", "src", "control", "models")),
        filename_prefix = "dqn_agent"
    ):
        if not os.path.exists(path):
            os.makedirs(path)
        local_path = os.path.join(path, f"{filename_prefix}_local.pth")
        target_path = os.path.join(path, f"{filename_prefix}_target.pth")
        optimizer_path = os.path.join(path, f"{filename_prefix}_optimizer.pth")
        torch.save(self.network_local.state_dict(), local_path)
        torch.save(self.network_target.state_dict(), target_path)
        torch.save(self.optimizer.state_dict(), optimizer_path)
    
    def load(
        self,
        path = os.path.join(get_package_share_directory("control"), "models"),
        filename_prefix = "dqn_agent",
        eval_mode = True
    ):
        local_path = os.path.join(path, f"{filename_prefix}_local.pth")
        target_path = os.path.join(path, f"{filename_prefix}_target.pth")
        optimizer_path = os.path.join(path, f"{filename_prefix}_optimizer.pth")
        if not os.path.exists(local_path) or not os.path.exists(target_path) or not os.path.exists(optimizer_path):
            print(f"Warning: Model files not found at {path}. Use 'colcon build' to install the models in the share directory.")
            return
        self.network_local.load_state_dict(torch.load(local_path, map_location = self.device))
        self.network_target.load_state_dict(torch.load(target_path, map_location = self.device))
        if eval_mode:
            self.network_local.eval()
            self.network_target.eval()
        else:
            self.optimizer.load_state_dict(torch.load(optimizer_path))
            self.network_local.train()
            self.network_target.train()