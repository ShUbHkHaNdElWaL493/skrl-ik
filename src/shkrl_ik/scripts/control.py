#!/usr/bin/env python3

#   CS22B1090
#   Shubh Khandelwal

from shkrl_ik.scripts.definitions import DQNAgent, ManipulatorEnv
import numpy as np
import rclpy
import time

def main(args = None):

    rclpy.init(args = args)
    manipulator_env = ManipulatorEnv()
    model = DQNAgent(int(np.prod(manipulator_env.observation_space.shape)), manipulator_env.action_space.n)
    model.load()

    try:

        episodes = 10
        for episode in range(episodes):

            manipulator_env.get_logger().info(f"\nEpisode: {episode + 1}/{episodes}")
            state, info = manipulator_env.reset()
            manipulator_env.get_logger().info(f"Episode: {episode + 1}| Initial Gripper Position: {manipulator_env.gripper}")
            manipulator_env.get_logger().info(f"Episode: {episode + 1}| Initial Goal Position: {manipulator_env.goal}")

            episode_reward = 0
            done = False
            while not done:
                action = model.act(state, epsilon = 0.01)
                next_state, reward, terminated, truncated, info = manipulator_env.step(action)
                episode_reward += reward
                done = terminated or truncated
                model.step(state, action, reward, next_state, done)
                state = next_state

            manipulator_env.get_logger().info(f"Episode: {episode + 1} | Steps: {manipulator_env.steps} | Reward: {episode_reward}")
            if terminated:
                manipulator_env.get_logger().info("Episode terminated.")
            elif truncated:
                manipulator_env.get_logger().info("Episode truncated.")
            time.sleep(1)

    except Exception as e:
        manipulator_env.get_logger().error(f"Error: {e}", once=True)
    finally:
        manipulator_env.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()