import torch
import torch.nn as nn
import numpy as np
from Drone_env import DroneEnv  # Import your custom environment

# Define the same DQN model structure used during training
class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(DQN, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, output_dim)
        )

    def forward(self, x):
        return self.fc(x)

def test_model(model_path="drone_dqn.pth", test_episodes=10):
    env = DroneEnv()
    state_size = env.observation_space[0]
    action_size = len(env.action_space)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    model = DQN(state_size, action_size).to(device)
    model.load_state_dict(torch.load(model_path))
    model.eval()

    for episode in range(test_episodes):
        state = env.reset()
        state = torch.FloatTensor(state).to(device)
        total_reward = 0

        print(f"\n--- Episode {episode + 1} ---")
        for step in range(300):
            with torch.no_grad():
                action = torch.argmax(model(state)).item()

            next_state, reward, done, _ = env.step(action, episode, step, 1)
            next_state = torch.FloatTensor(next_state).to(device)

            total_reward += reward
            state = next_state

            if done:
                print(f"Episode {episode + 1} ended after {step + 1} steps. Total reward: {total_reward:.2f}")
                break

        if not done:
            print(f"Episode {episode + 1} completed. Total reward: {total_reward:.2f}")

    env.client.reset()
    print("\nTesting completed!")

if __name__ == "__main__":
    test_model()
