import torch
import torch.optim as optim
import numpy as np
from Drone_env import DroneEnv
from train_dqn import DQN

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def train_dqn():
    env = DroneEnv()
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.n  # ✅ Fixed

    model = DQN(state_dim, action_dim).to(device)
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    # Epsilon-greedy parameters
    epsilon = 1.0  # Initial exploration rate
    epsilon_min = 0.01  # Minimum epsilon value
    epsilon_decay = 0.995  # Decay rate per episode

    num_episodes = 100

    # Reset training log before starting
    with open("train_dqn.log", "w") as log_file:
        log_file.write("Training started, resetting log...\n")
        log_file.write("======================================\n")

    for episode in range(num_episodes):
        state = env.reset()
        total_reward = 0
        done = False
        step_count = 0

        with open("train_dqn.log", "a") as log_file:  # Open in append mode for each episode
            while not done:
                state_tensor = torch.FloatTensor(state).unsqueeze(0).to(device)

                # Epsilon-greedy action selection
                if np.random.rand() < epsilon:
                    action = env.action_space.sample()  # Random action (exploration)
                else:
                    action = model(state_tensor).argmax().item()  # Best action (exploitation)

                # Perform action in environment
                next_state, reward, done, _ = env.step(action)
                total_reward += reward
                step_count += 1

                # Log Q-values predicted by the model
                q_values = model(state_tensor).detach().cpu().numpy()
                log_message = f"Episode {episode+1}, Step {step_count}: Predicted Q-values {q_values}, Action {action}, Reward {reward:.2f}\n"
                print(log_message, end="")  # Print Q-values in the terminal
                log_file.write(log_message)

                state = next_state

            # Log after every episode
            summary_message = f"✅ Episode {episode+1} Completed! Total Reward: {total_reward:.2f}\n"
            print(summary_message)
            log_file.write(summary_message)
            log_file.flush()

            # Decay epsilon
            epsilon = max(epsilon * epsilon_decay, epsilon_min)

    # Save model after training
    torch.save(model.state_dict(), "dqn_model.pth")
    print("💾 Model saved as dqn_model.pth")

if __name__ == "__main__":
    train_dqn()
