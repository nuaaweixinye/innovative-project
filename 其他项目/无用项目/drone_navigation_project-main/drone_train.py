import torch
import torch.optim as optim
import numpy as np
from Drone_env import DroneEnv
from train_dqn import DQN

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def train_dqn():
    env = DroneEnv()
    state_dim = env.observation_space[0]  
    action_dim = len(env.action_space) 

    model = DQN(state_dim, action_dim).to(device)
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    epsilon = 1.0  
    epsilon_min = 0.01  
    epsilon_decay = 0.995  

    num_episodes = 100
    reward_threshold = 200  

    with open("train_dqn.log", "w") as log_file:
        log_file.write("Training started, resetting log...\n")
        log_file.write("======================================\n")

    for episode in range(num_episodes):
        state = env.reset()
        total_reward = 0
        done = False
        step_count = 0

        while not done:
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(device)

            if np.random.rand() < epsilon:
                action = np.random.choice(len(env.action_space)) 
            else:
                action = model(state_tensor).argmax().item()  

            next_state, reward, done, _ = env.step(action)
            total_reward += reward
            step_count += 1

            # ✅ Print step-wise details (but don't save them)
            q_values = model(state_tensor).detach().cpu().numpy()
            log_message = f"Episode {episode+1}, Step {step_count}: Q-values {q_values}, Action {action}, Reward {reward:.2f}"
            print(log_message)  

            state = next_state

        # ✅ Save only episode summary to log
        with open("train_dqn.log", "a") as log_file:
            summary_message = f"Episode {episode+1} Completed! Total Reward: {total_reward:.2f}\n"
            print(summary_message)
            log_file.write(summary_message)
            log_file.flush()  # Ensures logs are written immediately

        epsilon = max(epsilon * epsilon_decay, epsilon_min)

        # ✅ Auto-stop if optimal reward is achieved
        if total_reward >= reward_threshold:
            print(f"✅ Optimal reward {total_reward} achieved! Stopping training early.")
            break

    print("Training completed.")

if __name__ == "__main__":
    train_dqn()
