# Autonomous Drone Navigation using Deep Reinforcement Learning  

This project uses **Deep Reinforcement Learning (DRL)** to train a drone for **autonomous navigation and obstacle avoidance** in a simulated environment.  
This project implements Deep Reinforcement Learning for autonomous drone navigation in AirSim. The model learns obstacle avoidance and optimal pathfinding using DQN.  

 Features  
 **Deep Q-Network (DQN) / PPO-based Navigation**  
 **Obstacle Detection & Avoidance**   
 **Simulation in AirSim**  



Installation  
# Clone the Repository  
git clone https://github.com/harshraj2008/drone_navigation_project.git


cd drone_navigation_project

# Install Dependencies
Ensure you have Python 3.8+ installed, 


# Train the Model
Run the training script in the terminal :
python train_dqn.py
This will train the DQN agent and save the model as dqn_model.pth.

# Setup the Drone Environment
Run the script in the terminal :
python Drone_env.py


# copy this code in your settings.json file in your Airsim folder to enable drone flight
{
  "SettingsVersion": 1.2,
  "SimMode": "Multirotor",
  "ClockSpeed": 1,
  "LocalHostIp": "127.0.0.1",
  "ViewMode": "FlyWithMe",
  "RpcEnabled": true,
  "Vehicles": {
    "Drone1": {
      "VehicleType": "SimpleFlight",
      "AutoCreate": true
    }
  },
  "DefaultVehicleConfig": "Drone1",
  "Recording": {
    "RecordInterval": 0.05,
    "Cameras": [
      { "CameraName": "front_center", "ImageType": 0, "PixelsAsFloat": false, "Compress": true }
    ]
  }
}

**This will enable your  your drone to fly in the application**

# Train the RL Model for drone environment
Run the training script in the terminal :
python drone_train.py
then open airsimNH application  in airsimNH folder 
This will train the DQN agent and save the model as dqn_model.pth.

# After training, evaluate the model using:
python test_dqn.py

# Drive Link for Video for running the simulation along with Training and Testing data video :
https://drive.google.com/drive/folders/13mVUcty2tl90VBDDeFINvF75wmFfbwMI?usp=drive_link



# The script checks:

Environment initialization

Model inference

Trained model action prediction

# Implementation Details
 State Representation: The drone's state consists of its position, velocity, and nearby obstacles.

 Action Space: The agent can move forward, left, right, or hover.

 Reward Function: Encourages efficient movement while penalizing collisions.

 Neural Network Architecture: A fully connected DQN with two hidden layers.

# Results
The drone successfully learns collision-free navigation.

Performance improves as epsilon decays, reducing random exploration.

Training logs and evaluation results can be found in train_dqn.log.

# Future Enhancements
 Integrate Lidar/GPS sensor data for real-world testing.

 Implement Deep Deterministic Policy Gradient (DDPG) for continuous action control.

 Train using AirSim or Gazebo for realistic simulations.

# Acknowledgments
Deep Q-Networks (DQN) Paper: Mnih et al., 2015

Reinforcement Learning Resources: OpenAI Gym
