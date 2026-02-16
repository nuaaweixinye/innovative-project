import numpy as np
import time
import airsim

class DroneEnv:
    def __init__(self):
        # Initialize AirSim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        # Enable API control
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        
        # Move to a safe altitude to prevent ground collisions
        self.client.moveToZAsync(-3, 2).join()

        # Action space: [Throttle Up, Throttle Down, Move Forward, Move Backward, Hover]
        self.action_space = [0, 1, 2, 3, 4]  
        self.observation_space = (5,)  # [pos.x, pos.y, pos.z, vel.x, vel.y]

    def get_state(self):
        # Get current position and velocity
        state = self.client.getMultirotorState()
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        return np.array([pos.x_val, pos.y_val, pos.z_val, vel.x_val, vel.y_val])

    def check_collision(self):
        # Retrieve collision information from AirSim
        collision_info = self.client.simGetCollisionInfo()
        return collision_info.has_collided

    def calculate_reward(self, state):
        z = state[2]  # Altitude (z-value)

        if self.check_collision():  # Collision detected
            return -50, True  # Large penalty and end episode
        
        if z < -5:  # Too low (risk of crashing)
            return -10, True  
        elif z > -1.5:  # Too high (out of desired range)
            return -5, False  
        else:  # Optimal height
            return 10, False  

    def step(self, action):
        # Ensure API control is still enabled
        if not self.client.isApiControlEnabled():
            print("Re-enabling API control...")
            self.client.enableApiControl(True)
            self.client.armDisarm(True)

        # Execute action
        if action == 0:  # Throttle Up
            self.client.moveByVelocityZAsync(0, 0, -1, 3).join()  # Increased duration
        elif action == 1:  # Throttle Down
            self.client.moveByVelocityZAsync(0, 0, 1, 3).join()
        elif action == 2:  # Move Forward
            self.client.moveByVelocityAsync(1, 0, 0, 3).join()
        elif action == 3:  # Move Backward
            self.client.moveByVelocityAsync(-1, 0, 0, 3).join()
        elif action == 4:  # Hover
            self.client.hoverAsync().join()

        time.sleep(1)  

        # Get new state and check for collisions
        state = self.get_state()
        reward, done = self.calculate_reward(state)

        print(f"New State: {state}, Reward: {reward}, Done: {done}")  # Debugging log

        return state, reward, done, {}

    def reset(self):
        if self.check_collision():
            print("Collision detected! Resetting drone...")
            self.client.reset()
            time.sleep(1)  # Allow time to stabilize
            self.client.enableApiControl(True)
            self.client.armDisarm(True)
            self.client.takeoffAsync().join()
            self.client.moveToZAsync(-3, 2).join()  # Move up after reset

        return self.get_state()  
