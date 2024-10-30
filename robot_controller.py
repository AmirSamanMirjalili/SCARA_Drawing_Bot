import numpy as np
import os

class SCARAController:
    def __init__(self, L1=0.4, L2=0.4):
        self.L1 = L1
        self.L2 = L2
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def calculate_ik(self, target_pos):
        """Analytical inverse kinematics for SCARA robot"""
        x, y, z = target_pos
        
        # Calculate the position for the horizontal plane (x-y)
        r = np.sqrt(x*x + y*y)
        
        # Check if target is reachable
        if r > (self.L1 + self.L2) - 0.01:
            print(f"Target position {target_pos} is too far!")
            return None
        if r < abs(self.L1 - self.L2) + 0.01:
            print(f"Target position {target_pos} is too close!")
            return None
        
        # Calculate theta2 using cosine law
        cos_theta2 = (r*r - self.L1*self.L1 - self.L2*self.L2) / (2*self.L1*self.L2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        theta2 = np.arccos(cos_theta2)
        
        # Calculate theta1
        beta = np.arctan2(self.L2 * np.sin(theta2), self.L1 + self.L2 * np.cos(theta2))
        theta1 = np.arctan2(y, x) - beta
        
        # Normalize angles
        theta1 = self.normalize_angle(theta1)
        theta2 = self.normalize_angle(theta2)
        
        # Z-axis control
        joint3_pos = np.clip(z + 0.12, -0.2, 0)
        
        print(f"IK solution: theta1={np.degrees(theta1):.2f}°, theta2={np.degrees(theta2):.2f}°, z={joint3_pos:.3f}m")
        
        return np.array([theta1, theta2, joint3_pos]) 