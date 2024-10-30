import mujoco
import mujoco.viewer
import numpy as np
import time
from dxf_parser import load_dxf_text, scale_and_offset_paths
import os
from create_text_dxf import create_text_dxf

# Maximum number of trace points to store
MAX_TRACE_POINTS = 1000
trace_points = []

# Path following parameters
current_path_index = 0
current_point_index = 0
paths = []
DISTANCE_THRESHOLD = 0.01

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calculate_scara_ik(target_pos, L1=0.4, L2=0.4):
    """
    Analytical inverse kinematics for SCARA robot
    L1, L2: lengths of the two links
    Returns: [theta1, theta2, z_pos]
    """
    x, y, z = target_pos
    
    # Calculate the position for the horizontal plane (x-y)
    r = np.sqrt(x*x + y*y)
    
    # Check if target is reachable
    if r > (L1 + L2) - 0.01:  # Add small tolerance
        print(f"Target position {target_pos} is too far!")
        return None
    if r < abs(L1 - L2) + 0.01:  # Add small tolerance
        print(f"Target position {target_pos} is too close!")
        return None
    
    # Calculate theta2 using cosine law
    cos_theta2 = (r*r - L1*L1 - L2*L2) / (2*L1*L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  # Ensure value is in valid range
    theta2 = np.arccos(cos_theta2)  # elbow-up solution
    
    # Calculate theta1
    beta = np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    theta1 = np.arctan2(y, x) - beta
    
    # Normalize angles to [-pi, pi]
    theta1 = normalize_angle(theta1)
    theta2 = normalize_angle(theta2)
    
    # Z-axis control (joint3) is direct
    joint3_pos = np.clip(z + 0.12, -0.2, 0)  # Add tool offset and clip to joint limits
    
    print(f"IK solution: theta1={np.degrees(theta1):.2f}°, theta2={np.degrees(theta2):.2f}°, z={joint3_pos:.3f}m")
    
    return np.array([theta1, theta2, joint3_pos])

def render_debug_info(viewer, model, data, target_pos):
    """Render debug visualization"""
    if target_pos is not None:
        # Render target position
        viewer.user_scn.ngeom = 0
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[0],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.02, 0, 0],
            pos=target_pos,
            mat=np.eye(3).flatten(),
            rgba=[0, 1, 0, 0.5]  # Green sphere for target
        )
        viewer.user_scn.ngeom += 1
        
        # Render current end effector position
        end_effector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
        current_pos = data.xpos[end_effector_id].copy()
        current_pos[2] += -0.12
        
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[1],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.02, 0, 0],
            pos=current_pos,
            mat=np.eye(3).flatten(),
            rgba=[0, 0, 1, 0.5]  # Blue sphere for current position
        )
        viewer.user_scn.ngeom += 1

def load_text_paths(filename):
    global paths
    raw_paths = load_dxf_text(filename)
    paths = scale_and_offset_paths(raw_paths)

def get_target_position():
    if not paths or current_path_index >= len(paths):
        return None
    
    current_path = paths[current_path_index]
    if current_point_index >= len(current_path):
        return None
    
    return current_path[current_point_index]

def add_trace_point(model, data):
    end_effector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
    end_effector_pos = data.xpos[end_effector_id].copy()
    tool_tip_pos = end_effector_pos + np.array([0, 0, -0.12])
    
    trace_points.append(tool_tip_pos)
    if len(trace_points) > MAX_TRACE_POINTS:
        trace_points.pop(0)

def render_trace(viewer):
    viewer.user_scn.ngeom = 0
    for i, point in enumerate(trace_points):
        if i >= viewer.user_scn.maxgeom:
            break
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[i],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.005, 0, 0],
            pos=point,
            mat=np.eye(3).flatten(),
            rgba=[1, 0, 0, 0.5]
        )
        viewer.user_scn.ngeom += 1

def main():
    global current_path_index, current_point_index
    
    # Create DXF file if it doesn't exist
    dxf_file = 'text.dxf'
    if not os.path.exists(dxf_file):
        print(f"Creating {dxf_file}...")
        create_text_dxf()
    
    # Load the DXF file
    print(f"Loading {dxf_file}...")
    load_text_paths(dxf_file)
    
    # Load the model
    model = mujoco.MjModel.from_xml_path('scara_robot.xml')
    data = mujoco.MjData(model)

    # Initialize position
    data.qpos[:] = [0, 0, 0]  # Start at home position
    mujoco.mj_forward(model, data)

    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Get next target position
            target_pos = get_target_position()
            
            if target_pos is not None:
                # Calculate joint angles using analytical IK
                joint_angles = calculate_scara_ik(target_pos)
                
                if joint_angles is not None:
                    # Apply joint positions through control
                    data.ctrl[:] = joint_angles
                    
                    # Debug visualization
                    render_debug_info(viewer, model, data, target_pos)
                    
                    # Check if target is reached
                    end_effector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
                    current_pos = data.xpos[end_effector_id].copy()
                    current_pos[2] += -0.12
                    
                    distance = np.linalg.norm(target_pos - current_pos)
                    print(f"\nCurrent joints: {np.degrees(data.qpos)}")
                    print(f"Target: {target_pos}")
                    print(f"Current: {current_pos}")
                    print(f"Distance: {distance}")
                    
                    if distance < DISTANCE_THRESHOLD:
                        print("Target reached, moving to next point")
                        current_point_index += 1
                        if current_point_index >= len(paths[current_path_index]):
                            current_point_index = 0
                            current_path_index += 1
            
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Add current position to trace
            add_trace_point(model, data)
            
            # Render the trace
            render_trace(viewer)
            
            viewer.sync()
            time.sleep(0.01)

if __name__ == "__main__":
    main() 