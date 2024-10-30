import mujoco
import mujoco.viewer
import time
import os
import numpy as np
from robot_controller import SCARAController
from visualization import RobotVisualizer
from path_manager import PathManager
from dxf_parser import load_dxf_text, scale_and_offset_paths, print_path_info
from create_text_dxf import create_text_dxf
import sys

DISTANCE_THRESHOLD = 0.01

# Constants for pen movement
PEN_UP_HEIGHT = 0.05
PEN_DOWN_HEIGHT = 0.0
INTERPOLATION_POINTS = 10

# Get the absolute path to the project directory
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
# Add the project directory to Python path
sys.path.append(PROJECT_DIR)

# Update the XML path to use absolute path
MODEL_PATH = os.path.join(PROJECT_DIR, 'scara_robot.xml')

def check_model_file():
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model file not found at: {MODEL_PATH}")
        
def main():
    # Check for model file
    check_model_file()
    
    # Initialize components
    controller = SCARAController()
    visualizer = RobotVisualizer()
    path_manager = PathManager()
    
    # Create DXF file if it doesn't exist
    dxf_file = 'text.dxf'
    if not os.path.exists(dxf_file):
        print(f"Creating {dxf_file}...")
        create_text_dxf()
    
    # Load the DXF file with debug info
    print(f"Loading {dxf_file}...")
    raw_paths = load_dxf_text(dxf_file)
    paths = scale_and_offset_paths(
        raw_paths,
        scale=0.02,
        offset=[0.4, 0, 0],
        num_interpolation_points=INTERPOLATION_POINTS
    )
    print_path_info(paths)
    path_manager.load_paths(paths)
    
    # Load the model using absolute path
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    # Initialize position
    data.qpos[:] = [0, 0, 0]
    mujoco.mj_forward(model, data)

    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # Get next target position
            target_pos = path_manager.get_target_position()
            
            if target_pos is not None:
                # Calculate joint angles using analytical IK
                joint_angles = controller.calculate_ik(target_pos)
                
                if joint_angles is not None:
                    # Apply joint positions through control
                    data.ctrl[:] = joint_angles
                    
                    # Debug visualization
                    visualizer.render_debug_info(viewer, model, data, target_pos)
                    
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
                        path_completed = path_manager.advance_to_next_point()
                        if path_completed:
                            print("Path completed!")
            
            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Add current position to trace
            visualizer.add_trace_point(model, data)
            
            # Render the trace
            visualizer.render_trace(viewer)
            
            viewer.sync()
            time.sleep(0.01)

if __name__ == "__main__":
    main() 