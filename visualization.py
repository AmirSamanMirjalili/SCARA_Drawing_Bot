import mujoco
import numpy as np

class RobotVisualizer:
    def __init__(self, max_trace_points=1000):
        self.trace_points = []
        self.max_trace_points = max_trace_points
    
    def render_debug_info(self, viewer, model, data, target_pos):
        """Render debug visualization"""
        if target_pos is not None:
            viewer.user_scn.ngeom = 0
            # Target position sphere
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[0],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.02, 0, 0],
                pos=target_pos,
                mat=np.eye(3).flatten(),
                rgba=[0, 1, 0, 0.5]
            )
            viewer.user_scn.ngeom += 1
            
            # Current position sphere
            end_effector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
            current_pos = data.xpos[end_effector_id].copy()
            current_pos[2] += -0.12
            
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[1],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.02, 0, 0],
                pos=current_pos,
                mat=np.eye(3).flatten(),
                rgba=[0, 0, 1, 0.5]
            )
            viewer.user_scn.ngeom += 1
    
    def add_trace_point(self, model, data):
        end_effector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "end_effector")
        end_effector_pos = data.xpos[end_effector_id].copy()
        tool_tip_pos = end_effector_pos + np.array([0, 0, -0.12])
        
        self.trace_points.append(tool_tip_pos)
        if len(self.trace_points) > self.max_trace_points:
            self.trace_points.pop(0)
    
    def render_trace(self, viewer):
        """Render the trace with proper geometry management"""
        max_points = min(len(self.trace_points), viewer.user_scn.maxgeom - 2)
        
        for i in range(max_points):
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[i],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[0.002, 0, 0],
                pos=self.trace_points[i],
                mat=np.eye(3).flatten(),
                rgba=[1, 0, 0, 0.8]
            )
            viewer.user_scn.ngeom += 1 