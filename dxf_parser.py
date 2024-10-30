import ezdxf
import numpy as np

def load_dxf_text(filename):
    """Load text paths from DXF file and return as list of points"""
    doc = ezdxf.readfile(filename)
    msp = doc.modelspace()
    
    paths = []
    current_path = []
    
    # Extract all entities
    for entity in msp:
        if entity.dxftype() == 'LINE':
            # Create a path for each line
            line_path = [
                np.array([entity.dxf.start.x, entity.dxf.start.y]),
                np.array([entity.dxf.end.x, entity.dxf.end.y])
            ]
            paths.append(np.array(line_path))
            
        elif entity.dxftype() == 'LWPOLYLINE':
            # Create a path for the polyline
            points = list(entity.get_points())
            polyline_path = [np.array([p[0], p[1]]) for p in points]
            paths.append(np.array(polyline_path))
            
    return paths

def interpolate_points(start, end, num_points=10):
    """Create interpolated points between start and end"""
    return np.array([
        start + (end - start) * i / (num_points - 1)
        for i in range(num_points)
    ])

def add_pen_movements(path, pen_up_height=0.05, pen_down_height=0.0):
    """Add pen up/down movements to a path"""
    result = []
    
    # Start point with pen up
    start = np.array([path[0][0], path[0][1], pen_up_height])
    result.append(start)
    
    # Move down to first point
    down = np.array([path[0][0], path[0][1], pen_down_height])
    result.append(down)
    
    # Add path points
    for point in path:
        point_3d = np.array([point[0], point[1], pen_down_height])
        result.append(point_3d)
    
    # Move up at end
    up = np.array([path[-1][0], path[-1][1], pen_up_height])
    result.append(up)
    
    return np.array(result)

def scale_and_offset_paths(paths, scale=0.02, offset=[0.4, 0, 0], num_interpolation_points=10):
    """Scale paths and add pen movements"""
    processed_paths = []
    
    for path in paths:
        # Scale XY coordinates
        scaled_path = path * scale
        
        # Interpolate between points
        interpolated_points = []
        for i in range(len(scaled_path) - 1):
            points = interpolate_points(
                scaled_path[i], 
                scaled_path[i + 1],
                num_interpolation_points
            )
            interpolated_points.extend(points)
        
        # Add pen movements and Z coordinate
        path_with_pen = add_pen_movements(np.array(interpolated_points))
        
        # Add offset
        offset_path = path_with_pen + np.array(offset)
        processed_paths.append(offset_path)
        
        # Add a transition path to the next starting point if there is one
        if len(processed_paths) > 0 and len(paths) > len(processed_paths):
            next_path = paths[len(processed_paths)] * scale
            transition = np.array([
                offset_path[-1],  # Last point of current path (pen up)
                np.array([next_path[0][0] * scale + offset[0],
                         next_path[0][1] * scale + offset[1],
                         offset[2] + 0.05])  # First point of next path (pen up)
            ])
            processed_paths.append(transition)
    
    return processed_paths

def print_path_info(paths):
    """Debug function to print information about the paths"""
    print(f"\nPath Information:")
    print(f"Number of paths: {len(paths)}")
    for i, path in enumerate(paths):
        print(f"\nPath {i}:")
        print(f"Number of points: {len(path)}")
        print(f"Start point: {path[0]}")
        print(f"End point: {path[-1]}")
        print(f"Z range: {np.min(path[:, 2]):.3f} to {np.max(path[:, 2]):.3f}")