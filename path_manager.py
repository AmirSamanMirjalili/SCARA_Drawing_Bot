class PathManager:
    def __init__(self):
        self.current_path_index = 0
        self.current_point_index = 0
        self.paths = []
        
    def load_paths(self, paths):
        self.paths = paths
        self.current_path_index = 0
        self.current_point_index = 0
    
    def get_target_position(self):
        if not self.paths or self.current_path_index >= len(self.paths):
            return None
        
        current_path = self.paths[self.current_path_index]
        if self.current_point_index >= len(current_path):
            return None
        
        return current_path[self.current_point_index]
    
    def advance_to_next_point(self):
        self.current_point_index += 1
        if self.current_point_index >= len(self.paths[self.current_path_index]):
            self.current_point_index = 0
            self.current_path_index += 1
            return True  # Path completed
        return False  # Still on current path 