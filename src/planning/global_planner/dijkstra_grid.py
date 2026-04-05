from .dijkstra import DijkstraPlanner

# Takes a pre-built grid and skips obstacle-map rebuild
class GridDijkstraPlanner(DijkstraPlanner):
    def __init__(self, obstacle_map, origin_x_m, origin_y_m, resolution):
        self.resolution = resolution # meters per cell
        self.robot_radius = 0.0 # grid already pre-built

        self.x_width = len(obstacle_map)
        self.y_width = len(obstacle_map[0]) if obstacle_map else 0

        self.min_x = origin_x_m
        self.min_y = origin_y_m
        
        self.max_x = self.min_x + self.x_width * resolution
        self.max_y = self.min_y + self.y_width * resolution

        self.obstacle_map = obstacle_map
        self.motion = self.get_motion_model()