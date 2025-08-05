import numpy as np
from .RRT import RRT

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
    
    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose):

        valid = True # flag to indicate if the pose is valid

        # TODO: convert world robot position to map coordinates using method __position_to_map__
        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute)
        if self.distance > 0:
            robot_cell_occupancy = np.floor(self.distance/self.resolution).astype(int)
            for i in range(-robot_cell_occupancy, robot_cell_occupancy+1):
                for j in range(-robot_cell_occupancy, robot_cell_occupancy+1):
                    if np.linalg.norm([i,j]) <= robot_cell_occupancy: # Check if the cell is inside the circle
                        cell = self.__position_to_map__(pose + np.array([i*self.resolution, j*self.resolution]))
                        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
                        # If checked position is outside the map bounds consider it as unknown.
                        if cell is None or self.map[cell[0],cell[1]] == -1: # Cell outside the map or unknown
                            valid = self.is_unknown_valid
                        elif self.map[cell[0],cell[1]] == 100: # Occupied
                            return False
        return valid # Free

    # Given a path, returns true if the path is not in collision and false othewise.
    def check_path(self, path):
        if not self.there_is_map:
            return True  # Assume valid if no map
        
        step_size = 2*self.distance
        for i in range(1, len(path)):
            # Calculate distance between points
            segment_length = np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))
            if segment_length == 0:
                continue
                
            # Calculate number of steps needed
            num_steps = int(np.ceil(segment_length / step_size))
            if num_steps == 0:
                num_steps = 1
                
            # Check points along the segment
            for t in np.linspace(0, 1, num_steps + 1):
                p = np.array(path[i-1]) + t * (np.array(path[i]) - np.array(path[i-1]))
                if not self.is_valid(p):
                    return False
        return True
    
    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates. 
        m = (p-self.origin)/self.resolution
        # Return None if the position is outside the map bounds
        if m[0] < 0 or m[1] < 0 or m[0] >= self.map.shape[0] or m[1] >= self.map.shape[1]:
            return None
        else:
            return np.floor(m).astype(int)
    
# Define Planner class (you can take code from Autonopmous Systems course!)
class Planner:
    def  __init__(self, state_validity_checker, max_iterations=5000, dominion=[-10, 10, -10, 10], step_size=0.2,p=0.2):
        self.svc = state_validity_checker
        self.max_iterations = max_iterations
        self.dominion = dominion
        self.step_size = step_size
        self.sample_goal_prob = p
  
    def compute_path(self, q_start, q_goal):
        # Implement RRT algorithm that utilizes the state_validity_checker object to find a path from q_start to q_goal.
        # Use the state_validity_checker object to see if a position is valid or not.
        rrt = RRT(q_start,q_goal,self.dominion,self.max_iterations,self.step_size,self.svc,self.sample_goal_prob)
        return rrt.rrt_planning()

    def smooth_path(self,path):
        # Optionally, you can implement a function to smooth the RRT path.
        if len(path)>0:
            next_node = path[-1] #Set as goal
            i = 0
            smooth_path = [path[-1]] # Add goal to smooth-path
            while (smooth_path[-1]!=path[0]).all(): # Check if start is reached
                if self.svc.check_path([path[i],next_node]): # Check if a direct path is free from node i to next_node
                    smooth_path.append(path[i]) 
                    next_node = path[i] # Set next_node as node i and repeat
                    i = 0
                else:
                    i+=1
            smooth_path.reverse() # Reverse path 
            return smooth_path
    
        else:
            return [] 


# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, dominion, max_iterations=5000):
    # TODO: Plan a path from start_p to goal_p inside dominiont using a Planner Object and the 
    # StateValidityChecker Object previously defined.
    # TODO: if solved, return a list with the [x, y] points in the solution path.
    # example: [[x1, y1], [x2, y2], ...]
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!
    planner = Planner(state_validity_checker, max_iterations, dominion)
    path = planner.compute_path(start_p, goal_p)
    path = planner.smooth_path(path)
    return path


# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # Get current position and goal position
    current_x , current_y, current_yaw = current[0], current[1], current[2]
    goal_x, goal_y = goal[0], goal[1]

    # Compute the angle between the current position and the goal
    angle = np.arctan2(goal_y - current_y, goal_x - current_x)

    # TODO: Use a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # To avoid strange curves, first correct the orientation and then the distance.
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    w = Kw * wrap_angle(current_yaw - angle)
    if np.abs(w) > 0.1: # If the robot is not oriented towards the goal, do not move forward
        v = 0
    else:
        v = Kv * np.linalg.norm([goal_x - current_x, goal_y - current_y])
    
    # This function should return only  linear velocity (v) and angular velocity (w)
    return v, w
