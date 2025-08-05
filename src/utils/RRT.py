import numpy as np
from .Point import Point

class RRT:
    """Rapidly-Exploring Random Trees (RRT) planner."""

    # Constructor
    def __init__(self, start, goal, dominion, max_iter, step_size, state_validity_checker, sample_goal_prob=0.2, distance_threshold=0.1):
        # Initialize start and goal points
        self.start = Point(start[0],start[1])
        self.goal = Point(goal[0],goal[1])
        # Define the domain boundaries
        self.x_domain = dominion[0:2]
        self.y_domain = dominion[2:4]
        # Set the maximum number of iterations and step size
        self.max_iter = max_iter
        self.step_size = step_size
        # Probability of sampling the goal
        self.sample_goal_prob = sample_goal_prob
        # Distance threshold to consider the goal reached
        self.distance_threshold = distance_threshold
        # State validity checker
        self.svc = state_validity_checker
        # Initialize the tree with the start point
        self.tree = {self.start: None}

    def rrt_planning(self):
        """Main RRT planning function."""
        for _ in range(self.max_iter):
            # Sample a random point
            q_rand = self.sample_random_point()
            # Find the nearest point in the tree
            q_near = self.get_nearest(q_rand)
            # Extend the tree towards the random point
            q_new = self.extend(q_near, q_rand)
            # Check if the new point is valid and the path to it is clear
            if self.svc.is_valid(q_new.numpy()) and self.svc.check_path([q_near.numpy(), q_new.numpy()]):
                # Add the new point to the tree
                self.tree[q_new] = q_near
                # Check if the goal is reached
                if q_new.dist(self.goal) < self.distance_threshold:
                    return self.reconstruct_path(q_new)
        return []
    
    def sample_random_point(self):
        """Sample a random point in the domain."""
        if np.random.uniform(0,1) < self.sample_goal_prob:
            return self.goal
        else:
            return Point(np.random.uniform(self.x_domain[0], self.x_domain[1]), np.random.uniform(self.y_domain[0], self.y_domain[1]))
        
    def get_nearest(self, q_rand):
        """Find the nearest point in the tree to the random point."""
        min_dist = float('inf')
        nearest = None
        for node in self.tree.keys():
            dist = node.dist(q_rand)
            if dist < min_dist and dist != 0:
                min_dist = dist
                nearest = node
        return nearest
    
    def extend(self, q_near, q_rand):
        """Extend the tree from the nearest point towards the random point."""
        direction = q_near.vector(q_rand)
        distance = direction.norm()
        unit_vector = direction.unit()

        if distance == 0:
            return q_near
        
        return q_near.__add__(unit_vector * min(self.step_size, distance))
    
    def reconstruct_path(self, q_new):
        """Reconstruct the path from the start to the goal."""
        path = [np.array([q_new.x, q_new.y])]
        while self.tree[q_new] is not None:
            q_new = self.tree[q_new]
            path.append(np.array([q_new.x, q_new.y]))
        path.reverse()
        return path

