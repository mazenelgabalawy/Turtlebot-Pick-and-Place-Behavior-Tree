import rospy
from geometry_msgs.msg import Pose,PoseStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
import py_trees
import numpy as np
import tf

from utils.online_planning import StateValidityChecker
from utils.RRT import RRT

# Behavior for planning path
class PlanPath(py_trees.behaviour.Behaviour):
    def __init__(self, name,max_iterations = 5000):
        super(PlanPath, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)
        self.goal=PoseStamped()
        # planner class object
        self.planner=Planner("/turtlebot/kobuki/odom_ground_truth","/projected_map",
                             StateValidityChecker(distance=0.19),max_iterations=max_iterations,dominion=[-1,4,-1.5,5])
        self.waypoints=[]

    def setup(self):
        self.logger.debug("  %s [PlanPath::setup()]" % self.name)
        self.pathPub=rospy.Publisher("/turtlebot/planned_path",Path,queue_size=1,latch=True)

    def initialise(self):
        self.logger.debug("  %s [PlanPath::initialise()]" % self.name)
        self.planner.waypoints=self.waypoints
        self.path:Path=rospy.wait_for_message("/turtlebot/kobuki/odom_ground_truth",Path)


    def update(self):
        try:
            self.logger.debug("  {}: computing path".format(self.name))
            self.goal = self.blackboard.get('goal')
            path:Path=self.planner.plan(self.goal)
            
            if len(path.poses) != 0:
                self.logger.debug("  {}: Path found".format(self.name))
                self.pathPub.publish(path)
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.debug("  {}: Path not found".format(self.name))
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug("  {}: Error, path not computed".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [PlanPath::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


# simple planner class, replace with your RRT path planner
class Planner:
    def __init__(self,odom_topic, map_topic,svc,max_iterations=5000,dominion=[-5,5,-5,5], step_size=0.2, sample_goal_prob=0.2, distance_threshold = 0.05):
        # initialize planner params
        self.svc = svc
        self.max_iterations = max_iterations
        self.dominion = dominion
        self.step_size = step_size
        self.sample_goal_probability = sample_goal_prob
        self.distance_threshold = distance_threshold
        self.current_pose = None
        self.last_map_time = rospy.Time.now()
        self.waypoints = []

        # Subscribers
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)

    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def odom_callback(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])

        # TODO: Store current position (x, y, yaw) as a np.array in self.current_pose var.
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

    def map_callback(self,gridmap):
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 5:            
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)


    def plan(self, goal: PoseStamped):
        # Convert goal to point for RRT
        goal_point = [goal.pose.position.x, goal.pose.position.y]
        
        # Use RRT for planning the path
        self.waypoints = self.compute_path(self.current_pose, goal_point)
        self.waypoints = self.smooth_path(self.waypoints)
        
        # Convert RRT path to PoseStamped messages
        path = Path()
        path.header.frame_id = "world_ned"
        for wp in self.waypoints:
            poseS = PoseStamped()
            poseS.pose.orientation.w = 1  # assuming flat terrain
            poseS.pose.position.x = wp[0]
            poseS.pose.position.y = wp[1]
            path.poses.append(poseS)
        
        return path
    
    def compute_path(self,start_p,goal_p):
        rrt = RRT(start_p,goal_p,self.dominion,self.max_iterations,self.step_size,self.svc,self.sample_goal_probability,self.distance_threshold)
        return rrt.rrt_planning()

    def smooth_path(self,path):
        # Optionally, you can implement a function to smooth the RRT path.
        if len(path)>0:
            next_node = path[-1] #Set as goal
            i = 0
            smooth_path = [path[-1]] # Add goal to smooth-path
            try:
                while (smooth_path[-1]!=path[0]).all(): # Check if start is reached
                    if self.svc.check_path([path[i],next_node]): # Check if a direct path is free from node i to next_node
                        smooth_path.append(path[i]) 
                        next_node = path[i] # Set next_node as node i and repeat
                        i = 0
                    else:
                        i+=1
                smooth_path.reverse() # Reverse path 
                return smooth_path
            except:
                print("Problem in Check path")
    
        else:
            return [] 
