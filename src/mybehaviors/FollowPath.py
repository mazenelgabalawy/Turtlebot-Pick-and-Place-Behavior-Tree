import rospy
from geometry_msgs.msg import Pose,PoseStamped,Twist
from nav_msgs.msg import Path,Odometry, OccupancyGrid
import numpy as np
import tf
import py_trees

from utils.online_planning import StateValidityChecker

# Behavior for path following
class FollowPath(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FollowPath, self).__init__(name)
        self.goal=PoseStamped()
        # controller class object
        self.controller=Controller("/turtlebot/kobuki/odom_ground_truth","/projected_map",
                                   "/turtlebot/kobuki/commands/velocity",distance_threshold=0.05)
        self.path=Path()

    def setup(self):
        self.logger.debug("  %s [FollowPath::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [FollowPath::initialise()]" % self.name)
        print("Getting path topic")
        self.path:Path=rospy.wait_for_message("/turtlebot/planned_path",Path)
        print("Following path")

                # Reset controller state
        self.controller.path = []
        self.controller.path_invalid = False
        self.controller.goal_reached = False

        # Load new path
        for wp in self.path.poses:
            point = [wp.pose.position.x, wp.pose.position.y]
            self.controller.path.append(point)

        # Timers
        self.timer=rospy.Timer(rospy.Duration(0.1), self.controller.controller)

    def update(self):
        try:
            # self.logger.debug("  {}: following path".format(self.name))
            if self.controller.goal_reached:
                self.timer.shutdown()
                self.controller.__send_commnd__(0,0)
                return py_trees.common.Status.SUCCESS
                
            elif self.controller.path_invalid:
                self.logger.debug("Path Error  %s [FollowPath::update()]" % self.name)
                self.timer.shutdown()
                self.controller.__send_commnd__(0,0)
                return py_trees.common.Status.FAILURE
            
            else:
                return py_trees.common.Status.RUNNING
            
            
        except:
            self.logger.debug("  {}: Error, something happened".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [FollowPath::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )

class Controller:
    def __init__(self, odom_topic,map_topic,cmd_vel_topic, distance_threshold):

        # Attributes
        self.distance_threshold = distance_threshold  # Distance threshold to way point
        self.current_pose = None  # Current robot SE2 pose
        self.goal = None  # A goal is set
        self.goal_reached=False
        self.path_invalid=False # Check if path is invalid
        self.path = []  # List of points which define the plan. None if there is no plan
        self.svc = StateValidityChecker(distance=0.225)
        # Parameters
        self.Kv = 0.5  # Proportional linear velocity controller
        self.Kw = 0.5  # Proportional angular velocity controller
        self.v_max = 0.15  # Maximum linear velocity control action
        self.w_max = 0.3  # Maximum angular velocity control action
        self.last_map_time = rospy.Time.now()
        # Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        # Subscribers
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom)
        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid,self.map_callback)
        
        # Wrap angle between -pi and pi
    def wrap_angle(self,angle):
        return angle + (2.0 * np.pi * np.floor((np.pi - angle) / (2.0 * np.pi)))


    # Controller
    def move_to_point(self,current, goal, Kv=0.5, Kw=0.5):
        """Computes the control command to move from current position to goal."""
        theta_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
        w = Kw * self.wrap_angle(theta_d - current[2])
        v = 0
        if abs(w) < 0.05:  # to avoid move while turning
            v = Kv * np.linalg.norm(goal - current[0:2])
        return v, w

    # Odometry callback
    def get_odom(self, odom:Odometry):
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ]
        )
        self.current_pose = np.array(
            [odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]
        )

    def map_callback(self,gridmap):
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 5.0:            
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)

            # If the robot is following a path, check if it is still valid
            if len(self.path) > 0:
                # create total_path adding the current position to the rest of waypoints in the path
                total_path = [self.current_pose[0:2]] + self.path
                if not self.svc.check_path(total_path):
                    print("Path not valid, replanning")
                    self.path = []
                    self.path_invalid=True
                else:
                    print("Path valid")
                    self.path_invalid=False

    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self, event):
        v = 0
        w = 0
        if self.path is not None and len(self.path) > 0:
            # If current waypoint reached with some tolerance move to next point otherwise move to current point
            if (np.linalg.norm(self.path[0] - self.current_pose[0:2])< self.distance_threshold):
                print("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    self.goal_reached=True
                    print("Final position reached!")
            else:
                self.goal_reached=False
                v, w = self.move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        self.__send_commnd__(v, -w)

    # Publishers
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)



