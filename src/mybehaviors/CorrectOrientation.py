import rospy
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf
import py_trees

# Behavior for path following
class CorrectOrientation(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CorrectOrientation, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)
        self.goal=PoseStamped()
        # controller class object
        self.controller=OrientationController("/turtlebot/kobuki/odom_ground_truth",
                                   "/turtlebot/kobuki/commands/velocity",orientation_threshold=0.05)

    def setup(self):
        self.logger.debug("  %s [CorrectOrientation::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [CorrectOrientation::initialise()]" % self.name)
        print("Following path")

        # Reset controller state
        self.goal_pose = self.blackboard.get('goal')

        _, _ , self.controller.goal_yaw = tf.transformations.euler_from_quaternion(
            [
                self.goal_pose.pose.orientation.x,
                self.goal_pose.pose.orientation.y,
                self.goal_pose.pose.orientation.z,
                self.goal_pose.pose.orientation.w,
            ]
        )
        self.controller.goal_reached = False

        self.timer=rospy.Timer(rospy.Duration(0.1), self.controller.controller)

    def update(self):
        try:
            if self.controller.goal_reached:
                self.timer.shutdown()
                self.controller.__send_commnd__(0,0)
                return py_trees.common.Status.SUCCESS
                
            else:
                return py_trees.common.Status.RUNNING
            
        except:
            self.logger.debug("  {}: Error, something happened".format(self.name))
            self.timer.shutdown()
            self.controller.__send_commnd__(0,0)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [CorrectOrientation::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )

class OrientationController:
    def __init__(self, odom_topic,cmd_vel_topic, orientation_threshold):

        # Attributes
        self.orientation_threshold = orientation_threshold  # Distance threshold to way point
        self.current_pose = None  # Current robot SE2 pose
        self.goal_yaw = None  # A goal is set
        self.goal_reached=False
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
        
    # Wrap angle between -pi and pi
    def wrap_angle(self,angle):
        return angle + (2.0 * np.pi * np.floor((np.pi - angle) / (2.0 * np.pi)))

    # Controller
    def rotate_to_goal(self, current_orientation, goal_orientation):
        w = self.Kw*self.wrap_angle(goal_orientation - current_orientation)
        return 0, w # Only angular velocity

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

    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self, event):
        v = 0
        w = 0
        if self.goal_yaw is not None and self.wrap_angle(self.goal_yaw - self.current_pose[2]) > self.orientation_threshold:
            v, w = self.rotate_to_goal(self.current_pose[2], self.goal_yaw)
            self.goal_reached = False
        else:
            self.goal_reached = True
            v, w = 0, 0
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
