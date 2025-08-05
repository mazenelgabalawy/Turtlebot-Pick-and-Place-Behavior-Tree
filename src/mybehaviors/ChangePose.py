import rospy
import rospkg
from geometry_msgs.msg import Twist
import numpy as np
import tf
import py_trees

# Behavior for calling `let_object`
class ChangePose(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ChangePose, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)

    def setup(self):
        self.logger.debug("  %s [ChangePose::setup()]" % self.name)
        
    def initialise(self):
        self.logger.debug("  %s [ChangePose::initialise()]" % self.name)
        self.controller = Controller("/turtlebot/kobuki/commands/velocity")
        self.timer=rospy.Timer(rospy.Duration(0.1), self.controller.controller)


    def update(self):
        try:
            self.logger.debug("  {}: Changing Robot Pose".format(self.name))
            
            if self.controller.pose_changed:
                self.timer.shutdown()
                self.controller.__send_commnd__(0,0)
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
            
        except Exception as e:
            self.logger.debug("  {}: Error, Goal not Set".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [ChangePose::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class Controller:
    def __init__(self,cmd_vel_topic):
        # Parameters
        self.Kv = 0.25  # Proportional linear velocity controller
        self.Kw = 0.25  # Proportional angular velocity controller
        self.v_max = 0.15  # Maximum linear velocity control action
        self.w_max = 0.3  # Maximum angular velocity control action
        self.last_map_time = rospy.Time.now()
        self.pose_changed = False
        # Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        
    # Controller
    def controller(self, event):        
        if (rospy.Time.now() - self.last_map_time).to_sec() >= 3.25:
            # If current waypoint reached with some tolerance move to next point otherwise move to current point
                    self.pose_changed=True
                    v = 0.0
        else:
            v = -0.1
            self.pose_changed=False

        self.__send_commnd__(v, 0.0)

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
