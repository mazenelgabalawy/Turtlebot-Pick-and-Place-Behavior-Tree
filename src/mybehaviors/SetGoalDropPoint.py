import rospy
from geometry_msgs.msg import PoseStamped
import tf
import py_trees


# Behavior for calling `let_object`
class SetGoalDropPoint(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SetGoalDropPoint, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("Drop_point", access=py_trees.common.Access.READ)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [SetGoalDropPoint::setup()]" % self.name)
        
    def initialise(self):
        self.logger.debug("  %s [SetGoalDropPoint::initialise()]" % self.name)
        # print("Goal Position: % " % self.blackboard.get("goal"))


    def update(self):
        try:
            self.logger.debug("  {}: Setting Goal to Drop point".format(self.name))
            new_goal = self.blackboard.get("Drop_point")

            goal_orientation = tf.transformations.quaternion_from_euler(0,0,new_goal[2])


            # Create PoseStamped message
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()  # Set timestamp
            goal_pose.header.frame_id = "world_ned"  # Set the frame

            # Set position from ball_positions list
            goal_pose.pose.position.x = new_goal[0]
            goal_pose.pose.position.y = new_goal[1]
            goal_pose.pose.position.z = 0.0

            # Set a default orientation
            goal_pose.pose.orientation.x = goal_orientation[0]
            goal_pose.pose.orientation.y = goal_orientation[1]
            goal_pose.pose.orientation.z = goal_orientation[2]
            goal_pose.pose.orientation.w = goal_orientation[3]

            # Store goal pose in blackboard
            self.blackboard.goal = goal_pose

            self.logger.debug("  {}: Goal Set to Position {}".format(self.name, goal_pose.pose.position))
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.logger.debug("  {}: Error, Goal  not set".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [SetGoalDropPoint::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )
