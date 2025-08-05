import rospy
import rospkg
import py_trees
from geometry_msgs.msg import PoseStamped
import tf

# Behavior for calling `let_object`
class SetGoalNextBall(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(SetGoalNextBall, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("Iterator", access=py_trees.common.Access.READ)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("Balls_positions", access=py_trees.common.Access.READ)

        self.iterator = self.blackboard.get("Iterator")
        self.ball_positions = self.blackboard.get("Balls_positions")

    def setup(self):
        self.logger.debug("  %s [SetGoalNextBall::setup()]" % self.name)
        
    def initialise(self):
        self.logger.debug("  %s [SetGoalNextBall::initialise()]" % self.name)

    def update(self):
        try:
            # Get the iterator value
            iterator = self.blackboard.get("Iterator")
            ball_positions = self.blackboard.get("Balls_positions")

            if ball_positions is None or iterator is None or iterator >= len(ball_positions):
                self.logger.debug("  {}: Error, Invalid Iterator or Balls_positions".format(self.name))
                return py_trees.common.Status.FAILURE

            # Create PoseStamped message
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()  # Set timestamp
            goal_pose.header.frame_id = "world_ned"  # Set the frame

            goal_orientation = tf.transformations.quaternion_from_euler(0,0,ball_positions[iterator][2])

            # Set position from ball_positions list
            goal_pose.pose.position.x = ball_positions[iterator][0]
            goal_pose.pose.position.y = ball_positions[iterator][1]
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
            self.logger.debug("  {}: Error, Goal not Set. Exception: {}".format(self.name, str(e)))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [SetGoalNextBall::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )
