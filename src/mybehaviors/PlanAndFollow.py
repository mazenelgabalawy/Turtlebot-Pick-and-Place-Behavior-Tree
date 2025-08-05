import py_trees
from mybehaviors.PlanSubTree import *
from mybehaviors.FollowPath import *
from mybehaviors.CorrectOrientation import *

class PlanAndFollow(py_trees.composites.Sequence):
    def __init__(self, name):
        super(PlanAndFollow, self).__init__(name,memory=True)

        # Define the sub-tree structure
        plan = PlanSubTree("Plan Sub-Tree")
        follow = FollowPath("follow_path")
        orient = CorrectOrientation("correct_orientation")

        # Add behaviors to selector
        self.add_children([plan,
                           py_trees.decorators.Timeout("Follow Time-out", child=follow,duration=60.0),
                           orient,
                           ])
