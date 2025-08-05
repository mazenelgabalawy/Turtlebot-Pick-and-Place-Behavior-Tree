import py_trees
from mybehaviors.ChangePose import *
from mybehaviors.PlanPath import *

class PlanSubTree(py_trees.composites.Selector):
    def __init__(self, name):
        super(PlanSubTree, self).__init__(name,memory=True)

        # Define the sub-tree structure
        plan = PlanPath("plan")
        change_pose = ChangePose("change_pose")

        # Add behaviors to selector
        self.add_children([py_trees.decorators.Timeout("Retry Planning for 2 mins", child=plan,duration=120.0),
                           py_trees.decorators.Inverter("Always False",child=change_pose)
                           ])
