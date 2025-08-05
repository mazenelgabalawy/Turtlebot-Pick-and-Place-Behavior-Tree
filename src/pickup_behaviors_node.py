#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import py_trees
import time
from mybehaviors import CheckObject, GetObject, LetObject, SetGoalDropPoint, SetGoalNextBall, ChangePose,PlanAndFollow

# TODO: Create any other required behavior like those to move the robot to a point,
#       add or check elements in the blackboard, ...

if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    # Initialize Blackboard
    blackboard = py_trees.blackboard.Client(name="RobotBlackboard")
    blackboard.register_key(key="Iterator",access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="Balls_positions", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="Drop_point", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="goal", access=py_trees.common.Access.WRITE)

    # Set Initial Values
    blackboard.Iterator = int(0)
    blackboard.Balls_positions = [
                        (1.65,0.75,np.pi),
                        (2.2,3.0,0.633),
                        (0.7,2.2,np.pi/2)
                        ]
    blackboard.Drop_point = (-0.2,3.5,3.14)
    blackboard.goal = None

    # Create Behaviors
    set_goal_next_ball = SetGoalNextBall("set_goal_next_ball")
    set_goal_drop_point = SetGoalDropPoint("set_goal_drop_point")

    plan_and_follow_path1 = PlanAndFollow("plan_and_follow_path1")
    plan_and_follow_path2 = PlanAndFollow("plan_and_follow_path2")

    change_pose= ChangePose("change_pose")

    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    let_object = LetObject("let_object")

    # Wrap behaviors in a Sequence before applying Retry
    go_to_ball_sequence = py_trees.composites.Sequence(
        name="Go to Ball Sequence",
        children=[plan_and_follow_path1, check_object, get_object],
        memory=True
    )
    go_to_ball = py_trees.decorators.Retry(
        name="Repeat Until Object is Picked",
        child=go_to_ball_sequence,  # Single child (the Sequence)
        num_failures=float('inf')
    )

    go_to_drop_sequence = py_trees.composites.Sequence(
        name="Go to Drop Sequence",
        children=[plan_and_follow_path2],
        memory=True
    )
    go_to_drop = py_trees.decorators.Retry(
        name="Repeat Until Object is Dropped",
        child=go_to_drop_sequence,  # Single child (the Sequence)
        num_failures=float('inf')
    )

    repeat_three_times_sequence = py_trees.composites.Sequence(
        name="Repeat Three Times Sequence",
        children=[set_goal_next_ball, 
                  go_to_ball, 
                  change_pose,
                  set_goal_drop_point,
                  go_to_drop,
                  let_object],
        memory=True)
    repeat_three_times = py_trees.decorators.Repeat("Repeat Three Times",
                                                    child=repeat_three_times_sequence,num_success=len(blackboard.get("Balls_positions")))

    # create tree, define root and add behaviors
    root = py_trees.composites.Sequence(name="Sequence", memory=True)
    root.add_children([repeat_three_times])
    behavior_tree = py_trees.trees.BehaviourTree(root=root)
    # call setup method of all tree behaviors
    behavior_tree.setup(timeout=15)

    # save tree as image
    rospack = rospkg.RosPack()
    filepath = rospack.get_path("pick_up_objects_task")
    py_trees.display.render_dot_tree(root=root, target_directory=filepath)

    # tick the tree
    try:
        while not rospy.is_shutdown():
            if behavior_tree.root.status!=py_trees.common.Status.SUCCESS:
                behavior_tree.tick()
                time.sleep(0.5)
            else:
                print("root returned success, tree done")
                break
        print("\n")
    except KeyboardInterrupt:
        print("")
        pass
