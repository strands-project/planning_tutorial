#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.msg import *

def create_ltl_task_spec(ltl_formula):
    """
    Turn an LTL formula into an MDP domain specification. There is a navigation MDP which this is addeed to. 
    """
    mdp_spec = MdpDomainSpec()
    mdp_spec.ltl_task = ltl_formula
    return mdp_spec

def create_ltl_policy_goal(ltl_formula):
    """
    Create an actionlib goal to execute the given LTL formula.
    """
    return ExecutePolicyGoal(spec = create_ltl_task_spec(ltl_formula))

def mdp_exec_feedback(feedback):
    # print feedback
    pass


if __name__ == '__main__':

    rospy.init_node('ltl_nav')
        
    goal_formula = '(F "WayPoint4")'
    
    # create the goal to execute
    goal = create_ltl_policy_goal(goal_formula)

    # contact the execution server
    mdp_exec_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy', ExecutePolicyAction)
    mdp_exec_client.wait_for_server()                                

    # send the goal
    mdp_exec_client.send_goal(goal, feedback_cb = mdp_exec_feedback)

    # and wait for the executive to complete it 
    mdp_exec_client.wait_for_result()

