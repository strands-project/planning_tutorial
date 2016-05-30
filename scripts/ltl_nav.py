#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.msg import *


def create_ltl_task_spec(ltl_formula):
    mdp_spec = MdpDomainSpec()
    mdp_spec.ltl_task = ltl_formula
    return mdp_spec

def create_ltl_policy_goal(ltl_formula):
    return ExecutePolicyExtendedGoal(spec = create_ltl_task_spec(ltl_formula))

def mdp_exec_feedback(feedback):
    print feedback

def main():
    goal = create_ltl_policy_goal('(F "WayPoint4")')
    mdp_exec_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy_extended', ExecutePolicyExtendedAction)
    mdp_exec_client.wait_for_server()                                
    mdp_exec_client.send_goal(goal, feedback_cb = mdp_exec_feedback)
    mdp_exec_client.wait_for_result()

if __name__ == '__main__':
    rospy.init_node('ltl_nav')
    main()
