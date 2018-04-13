#!/usr/bin/env python

import rospy
import actionlib
from strands_executive_msgs.msg import *
from actionlib_msgs.msg import GoalStatus
from strands_executive_msgs.mdp_action_utils import add_string_argument

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


def create_mdp_action_for_announcement(waypoint, announcement):

    # The name for this action in the MDP
    action_name = 'announce_at_' + waypoint  
    
    # The state variable which will correspond to the completion of the action.
    # We use this state variable to track whether or not we've executed the action, as there is no state-relevant outcome we can track.
    # This prevents us re-executing it
    state_var_name = 'announced_' + action_name
    # Create the state var object, initialise to 0 (false)
    state_var = MdpStateVar(name = state_var_name, init_val = 0, min_range = 0, max_range = 1)


    # Next we need to consider the possible outcomes of the action. 
    # The action server can either return successfully (implying the speech was given), or it could return in some other state (implying the speech was not completed).
    # We capture these different outcomes in the following objects. Note the the probabilities are made up, but could be learnt from data.

    successful_outcome = MdpActionOutcome(probability = 0.99,
                post_conds = [StringIntPair(string_data = state_var_name, int_data = 1)],
                duration_probs = [1.0],
                durations = [20.0], 
                # if the action server returns with SUCCEEDED then this outcome is used
                status = [GoalStatus.SUCCEEDED])

    unsuccessful_outcome = MdpActionOutcome(probability = (1 - successful_outcome.probability),
                post_conds = [StringIntPair(string_data = state_var_name, int_data = 0)],
                duration_probs = [1.0],
                durations = [10.0], 
                # if the action server returns with SUCCEEDED then this outcome is used
                status = [GoalStatus.ABORTED, GoalStatus.PREEMPTED])


    # Finally we tie all this together to an actionlib server
    action = MdpAction(name=action_name,
            # This is the actual action server topic to be used 
            action_server='/speak', 
            # The action will only be attemped if the preconditions are satisfied. In this case we can't have succeeded in the action before 
            pre_conds=[StringIntPair(string_data=state_var_name, int_data=0)],
            # These are the possible outcomes we defined above
            outcomes=[successful_outcome, unsuccessful_outcome],
            # And this is where we can execute the action. 
            waypoints = [waypoint])

    # And this adds some arguments to the actionserver call
    add_string_argument(action, announcement)

    return state_var, action

if __name__ == '__main__':

    rospy.init_node('waypoint_announce')
      
    # The announcement to make (call a meeting)
    announcement = "Hello everyone. Please go to the meeting room, the meeting is about to start."

    # The list of waypoints to speak at
    waypoints = ['WayPoint8', 'WayPoint2']
    
    
    

    # Collect mdp state variables and actions for each waypoint
    vars_and_actions = [create_mdp_action_for_announcement(wp, announcement) for wp in waypoints]


    mdp_spec = MdpDomainSpec()
        

    for state_var, action in vars_and_actions:           
        mdp_spec.vars.append(state_var)
        mdp_spec.actions.append(action)
        mdp_spec.ltl_task += '(F %s=1) & ' % state_var.name                
           
    mdp_spec.ltl_task = mdp_spec.ltl_task[:-3]

    goal = ExecutePolicyGoal(spec = mdp_spec)

    # contact the execution server
    mdp_exec_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy', ExecutePolicyAction)
    mdp_exec_client.wait_for_server()                                

    # send the goal
    mdp_exec_client.send_goal(goal, feedback_cb = mdp_exec_feedback)

    # and wait for the executive to complete it 
    print mdp_exec_client.wait_for_result()

