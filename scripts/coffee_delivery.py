#!/usr/bin/env python

import rospy

from strands_executive_msgs.msg import MdpStateVar, StringIntPair, StringTriple, MdpAction, MdpActionOutcome, MdpDomainSpec, MdpTask, ExecutePolicyAction, ExecutePolicyGoal
from strands_executive_msgs import mdp_action_utils as mu

from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus



class GetCoffee(object):

    def __init__(self):
        self.ask_timeout = 10  #number of seconds to wait for an answer re coffee
        self.get_coffee_timeout = 60.0*2 #number of seconds to wait for a coffee at the machine
        self.deliver_timeout = 30 #number of seconds to wait for the person to retrieve the coffee
        
        self.coffee_wp = 'WayPoint8'  #location of the coffee machine
        
        self.people= ['John', 'Paul']  #people to serve 
        self.initial_location = { 'John':{'WayPoint1':0.8, 'WayPoint2':0.1}, 'Paul':{'WayPoint1':0.8, 'WayPoint2':0.1}}  #distribution of people's location in the environment
        self.wants_coffee = {'John':0.7, 'Paul':0.2} #probability of people wanting coffee
        
        # define the mdp action client 
        self.mdp_client = SimpleActionClient('mdp_plan_exec/execute_policy', ExecutePolicyAction)
        self.mdp_client.wait_for_server()
        

        # build planning problem
        model = self.build_mdp_spec()

        # send goal to mdp action server
        self.mdp_client.send_goal(ExecutePolicyGoal(spec = model))
        self.mdp_client.wait_for_result()
        
    
    
    def build_mdp_spec(self):
  
        mdp_vars = [] #the state features other than robot location
        actions = [] #the actions other than navigation
        
        holding_coffees_var_name = "n_coffees"
        holding_coffees_var =  MdpStateVar(name = holding_coffees_var_name,
                                     init_val = 0,
                                     min_range = 0,
                                     max_range = 3) #state feature representing number of coffees the robot is holding. The robot starts with 0, and can carry up to 3 coffees at the same time
        mdp_vars += [holding_coffees_var]

        for person in self.people: #looping for each person
            asked_var_name = 'asked_' + person
            asked_var = MdpStateVar(name = asked_var_name,
                                     init_val = 0,
                                     min_range = 0,
                                     max_range = 1)  #boolean state feature representing whether the person was asked wheter they want coffee.
            delivered_var_name = 'delivered_to_' + person
            delivered_at_var = MdpStateVar(name = delivered_var_name,
                                     init_val = 0,
                                     min_range = 0,
                                     max_range = 1) #boolean state feature representing whether person has received coffee.
            mdp_vars += [asked_var, delivered_at_var]
            for (possible_init_location, prob) in self.initial_location[person].items(): #looping for each possible location of this person and corresponding probability
                wants_var_name =  person + 'wants_at_' + possible_init_location
                wants_at_var = MdpStateVar(name = wants_var_name,
                                     init_val = -1,
                                     min_range = -1,
                                     max_range = 1) #state feature representing whether person wants coffee at this location. -1 represents the robot has not asked, 0 represents the person responded no coffee, 1 represents the person responding yes to coffee. In this case we need to remember the location to bring the coffee back, so we add a feature for each possible (person, location) pair.
              
                mdp_vars += [wants_at_var]
                
                #action encoding of asking person whether they want coffee, at possible_init_location
                ask_action = MdpAction(name = "ask_" + person + '_at_' + possible_init_location,  #the name of the action to be used in the MDP model
                            action_server = 'ask_question_server',  #the actionlib server implementing the behaviour. This will be called by the policy executor
                            waypoints = [possible_init_location], #the waypoint(s) where the behavior can be executed
                            pre_conds = [StringIntPair(wants_var_name, -1), StringIntPair(asked_var_name,0)], #preconditions for this action: it must be unknown whether the person is at this location and the person must not have been asked whether they want coffee
                            outcomes = []) #will be filled later
                #adding arguments to be passed to the 'ask_question_server' action server during execution. These are in line with the definition of the action msg for the action server located at ros_ws/src/isr_monarch_robot/mbot_task_planning/mbot_action_msgs/action/AskQuestion.action.
                mu.add_string_argument(ask_action, "Hi " + person + ". Would you like some coffee?") #What the robot should say.
                mu.add_string_argument(ask_action, "yes") #Answer to trigger success
                mu.add_float_argument(ask_action, self.ask_timeout) #waiting for reply timeout
                mu.add_string_argument(ask_action, "OK, I will bring it as soon as I can.") #What to say if the person was here and said yes to coffee
                mu.add_string_argument(ask_action, "Good call, caffeine is bad for you and I have more important things to do.") #what to say if the person was here but said no to coffee
                mu.add_string_argument(ask_action, "Looks like no one is here.") #what to say if person is not here

                #the outcomes for the asking action
                person_here_prob =  self.initial_location[person][possible_init_location] #probability the person is in  location possible_init_location
                
                #person was not in possible_init_location
                no_person_outcome =  MdpActionOutcome(probability = 1 - person_here_prob, #probability of this outcome, to be used in the MDP model
                                    waypoint = possible_init_location, #location of the robot after executing the action. In this case the robot does not move, so it ends in the same place. This will be the case for all actions in this example.
                                    post_conds = [StringIntPair(wants_var_name, 0)], #state feature update for this outcome. the state feature representing whether the robot the person wants coffee at this location becomes is there becomes 0.
                                    duration_probs = [1.0], #duration probabilities. This does not apply to tis example and will always be one.
                                    durations = [self.ask_timeout/2], #expected time for action execution, to be used in the MDP model. We set expected times to half the timeout. One could also learn these from experience.
                                    status = [GoalStatus.SUCCEEDED], #After executing the action in the robot, we check if the actionlib status  matched a value in this array. If so, we  perform an extra check using the result attribute below.
                                    result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'True')]) #After executing the action in the robot, if the check above succeeded, we check if the actionlib result matched the conditions given here. In this case, we check whether the attribute 'timeout' of the result object is equal to the boolean True.
                # person is in possible_init_location and wants coffee                   
                wants_coffee_outcome =  MdpActionOutcome(probability = person_here_prob*self.wants_coffee[person], 
                                    waypoint = possible_init_location,
                                    post_conds = [StringIntPair(asked_var_name, 1), StringIntPair(wants_var_name, 1)], #in this case, 2 state features are updated. The person has been asked about coffee, and the person wants coffee.
                                    duration_probs = [1.0],
                                    durations = [self.ask_timeout/2],
                                    status = [GoalStatus.SUCCEEDED],
                                    result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'False'), StringTriple('response', MdpActionOutcome.BOOL_TYPE, 'True')]) # for this outcome to occur during execution, the timeout attribute of the action result must be False, and the response outcome must be True, meaning the person responded 'yes'
                doesnt_want_coffee_outcome =  MdpActionOutcome(probability = person_here_prob*(1-self.wants_coffee[person]),
                                    waypoint = possible_init_location,
                                    post_conds = [StringIntPair(asked_var_name, 1), StringIntPair(wants_var_name, 0)],
                                    duration_probs = [1.0],
                                    durations = [self.ask_timeout/2],
                                    status = [GoalStatus.SUCCEEDED],
                                    result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'False'), StringTriple('response', MdpActionOutcome.BOOL_TYPE, 'False')])
                ask_action.outcomes = [no_person_outcome, wants_coffee_outcome, doesnt_want_coffee_outcome]
                
                #actions to deliver the coffees. We need to define one action for each amount of coffees the robot is holding
                #holding 1 coffee
                deliver_action1 = MdpAction(name = "deliver_coffee_to_" + person + '_at_' + possible_init_location,
                            action_server = 'ask_question_server', #we'll use the same action server as before, but with different arguments.
                            waypoints = [possible_init_location],
                            pre_conds = [StringIntPair(holding_coffees_var_name, 1), StringIntPair(wants_var_name, 1)], #delivering coffee when holding 1 coffee
                            outcomes = [])
                mu.add_string_argument(deliver_action1, "Hi " + person + ". Here is your coffee. Please take it out of my cupholder and tell me ok when you are done.")
                mu.add_string_argument(deliver_action1, "ok")
                mu.add_float_argument(deliver_action1, self.deliver_timeout)
                mu.add_string_argument(deliver_action1, "Enjoy. I hope it is still hot.")
                mu.add_string_argument(deliver_action1, "You asked for it, just take it.")
                mu.add_string_argument(deliver_action1, person + "? Get your coffee or it will get cold")
                succ_del1 = MdpActionOutcome(probability = 0.8,
                            waypoint = possible_init_location,
                            post_conds = [StringIntPair(holding_coffees_var_name, 0), StringIntPair(delivered_var_name, 1)], #now the robot has 0 coffees
                            duration_probs = [1.0],
                            durations = [self.deliver_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'False')]) 
                failed_del1 =  MdpActionOutcome(probability = 0.2,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 1), StringIntPair(wants_var_name, 1)], #failure. state remains the same in this case, these state feature updates are just for clarity: robot still has 1 coffee and person still wants it. In this simple example we will retry giving the coffee until it is taken.
                            duration_probs = [1.0],
                            durations = [self.deliver_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'True')])
                deliver_action1.outcomes = [succ_del1, failed_del1]
                
                #holding 2 coffees
                deliver_action2 = MdpAction(name = "deliver_coffee_to_" + person + '_at_' + possible_init_location,
                            action_server = 'ask_question_server',
                            waypoints = [possible_init_location],
                            pre_conds = [StringIntPair(holding_coffees_var_name, 2), StringIntPair(wants_var_name, 1)], #delivering coffee when holding 2 coffees
                            outcomes = [])
                mu.add_string_argument(deliver_action2, "Hi " + person + ". Here is your coffee")
                mu.add_string_argument(deliver_action2, "ok")
                mu.add_float_argument(deliver_action2, self.deliver_timeout)
                mu.add_string_argument(deliver_action2, "Enjoy. I hope it is still hot.")
                mu.add_string_argument(deliver_action2, "You asked for it, just take it.")
                mu.add_string_argument(deliver_action2, person + "? Get your coffee or it will get cold")
                succ_del2 = MdpActionOutcome(probability = 0.8,
                            waypoint = possible_init_location,
                            post_conds = [StringIntPair(holding_coffees_var_name, 1), StringIntPair(delivered_var_name, 1)], #now the robot has 1 coffee
                            duration_probs = [1.0],
                            durations = [self.deliver_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'False')]) 
                failed_del2 =  MdpActionOutcome(probability = 0.2,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 2), StringIntPair(wants_var_name, 1)], 
                            duration_probs = [1.0],
                            durations = [self.deliver_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'True')])
                deliver_action2.outcomes = [succ_del2, failed_del2]
                
                #3 coffees
                deliver_action3 = MdpAction(name = "deliver_coffee_to_" + person + '_at_' + possible_init_location,
                            action_server = 'ask_question_server',
                            waypoints = [possible_init_location],
                            pre_conds = [StringIntPair(holding_coffees_var_name, 2), StringIntPair(wants_var_name, 1)], #delivering coffee when holding 3 coffees
                            outcomes = [])
                mu.add_string_argument(deliver_action3, "Hi " + person + ". Here is your coffee")
                mu.add_string_argument(deliver_action3, "ok")
                mu.add_float_argument(deliver_action3, self.deliver_timeout)
                mu.add_string_argument(deliver_action3, "Enjoy. I hope it is still hot.")
                mu.add_string_argument(deliver_action3, "You asked for it, just take it.")
                mu.add_string_argument(deliver_action3, person + "? Get your coffee or it will get cold")
                succ_del3 = MdpActionOutcome(probability = 0.8,
                            waypoint = possible_init_location,
                            post_conds = [StringIntPair(holding_coffees_var_name, 2), StringIntPair(delivered_var_name, 1)], #now the robot has 2 coffee
                            duration_probs = [1.0],
                            durations = [self.deliver_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'False')]) 
                failed_del3 =  MdpActionOutcome(probability = 0.2,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 3), StringIntPair(wants_var_name, 1)], 
                            duration_probs = [1.0],
                            durations = [self.deliver_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'True')])
                deliver_action3.outcomes = [succ_del3, failed_del3]
                
                
                actions += [ask_action, deliver_action1, deliver_action2, deliver_action3]
              
              
              
              
        #actions to get coffees. we only allow to get coffees when the robot isnt holding any, for simplicity.
        #getting one coffee
        get_action1 = MdpAction(name = "get_1coffee",
                    action_server = 'ask_question_server',
                    waypoints = [self.coffee_wp],
                    pre_conds = [StringIntPair(holding_coffees_var_name, 0)],
                    outcomes = [])
        mu.add_string_argument(get_action1, "Please put one coffee in my cupholder and tell me ok when you are done.")
        mu.add_string_argument(get_action1, "ok")
        mu.add_float_argument(get_action1, self.get_coffee_timeout)
        mu.add_string_argument(get_action1, "Thanks. I really appreciate your help.")
        mu.add_string_argument(get_action1, "Please someone help. I am trying to give some coffee")
        mu.add_string_argument(get_action1, "Please someone help. I am trying to give some coffee.")
        got_1coffee_outcome =  MdpActionOutcome(probability = 0.8,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 1)], #now the robot has 1 coffee
                            duration_probs = [1.0],
                            durations = [self.get_coffee_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'False')])
        didnt_get_1coffee_outcome =  MdpActionOutcome(probability = 0.2,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 0)],
                            duration_probs = [1.0],
                            durations = [self.get_coffee_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'True')])
        get_action1.outcomes = [got_1coffee_outcome, didnt_get_1coffee_outcome]
        
        #holding 2 coffees
        get_action2 = MdpAction(name = "get_2coffee",
                    action_server = 'ask_question_server',
                    waypoints = [self.coffee_wp],
                    pre_conds = [StringIntPair(holding_coffees_var_name, 0)],
                    outcomes = [])
        mu.add_string_argument(get_action2, "Please put two coffees in my cupholder and tell me ok when you are done.")
        mu.add_string_argument(get_action2, "ok")
        mu.add_float_argument(get_action2, self.get_coffee_timeout)
        mu.add_string_argument(get_action2, "Thanks. I really appreciate your help.")
        mu.add_string_argument(get_action2, "Please someone help. I am trying to give some coffee")
        mu.add_string_argument(get_action2, "Please someone help. I am trying to give some coffee.")
        got_2coffee_outcome =  MdpActionOutcome(probability = 0.8,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 2)], #now the robot has 2 coffees
                            duration_probs = [1.0],
                            durations = [self.get_coffee_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'False')])
        didnt_get_2coffee_outcome =  MdpActionOutcome(probability = 0.2,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 0)],
                            duration_probs = [1.0],
                            durations = [self.get_coffee_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'True')])
        get_action2.outcomes = [got_2coffee_outcome, didnt_get_2coffee_outcome]
        
        #get 3 coffees
        get_action3 = MdpAction(name = "get_3coffee",
                    action_server = 'ask_question_server',
                    waypoints = [self.coffee_wp],
                    pre_conds = [StringIntPair(holding_coffees_var_name, 0)],
                    outcomes = [])
        mu.add_string_argument(get_action3, "Please put three coffees in my cupholder and tell me ok when you are done.")
        mu.add_string_argument(get_action3, "ok")
        mu.add_float_argument(get_action3, self.get_coffee_timeout)
        mu.add_string_argument(get_action3, "Thanks. I really appreciate your help.")
        mu.add_string_argument(get_action3, "Please someone help. I am trying to give some coffee")
        mu.add_string_argument(get_action3, "Please someone help. I am trying to give some coffee.")
        got_3coffee_outcome =  MdpActionOutcome(probability = 0.8,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 3)], #now the robot has 3 coffees
                            duration_probs = [1.0],
                            durations = [self.get_coffee_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'False')])
        didnt_get_3coffee_outcome =  MdpActionOutcome(probability = 0.2,
                            waypoint = self.coffee_wp,
                            post_conds = [StringIntPair(holding_coffees_var_name, 0)],
                            duration_probs = [1.0],
                            durations = [self.get_coffee_timeout/2],
                            status = [GoalStatus.SUCCEEDED],
                            result = [StringTriple('timeout', MdpActionOutcome.BOOL_TYPE, 'True')])
        get_action3.outcomes = [got_3coffee_outcome, didnt_get_3coffee_outcome]
        
        actions += [get_action1, get_action2, get_action3]
                
                
         
        #define the ltl task. currently hardcoded. 
        #ltl_task = "F (" + delivered_var_name + " = 1)"
        #ltl_task = "(F (delivered_to_John = 1)) & (F (delivered_to_Paul = 1))"
        ltl_task = "(F (delivered_to_John = 1)) & (F (delivered_to_Paul = 1))"
        
        #build the mdp spec
        mdp_spec = MdpDomainSpec(vars = mdp_vars, actions = actions, ltl_task = ltl_task)
        return mdp_spec


         
    


if __name__ == '__main__':
    rospy.init_node("mbot_delivery")
    coffee = GetCoffee()
