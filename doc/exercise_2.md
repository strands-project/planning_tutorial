# Exercise 2

In this exercise you will use our MDP/LTL planning framework to get the robot to perform announcements, telling people to go to a meeting. This example connects many of the different elements of our robot system, and may seem complex at first. If something is unclear, or you want more information, please just ask.



# Background

You must first run some basic elements from the STRANDS system. You should ideally run each of these in a separate terminal where you have sourced both the ROS and your local workspace `setup.bash` files, as described in [tutorial_prep.md](./tutorial_prep.md).

If you still have the nodes  running from [Exercise 1](./exercise_1.md),  you can skip to the [Speech Nodes](#speech-nodes) configuration.

## MongoDB Store

First, check the `db` directory exists (which you should've created following [tutorial_prep.md](./tutorial_prep.md)). The following should not list any files or report an error:

```bash 
ls `rospack find planning_tutorial`/db
```

If that is ok, then launch [mongodb_store](http://wiki.ros.org/mongodb_store) using that directory as its data path:

```bash
roslaunch mongodb_store mongodb_store.launch db_path:=`rospack find planning_tutorial`/db
```

## MORSE Simulation

In another terminal, launch our simplified simulation of the [Transport Systems Catapult](http://ts.catapult.org.uk) (TSC). 

```bash
roslaunch strands_morse tsc_fast_morse.launch 
```

If you press the 'h' key in MORSE you can see a list of available keyboard commands.

## 2D and Topological Navigation

We have predefined a simple topological map for you to use in this tutorial. The first time (and only the first time!) you want to use topological navigation in the TSC simulation, you must add this map to the mongodb store. Do it with the following command:

```bash
rosrun topological_utils load_yaml_map.py `rospack find planning_tutorial`/maps/plan_tut_top_map.yaml
```

You can check the result of this with the following command which should print a description of the topological map `plan_tut`.

```
rosrun topological_utils list_maps 
```

If this was successful you can launch the 2D (amcl and move_base) and topological localisation and navigation for your simulated robot.

```bash
roslaunch planning_tutorial tsc_navigation.launch
```

To see everything running, launch the ROS visualisation tool `rviz` with the provided config file:

```bash
rviz -d `rospack find planning_tutorial`/plan_tut.rviz
```

If you click on a green arrow in a topological node, the robot should start working its way there. Feel free to add whatever extra visualisation parts you want to this (or ask us what the various bits are if you're new to robotics).


## Edge Prediction and MDP Planning

For this tutorial we're going to interfere with the expected duration and success probabilities for navigating edges in the topological map. Usually these are computed from data gathered as navigation happens, but for now we're going to fix them. To launch a node which reports fixed predictions for map edges, do the following in a new terminal (the argument is the file which contains the values to be reported):

```bash
rosrun topological_navigation manual_edge_predictions.py `rospack find planning_tutorial`/maps/plan_tut_edges.yaml
```

Once this is running you can launch the MDP-based task executive system in (yet another!) new terminal:

```bash
roslaunch mdp_plan_exec mdp_plan_exec.launch
```


## Speech Nodes

We will be using a framework for *text-to-speech* and use it to have the robot call people for a meeting. To launch the text-to-speech nodes, open a new terminal and do:


```bash
roslaunch mary_tts ros_mary.launch
```

# Exercise 2a

In [Exercise 1](./exercise_1.md) you exploited the fact that the execution framework automatically creates an MDP for navigation across the topological map. In this exercise we will extend this MDP with additional actions which connect ROS [actionlib servers](http://wiki.ros.org/actionlib) to actions in the MDP. 

In this part we will walk through an example where the outcome of the invocation of an action server is tied to a probabilistic outcome of an action in an MDP. After showing you how to do this, the next step will be for you to edit the file to change how the action is encoded in the MDP.

All of the code for this part of the exercise is in `planning_tutorial/scripts/announce_at_waypoints.py` so only the important fragments will be included here.

The task we will undertake is to trigger the `/speak` action server with argument `Hello everyone. Please go to the meeting room, the meeting is about to start.` (in order to tell people to go to a meetig) at a set of waypoints in the topological map. The key idea is that we associate the action server with an MDP action which we add to the navigation MDP. To make sure we only complete the action once, we connect the successful completion of this action to a state variable in our MDP specification, which can be used in pre- and post-conditions for our action. 

We start by creating a name for our action. As we care about the success of an action at a waypoint, we need a different action for each waypoint. This is captured in the naming of the action, which should be different for each waypoint, i.e.

```python
    action_name = 'announce_at_' + waypoint  
```    

Next we create a the MDP state variable for tracking the success of the action. This state variable will form part of our goal statement, e.g. if we have the state variable `executed_announcement_at_WayPoint1` the goal to eventually make this true would be `(F executed_announcement_at_WayPoint1=1)`.

```python
    state_var_name = 'executed_' + action_name
    # Create the state var object, initialise to 0 (false)
    state_var = MdpStateVar(name = state_var_name, init_val = 0, min_range = 0, max_range = 1) 
```

Following this we need to encode the possible outcomes of the action and the way they can change the state in the MDP. Although there is no restriction on the number of outcomes an action can have (over one), we will use two: succeeded or not succeeded. The `/speak` action reports its outcomes when it completes, so we will use this return signal to tell the MDP which outcome occurred (allowing it to update its internal state correctly). Each outcome has a probability of occurring (which is used when creating a policy to solve the MDP), and a probability distribution over how long it might take to reach the outcome. In this example we make up these values, but in our full system we can learn them from experience at execution time.


```python
    successful_outcome = MdpActionOutcome(
    			# the probability of this outcome occurring
    			probability = 0.99,
    			# the effects of this action on the MDP state, in this case setting the state variable to 1
                post_conds = [StringIntPair(string_data = state_var_name, int_data = 1)],
                # how long it will take to reach this outcome, and with what probability
                duration_probs = [1.0],
                durations = [20.0], 
                # And the action server outcome which will tell us if this outcome occurred. In this case if the action server returns with SUCCEEDED 
                status = [GoalStatus.SUCCEEDED])
```

Below is a similar encoding for the unsuccessful outcome, i.e. the cases where the action server reports that it has aborted or was preempted.

```python
    unsuccessful_outcome = MdpActionOutcome(probability = (1 - successful_outcome.probability),
                post_conds = [StringIntPair(string_data = state_var_name, int_data = 0)],
                duration_probs = [1.0],
                durations = [10.0],               
                status = [GoalStatus.ABORTED, GoalStatus.PREEMPTED])
```

The final step is to link the MDP action to the actionlib server. This is done in the code below. The `action_server` argument is used to configure the action server client, and the `add_string_argument` call is used to embed the parameters for the call into the action itself (see `strands_executive_msgs.mdp_action_utils` for more options).

```python
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
    add_string_argument(action, 'Hello everyone. Please go to the meeting room, the meeting is about to start.')

```

For the purposes of the tutorial you should understand the overall approach we are taking for this, and be able to map this understanding into the code. To see it working you can run:

```bash
rosrun planning_tutorial announce_at_waypoints.py
```
Feel free to edit the code as you wish and play around with the file. You could add more waypoints to the list, or add additional LTL navigation goals (as in [Exercise 1](./exercise_1.md)) to the string `mdp_spec.ltl_task` to change the robot's behaviour (e.g. get it to do an announcement without ever visiting a given (dangerous) waypoint).


# Exercise 2b

To successfully complete the following you need to be competent with both Python and ROS. If you're not comfortable with these, please pair up with someone who is.

To test your understanding, create a copy of `planning_tutorial announce_at_waypoints.py` and re-write it such that instead of completing an announcement at every waypoint, the problem is to simply perform a single announcement at any point at the map. However you should be able to specify that the chance of the action succeeding is different at different waypoints (with you inventing the numbers), and this should result in the robot choosing the waypoint with the highest chance of success for the annoucement first. 

# Exercise 2c

What happens when you combine your answer to 2b with the ability to change the probabilities of success on the edges of the topological map? Talk to one of the lecturers for help with getting this running if you want to try it.

