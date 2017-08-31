# Exercise 3 - Coffee delivery with the MBOT

In this exercise you will use our MDP/LTL planning framework to encode a coffee delivery task in a real robot (ISR's own MBOT). This example connects many of the different elements of our robot system, and may seem complex at first. If something is unclear, or you want more information, please just ask. Also, there might be more than one team working in the robot at the same time, so please talk to us before running anything.


# Background

You must first ssh into the robot. Connect to the ISR network, and add the following line in ``/etc/hosts`` (you need root access to do so):

```bash 
10.1.15.14 mbot01n
```
Now you can ssh into the mbot (ask the password to one of us):

```bash 
ssh socrob@mbot01n
```

In order for you to be able to use rviz to localise the robot, you'll also have to make sure your laptop is know to the robot. Check your IP using ifconfig, and add a line into  ``/etc/hosts`` in the robot:


```bash 
your_ip your_pc_name
```

In order to start the basic robot nodes, we have written a tmux script. You can find it at ``/home/socrob/ros_ws/src/planning_tutorial/scripts/start_rob.tmux``. Ask one of us to sow you how it works. This starts some of same nodes you have started in the previous exercises, and some robot specific nodes.


Once the configuration above is done and the nodes are running, you should be able to use rviz to see the from your laptop. Open a terminal and run 

```bash 
export ROS_MASTER_URI=http://mbot01n:11311
rviz -d `rospack find planning_tutorial`/plan_tut_mbot.rviz
```

You should now see the robot and a topological map over ISR's 8th floor map. This map covers the robot lab and the coffee room. Try to localise the robot using rviz's 2D Pose Estimate tool.

# Coffee Delivery

In this exercise, we will tackle a more involved planning problem, with more dependencies and reasoning needed to achieve the goal. We will also use an mbot skill implemented in actionlib.

The goal of this exercise is to have the robot asking people in the environment if they want coffee, and if so having the robot deliver them a coffee. We will assume that there is a predefined set of people to serve, and that there is a known probability distribution of their location in the environment and their need for coffee. As an example:


```bash
self.people= ['John', 'Paul']  #people to serve 
self.initial_location = { 'John':{'WayPoint1':0.8, 'WayPoint2':0.1}, 'Paul':{'WayPoint1':0.3, 'WayPoint7':0.6}}  #distribution of people's location in the environment
self.wants_coffee = {'John':0.7, 'Paul':0.2} #probability of people wanting coffee
```

The above means that we have two peolpe to look for, John and Paul. John is in WayPoint1 with probability 0.8, or in WayPoint2 with probability 0.1 (note that there is also a probability of John not being in any of these waypoints). Furthermore, John wants a coffee 70% of the time. A similar definition is made for Paul.


The idea is for the robot to go to the waypoints where people might be present and ask them whether they want coffee. If no one replies, we will assume that the person is not there. The person can also reply with 'yes' or 'no'. If the person says yes, then the robot should go to the coffee room, ask someone to place a coffee on its cupholder, and bring it to the location where the person replied.

# Exercise 3a


Check bla for an example of the search part of this task. Note that it uses an actionlib action server implementing a skill for the MBOT: make a question, and return a response according to the human's answer, or timeout. This integrates a sound output module and a speech recognition module into an actionlib server. You can see the definition of the action in bla bla.

Run the node a couple of times. Note that with the predifined probabilities, the robot always asks whether John want coffee in WayPoint1 first. This is because even though the ordering of the nodes doesn't change with how wsypoints orderings, the expected time for task completion does. If the probability of John being in WayPoint2 is very low, it is quicker to try WayPoint1 first, instead of wasting time executing the ask question action when probably no one is there. Increase the probability of John being in WayPoint1 and see how that changes the robot behaviour.

# Exercise 3b

Complete the example file to achieve the full coffee delivery behaviour. Use the speech action with different arguments for retrieving and delivering the coffee. For example, when the robot gets next to the coffe machine, it has an action available where it asks to put a coffee in the cupboard and waits for an 'ok' to be said by the human. If that happens within the timeout, then the robot can assume it is carrying a coffee. Similar actions can be defined for delivering the coffee to a person at a waypoint - make sure the robot delivers coffees on the correct waypoint where the person was found. You'll need to add state features that represent the number of coffees the robot is carrying, and whether the coffee has been delivered to each person; and an action to retrieve coffee, and actions to deliver it (try to adapt the action for asking if people want coffee to implement these). Call us if you need help understanding the code or have any doubt.





