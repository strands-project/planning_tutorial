# Exercise 1

In this first exercise you will use our MDP/LTL planning framework to make the robot drive around the map, and also play with the edges of the underlying map to see how different long-term robot experiences influence its behaviour when creating navigation policies.


# Background

You must first run some basic elements from the STRANDS system. You should ideally run each of these in a separate terminal where you have sourced both the ROS and your local workspace `setup.bash` files, as described in [tutorial_prep.md](./tutorial_prep.md). 

## MongoDB Store

First, check the `db` directory exists (which you should've created following [tutorial_prep.md](./tutorial_prep.md)). The following should not list any files or report an error:

```bash 
ls `rospack find planning_tutorial`/db
```

If that is ok, then launch [mongodb_store](http://wiki.ros.org/mongodb_store) using that directory as its data path:

```bash
roslaunch mongodb_store mongodb_store.launch db_path:=`rospack find planning_tutorial`/db
```

### MORSE Simulation

In another terminal, launch our simplified simulation of the [Transport Systems Catapult](http://ts.catapult.org.uk) (TSC). 

```bash
roslaunch strands_morse tsc_morse.launch 
```

### 2D and Topological Navigation

We have predefined a simple topological map for you to use in this tutorial. The first time (and only the first time!) you want to use topological navigation, you must add this mapt to the mongodb store. Do it with the following command:

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




