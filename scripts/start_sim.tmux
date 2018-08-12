#!/bin/bash

SESSION=sim

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
DEVELOPMENT_WS=`rospack find planning_tutorial`/../../devel/setup.bash
_SRC_ENV="tmux send-keys source Space $DEVELOPMENT_WS C-m "

tmux -2 new-session -d -s $SESSION
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo'
tmux new-window -t $SESSION:2 -n 'morse'
tmux new-window -t $SESSION:3 -n 'move_base'
tmux new-window -t $SESSION:4 -n 'monitored_nav'
tmux new-window -t $SESSION:5 -n 'topo_nav'
tmux new-window -t $SESSION:6 -n 'exec'
tmux new-window -t $SESSION:7 -n 'behaviour'




tmux select-window -t $SESSION:0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 50
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch mongodb_store mongodb_store.launch db_path:=`rospack find planning_tutorial`/db"

tmux select-window -t $SESSION:2
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "foo"
tmux select-pane -t 1
tmux send-keys "roslaunch strands_morse tsc_fast_morse.launch"

tmux select-window -t $SESSION:3
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch planning_tutorial tsc_move_base.launch"

tmux select-window -t $SESSION:4
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch planning_tutorial monitored_navigation.launch recoveries:=true mon_nav_config_file:=`rospack find planning_tutorial`/config/carpet_monitored_nav_config.yaml"
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "rosrun smach_viewer smach_viewer.py"
tmux select-pane -t 0

tmux select-window -t $SESSION:5
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "foo"
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch  planning_tutorial topological_navigation.launch"


tmux select-window -t $SESSION:6
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait planning_tutorial just-mdp-executor.launch combined_sort:=true"
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch --wait planning_tutorial mdp-executor-dependencies.launch"

tmux select-window -t $SESSION:7
tmux split-window -v
tmux select-pane -t 0
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux select-pane -t 1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "rosrun planning_tutorial coffee_delivery_fill.py"


# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
