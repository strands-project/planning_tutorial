#!/bin/bash

SESSION=lucia

# Set this variable in order to have a development workspace sourced, surplus/instead of the .bashrc one
DEVELOPMENT_WS=/home/socrob/ros_ws/devel/setup.bash
#DEVELOPMENT_WS=/opt/ros/kinetic/setup.bash
_SRC_ENV="tmux send-keys source Space $DEVELOPMENT_WS C-m "

tmux -2 new-session -d -s $SESSION
tmux new-window -t $SESSION:0 -n 'roscore'
tmux new-window -t $SESSION:1 -n 'mongo'
tmux new-window -t $SESSION:2 -n 'mbot'
tmux new-window -t $SESSION:3 -n 'teleop'
tmux new-window -t $SESSION:4 -n 'mbot_nav'
tmux new-window -t $SESSION:5 -n 'microphone'
tmux new-window -t $SESSION:6 -n 'exec'




tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roscore" C-m
tmux resize-pane -U 50
tmux select-pane -t 1
tmux send-keys "htop" C-m

tmux select-window -t $SESSION:1
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch mongodb_store mongodb_store.launch  db_path:=/home/socrob/ros_ws/mongo"

tmux select-window -t $SESSION:2
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roslaunch mbot_bringup robot.launch"
tmux select-pane -t 1
tmux send-keys "rosrun robot_pose_publisher robot_pose_publisher"

tmux select-window -t $SESSION:3
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux send-keys "roslaunch mbot_teleop_joypad teleop_joypad.launch"

tmux select-window -t $SESSION:4
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "export ROBOT_ENV=isr-8-floor" C-m
tmux send-keys "roslaunch mbot_2dnav 2dnav.launch"
tmux select-pane -t 1
tmux send-keys "roslaunch planning_tutorial topological_navigation.launch map:=isr-8-floor"

tmux select-window -t $SESSION:5
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "rosrun mbot_actions ask_question_server_node"
tmux select-pane -t 1
tmux send-keys "roslaunch vocon_speech_recognizer speech_recognition.launch"


tmux select-window -t $SESSION:6
[ -f $DEVELOPMENT_WS ] && `$_SRC_ENV`
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "roslaunch task_executor mdp-executor.launch combined_sort:=true"
tmux select-pane -t 1
tmux send-keys "rosrun planning_tutorial coffee_delivery_fill.py"





# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION
