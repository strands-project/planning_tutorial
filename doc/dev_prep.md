# Installation

To go beyond the basic tutorial and do some development of your own code around either the STRANDS planning or navigation systems, you will need some additional dependencies and setup. This file explains the current approach needed to run monitored navigation and recovery behaviours in a ROS Kinetic system on Ubuntu 16.04.

1. First complete the steps outlined in [tutorial_prep.md](tutorial_prep.md)

2. Then follow [One time setup on a computer (by sys admin)](https://github.com/LCAS/rosdistro/wiki#one-time-setup-on-a-computer-by-sys-admin) and [
One time setup in a user's account (for a developer)](https://github.com/LCAS/rosdistro/wiki#one-time-setup-in-a-users-account-for-a-developer) from the [LCAS rosdistro wiki](https://github.com/LCAS/rosdistro/wiki)

3. The LCAS dependencies are a little out of date, so we also need to add our own overrides. To do this add the line `yaml file:///home/USERNAME/local_custom_rosdep.yaml` to the **top** of the file `/etc/ros/rosdep/sources.list.d/50-lcas.list`. This will allow us to specify our own rosdep definitions in the indicated file.

4. Next create the file `/home/USERNAME/local_custom_rosdep.yaml` and put the following into it.
```bash
strands-twython:
  ubuntu: [python-twython]
```

5. Initialise the source dir your catkin workspace for use with `wstool` in order to automatically manage source repos.
```bash
cd $WS_ROOT_DIR/src
wstool init
```

6. Bring in the extra source needed for running and developing parts of the STRANDS executive behaviour stack:
```bash
cd $WS_ROOT_DIR/src
curl https://raw.githubusercontent.com/strands-project/planning_tutorial/ori/config/tut_dev.rosinstall | wstool merge -k -y -
```

7. In your catkin worskpace, blacklist some packages which can be problematic to build:
```
cd $WS_ROOT_DIR
catkin config --blacklist scheduler scipoptsuite datamatrix_read
```

8. Rebuild 
```bash
catkin build
```

9. Resource
```bash
source $WS_ROOT_DIR/devel/setup.bash
```


