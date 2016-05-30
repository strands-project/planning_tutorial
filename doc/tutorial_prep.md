# Support

If you have difficulty following the steps in this document, please either contact Nick Hawes (n.a.hawes@cs.bham.ac.uk) or use the gitter chat at https://gitter.im/strands-project/planning_tutorial

# Computer Setup

To take part in the practical session of the tutorial, you will need to have your own laptop configured with the following software.

1. Install Ubuntu Linux 14.04LTS 64bit on your computer. Please make sure that you have exactly this version installed: 14.04 for 64bit. Download the image from here: http://releases.ubuntu.com/14.04.4/ (the image download is ubuntu-14.04.4-desktop-amd64.iso). Note, that you can perfectly install Ubuntu 14.04 alongside an existing operating system (even on a MacBook), or you could run this in a virtual machine.

2. Within Ubuntu, follow the instructions at https://github.com/strands-project-releases/strands-releases/wiki#using-the-strands-repository to install both ROS and the STRANDS software packages. We assume a basic understanding of Unix-like operating systems and shell usage here. If you need help using the command line, this might be a good start: https://help.ubuntu.com/community/UsingTheTerminal. 
The relevant installation steps are copied below for your convenience:
    1. Enable the ROS repositories: Follow steps 1.1-1.3 in http://wiki.ros.org/indigo/Installation/Ubuntu#Installation. There is no need to do the steps after 1.3.
    2. Enable the STRANDS repositories:
        1. Add the STRANDS public key to verify packages:
       `curl -s http://lcas.lincoln.ac.uk/repos/public.key | sudo apt-key add -`
        2. Add the STRANDS repository: `sudo apt-add-repository http://lcas.lincoln.ac.uk/repos/release`
    3. update your index: `sudo apt-get update`
    4. install required packages: `sudo apt-get install ros-indigo-desktop-full ros-indigo-strands-desktop python-catkin-tools`

3. Try out the “MORSE” simulator (run all the following in your terminal): 
    1. configure ROS: `source /opt/ros/indigo/setup.bash`
    2. launch the simulator: `roslaunch strands_morse aloof_morse.launch`
    You should see the Morse simulator popping up with our robot platform being configured. 
  

If your laptop uses an NVidia graphics card it might be worth looking at: https://wiki.ubuntu.com/Bumblebee to use it to its full potential.
You should be all set now!

# Tutorial Packages

For the tutorial itself you will be asked to write some simple Python scripts to control a robot in ROS via our planning and execution packages. You will be taught about these packages in the tutorial itself, but  we will not have time to cover the basics of Python. If you are not familiar with Python, it would certainly help to run through a quick Python tutorial (there are many online) to get comfortable with the language. That said, we will try our hardest to ensure that you can access as much of the tutorial material as possible without knowing Python. 