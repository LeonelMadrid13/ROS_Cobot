# Requeriments

In ROS Noetic, you need to install them manually.

To ensure that they are installed, open a terminal and type:

```Bash
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-slam-gmapping
```

The first command installs the navigation stack.

The second command installed the SLAM package.

Now you can install Turtlebot3 as shown next.

## Trutlebot3 installation

Before installing Turtlebot3, make sure to make the following two commands:

```Bash
sudo apt-get update

sudo apt-get upgrade
```

**The installation may fail if you do not upgrade.**

Then, do the following (if you install for **noetic**, `make -b noetic-devel` to get the right branch)

```Bash
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b noetic-devel
git clone  https://github.com/ROBOTIS-GIT/turtlebot3.git -b noetic-devel
cd ~/catkin_ws && catkin_make
```

If you install on **melodic**, change `-b noetic-devel` with `-b melodic-devel`

This will install the core packages of Turtlebot3.

Afterward, and **after the correct compilation of the catkin_ws,** you can download and installation the simulation packages

```Bash
cd ~/catkin_ws/src/
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ~/catkin_ws && catkin_make
```

As such, the Turtlebot3 simulator should be installed.

Then, I made the modification in the .bashrch file as follows:

```Bash
cd
gedit .bashrc
```

Inside the bashrc file, put the following aliases to make it easier to access different executables in the **alias** section.

```Bash
alias burger='export TURTLEBOT3_MODEL=burger'
alias waffle='export TURTLEBOT3_MODEL=waffle'
alias tb3fake='roslaunch turtlebot3_fake turtlebot3_fake.launch'
alias tb3teleop='roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch'
alias tb3='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
alias tb3maze='roslaunch turtlebot3_gazebo turtlebot3_world.launch'
alias tb3house='roslaunch turtlebot3_gazebo turtlebot3_house.launch'
```

also, at the end of the file, write the following commands
```Bash
source /opt/ros/noetic/setup.bash
source /home/akoubaa/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
export SVGA_VGPU10=0
```
**The last command will let you open Gazebo on a Virtual Machine and avoid crashing its display.**

You can change export `TURTLEBOT3_MODEL=waffle` by `export TURTLEBOT3_MODEL=burger` if you want to use TB3 Burger.
