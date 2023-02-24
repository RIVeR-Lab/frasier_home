# frasier_home
Public facing FRASIER RoboCup@Home codebase. For 2023 Robocup@Home competition

# frasier_robocup
FRASIER RoboCup@Home codebase.

## Installation Instructions
Note this instructions are tested on Ubuntu 18.04.

Dependencies:
* [ROS Melodic](http://wiki.ros.org/melodic/Installation)
* [HSR Software](https://docs.hsr.io/manual_en/index.html)
* [OpenRAVE](http://openrave.org)
* [Gurobi](http://www.gurobi.com/)
* [TrajOpt](http://rll.berkeley.edu/trajopt/doc/sphinx_build/html/)


**HSR Installation** 
* BEFORE INSTALLING READ THIS WHOLE SECTION
* Follow the instructions [here](https://docs.hsr.io/archives/hsrb_user_manual/2004/en/howto/pc_install.html).
* Ignore all lines pertaining to couchDB for that installation refer to the instructions [here] (https://docs.couchdb.org/en/stable/install/unix.html#installation-using-the-apache-couchdb-convenience-binary-packages).
* Install couchDB before attempting to full install ros-melodic-tmc-desktop-full
* If you don't have the login information, please ask James Tukpah or check the Password Manager file inside of RIVeR-RoboCup_WRS   2023 Channel

**OpenRAVE Installation**   
Install dependencies:
* Follow the instructions [here](https://github.com/RIVeR-Lab/openrave).
* Add the following to `.bashrc`:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(openrave-config --python-dir)/openravepy/_openravepy_
export PYTHONPATH=$PYTHONPATH:$(openrave-config --python-dir)
```

**Gurobi Installation**
Install Gurobi by following the instructions. You can get a free acadamic license if you have .edu email.
1. Login/register for an account
2. Go to download center in the upper right hand corner of the screen
3. Click view on the Gurobi Optimizer card and install the most recent gurobi software
4. Extract gurobi to either the home or opt directory (your choice)
5. Go to License Center in the upper right hand corner of the screen
6. Click request
7. Generate Named-user Academic License
8. Run /path/to/gurobi1000/linux64/bin/grbgetkey xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx in terminal
9. Store the license key in a directory
10. Add the following to the `.bashrc`:
```
# Gurobi
export GUROBI_HOME=/path/to/gurobi1000/linux64
```


**TrajOpt Installation**   
Install dependencies:
```
sudo apt install libopenscenegraph-dev libeigen3-dev
```
Trajopt uses Gurobi for the optimization. 
1. `git clone https://github.com/tkelestemur/trajopt.git`
2. `cd trajopt && mkdir build`  
3. `cd cmake/modules && code FindGUROBI.cmake` and add gurobi100 to the list of gurobi options
4. `cmake .. -DGUROBI_LIBRARY=/opt/gurobi1000/linux64/lib/libgurobi100.so`
5. `make -j4`
6. Test out functionality by running `ctest` inside of the build folder, should return with 8 tests failed our of 10.
7. Add the following to `.bashrc`:
```
export TRAJOPT_HOME=/path/to/trajopt/folder
```

## VCS Installation

The `vcs` is going to install several packages which are the dependencies of the `frasier_robocup`. Before you run the below commands, make sure that the dependencies are installed by following the instructions [here](https://github.com/tkelestemur/frasier_openrave.git).

1. `cd catkin_ws/src`
2. `git clone git@github.com:RIVeR-Lab/frasier_home.git`
4. `vcs import --input robocup_2018.yaml ~/catkin_ws/src`
5. `cd catkin_ws`
6. `catkin build`

Note: In order to use `vcs`, install `sudo apt-get install python3-vcstool`

**Package Installation**   
Install dependencies:
```
sudo apt install ros-melodic-ecl
```
1. `cd catkin_ws/src`
2. `git clone  https://github.com/tkelestemur/frasier_openrave.git`
3. `git clone https://github.com/tkelestemur/point_cloud_proc.git`
4. `git clone https://github.com/tkelestemur/frasier_demos.git`
5. `catkin build`

**Instructions for Bringup**   

In order to get all the topics from HSR to one of the already setted up PCs, first make sure HSR is running.
1. ssh into `administrator@hsrb.local`
2. Press down the kill switch on back of the robot.
3. run `sudo systemctl restart docker.hsrb.monitor.service`
4. Pull up the kill switch (after that wait for HSR Start voice).
5. Run `hsrb_mode` on the main terminal (not in ssh terminal).




**OPERATION INSTRUCTIONS**
Note this instructions are tested on Ubuntu 18.04.

Follow these instructions to turn on and test the functionality of HSR 82
* There are currently problems with HSR 41, and those will be fixed over the coming weeks

**Powering On the Robot**
1. Push down the red E-stop button on the back of the robot
2. Press down and hold the power button for three seconds. After which release. This should cause there to be a start up tone.
3. After hearing the start up tone, release the E-Stop button by pressing it down and turning it to the left
4. The previous step should result in HSR speaking the phrase "HSR Start". If this does not happen and you have released the E-Stop Continue to resetting the robot.

**Reseting the Robot**
1. Press the E-Stop of the robot
2. Open terminal on a computer that is hard connected to HSR
3. Access `hsrb_mode`
4. Ssh into the robot from hsrb_mode by typing the following
```
ssh adminsitrator@hsrb.local
```
5. Grab the password from the password document 
6. Enter the following command into the terminal
```
sudo systemctl restart docker.hsrb.robot.service
```
7. After entering this command wait for a new terminal line to appear. Once it does release the E-stop
8. Wait until you hear the phrase "HSR START". If the phrase is heard, the robot is reset. If not repeat the instructions.


**Powering Down the Robot**
1. HOld down the power button until everything is shut down.
