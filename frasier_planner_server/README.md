## frasier_openrave_planner

### Installation

0. Install [frasier_openrave](https://github.com/tkelestemur/frasier_openrave) package.
1. `cd catkin_ws/src`
2. `git clone  https://github.com/RIVeR-Lab/frasier_interface.git`
3. `git checkout tarik-dev`
3. `catkin build`


### Usage

1. `roscore`
2. `rosrun frasier_planner_server frasier_planner_server`
3. `rosrun frasier_planner_server test_plan_req.py`
4. `Enjoy!`