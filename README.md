Extended STEAP
===================================================

The new extended version of STEAP (Simultaneous Trajectory Estimation and Planning) is a navigation framework that supports  different types of mobile robot (Omni and Differential Drive). This repository heavily relies on GTSAM and GPMP2, thus, we refer users to carefully read the information below for installation as well as configuration these packages. 

- Hoang, T. Estimation and Planning Differential Drive Robot Simultaneously - Technical Report. Preprints 2019, 2019020084 (doi: 10.20944/preprints201902.0084.v1).
- [DEMO](https://youtu.be/0G3FmNgEhMs) 

PIPER
===================================================

PIPER (Probabilistic Inference based Platform for Essential problems in Robotics) is a modular package that provides support for algorithms that use probabilistic inference on factor graphs to solve various robotics problems. Each module can be independently installed and implements an easy ROS interface for that algorithm to run on any simulated or real robots. Currently PIPER supports the following algorithms:

- [GPMP2](http://www.cc.gatech.edu/~bboots3/files/GPMP2.pdf) - Gaussian Process Motion Planner 2
- [STEAP](http://www.cc.gatech.edu/~bboots3/files/STEAP.pdf) - Simultaneous Trajectory Estimation and Planning
- more coming soon ...

PIPER is being developed by [Mustafa Mukadam](mailto:mmukadam3@gatech.edu) at the Georgia Tech Robot Learning Lab. See [documentation](doc/index.md) for information on usage and development.

---
Table of Contents
---
- [Prerequisites](#prerequisites)
- [Compilation and Installation](#compilation-and-installation)
- [Questions and Bug reporting](#questions-and-bug-reporting)
- [Citing](#citing)
- [FAQs](#faqs)
- [License](#license)

---
Prerequisites
------

- Install [ROS](http://wiki.ros.org/Distributions). We have tested up to ROS Kinetic on Ubuntu 16.04.
- Install [GPMP2](https://github.com/gtrll/gpmp2) C++ library.


Compilation and Installation
------

- Initialize a catkin workspace (if you are using an existing catkin workspace this step is not needed)
    
  ```bash
  mkdir -p ~/piper_ws/src
  cd ~/piper_ws/src
  catkin_init_workspace
  ```

  Before running setup the environment variables

  ```bash
  source ~/piper_ws/devel/setup.bash
  ```

- Clone this repository in ```~/piper_ws/src```

  ```bash
  git clone https://github.com/gtrll/piper.git
  ```

- To compile only the ```piperbase``` library, in the catkin workspace directory do
    
  ```bash
  catkin_make
  ```

- Otherwise, to install some module, for example,  _X_: first make sure to install dependencies for _X_, besides the prerequisites for piperbase, then do
    
  ```bash
  catkin_make -build_flag_X
  ```

- Similarly, to install multiple modules, for example, _X_ and _Y_: install all their dependencies and then use their appropriate flags together
    
  ```bash
  catkin_make -build_flag_X -build_flag_Y
  ```
  
  See table below for currently supported modules, their dependencies and build flags
  
  | Module | Other Dependencies | Build Flag |
  |:------:|:------------------:|:----------:|
  |GPMP2|None|```-DBUILD_GPMP2_INTERFACE:OPTION=ON```|
  |STEAP|None|```-DBUILD_STEAP_INTERFACE:OPTION=ON```|
  |ALL|All from above|```-DBUILD_ALL_INTERFACE:OPTION=ON```|


Questions and Bug reporting
-----

Please use Github issue tracker to report bugs. For other questions please contact [Mustafa Mukadam](mailto:mmukadam3@gatech.edu).


Citing
-----

If you use PIPER in an academic context, please cite any module/algorithm specific publications you use, and cite the following:

```
@article{mukadam2017piper,
  title={{PIPER}},
  author={Mukadam, Mustafa},
  journal={[Online] Available at \url{https://github.com/gtrll/piper}},
  year={2017}
}
```


FAQs
-----

- Q: Cannot find trajectory_control server error

  A: To be able to run the included examples you need to have an [action server](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29) set up in your real or simulated robot's API. See ```trajectory_control_topic``` in [robot config](https://github.com/gtrll/piper/blob/master/doc/config.md).


License
-----

PIPER is released under the BSD license. See LICENSE file in this directory.
