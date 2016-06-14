- **install just choreonoid**
```
$ sudo add-apt-repository ppa:hrg/daily
$ sudo apt-get update
$ sudo apt-get install choreonoid libcnoid-dev
```
- download source files
```
wstool set --git rtm-ros-robotics/rtmros_choreonoid https://github.com/start-jsk/rtmros_choreonoid.git
wstool update rtm-ros-robotics/rtmros_choreonoid
```

- catkin build hrpsys_choreonoid_tutorials

- rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch

---

- for JVRC (Old information), this may work with 'last_working_jvrc' tag
- install choreonoid see http://jvrc.github.io/tutorials/html-ja/install.html#installation-of-choreonoid  

- catkin build hrpsys_ros_bridge_jvrc and hrpsys_choreonoid

- roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_choreonoid.launch TASK:=O1
