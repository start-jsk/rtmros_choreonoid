- install choreonoid see http://jvrc.github.io/tutorials/html-ja/install.html#installation-of-choreonoid   

- **install just choreonoid** ```sudo apt-get install choreonoid libcnoid-dev```

- catkin build hrpsys_choreonoid_tutorials

- roslaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch

---

- for JVRC

- catkin build hrpsys_ros_bridge_jvrc and hrpsys_choreonoid

- roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_choreonoid.launch TASK:=O1
