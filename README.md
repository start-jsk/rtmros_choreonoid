- install choreonoid see http://jvrc.github.io/tutorials/html-ja/install.html#installation-of-choreonoid       

- catkin build hrpsys_ros_bridge_jvrc and hrpsys_choreonoid

- roscd hrpsys_choreonoid; ./install_plugins.sh ## install *.so to /usr/lib/choreonoid-1.5

- roslaunch hrpsys_ros_ros_bridge_jvrc jaxon_choreonoid.launch

- start simulation in choreonoid