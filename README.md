- install choreonoid see http://jvrc.github.io/tutorials/html-ja/install.html#installation-of-choreonoid   

- download task model file from http://www.jvrc.org/download.html / unzip under jvrc_models 

- catkin build hrpsys_ros_bridge_jvrc and hrpsys_choreonoid

- roscd hrpsys_ros_bridge_jvrc; sudo cp config/SensorReaderRTC.conf /usr/lib/choreonoid-1.5/rtc

- roslaunch hrpsys_ros_ros_bridge_jvrc jaxon_choreonoid.launch

- start simulation in choreonoid

- roslaunch hrpsys_ros_bridge_jvrc vision_connect.launch
