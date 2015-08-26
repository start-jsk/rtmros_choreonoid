- install choreonoid see http://jvrc.github.io/tutorials/html-ja/install.html#installation-of-choreonoid   

- **install just choreonoid** ```sudo apt-get install choreonoid libcnoid-dev```

- download task model file from http://www.jvrc.org/download.html / unzip under jvrc_models 

- catkin build hrpsys_ros_bridge_jvrc and hrpsys_choreonoid

- sudo mkdir -p /usr/lib/choreonoid-1.5/customizer; sudo mkdir /usr/lib/choreonoid-1.5/rtc # if needed

- roscd hrpsys_ros_bridge_jvrc; sudo cp config/SensorReaderRTC.*conf /usr/lib/choreonoid-1.5/rtc

- sudo cp ~/ros/indigo_parent/devel/lib/{HG,PD}controller.so /usr/lib/choreonoid-1.5/rtc

- roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_choreonoid.launch

- start simulation in choreonoid
