- install choreonoid see http://jvrc.github.io/tutorials/html-ja/install.html#installation-of-choreonoid   

- **install just choreonoid** ```sudo apt-get install choreonoid libcnoid-dev```

- catkin build hrpsys_ros_bridge_jvrc and hrpsys_choreonoid

- roscd jvrc_models; bash setup_jvrc.sh

- choreonoid $(rospack find jvrc_models)/model/O1.cnoid

- ~~roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_choreonoid.launch~~

- ~~start simulation in choreonoid~~

- ~~roscd hrpsys_ros_bridge_jvrc; sudo cp config/SensorReaderRTC.*conf /usr/lib/choreonoid-1.5/rtc~~

- ~~sudo cp ~/ros/indigo_parent/devel/lib/{HG,PD}controller.so /usr/lib/choreonoid-1.5/rtc~~

- ~~sudo mkdir -p /usr/lib/choreonoid-1.5/customizer; sudo mkdir /usr/lib/choreonoid-1.5/rtc # if needed~~
