for TASK in O1 O2 R11L R11M R12 R2AB R2C R3A R3B R4 R5; do

CNOID_FILE=JVRC_${TASK}.cnoid.in
\cp -f $(rospack find jvrc_models)/task_configs/${TASK}.cnoid ${CNOID_FILE}

### for test
sed -i -e "s@maxTime: 600@maxTime: 12000@" ${CNOID_FILE}
sed -i -e "s@timeLength: 600@timeLength: 12000@" ${CNOID_FILE}
# RTMROS_C=$(rospack find hrpsys_ros_bridge_jvrc)/..
# sed -i -e "s@/home/player/roslaunch.sh@/home/leus/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/scripts/roslaunch.sh@" ${CNOID_FILE}
sed -i -e "s@/home/player/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_choreonoid/PDcontroller.so@\${JVRC_RTC_DIRECTORY}/PDcontroller@" ${CNOID_FILE}
sed -i -e "s@/home/player/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/config/SensorReaderRTC.PD.conf@\${JVRC_CONF_DIRECTORY}/SensorReaderRTC.PD.conf@" ${CNOID_FILE}
sed -i -e "s@JAXON_JVRC/@../../jvrc_models/JAXON_JVRC/@" ${CNOID_FILE}
sed -i -e "s@tasks/@../../jvrc_models/model/tasks/@" ${CNOID_FILE}

done
