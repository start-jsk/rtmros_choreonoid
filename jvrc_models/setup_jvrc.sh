if [ -e model/JAXON_JVRC ]; then
    rm -rf model/JAXON_JVRC
fi
\cp -r JAXON_JVRC model
\cp task_configs/*.cnoid model

if [ ! -e /home/${USER}/roslaunch.sh ]; then
    cp $(rospack find hrpsys_ros_bridge_jvrc)/scripts/roslaunch.sh /home/${USER}
fi

if [ "$(grep CNOID_CUSTOMIZER_PATH /home/${USER}/.bashrc | wc -l)" == "0" ]; then
cat <<EOF >> /home/${USER}/.bashrc
##### <<< JVRC setting #####
export RTCTREE_NAMESERVERS=localhost:2809
export ROBOT=JAXON_RED
export ORBgiopMaxMsgSize=2147483648
export CNOID_CUSTOMIZER_PATH=\$(rospack find hrpsys_choreonoid)
##### >>> JVRC setting #####
EOF

cat <<EOF > /home/${USER}/rtc.conf.choreonoid
manager.is_master:YES
corba.nameservers:localhost:2809
naming.formats:%n.rtc
logger.file_name:/tmp/rtc%p.log
manager.shutdown_onrtcs:NO
manager.modules.load_path:/home/${USER}/ros/indigo_parent/devel/share/hrpsys/lib
manager.modules.preload:.so
manager.components.precreate:
example.SequencePlayer.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.ForwardKinematics.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.ImpedanceController.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.AutoBalancer.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.StateHolder.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.TorqueFilter.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.TorqueController.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.ThermoEstimator.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.ThermoLimiter.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.VirtualForceSensor.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.AbsoluteForceSensor.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.RemoveForceSensorLinkOffset.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.KalmanFilter.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.Stabilizer.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.CollisionDetector.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.SoftErrorLimiter.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.HGcontroller.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.PDcontroller.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.EmergencyStopper.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
example.RobotHardware.config_file:/home/${USER}/ros/indigo/src/rtm-ros-robotics/rtmros_choreonoid/hrpsys_ros_bridge_jvrc/models/JAXON_JVRC.conf
EOF
