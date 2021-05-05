rtmros_choreonoid  [![Build Status](https://travis-ci.org/start-jsk/rtmros_choreonoid.png)](https://travis-ci.org/start-jsk/rtmros_choreonoid)
-------------

## Install
Create catkin workspace and add rtmros_choreonoid
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
wstool init .
wstool set rtm-ros-robotics/rtmros_choreonoid https://github.com/start-jsk/rtmros_choreonoid --git -y
```
Set up workspace.

If you install openhrp3 and hrpsys from apt.
```
wstool merge https://raw.githubusercontent.com/start-jsk/rtmros_choreonoid/master/.travis.rosinstall -y
wstool merge https://raw.githubusercontent.com/start-jsk/rtmros_choreonoid/master/.travis.rosinstall.${ROS_DISTRO} -y
wstool update
```

Else if you install openhrp3 and hrpsys from source.
```
wstool merge https://raw.githubusercontent.com/start-jsk/rtmros_choreonoid/master/.from_source.rosinstall -y
wstool update
```

Install requisites for choreonoid
```
./choreonoid/misc/script/install-requisites-ubuntu-16.04.sh # if ubuntu 16.04
./choreonoid/misc/script/install-requisites-ubuntu-18.04.sh # if ubuntu 18.04
```
Apply JSK system settings
```
patch -p1 -d choreonoid < rtm-ros-robotics/rtmros_choreonoid/choreonoid.patch
```
Build
```
cd ..
source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install -r --from-paths src --ignore-src -y
catkin build hrpsys_choreonoid
source devel/setup.bash
```

### run test
```
catkin build hrpsys_choreonoid_tutorials
source devel/setup.bash
rtmlaunch hrpsys_choreonoid_tutorials jaxon_jvrc_choreonoid.launch
```
Launch another terminal and send command to robot. (python)
```
ipython -i `rospack find hrpsys_choreonoid_tutorials`/scripts/jaxon_red_setup.py "JAXON_RED(Robot)0"
hcf.abc_svc.goPos(1,0,0)
```
Launch another terminal and send command to robot. (euslisp)
```
roseus `rospack find hrpsys_choreonoid_tutorials`/euslisp/jaxon_jvrc-interface.l
(jaxon_jvrc-init)
(send *ri* :go-pos 1 0 0)
(send *ri* :start-grasp)
(send *ri* :stop-grasp)
(setq *robot* *jaxon_jvrc*)
(objects *robot*)
(send *robot* :reset-manip-pose)
(send *ri* :angle-vector (send *robot* :angle-vector) 5000)
```

If you get error, try `export ORBgiopMaxMsgSize=2097152000`.

See also [hrpsys_choreonoid_tutorials/README.md](/hrpsys_choreonoid_tutorials/README.md)

---
## **Control simulator via roseus**

### Get coordinates on simulation
~~~
(load "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")
(get-coords-on-simulation :robot "JAXON_RED" :link "WAIST")
~~~

### Apply external force to robot model using EusLisp

Assume hrpsys_choreonoid_tutorials/jaxon_red_choreonoid.launch is running,
~~~
(load "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")
(add-external-force :pos #f(0 0 0) :force #f(0 100 0) :tm 0.1) ;;force is expressed in world frame
~~~

## **Tips**

### Move viewpoint in the simulation (Choreonoidシミュレータ内での視点の移動方法）
Move cursor while pushing space key.

### Use middle button with thinkpad keyborad

Write functions below on .bashrc
~~~
function choreonoidinput () {
xinput set-prop "TPPS/2 IBM TrackPoint" "Evdev Wheel Emulation" 0
xinput set-prop "TPPS/2 IBM TrackPoint" "Evdev Wheel Emulation Button" 3
xinput set-prop "TPPS/2 IBM TrackPoint" "Evdev Wheel Emulation" 1
}

function defaultinput () {
xinput set-prop "TPPS/2 IBM TrackPoint" "Evdev Wheel Emulation" 0
xinput set-prop "TPPS/2 IBM TrackPoint" "Evdev Wheel Emulation Button" 2
xinput set-prop "TPPS/2 IBM TrackPoint" "Evdev Wheel Emulation" 1
}
~~~

Type 'choreonoidinput' for changing role of buttons. Then middle button can work to translate views and right button can work to zoom views.

For returning to default input, type 'defaultinput' in any terminals.

---

## for JVRC (Old information), this may work with 'last_working_jvrc' tag
```
$ sudo add-apt-repository ppa:hrg/daily
$ sudo apt-get update
$ sudo apt-get install choreonoid libcnoid-dev
```
- install choreonoid see http://jvrc.github.io/tutorials/html-ja/install.html#installation-of-choreonoid  

- catkin build hrpsys_ros_bridge_jvrc and hrpsys_choreonoid

- roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_choreonoid.launch TASK:=O1
