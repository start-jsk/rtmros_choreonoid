## **install choreonoid from source**
### compile choreonoid
~~~
$ sudo apt-get install libyaml-dev
$ export CNOID_INSTALL_DIR=/usr/local/choreonoid
$ git clone https://github.com/s-nakaoka/choreonoid.git
$ mkdir -p choreonoid/build
$ cd choreonoid/build
$ cmake .. -DCMAKE_INSTALL_PREFIX=${CNOID_INSTALL_DIR} -DENABLE_INSTALL_RPATH=ON -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON -DBUILD_HELLO_WORLD_SAMPLE=ON -DBUILD_SPRING_MODEL_SAMPLE=ON -DOPENRTM_DIR=${HOME}/ros/indigo_parent/devel
$ make -j8
$ sudo make install
~~~

### download source files
```
wstool set --git rtm-ros-robotics/rtmros_choreonoid https://github.com/start-jsk/rtmros_choreonoid.git
wstool update rtm-ros-robotics/rtmros_choreonoid
```

### compile BodyRTC etc.
~~~
$ export CNOID_INSTALL_DIR=/usr/local/choreonoid
$ export PKG_CONFIG_PATH=${CNOID_INSTALL_DIR}/lib/pkgconfig:$PKG_CONFIG_PATH
$ roscd hrpsys_choreonoid
$ catkin build --this
~~~

### add PATH
~~~
export PATH=${CNOID_INSTALL_DIR}/bin:$PATH
~~~

### run test

- catkin build hrpsys_choreonoid_tutorials

- rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch

---

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
