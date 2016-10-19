## **install just choreonoid**
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

## **install choreonoid from source**
### compile choreonoid
~~~
$ export CNOID_INSTALL_DIR=/usr/local/choreonoid
$ git clone https://github.com/s-nakaoka/choreonoid.git
$ mkdir -p choreonoid/build
$ cd choreonoid/build
$ cmake .. -DCMAKE_INSTALL_PREFIX=${CNOID_INSTALL_DIR} -DENABLE_INSTALL_RPATH=ON -DENABLE_CORBA=ON -DBUILD_CORBA_PLUGIN=ON -DBUILD_OPENRTM_PLUGIN=ON -DBUILD_HELLO_WORLD_SAMPLE=ON -DBUILD_SPRING_MODEL_SAMPLE=ON -DOPENRTM_DIR=${HOME}/ros/indigo_parent/devel
$ make -j8
$ sudo make install
~~~

### compile BodyRTC etc.
~~~
$ export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:${CNOID_INSTALL_DIR}/lib/pkgconfig
$ roscd hrpsys_choreonoid
$ catkin build --this
~~~

### add PATH
~~~
export PATH=$PATH:${CNOID_INSTALL_DIR}/bin
~~~

---

## for JVRC (Old information), this may work with 'last_working_jvrc' tag
- install choreonoid see http://jvrc.github.io/tutorials/html-ja/install.html#installation-of-choreonoid  

- catkin build hrpsys_ros_bridge_jvrc and hrpsys_choreonoid

- roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_choreonoid.launch TASK:=O1
