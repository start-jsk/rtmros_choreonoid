## JVRC2015

#### R2-AB

- launch footstep controller and multisense

```bash
rosrun drc_task_common jvrc_fc.sh
```

- launch footstep planner and rviz

```bash
rosrun drc_task_common jvrc_ocs.sh
```

- load euslisp

```bash
roseus `rospack find hrpsys_ros_bridge_jvrc`/euslisp/cross-step.l
```

#### R2-C : narrow space field

time limit : 5 minutes

- run simulator

```bash
sudo service omniorb4-nameserver restart
pkill -9 choreonoid
choreonoid $(rospack find hrpsys_ros_bridge_jvrc)/config/JVRCR2C.cnoid --start-simulation
```

![](images/task_R2-C.png)

###### How to complete this task

Open 3 terminals and execute the following commands. JAXON will start moving before rviz is displayed and keep walking until he collides the surrounding walls and stops walking automatically. After that, move JAXON with the interactive marker.

1. close hand as quickly as possible and then generate ``*ri*``

```bash
roseus `rospack find hrpsys_ros_bridge_jvrc`/euslisp/fast-hand-close-and-generate-ri.l
```

2. start walking as quickly as possible with ``hcf``

```python
xsel -b < `rospack find hrpsys_ros_bridge_jvrc`/config/R2C-path.py
ipython -i `rospack find hrpsys_tools`/scripts/hrpsys_tools_config.py -- --use-unstable-rtc --host localhost --port 2809
%paste
```

3. launch rviz and b-controller

```bash
roslaunch hrpsys_ros_bridge_jvrc R2C.launch
```

