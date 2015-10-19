## JVRC2015

#### O1, O2, R1, R2

##### LAUNCH PROGRAMS

- launch fc programs(almost taking over DRC programs)

```bash
rosrun drc_task_common jvrc_fc.sh
```

- launch ocs programs(rviz, furutaractive, controller-device)

```bash
rosrun drc_task_common jvrc_ocs.sh
```

##### EXECUTE TASK

- match transformable marker to cloud
- two choices
-- select the cylinder on plane
-- insert primitive with botton and match it manually

- solve ik
-- change params
--- height of waist[0cm, 15cm lower, 30cm lower]
--- ik mode[grasp, peep1(for hand-cam1), peep2(for hand-cam2)]
-- change robot stand-pos
-- solve-ik with button

- send to robot
-- walk to object
-- send angle to robot
 

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

#### R2-C

- run simulator

```bash
sudo service omniorb4-nameserver restart
pkill -9 choreonoid
choreonoid $(rospack find hrpsys_ros_bridge_jvrc)/config/JVRCR2C.cnoid --start-simulation
```

![](images/task_R2-C.png)

- close hand as quickly as possible and then generate ``*ri*``

```bash
roseus `rospack find hrpsys_ros_bridge_jvrc`/euslisp/fast-hand-close-and-generate-ri.l
```

- start walking as quickly as possible with ``hcf``

```python
xsel -b < `rospack find hrpsys_ros_bridge_jvrc`/config/R2C-path.py
ipython -i `rospack find hrpsys_tools`/scripts/hrpsys_tools_config.py -- --use-unstable-rtc --host localhost --port 2809
%paste
```

- launch rviz and b-controller

```bash
roslaunch hrpsys_ros_bridge_jvrc R2C.launch
```

