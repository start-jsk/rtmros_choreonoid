## JVRC2015

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

