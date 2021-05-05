
## **how to use**
### **JAXON environments**
simplest version: (JAXON_RED_RH_FLAT.cnoid)
```
$ rtmlaunch hrpsys_choreonoid_tutorials jaxon_jvrc_choreonoid.launch
```

DRCBOX: (JAXON_RED_DRCBOX.cnoid)
```
$ rtmlaunch hrpsys_choreonoid_tutorials jaxon_jvrc_choreonoid.launch USE_ROBOTHARDWARE:=false TASK:=DRCBOX
```

add VRML objects using yaml: (JAXON_RED_RH_LOAD_OBJ.cnoid)
```
$ rtmlaunch hrpsys_choreonoid_tutorials jaxon_jvrc_choreonoid.launch LOAD_OBJECTS:=true ENVIRONMENT_YAML:=$(rospack find hrpsys_choreonoid_tutorials)/config/simple_step.yaml
```

If simulatitors often end abnormaly, the stand-alone roscore may solve the problem (Please launch the roscore before rtmlaunch command.).
This enables you to reuse a single '*ri*' in simurations and to skip '(jaxon_red-init)'.

シミュレーションが頻繁に突然終了する時には，roscoreを別で立ち上げておくと改善される場合がある．
また，roscoreを立ち上げておくと，毎回シミュレーションを立ち上げなおしてもroseusにおいて`*ri*`を保持することができ，angle-vectorなどを使うために`(jaxon_red-init)`などし直す必要がない．


### **how to use hcf**
Run one of the environments and open a new terminal:
```
roscd hrpsys_choreonoid_tutorials/scripts
ipython -i jaxon_red_setup.py "JAXON_RED(Robot)0"
```

### **how to use simulator functions from roseus**
Run one of the environments and open a new terminal:
```
roseus
(load "package://hrpsys_choreonoid/scripts/choreonoid-service-client.l")
```
Example of add external force (the force direction is calculated in the "world frame" of the simulator)
```
(add-external-force :link "WAIST" :pos #f(0 0 0) :force #f(100 0 0) :tm 0.1)
```
Get absolute position and orientation in the simulation
```
(get-coords-on-simulation :link "WAIST")

```
Reset(sometimes work well)
```
(reset-position :coords (make-coords :pos #f(0 0 990))) ;; stが入っているとうまく行かない
(reset-simulation :coords (make-coords :pos #f(0 0 990))) ;; *ri* が必要(stを止めたりしてくれる)
```



### **chidori**
```
$ rtmlaunch hrpsys_choreonoid_tutorials chidori_choreonoid.launch
```
