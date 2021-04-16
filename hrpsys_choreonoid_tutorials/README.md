
## **how to use**
### **JAXON environments**
simplest version:
```
$ rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch
```
DRCBOX & vision:
```
$ rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch TASK:=DRCBOX
$ roslaunch hrpsys_choreonoid_tutorials jaxon_multisense_local.launch
```
add VRML objects using yaml:
```
rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch LOAD_OBJECTS:=true ENVIRONMENT_YAML:=$(rospack find hrpsys_choreonoid_tutorials)/config/simple_step.yaml
```

If you use the open-source jaxon model, please use jaxon_jvrc_choreonoid.launch instead of jaxon_red_choreonoid.launch.

オープンソースのJAXONモデルを用いている場合はjaxon_red_choreonoid.launchの代わりにjaxon_jvrc_choreonoid.launchを使う．

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
