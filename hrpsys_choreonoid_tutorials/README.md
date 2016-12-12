
## **how to use**
### **JAXON**
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

### **chidori**
```
$ rtmlaunch hrpsys_choreonoid_tutorials chidori_choreonoid.launch
```
