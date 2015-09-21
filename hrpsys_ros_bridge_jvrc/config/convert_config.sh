
for TASK in O1 O2 R11L R11M R12 R2AB R2C R3A R3B R4 R5; do
#CNOID_FILE=${TASK}.new.cnoid
CNOID_FILE=jaxon_pd${TASK}.cnoid
\cp -f $(rospack find jvrc_models)/model/tasks/${TASK}/${TASK}.cnoid ${CNOID_FILE}

sed -i -e "s@modelFile: \"@modelFile: \"../../jvrc_models/model/tasks/${TASK}/@" ${CNOID_FILE}
sed -i -e "s@\(info:.*\)@\1\n            - \n              id: 5\n              name: \"GLVisionSimulator\"\n              plugin: Body\n              class: GLVisionSimulatorItem\n              data: \n                enabled: true\n                targetBodies: [ JAXON_JVRC ]\n                targetSensors: [ HEAD_LEFT_CAMERA ]\n                maxFrameRate: 1000\n                maxLatency: 1\n                recordVisionData: false\n                useThread: true\n                useThreadsForSensors: true\n                bestEffort: false\n                allSceneObjects: false\n                rangeSensorPrecisionRatio: 2\n                depthError: 0\n                enableHeadLight: true\n                enableAdditionalLights: true\n            - \n              id: 6\n              name: \"GLVisionSimulator\"\n              plugin: Body\n              class: GLVisionSimulatorItem\n              data: \n                enabled: true\n                targetBodies: [ JAXON_JVRC ]\n                targetSensors: [ CHEST_CAMERA ]\n                maxFrameRate: 1000\n                maxLatency: 1\n                recordVisionData: false\n                useThread: true\n                useThreadsForSensors: true\n                bestEffort: false\n                allSceneObjects: false\n                rangeSensorPrecisionRatio: 2\n                depthError: 0\n                enableHeadLight: true\n                enableAdditionalLights: true\n            - \n              id: 7\n              name: \"GLVisionSimulator\"\n              plugin: Body\n              class: GLVisionSimulatorItem\n              data: \n                enabled: true\n                targetBodies: [ JAXON_JVRC ]\n                targetSensors: [ HEAD_RANGE ]\n                maxFrameRate: 1000\n                maxLatency: 1\n                recordVisionData: false\n                useThread: true\n                useThreadsForSensors: true\n                bestEffort: false\n                allSceneObjects: false\n                rangeSensorPrecisionRatio: 2\n                depthError: 0\n                enableHeadLight: true\n                enableAdditionalLights: true\n            - \n              id: 8\n              name: \"GLVisionSimulator\"\n              plugin: Body\n              class: GLVisionSimulatorItem\n              data: \n                enabled: true\n                targetBodies: [ JAXON_JVRC ]\n                targetSensors: [ LARM_CAMERA ]\n                maxFrameRate: 1000\n                maxLatency: 1\n                recordVisionData: false\n                useThread: true\n                useThreadsForSensors: true\n                bestEffort: false\n                allSceneObjects: false\n                rangeSensorPrecisionRatio: 2\n                depthError: 0\n                enableHeadLight: true\n                enableAdditionalLights: true\n            - \n              id: 9\n              name: \"GLVisionSimulator\"\n              plugin: Body\n              class: GLVisionSimulatorItem\n              data: \n                enabled: true\n                targetBodies: [ JAXON_JVRC ]\n                targetSensors: [ RARM_CAMERA ]\n                maxFrameRate: 1000\n                maxLatency: 1\n                recordVisionData: false\n                useThread: true\n                useThreadsForSensors: true\n                bestEffort: false\n                allSceneObjects: false\n                rangeSensorPrecisionRatio: 2\n                depthError: 0\n                enableHeadLight: true\n                enableAdditionalLights: true\n        - \n          id: 2\n          name: \"JAXON_JVRC\"\n          plugin: Body\n          class: BodyItem\n          data: \n            modelFile: \"../../jvrc_models/JAXON_JVRC/JAXON_JVRCmain_hrpsys_bush.wrl\"\n            currentBaseLink: \"WAIST\"\n            rootPosition: [ 3.2, 2.5, 1.332 ]\n            rootAttitude: [ \n              0, -1, 0, \n              1, 0, 0, \n              0, 0, 1 ]\n            jointPositions: [ \n               0.000054, -0.003093, -0.262419,  0.681091, -0.418672,  0.003093,  0.000054, -0.003093, -0.262401,  0.681084, \n              -0.418684,  0.003093,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.523599, -0.349066, \n              -0.087266, -1.396263,  0.000000,  0.000000, -0.349066,  0.000000,  0.523599,  0.349066,  0.087266, -1.396263, \n               0.000000,  0.000000, -0.349066,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]\n            initialRootPosition: [ 3.2, 2.5, 1.332 ]\n            initialRootAttitude: [ \n              0, -1, 0, \n              1, 0, 0, \n              0, 0, 1 ]\n            initialJointPositions: [ \n               0.000054, -0.003093, -0.262419,  0.681091, -0.418672,  0.003093,  0.000054, -0.003093, -0.262401,  0.681084, \n              -0.418684,  0.003093,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.523599, -0.349066, \n              -0.087266, -1.396263,  0.000000,  0.000000, -0.349066,  0.000000,  0.523599,  0.349066,  0.087266, -1.396263, \n               0.000000,  0.000000, -0.349066,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000,  0.000000 ]\n            zmp: [ 0, 0, 0 ]\n            collisionDetection: true\n            selfCollisionDetection: false\n            isEditable: true\n          children: \n            - \n              id: 3\n              name: \"BodyRTC\"\n              plugin: OpenRTM\n              class: BodyRTCItem\n              data: \n                isImmediateMode: true\n                moduleName: \"PDcontroller\"\n                confFileName: \"SensorReaderRTC.PD.conf\"\n                configurationMode: Use Configuration File\n                AutoConnect: false\n                InstanceName: JAXON_JVRC\n                bodyPeriodicRate: 0.002\n        - \n          id: 10\n          name: \"model-loader\"\n          plugin: Base\n          class: ExtCommandItem\n          data: \n            command: openhrp-model-loader\n            executeOnLoading: true@" ${CNOID_FILE}
sed -i -e "s@info: \"@info: \"../../jvrc_models/model/tasks/${TASK}/@" ${CNOID_FILE}

IDPOS=$(grep -n ' id:' ${CNOID_FILE} | sed -e 's@\([0-9]\+\).*@\1@')
VIEWPOS=$(grep -n '^views:' ${CNOID_FILE} | sed -e 's@\([0-9]\+\).*@\1@')

poslst=""
idcntr=0
for pos in ${IDPOS}; do
    #echo "${pos} ${VIEWPOS}"
    if [ ${pos} -le ${VIEWPOS} ]; then
        sed -i -e "${pos},${pos}s@id: \([0-9]\+\)@id: ${idcntr}@" ${CNOID_FILE}
        poslst="${poslst}${idcntr}, "
        idcntr=$(expr ${idcntr} + 1)
    fi
done

echo ${poslst}
sed -i -e "s@checked: .*@checked: [ ${poslst} ]@" ${CNOID_FILE}
done

sed -i -e "s@maxTime: 600@maxTime: 12000@" ${CNOID_FILE}
sed -i -e "s@timeLength: 600@timeLength: 12000@" ${CNOID_FILE}

# jaxon_pdO1.cnoid-            rootPosition: [ 3.2, 2.5, 1.332 ]
# jaxon_pdO2.cnoid-            rootPosition: [ 3.0, 1.5, 4.632 ]
# jaxon_pdR11L.cnoid-            rootPosition: [ 3.2, 1.5, 1.332 ]
# jaxon_pdR11M.cnoid-            rootPosition: [ 3.2, 1.5, 1.332 ]
# jaxon_pdR12.cnoid-            rootPosition: [ 3.2, 1.5, 1.332 ]
# jaxon_pdR2AB.cnoid-            rootPosition: [ 4.5, 2.0, 1.332 ]
# jaxon_pdR2C.cnoid-            rootPosition: [ 6.8, 2.0, 1.332 ]
# jaxon_pdR3A.cnoid-            rootPosition: [ 3.2, 2.0, 1.332 ]
# jaxon_pdR3B.cnoid-            rootPosition: [ 3.2, 2.0, 1.332 ]
# jaxon_pdR4.cnoid-            rootPosition: [ 4.6, 2, 1.332 ]
# jaxon_pdR5.cnoid-            rootPosition: [ 3.2, 2.6, 1.332 ]
