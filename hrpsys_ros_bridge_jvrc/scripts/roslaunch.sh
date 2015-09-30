#!/bin/bash

pkill -INT -f jaxon_jvrc_choreonoid.launch

while [ "$(pgrep -f jaxon_jvrc_choreonoid.launch | wc -l)" != "0" ]; do
    sleep 1
done

trap exit_this 1 2 3 9 15 EXIT

function exit_this {
    echo "trapped"
    echo "LAUNCH PID: ${LAUNCH_PID}"
    /bin/kill -INT ${LAUNCH_PID}
    exit 0
}

roslaunch hrpsys_ros_bridge_jvrc jaxon_jvrc_choreonoid.launch > /dev/null &
LAUNCH_PID=$!
echo "LAUNCH PID: ${LAUNCH_PID}"

wait ${LAUNCH_PID}
