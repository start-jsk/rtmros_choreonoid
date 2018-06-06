#!/bin/bash

ulimit -c unlimited

for count in `seq 100`
do
    rm -rf /tmp/core
    rm -rf /tmp/choreonoid_console.log
    echo ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;"
    echo ";;;;;"
    echo ";;;;; Start: $count"
    echo ";;;;;"
    echo ";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;"

    rtmlaunch hrpsys_choreonoid_tutorials jaxon_red_choreonoid.launch &> /tmp/choreonoid_console.log &
    sleep 180
    ### check / choreonoid dumped core
    if [ -e /tmp/core ]; then
        echo ";;;;;;; failed: $count"
        cp /tmp/core ~/core_fix_etc_choreonoid_$count;
    fi
    ### check / offsensor was connected
    roseus $(rospack find hrpsys_choreonoid_tutorials)/euslisp/check-offsensor.l
    if [ $? != 0 ]; then
        echo ";;;;;; offsensor failed"
        cp /tmp/choreonoid_console.log ~/console_fix_etc_choreonoid_$count;
    fi
    pkill -INT -f 'python /home/leus/ros/indigo/devel/bin/rtmlaunch'
    pkill -9 choreonoid
    sleep 20
done
