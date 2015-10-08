#!/usr/bin/env bash
CMDNAME=$(basename $0)

while getopts hf:o:ka OPT
do
    case $OPT in
        "f") FC_IP="$OPTARG";;
        "o") WALK_IP="$OPTARG";;
        "k") tmux kill-session -t walk; exit;;
        "a") tmux a -t walk; exit;;
        "h") echo "Usage: $CMDNAME [-f FC_IP] [-o WALK_IP]"; exit;;
    esac
done

tmux-newwindow() {
    if [ `tmux list-windows 2>/dev/null | grep $1 | sed -e 's/ //g'  >/dev/null 2>&1` ]; then
        echo $1 "already exists"
    else
        tmux new-window -k -n $1 -t walk
        tmux send-keys -t walk:$1 "$2" C-m
    fi
}

if `tmux has-session -t walk`; then
    echo -e "\e[1;33msession named walk already exists.\e[m"
    exit 1
else
    echo -e "\e[1;34mcreate new session named walk.\e[m"
    tmux new-session -d -s walk -n tmp
fi

tmux-newwindow o2 "rosrun hrpsys_ros_bridge_jvrc jaxon_o12.l"
tmux-newwindow r12 "rosrun hrpsys_ros_bridge_jvrc jaxon_r12.l"
tmux-newwindow walk "python `rospack find hrpsys_ros_bridge_jvrc`/scripts/walking.py -i --use-unstable-rtc --host `echo $ROS_HOSTNAME` --port 2809"
tmux send-keys -t walk:tmp "exit" C-m
tmux a -t walk

