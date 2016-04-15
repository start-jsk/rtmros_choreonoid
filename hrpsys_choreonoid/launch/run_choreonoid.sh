#!/bin/bash

cnoid_proj=""
if [ "$(echo $1 | grep \.cnoid$ | wc -l)" == 1 ]; then
    cnoid_proj=$1
fi
start_sim=""
enable_const=""
rtm_args=()
latch=0

for arg in $@; do
## debug
##  echo $arg 1>&2;
  if [ $latch = 1 ]; then
      rtm_args=("${rtm_args[@]}" $arg)
      latch=0
  fi
  if [ $arg = "-o" ]; then
      latch=1
  fi
  if [ $arg = "--start-simulation" ]; then
      start_sim=$arg
  fi
  if [ $arg = "--enable-constraint" ]; then
      enable_const="--python $(rospack find hrpsys_choreonoid)/launch/constraint_enable.py"
  fi
done

## debug
##echo "${rtm_args[@]}" 1>&2

rtc_conf="/tmp/rtc.conf.choreonoid"
if [ -e $rtc_conf ]; then
    rm -rf $rtc_conf
fi

for arg in "${rtm_args[@]}";
  do echo $arg >> $rtc_conf;
done

echo "$ choreonoid $cnoid_proj" 1>&2
echo "with rtc.conf file on $rtc_conf" 1>&2
echo "<BEGIN: rtc.conf>"
cat $rtc_conf 1>&2
echo "<END: rtc.conf>"

export RTCTREE_NAMESERVERS=localhost:2809
export ROBOT=JAXON_RED
export ORBgiopMaxMsgSize=2147483648
export CNOID_CUSTOMIZER_PATH=$(rospack find hrpsys_choreonoid)

(cd /tmp; choreonoid $enable_const $cnoid_proj $start_sim)
## for using gdb
#(cd /tmp; gdb -ex run --args choreonoid $cnoid_proj $start_sim)
