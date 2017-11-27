#!/bin/bash

### choose choreonoid binary
choreonoid_exe='choreonoid'

if [ "$(pgrep -x ${choreonoid_exe} | wc -l)" != 0 ]; then
    pkill -9 ${choreonoid_exe}
    echo "****************************************************" 1>&2
    echo "*                                                  *" 1>&2
    echo "*                                                  *" 1>&2
    echo "*   Old choreonoid process was found.              *" 1>&2
    echo "*   Process has been killed, please start again.   *" 1>&2
    echo "*                                                  *" 1>&2
    echo "*                                                  *" 1>&2
    echo "****************************************************" 1>&2
    exit 1
fi

cnoid_proj=""
if [ "$(echo $1 | grep \.cnoid$ | wc -l)" == 1 ]; then
    cnoid_proj=$1
fi
start_sim=""
enable_const=""
add_objects=""
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
  if [ $arg = "--add-objects" ]; then
      if [ "${EXTRA_CHOREONOID_OBJS}" != "" ]; then
          add_objects="--python $(rospack find hrpsys_choreonoid)/launch/add_objects.py"
      fi
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

echo "choreonoid will run with rtc.conf file of $rtc_conf" 1>&2
echo "contents of $rtc_conf are listed below" 1>&2
echo "<BEGIN: rtc.conf>" 1>&2
cat $rtc_conf 1>&2
echo "<END: rtc.conf>" 1>&2

export RTCTREE_NAMESERVERS=localhost:15005
export ORBgiopMaxMsgSize=2147483648
export CNOID_CUSTOMIZER_PATH=$(rospack find hrpsys_choreonoid)

if [ -z "$cnoid_proj" ]; then
    echo "choreonoid will be executed by the command below" 1>&2
    echo "$ (cd /tmp; $choreonoid_exe --python $(rospack find hrpsys_choreonoid)/scripts/create_environment.py)" 1>&2
    (cd /tmp; $choreonoid_exe --python $(rospack find hrpsys_choreonoid)/scripts/create_environment.py)
else
    echo "choreonoid will be executed by the command below" 1>&2
    echo "$ (cd /tmp; $choreonoid_exe $enable_const $add_objects $cnoid_proj $start_sim)" 1>&2
    (cd /tmp; $choreonoid_exe $enable_const $add_objects $cnoid_proj $start_sim)
    ## for using gdb
    #(cd /tmp; gdb -ex run --args $choreonoid_exe $enable_const $add_objects $cnoid_proj $start_sim)
fi
