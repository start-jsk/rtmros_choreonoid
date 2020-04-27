cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_choreonoid_tutorials)

find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge euscollada rostest rospy
  euslisp xacro control_msgs roseus hrpsys_ros_bridge_tutorials jvrc_models hrpsys_choreonoid)

include(FindPkgConfig)
pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
set(OPENHRP_SAMPLE_DIR ${openhrp3_PREFIX}/share/OpenHRP-3.1/sample)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)

catkin_python_setup()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS hrpsys_ros_bridge euscollada rospy
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

##
find_package(jsk_models QUIET)
if(NOT "${jsk_models_FOUND}")
  string(ASCII 27 Esc)
  message(WARNING "${Esc}[1;33mPackage jsk_models is not found, if you have right to access them please include source code in catkin workspace${Esc}[m")
else()
  set (JSK_MODELS_DIR ${jsk_models_SOURCE_PREFIX})
endif()

################################
## compile_openhrp_model
##  Generate OpenHRP3 .xml and .conf file.
##  Convert .wrl, .dae, and .l files
################################

### convert model for closed models
macro(compile_model_for_closed_robots _robot_wrl_file _OpenHRP2_robot_name)
  if(EXISTS ${_robot_wrl_file})
    compile_openhrp_model(
      ${_robot_wrl_file}
      ${_OpenHRP2_robot_name}
      ${ARGN})
  else()
    message(STATUS "\n\n\n\n ${_robot_wrl_file} is not found..\n\n\n\n")
  endif()
endmacro()
macro(compile_rbrain_model_for_closed_robots _OpenHRP2_robot_vrml_name _OpenHRP2_robot_dir _OpenHRP2_robot_name)
  compile_model_for_closed_robots(
    ${jvrc_models_MODEL_DIR}/${_OpenHRP2_robot_dir}/${_OpenHRP2_robot_vrml_name}main.wrl
    ${_OpenHRP2_robot_name}
    ${ARGN})
endmacro()
macro(gen_minmax_table_for_closed_robots _OpenHRP2_robot_vrml_name _OpenHRP2_robot_dir _OpenHRP2_robot_name)
  if (EXISTS ${jvrc_models_MODEL_DIR}/${_OpenHRP2_robot_dir}/${_OpenHRP2_robot_vrml_name}main.wrl)
    string(TOLOWER ${_OpenHRP2_robot_name} _sname)
    set(_workdir ${PROJECT_SOURCE_DIR}/models)
    set(_gen_jointmm_command_arg "\"\\(write-min-max-table-to-robot-model-file \\(${_sname}\\) \\\"${_workdir}/${_sname}.l\\\" :margin 1.0\\)\"")
    set(_gen_jointmm_conf_command_arg "\"\\(write-min-max-table-to-conf-file \\(${_sname}\\) \\\"${_workdir}/${_OpenHRP2_robot_name}.conf\\\"\\)\"")

    find_program(euslisp_exe irteusgl)
    if(NOT EXISTS ${euslisp_exe})
      message(FATAL_ERROR "failed to find euslisp, skipping generating min max table")
    else()
      message(STATUS "find euslisp on ${euslisp_exe}")
      add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_done
      COMMAND DISPLAY= ${euslisp_exe} ${hrpsys_ros_bridge_tutorials_SOURCE_PREFIX}/euslisp/make-joint-min-max-table.l ${_workdir}/${_sname}.l "\"${_gen_jointmm_command_arg}\"" "\"(exit)\"" && touch ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_done
      DEPENDS ${_workdir}/${_sname}.l)
      add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_conf_done
      COMMAND DISPLAY= ${euslisp_exe} ${hrpsys_ros_bridge_tutorials_SOURCE_PREFIX}/euslisp/make-joint-min-max-table.l ${_workdir}/${_sname}.l "\"${_gen_jointmm_conf_command_arg}\"" "\"(exit)\"" && touch ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_conf_done
      DEPENDS ${_workdir}/${_sname}.l ${_workdir}/${_OpenHRP2_robot_name}.xml ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_done)
    add_custom_target(${_sname}_${PROJECT_NAME}_compile_joint_minmax DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_done ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_conf_done)
    add_dependencies(${_sname}_${PROJECT_NAME}_compile ${_sname}_${PROJECT_NAME}_compile_joint_minmax)
    endif()
  endif()
endmacro()

if(NOT ENV{TRAVIS_JOB_ID})
# JAXON_JVRC
compile_rbrain_model_for_closed_robots(JAXON_JVRC JAXON_JVRC JAXON_JVRC
  --conf-dt-option "0.002"
  --simulation-timestep-option "0.002"
  --robothardware-conf-file-option "model: file://${jvrc_models_MODEL_DIR}/JAXON_JVRC/JAXON_JVRCmain_hrpsys.wrl"
  --robothardware-conf-file-option "pdgains.file_name: ${hrpsys_ros_bridge_tutorials_SOURCE_PREFIX}/models/PDgains.sav"
  --conf-file-option "model: file://${jvrc_models_MODEL_DIR}/JAXON_JVRC/JAXON_JVRCmain_hrpsys.wrl"
  --conf-file-option "pdgains_sim_file_name: ${PROJECT_SOURCE_DIR}/models/JAXON_JVRC.PDgains_sim.dat"
  --conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/JAXON_JVRC.PDgains.sav"
  --conf-file-option "abc_leg_offset: 0.0, 0.1, 0.0"
  --conf-file-option "end_effectors: rleg,RLEG_JOINT5,WAIST,0.0,0.0,-0.1,0.0,0.0,0.0,0.0, lleg,LLEG_JOINT5,WAIST,0.0,0.0,-0.1,0.0,0.0,0.0,0.0, rarm,RARM_JOINT7,CHEST_JOINT2,0.0,0.055,-0.217,0,1.0,0.0,1.5708, larm,LARM_JOINT7,CHEST_JOINT2,0.0,-0.055,-0.217,0,1.0,0.0,1.5708,"
  --conf-file-option "collision_pair: RLEG_JOINT2:LLEG_JOINT2 RLEG_JOINT2:LLEG_JOINT3 RLEG_JOINT2:LLEG_JOINT5 RLEG_JOINT2:RARM_JOINT3 RLEG_JOINT2:RARM_JOINT4 RLEG_JOINT2:RARM_JOINT5 RLEG_JOINT2:RARM_JOINT6 RLEG_JOINT2:LARM_JOINT3 RLEG_JOINT2:LARM_JOINT4 RLEG_JOINT2:LARM_JOINT5 RLEG_JOINT2:LARM_JOINT6 RLEG_JOINT3:LLEG_JOINT2 RLEG_JOINT3:LLEG_JOINT3 RLEG_JOINT3:LLEG_JOINT5 RLEG_JOINT3:RARM_JOINT3 RLEG_JOINT3:RARM_JOINT4 RLEG_JOINT3:RARM_JOINT5 RLEG_JOINT3:RARM_JOINT6 RLEG_JOINT3:LARM_JOINT3 RLEG_JOINT3:LARM_JOINT4 RLEG_JOINT3:LARM_JOINT5 RLEG_JOINT3:LARM_JOINT6 RLEG_JOINT5:LLEG_JOINT2 RLEG_JOINT5:LLEG_JOINT3 RLEG_JOINT5:LLEG_JOINT5 RLEG_JOINT5:RARM_JOINT3 RLEG_JOINT5:RARM_JOINT4 RLEG_JOINT5:RARM_JOINT5 RLEG_JOINT5:RARM_JOINT6 RLEG_JOINT5:LARM_JOINT3 RLEG_JOINT5:LARM_JOINT4 RLEG_JOINT5:LARM_JOINT5 RLEG_JOINT5:LARM_JOINT6 LLEG_JOINT2:RARM_JOINT3 LLEG_JOINT2:RARM_JOINT4 LLEG_JOINT2:RARM_JOINT5 LLEG_JOINT2:RARM_JOINT6 LLEG_JOINT2:LARM_JOINT3 LLEG_JOINT2:LARM_JOINT4 LLEG_JOINT2:LARM_JOINT5 LLEG_JOINT2:LARM_JOINT6 LLEG_JOINT3:RARM_JOINT3 LLEG_JOINT3:RARM_JOINT4 LLEG_JOINT3:RARM_JOINT5 LLEG_JOINT3:RARM_JOINT6 LLEG_JOINT3:LARM_JOINT3 LLEG_JOINT3:LARM_JOINT4 LLEG_JOINT3:LARM_JOINT5 LLEG_JOINT3:LARM_JOINT6 LLEG_JOINT5:RARM_JOINT3 LLEG_JOINT5:RARM_JOINT4 LLEG_JOINT5:RARM_JOINT5 LLEG_JOINT5:RARM_JOINT6 LLEG_JOINT5:LARM_JOINT3 LLEG_JOINT5:LARM_JOINT4 LLEG_JOINT5:LARM_JOINT5 LLEG_JOINT5:LARM_JOINT6 CHEST_JOINT1:RARM_JOINT2 CHEST_JOINT1:RARM_JOINT3 CHEST_JOINT1:RARM_JOINT4 CHEST_JOINT1:RARM_JOINT5 CHEST_JOINT1:RARM_JOINT6 CHEST_JOINT1:LARM_JOINT2 CHEST_JOINT1:LARM_JOINT3 CHEST_JOINT1:LARM_JOINT4 CHEST_JOINT1:LARM_JOINT5 CHEST_JOINT1:LARM_JOINT6 HEAD_JOINT1:RARM_JOINT3 HEAD_JOINT1:RARM_JOINT4 HEAD_JOINT1:RARM_JOINT5 HEAD_JOINT1:RARM_JOINT6 HEAD_JOINT1:LARM_JOINT3 HEAD_JOINT1:LARM_JOINT4 HEAD_JOINT1:LARM_JOINT5 HEAD_JOINT1:LARM_JOINT6 RARM_JOINT0:LARM_JOINT4 RARM_JOINT0:LARM_JOINT5 RARM_JOINT0:LARM_JOINT6 RARM_JOINT2:LARM_JOINT4 RARM_JOINT2:LARM_JOINT5 RARM_JOINT2:LARM_JOINT6 RARM_JOINT2:WAIST RARM_JOINT3:LARM_JOINT3 RARM_JOINT3:LARM_JOINT4 RARM_JOINT3:LARM_JOINT5 RARM_JOINT3:LARM_JOINT6 RARM_JOINT3:WAIST RARM_JOINT4:LARM_JOINT0 RARM_JOINT4:LARM_JOINT2 RARM_JOINT4:LARM_JOINT3 RARM_JOINT4:LARM_JOINT4 RARM_JOINT4:LARM_JOINT5 RARM_JOINT4:LARM_JOINT6 RARM_JOINT4:WAIST RARM_JOINT5:LARM_JOINT0 RARM_JOINT5:LARM_JOINT2 RARM_JOINT5:LARM_JOINT3 RARM_JOINT5:LARM_JOINT4 RARM_JOINT5:LARM_JOINT5 RARM_JOINT5:LARM_JOINT6 RARM_JOINT5:WAIST RARM_JOINT6:LARM_JOINT0 RARM_JOINT6:LARM_JOINT2 RARM_JOINT6:LARM_JOINT3 RARM_JOINT6:LARM_JOINT4 RARM_JOINT6:LARM_JOINT5 RARM_JOINT6:LARM_JOINT6 RARM_JOINT6:WAIST LARM_JOINT2:WAIST LARM_JOINT3:WAIST LARM_JOINT4:WAIST LARM_JOINT5:WAIST LARM_JOINT6:WAIST RLEG_JOINT2:LARM_JOINT7 RLEG_JOINT3:LARM_JOINT7 RLEG_JOINT5:LARM_JOINT7 LLEG_JOINT2:LARM_JOINT7 LLEG_JOINT3:LARM_JOINT7 LLEG_JOINT5:LARM_JOINT7 CHEST_JOINT1:LARM_JOINT7 HEAD_JOINT1:LARM_JOINT7 RARM_JOINT0:LARM_JOINT7 RARM_JOINT2:LARM_JOINT7 RARM_JOINT3:LARM_JOINT7 RARM_JOINT4:LARM_JOINT7 RARM_JOINT5:LARM_JOINT7 RARM_JOINT7:LARM_JOINT7 LARM_JOINT7:WAIST RLEG_JOINT2:RARM_JOINT7 RLEG_JOINT3:RARM_JOINT7 RLEG_JOINT5:RARM_JOINT7 LLEG_JOINT2:RARM_JOINT7 LLEG_JOINT3:RARM_JOINT7 LLEG_JOINT5:RARM_JOINT7 CHEST_JOINT1:RARM_JOINT7 HEAD_JOINT1:RARM_JOINT7 RARM_JOINT7:LARM_JOINT0 RARM_JOINT7:LARM_JOINT2 RARM_JOINT7:LARM_JOINT3 RARM_JOINT7:LARM_JOINT4 RARM_JOINT7:LARM_JOINT5 RARM_JOINT7:WAIST CHEST_JOINT2:RARM_JOINT3 CHEST_JOINT2:RARM_JOINT4 CHEST_JOINT2:RARM_JOINT5 CHEST_JOINT2:RARM_JOINT6 CHEST_JOINT2:RARM_JOINT7 CHEST_JOINT2:LARM_JOINT3 CHEST_JOINT2:LARM_JOINT4 CHEST_JOINT2:LARM_JOINT5 CHEST_JOINT2:LARM_JOINT6 CHEST_JOINT2:LARM_JOINT7 CHEST_JOINT2:LARM_JOINT2 CHEST_JOINT2:RARM_JOINT2"
  --conf-file-option "collision_model: convex hull"
  --conf-file-option "# SequencePlayer optional data (contactStates x 4 + controlSwingTime x 4 (4 is rfsensor, lfsensor, rhsensor, lhsensor)"
  --conf-file-option "seq_optional_data_dim: 8"
  --simulation-joint-properties-option "RARM_JOINT2.angle,-0.191986,LARM_JOINT2.angle,0.191986"
)
gen_minmax_table_for_closed_robots(JAXON_JVRC JAXON_JVRC JAXON_JVRC)
endif()

compile_rbrain_model_for_closed_robots("" JVRC-1 JVRC1
  --conf-dt-option "0.002"
  --simulation-timestep-option "0.002"
  --robothardware-conf-file-option "pdgains.file_name: ${hrpsys_ros_bridge_tutorials_SOURCE_PREFIX}/models/PDgains.sav"
)
##
find_package(euscollada)
if (euscollada_SOURCE_PREFIX)
  set(euscollada_PACKAGE_PATH ${euscollada_SOURCE_PREFIX})
else ()
  set(euscollada_PACKAGE_PATH ${euscollada_PREFIX}/share/euscollada)
endif()
macro (attach_sensor_and_endeffector_to_hrp2jsk_urdf
    _robot_name _urdf_file _out_file _yaml_file)
  set(_model_dir "${PROJECT_SOURCE_DIR}/models/")
  set(_in_urdf_file "${_model_dir}/${_urdf_file}")
  set(_in_yaml_file "${_model_dir}/${_yaml_file}")
  set(_out_urdf_file "${_model_dir}/${_out_file}")
  set(_script_file ${euscollada_PACKAGE_PATH}/scripts/add_sensor_to_collada.py)
  add_custom_command(OUTPUT ${_out_urdf_file}
    COMMAND ${_script_file}
    ${_in_urdf_file} -O ${_out_urdf_file} -C ${_in_yaml_file}
    DEPENDS ${_in_urdf_file} ${_in_yaml_file} ${_script_file})
  add_custom_target(${_out_file}_generate DEPENDS ${_out_urdf_file})
  list(APPEND compile_urdf_robots ${_out_file}_generate)
endmacro()
attach_sensor_and_endeffector_to_hrp2jsk_urdf(JAXON_JVRC JAXON_JVRC.urdf
  JAXON_JVRC_SENSORS.urdf jaxon_jvrc.yaml)
if (DEFINED compile_urdf_robots)
  add_custom_target(all_urdf_model_generate ALL DEPENDS ${compile_urdf_robots})
endif()
##
################################
## Generate default simulation launch files and euslisp interface files
################################
macro (generate_default_launch_eusinterface_files_for_jsk_closed_rbrain_robots ROBOT_DIR ROBOT_NAME)
  set(_arg_list ${ARGV})
  # remove arguments of this macro
  list(REMOVE_AT _arg_list 0 1)
  if(EXISTS ${jvrc_models_MODEL_DIR}/${ROBOT_DIR}/${ROBOT_NAME}main.wrl)
    generate_default_launch_eusinterface_files("${jvrc_models_MODEL_DIR}/${ROBOT_DIR}/${ROBOT_NAME}main.wrl" hrpsys_ros_bridge_jvrc ${ROBOT_NAME} ${_arg_list})
  endif()
endmacro ()

if(NOT ENV{TRAVIS_JOB_ID})
# generate_default_launch_eusinterface_files_for_jsk_closed_rbrain_robots(JAXON_JVRC JAXON_JVRC "--use-robot-hrpsys-config")
endif()
generate_default_launch_eusinterface_files_for_jsk_closed_rbrain_robots(JVRC-1 JVRC-1 "--use-robot-hrpsys-config")

###
#if (DEFINED compile_urdf_robots)
#  add_custom_target(all_urdf_model_generate ALL DEPENDS ${compile_urdf_robots})
#endif()

install(DIRECTORY euslisp launch scripts models test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(CODE
  "execute_process(COMMAND echo \"fix \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/model/* ${CATKIN_DEVEL_PREFIX} -> ${CMAKE_INSTALL_PREFIX}\")
   file(GLOB _conf_files \"\$ENV{DISTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/model/*/*.conf\")
   foreach(_conf_file \${_conf_files})
     execute_process(COMMAND sed -i s@${CATKIN_DEVEL_PREFIX}@${CMAKE_INSTALL_PREFIX}@g \${_conf_file})
     execute_process(COMMAND sed -i s@${PROJECT_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}@g \${_conf_file})
  endforeach()
")

set(JVRC_RTC_DIRECTORY ${hrpsys_choreonoid_SOURCE_PREFIX})
set(JVRC_CONF_DIRECTORY ${PROJECT_SOURCE_DIR}/config)

###
#JAXON_RED conid
###
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_FLAT.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_FLAT.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_VALVE.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_VALVE.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_DRCBOX.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_DRCBOX.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_DOOR.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_DOOR.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_RH_FLAT.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_RH_FLAT.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_LOAD_OBJ.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_LOAD_OBJ.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_LOAD_OBJ_NW.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_LOAD_OBJ_NW.cnoid @ONLY)

configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_TUTORIALS.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_TUTORIALS.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_HILLS.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_HILLS.cnoid @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_RED_STEPS.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_RED_STEPS.cnoid @ONLY)

###
#JAXON_BLUE conid
###
if (${jsk_models_FOUND})
configure_file(${PROJECT_SOURCE_DIR}/config/JAXON_BLUE_RH_FLAT.cnoid.in ${PROJECT_SOURCE_DIR}/config/JAXON_BLUE_RH_FLAT.cnoid @ONLY)
endif()

###
#CHIDORI conid
###
if (${jsk_models_FOUND})
configure_file(${PROJECT_SOURCE_DIR}/config/CHIDORI_RH_FLAT.cnoid.in ${PROJECT_SOURCE_DIR}/config/CHIDORI_RH_FLAT.cnoid @ONLY)
endif()

###
# scene.yaml
###
configure_file(${PROJECT_SOURCE_DIR}/config/flat.yaml.in ${PROJECT_SOURCE_DIR}/config/flat.yaml @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/soccer.yaml.in ${PROJECT_SOURCE_DIR}/config/soccer.yaml @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/config/footsal.yaml.in ${PROJECT_SOURCE_DIR}/config/footsal.yaml @ONLY)
