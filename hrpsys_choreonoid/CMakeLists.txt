cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_choreonoid)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

include(FindPkgConfig)
pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
set(OPENHRP_SAMPLE_DIR ${openhrp3_PREFIX}/share/OpenHRP-3.1/sample)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)

pkg_check_modules(cnoid-plugin choreonoid-body-plugin)
find_package(Boost REQUIRED system filesystem)
##catkin_python_setup()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

if(${cnoid-plugin_FOUND})
  include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${cnoid-plugin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${cnoid-plugin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${hrpsys_LIBRARY_DIRS})
  add_library(PDcontroller src/PDcontroller.cpp)
  target_link_libraries(PDcontroller ${openrtm_aist_LIBRARIES} ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES})
  set_target_properties(PDcontroller PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
###
  include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${cnoid-plugin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${cnoid-plugin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${hrpsys_LIBRARY_DIRS})
  add_library(JAXONCustomizer src/JAXONCustomizer.cpp)
  target_link_libraries(JAXONCustomizer ${cnoid-plugin_LIBRARIES})
  set_target_properties(JAXONCustomizer PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
##
  include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${cnoid-plugin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${cnoid-plugin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${hrpsys_LIBRARY_DIRS})
  add_library(DOORCustomizer src/DOORCustomizer.cpp)
  target_link_libraries(DOORCustomizer ${cnoid-plugin_LIBRARIES})
  set_target_properties(DOORCustomizer PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
##
  include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${cnoid-plugin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${cnoid-plugin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${hrpsys_LIBRARY_DIRS})
  add_executable(JointStateROSBridge src/JointStateROSBridge.cpp src/JointStateROSBridgeComp.cpp)
  target_link_libraries(JointStateROSBridge ${openrtm_aist_LIBRARIES} ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES})

  add_executable(GroundTruthROSBridge src/GroundTruthROSBridge.cpp src/GroundTruthROSBridgeComp.cpp)
  target_link_libraries(GroundTruthROSBridge ${openrtm_aist_LIBRARIES} ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES})
  
endif()

## Build only choreonoid iob
find_package(PkgConfig)
pkg_check_modules(omniorb omniORB4 REQUIRED)
pkg_check_modules(omnidynamic omniDynamic4 REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
### hotfix for https://github.com/fkanehiro/hrpsys-base/pull/803
list(GET hrpsys_INCLUDE_DIRS 0 hrpsys_first_incdir)
list(APPEND hrpsys_INCLUDE_DIRS ${hrpsys_first_incdir}/hrpsys)
list(APPEND hrpsys_INCLUDE_DIRS ${hrpsys_first_incdir}/hrpsys/idl)
list(APPEND hrpsys_INCLUDE_DIRS ${hrpsys_first_incdir}/hrpsys/io)
if(EXISTS ${hrpsys_SOURCE_DIR})
  set(ROBOTHARDWARE_SOURCE ${hrpsys_SOURCE_DIR}/src/rtc/RobotHardware)
  set(HRPEC_SOURCE         ${hrpsys_SOURCE_DIR}/src/ec/hrpEC)
elseif(EXISTS ${hrpsys_SOURCE_PREFIX})
  set(ROBOTHARDWARE_SOURCE ${hrpsys_SOURCE_PREFIX}/src/rtc/RobotHardware)
  set(HRPEC_SOURCE         ${hrpsys_SOURCE_PREFIX}/src/ec/hrpEC)
else()
  set(ROBOTHARDWARE_SOURCE ${hrpsys_PREFIX}/share/hrpsys/src/rtc/RobotHardware)
  set(HRPEC_SOURCE         ${hrpsys_PREFIX}/share/hrpsys/src/ec/hrpEC)
endif()
include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
link_directories(${CATKIN_DEVEL_PREFIX}/lib ${hrpsys_PREFIX}/lib ${openhrp3_LIBRARY_DIRS} /opt/ros/$ENV{ROS_DISTRO}/lib/)
add_subdirectory(iob)

add_custom_target(hrpsys_choreonoid_iob ALL DEPENDS RobotHardware_choreonoid)
