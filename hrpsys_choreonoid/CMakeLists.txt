cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_choreonoid)

cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0057 NEW)

find_package(catkin REQUIRED COMPONENTS roscpp tf)

# activate C++ 11 for choreonoid-1.7
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

find_package(choreonoid)
if (${choreonoid_FOUND})
  ### for chorenoid with catkin
  set(cnoid-plugin_FOUND ${choreonoid_FOUND} )
  set(cnoid-plugin_INCLUDE_DIRS ${CHOREONOID_INCLUDE_DIRS} )
  set(cnoid-plugin_LIBRARY_DIRS ${CHOREONOID_LIBRARY_DIRS})
  set(cnoid-plugin_LIBRARIES ${CHOREONOID_LIBRARIES})
else ()
  pkg_check_modules(cnoid-plugin choreonoid-body-plugin)
endif()

find_package(Boost REQUIRED system filesystem)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
##catkin_python_setup()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

include_directories(${hrpsys_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${hrpsys_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
if(${cnoid-plugin_FOUND})
  include_directories(${cnoid-plugin_INCLUDE_DIRS})
  link_directories(${cnoid-plugin_LIBRARY_DIRS})
  ##
  add_library(PDcontroller src/PDcontroller.cpp)
  target_link_libraries(PDcontroller ${openrtm_aist_LIBRARIES} ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES})
  set_target_properties(PDcontroller PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
  ##
  add_library(JAXONCustomizer src/JAXONCustomizer.cpp)
  target_link_libraries(JAXONCustomizer ${cnoid-plugin_LIBRARIES} ${YAML_CPP_LIBRARIES})
  set_target_properties(JAXONCustomizer PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
  ##
  add_library(DOORCustomizer src/DOORCustomizer.cpp)
  target_link_libraries(DOORCustomizer ${cnoid-plugin_LIBRARIES})
  set_target_properties(DOORCustomizer PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
  ##
  add_executable(JointStateROSBridge src/JointStateROSBridge.cpp src/JointStateROSBridgeComp.cpp)
  target_link_libraries(JointStateROSBridge ${openrtm_aist_LIBRARIES} ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES})
  ##
  add_executable(TransformROSBridge src/TransformROSBridge.cpp src/TransformROSBridgeComp.cpp)
  target_link_libraries(TransformROSBridge ${openrtm_aist_LIBRARIES} ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES})
  ##
  add_executable(ImuROSBridge src/ImuROSBridge.cpp src/ImuROSBridgeComp.cpp)
  target_link_libraries(ImuROSBridge ${openrtm_aist_LIBRARIES} ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES})
endif()

## Build only choreonoid iob
pkg_check_modules(openhrp3 REQUIRED openhrp3.1)
pkg_check_modules(omniorb omniORB4 REQUIRED)
pkg_check_modules(omnidynamic omniDynamic4 REQUIRED)
### hotfix for https://github.com/fkanehiro/hrpsys-base/pull/803
list(GET hrpsys_INCLUDE_DIRS 0 hrpsys_first_incdir)
include_directories(BEFORE ${hrpsys_first_incdir}/hrpsys)
include_directories(BEFORE ${hrpsys_first_incdir}/hrpsys/idl)
include_directories(BEFORE ${hrpsys_first_incdir}/hrpsys/io)
include_directories(${openhrp3_INCLUDE_DIRS})
link_directories(${openhrp3_LIBRARY_DIRS})
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
add_subdirectory(iob)

add_custom_target(hrpsys_choreonoid_iob ALL DEPENDS RobotHardware_choreonoid)
