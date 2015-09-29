cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_choreonoid)

find_package(catkin REQUIRED COMPONENTS roscpp)

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
  ##
#  add_custom_command(OUTPUT /usr/lib/choreonoid-1.5/rtc/PDcontroller.so
#    COMMAND sudo -n mkdir -p /usr/lib/choreonoid-1.5/rtc
#    COMMAND sudo -n cp ${PROJECT_SOURCE_DIR}/PDcontroller.so /usr/lib/choreonoid-1.5/rtc/
#    DEPENDS PDcontroller)
  ##
#  add_custom_target(copy_to_rtc ALL DEPENDS /usr/lib/choreonoid-1.5/rtc/PDcontroller.so)
###
  include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${cnoid-plugin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${cnoid-plugin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${hrpsys_LIBRARY_DIRS})
  add_library(JAXONCustomizer src/JAXONCustomizer.cpp)
  target_link_libraries(JAXONCustomizer ${cnoid-plugin_LIBRARIES})
  set_target_properties(JAXONCustomizer PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
  ##
#  add_custom_command(OUTPUT /usr/lib/choreonoid-1.5/customizer/JAXONCustomizer.so
#    COMMAND sudo -n mkdir -p /usr/lib/choreonoid-1.5/customizer
#    COMMAND sudo -n cp ${PROJECT_SOURCE_DIR}/JAXONCustomizer.so /usr/lib/choreonoid-1.5/customizer/
#    DEPENDS JAXONCustomizer)
  ##
#  add_custom_target(copy_to_customizer ALL DEPENDS /usr/lib/choreonoid-1.5/customizer/JAXONCustomizer.so)
endif()
