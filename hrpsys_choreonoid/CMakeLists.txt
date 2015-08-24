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

# if(${cnoid-plugin_FOUND})
#   include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${cnoid-plugin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
#   link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${cnoid-plugin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${hrpsys_LIBRARY_DIRS})
#   add_library(VisionReaderRTC src/VisionReaderRTC.cpp)
#   target_link_libraries(VisionReaderRTC ${openrtm_aist_LIBRARIES} ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES} hrpsysBaseStub)
#   set_target_properties(VisionReaderRTC PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

#   add_library(IOBSimpleController src/IOBSimpleController.cpp)
#   target_link_libraries(IOBSimpleController ${cnoid-plugin_LIBRARIES} ${Boost_LIBRARIES} ${hrpsys_LIBRARIES} ${catkin_LIBRARIES} hrpsysBaseStub)
#   set_target_properties(IOBSimpleController PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

#   add_custom_command(OUTPUT /usr/lib/choreonoid-1.5/simplecontroller/IOBSimpleController.so
#     COMMAND sudo cp ${PROJECT_SOURCE_DIR}/IOBSimpleController.so /usr/lib/choreonoid-1.5/simplecontroller/
#     DEPENDS IOBSimpleController)
#   add_custom_command(OUTPUT /usr/lib/choreonoid-1.5/rtc/VisionReaderRTC.so
#     COMMAND sudo cp ${PROJECT_SOURCE_DIR}/VisionReaderRTC.so /usr/lib/choreonoid-1.5/rtc/
#     DEPENDS VisionReaderRTC)

#   add_custom_target(copy_to_lib ALL DEPENDS /usr/lib/choreonoid-1.5/simplecontroller/IOBSimpleController.so /usr/lib/choreonoid-1.5/rtc/VisionReaderRTC.so)

# endif()

if(${cnoid-plugin_FOUND})
  include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${cnoid-plugin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS} ${openrtm_aist_LIBRARY_DIRS} ${cnoid-plugin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${hrpsys_LIBRARY_DIRS})
  add_library(JAXONCustomizer src/JAXONCustomizer.cpp)
  target_link_libraries(JAXONCustomizer ${cnoid-plugin_LIBRARIES})
  set_target_properties(JAXONCustomizer PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
  ##
  add_custom_command(OUTPUT /usr/lib/choreonoid-1.5/customizer/JAXONCustomizer.so
    COMMAND sudo cp ${PROJECT_SOURCE_DIR}/JAXONCustomizer.so /usr/lib/choreonoid-1.5/customizer/
    DEPENDS JAXONCustomizer)
  ##
  add_custom_target(copy_to_lib ALL DEPENDS /usr/lib/choreonoid-1.5/customizer/JAXONCustomizer.so)
endif()
