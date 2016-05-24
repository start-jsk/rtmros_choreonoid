add_definitions(-DHRPSYS_PACKAGE_VERSION=\"\\"${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}\\"\")

# hrpIo
add_library(hrpIo_choreonoid SHARED iob.cpp)
#add_dependencies(hrpIo_choreonoid hrpsys_choreonoid_msgs_gencpp)
include_directories(${ROBOTHARDWARE_SOURCE})

if (NOT APPLE AND NOT QNXNTO)
   target_link_libraries(hrpIo_choreonoid rt)
endif()

target_link_libraries(hrpIo_choreonoid roscpp rostime)

# RobotHardware
#set(comp_source  ${ROBOTHARDWARE_SOURCE}/robot.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardware.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardwareService_impl.cpp)
message ("RobotHardware source directory: ${ROBOTHARDWARE_SOURCE}")
set(comp_source  ${ROBOTHARDWARE_SOURCE}/robot.cpp RobotHardware_choreonoid.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardware.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardwareService_impl.cpp)
set(libs hrpIo_choreonoid hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub)

add_library(RobotHardware_choreonoid SHARED ${comp_source})

target_link_libraries(RobotHardware_choreonoid ${libs})
set_target_properties(RobotHardware_choreonoid PROPERTIES PREFIX "")
set_target_properties(RobotHardware_choreonoid PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(RobotHardware_choreonoid PROPERTIES OUTPUT_NAME RobotHardware_choreonoid)

# add_executable(RobotHardwareComp_choreonoid ${ROBOTHARDWARE_SOURCE}/RobotHardwareComp.cpp ${comp_source})
# target_link_libraries(RobotHardwareComp_choreonoid ${libs} ${omniorb_LIBRARIES} ${omnidynamic_LIBRARIES} RTC coil dl pthread)
# set_target_properties(RobotHardwareComp_choreonoid PROPERTIES OUTPUT_NAME RobotHardwareComp)

# add_library(hrpEC_choreonoid SHARED ${HRPEC_SOURCE}/hrpEC.cpp ${HRPEC_SOURCE}/hrpEC-common.cpp)
# target_link_libraries(hrpEC_choreonoid hrpIo_choreonoid hrpsysBaseStub)
# set_target_properties(hrpEC_choreonoid PROPERTIES PREFIX "")
# set_target_properties(hrpEC_choreonoid PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set_target_properties(hrpEC_choreonoid PROPERTIES OUTPUT_NAME hrpEC)
# set_target_properties(hrpEC_choreonoid PROPERTIES COMPILE_DEFINITIONS "OPENRTM_VERSION110")

install(TARGETS hrpIo_choreonoid
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo
)
##install(TARGETS RobotHardwareComp_choreonoid RobotHardware_choreonoid hrpEC_choreonoid
install(TARGETS RobotHardware_choreonoid
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} CONFIGURATIONS Release Debug RelWithDebInfo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib CONFIGURATIONS Release Debug RelWithDebInfo
)
