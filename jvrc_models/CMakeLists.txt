cmake_minimum_required(VERSION 2.8.3)
project(jvrc_models)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(CFG_EXTRAS jvrc_models-extras.cmake)

if (EXISTS ${PROJECT_SOURCE_DIR}/model)
  execute_process(COMMAND git pull
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/model)
else ()
  execute_process(COMMAND git clone https://github.com/jvrc/model.git
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()
