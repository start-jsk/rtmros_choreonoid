cmake_minimum_required(VERSION 2.8.3)
project(jvrc_models)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(CFG_EXTRAS jvrc_models-extras.cmake)

if (EXISTS ${PROJECT_SOURCE_DIR}/model)
  execute_process(COMMAND git pull
  ## temporary fix version
#  execute_process(COMMAND git checkout 540ab2709d7d491a88b1a716aac085ae91e0889e
#  execute_process(COMMAND git checkout 978fea941ce938a29a1765dbb170ede41e3533aa
#  execute_process(COMMAND git checkout 69a151665f6400fda1e69f2361adfb7cb2d30c7e
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/model)
else ()
  execute_process(COMMAND git clone https://github.com/jvrc/model.git
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()
