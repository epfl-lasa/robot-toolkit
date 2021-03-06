cmake_minimum_required(VERSION 2.8.3)
project(rtk_examples)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mathlib
  robotlib
  std_tools
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES RTK_examples
#  CATKIN_DEPENDS mathlib robotlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(ExampleConsoleModule/include ExampleForceSensorModule/include ExampleGravityCompModule/include ExampleInverseKinematics/include ExampleOpenLoopPIDControlModule/include ExamplePositionControl/include ExampleTracker/include ExampleVelocityControl/include ExampleWorldControlModule/include WorldNetworkClient/include WorldNetworkServer/include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
FILE(GLOB theSourceFiles ExampleConsoleModule/src/*.cpp)
add_library(ExampleConsoleModule ${theSourceFiles})

FILE(GLOB theSourceFiles ExampleForceSensorModule/src/*.cpp)
add_library(ExampleForceSensorModule ${theSourceFiles})

FILE(GLOB theSourceFiles ExampleGravityCompModule/src/*.cpp)
add_library(ExampleGravityCompModule ${theSourceFiles})

FILE(GLOB theSourceFiles ExampleInverseKinematics/src/*.cpp)
add_library(ExampleInverseKinematics ${theSourceFiles})

FILE(GLOB theSourceFiles ExampleOpenLoopPIDControlModule/src/*.cpp)
add_library(ExampleOpenLoopPIDControlModule ${theSourceFiles})

FILE(GLOB theSourceFiles ExamplePositionControl/src/*.cpp)
add_library(ExamplePositionControl ${theSourceFiles})

FILE(GLOB theSourceFiles ExampleTracker/src/*.cpp)
add_library(ExampleTracker ${theSourceFiles})

FILE(GLOB theSourceFiles ExampleVelocityControl/src/*.cpp)
add_library(ExampleVelocityControl ${theSourceFiles})

FILE(GLOB theSourceFiles ExampleWorldControlModule/src/*.cpp)
add_library(ExampleWorldControlModule ${theSourceFiles})

FILE(GLOB theSourceFiles WorldNetworkClient/src/*.cpp)
add_library(WorldNetworkClient ${theSourceFiles})

FILE(GLOB theSourceFiles WorldNetworkServer/src/*.cpp)
add_library(WorldNetworkServer ${theSourceFiles})

## Declare a cpp executable
# add_executable(RTK_examples_node src/RTK_examples_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(RTK_examples_node RTK_examples_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(RTK_examples_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS RTK_examples RTK_examples_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_RTK_examples.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
###################################
## RobotToolKit specific stuff  ###
###################################
# to comply with the file structure of RobotToolKit, you need to create symlinks 
# to any compiled modules in the RobotToolKit/module folder. This is automated below.
# just add the modules of this package as set(rtk_modules myAwesomeModule1 myAwesomeModule2)
# modules can be world or robot modules. Note that you need to specify your targets using 
# standard cmake/catkin procedures above. This is step is only for linking the build modules.
set(rtk_modules ExampleWorldControlModule ExampleGravityCompModule ExamplePositionControl ExampleConsoleModule ExampleForceSensorModule ExampleInverseKinematics ExampleOpenLoopPIDControlModule ExampleTracker ExampleVelocityControl)
execute_process(COMMAND rospack find rtk_pkg_tools OUTPUT_VARIABLE rtk_pkg_tools_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
include(${rtk_pkg_tools_DIR}/rtk_create_symlinks.cmake)

# Uncomment the following line if you want catkin dependencies you specify in CMakeLists.txt to be automatically copied to the package.xml
execute_process(COMMAND ${rtk_pkg_tools_DIR}/scripts/CopyDepsFromCMakeListsToPackageXML.py ${PROJECT_NAME})

