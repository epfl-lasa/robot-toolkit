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

# Check for RTK package tools.
if(rtk_pkg_tools_DIR STREQUAL "")
  message(FATAL_ERROR
"Error: Could not find RTK package tools.
Did you set the path?")
endif()

include(${rtk_pkg_tools_DIR}/rtk_create_symlinks.cmake)

# Uncomment the following line if you want catkin dependencies you specify in CMakeLists.txt to be automatically copied to the package.xml
execute_process(COMMAND ${rtk_pkg_tools_DIR}/scripts/CopyDepsFromCMakeListsToPackageXML.py ${PROJECT_NAME})
