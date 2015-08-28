cmake_minimum_required(VERSION 2.8.3)
project(robot_simulator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mathlib
  robot_gui
  robotlib
  simulator_dynamics
  std_tools
  gl_tools
  rtk_pkg_tools
  rtk_libode
  rtk_libbullet
  roslib
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(PM_QTMOC_FILES MainWindow ConsoleWidget WorldRendererWidget)
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_INCLUDES})
IF(DEFINED PM_QTMOC_FILES)
  SET(MOC_SOURCES "")
  FOREACH(MOC_NAME ${PM_QTMOC_FILES})
    SET(MOC_SOURCES ${MOC_SOURCES} ${PROJECT_SOURCE_DIR}/include/${MOC_NAME}.h)
  ENDFOREACH(MOC_NAME)
  message(STATUS "MOC_SOURCES:  ${MOC_SOURCES}")
  QT4_WRAP_CPP(MOCED_SOURCES
    ${MOC_SOURCES})
ENDIF(DEFINED PM_QTMOC_FILES)
include_directories(${CMAKE_CURRENT_BINARY_DIR}) 
message(STATUS "CMAKE_CURRENT_BINARY_DIR:  ${CMAKE_CURRENT_BINARY_DIR}")

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
#  LIBRARIES robot_simulator
#  CATKIN_DEPENDS mathlib robot_gui robotlib simulator_dynamics std_tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} 
)


FILE(GLOB theSourceFiles src/*.cpp)
 add_executable(robot_simulator ${theSourceFiles})


 target_link_libraries(robot_simulator
    ${catkin_LIBRARIES} ${QT_LIBRARIES} freeimage
  )


# link the executable to the traditional location
set(ROBOTTOOLKIT_PATH ${rtk_pkg_tools_SOURCE_DIR}/..)
get_property(exec_output_dir TARGET robot_simulator PROPERTY RUNTIME_OUTPUT_DIRECTORY)
add_custom_command(TARGET robot_simulator POST_BUILD COMMAND ln -sf ${exec_output_dir}/robot_simulator ${ROBOTTOOLKIT_PATH}/bin/robot_simulator )


