cmake_minimum_required(VERSION 2.8.3)
project(rtk_libbullet)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#execute_process(COMMAND "python" "${PROJECT_SOURCE_DIR}/rtk_libode_scripts/rtk_libode_setup.py")

catkin_package(
INCLUDE_DIRS build/include build/include/bullet
#LIBRARIES rtk_libode ode
LIBRARIES rtk_libbullet
#LIBRARIES ${PROJECT_SOURCE_DIR}/lib/libode.so
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#message("hello catkin devel prefix ${CATKIN_DEVEL_PREFIX}")
# add fake target
# add postbuild command for linking to CATKIN_DEVEL_PREFIX
add_custom_target(rtk_libbullet_setup_dummy_target ALL)
add_custom_command(TARGET rtk_libbullet_setup_dummy_target  PRE_BUILD COMMAND "python" "${PROJECT_SOURCE_DIR}/rtk_libbullet_scripts/rtk_libbullet_setup.py")

set(BulletLibs BulletDynamics BulletCollision BulletSoftBody LinearMath)
foreach(lib ${BulletLibs})
add_library(${lib} SHARED IMPORTED)
set_property(TARGET ${lib} PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/build/lib/lib${lib}.so)
#message(${PROJECT_SOURCE_DIR}/build/lib/lib${lib}.so)
set_target_properties(${lib} PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(${lib} rtk_libbullet_setup_dummy_target)
endforeach(lib)

# add_library(BulletDynamics SHARED IMPORTED)
# set_property(TARGET BulletDynamics PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/build/lib/libBulletDynamics.so)
# add_library(BulletCollision SHARED IMPORTED)
# set_property(TARGET BulletCollision PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/build/lib/libBulletCollision.so)
# add_library(BulletSoftBody SHARED IMPORTED)
# set_property(TARGET BulletSoftBody PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/build/lib/libBulletSoftBody.so)
# add_library(LinearMath SHARED IMPORTED)
# set_property(TARGET BulletDynamics PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/build/lib/libLinearMath.so)

#set_target_properties(ode PROPERTIES LINKER_LANGUAGE CXX)
add_library(rtk_libbullet SHARED)
set_target_properties(rtk_libbullet PROPERTIES LINKER_LANGUAGE CXX)
add_dependencies(rtk_libbullet rtk_libbullet_setup_dummy_target)
target_link_libraries(rtk_libbullet ${BulletLibs})
#add_dependencies(rtk_libode ode rtk_libode_setup_dummy_target)
#   src/${PROJECT_NAME}/rtk_libode.cpp
# )

## Declare a cpp executable
# add_executable(rtk_libode_node src/rtk_libode_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rtk_libode_node rtk_libode_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(rtk_libode_node
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
# install(TARGETS rtk_libode 
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rtk_libode.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
