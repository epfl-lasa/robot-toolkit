
#rosbuild_find_ros_package(${PROJECT_NAME})


#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCHECK_N_WRAP_MALLOCS")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--wrap=malloc,--wrap=free")
#set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--wrap=malloc,--wrap=free")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--wrap=malloc,--wrap=free")




#rosbuild_find_ros_stack(RobotToolKit)
# doing this in another way since rosbuild_find_ros_stack is only 
# available from ros electric onwards
#just to get the path to some package that we know excctly where it is..
#rosbuild_find_ros_package(RobotLib)
# remove strip the path such that it give the path to RobotToolKit
string(REPLACE "/rtk_pkg_tools" "" RobotToolKit_STACK_PATH ${rtk_pkg_tools_SOURCE_DIR})

# QT MOC, these will be included to everything that is built by this package
IF(DEFINED QTMOC_FILES)
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
SET(QT_USE_QTOPENGL true)
INCLUDE(${QT_USE_FILE})
INCLUDE_DIRECTORIES(${QT_INCLUDES})

  SET(MOC_SOURCES "")
  FOREACH(MOC_NAME ${QTMOC_FILES})
    SET(MOC_SOURCES ${MOC_SOURCES}  ${${PROJECT_NAME}_PACKAGE_PATH}/${MOC_NAME} )
  ENDFOREACH(MOC_NAME)
  message(STATUS "MOC_SOURCES:  ${MOC_SOURCES}")
  QT4_WRAP_CPP(MOCED_SOURCES ${MOC_SOURCES})
  message(STATUS "MOCED_SOURCES: ${MOCED_SOURCES}")

include_directories(${CMAKE_CURRENT_BINARY_DIR}) 
message(STATUS "CMAKE_CURRENT_BINARY_DIR:  ${CMAKE_CURRENT_BINARY_DIR}")
ENDIF(DEFINED QTMOC_FILES)

# QT MOC, these will be included to everything that is built by this package
IF(DEFINED QTFORMS_FILES)
	SET(MOC_SOURCES "")
	FOREACH(UI_NAME ${QTFORMS_FILES})
		SET(QTFORMS_SOURCE ${QTFORMS_SOURCE}  ${${PROJECT_NAME}_PACKAGE_PATH}/${UI_NAME} )
	ENDFOREACH(UI_NAME)
	message(STATUS "FORM SOURCES:  ${QTFORMS_SOURCE}")
	QT4_WRAP_UI(BUILT_QT_FORMS ${QTFORMS_SOURCE})
	message(STATUS "BUILT FORM SOURCES:  ${BUILT_QT_FORMS}")
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})	
ENDIF(DEFINED QTFORMS_FILES)

# build the libraries, this is a bit ugly but it works..
# since shared libraries cannot depend on each other in a cyclic manner
# they have to be defined in the correct order if they depend on each other.
if(PackageLibraries)
  list(REVERSE PackageLibraries)
  set(BuiltLibraries)
  foreach(PackageLibrary ${PackageLibraries})
    # find the source and header files
    FILE(GLOB theSourceFiles ${PackageLibrary}/src/*)
    FILE(GLOB theHeaderFiles ${PackageLibrary}/include/*.h)
    # add the include dirs
    include_directories(${PackageLibrary}/include)
    # build the library
    add_library(${PackageLibrary} ${theSourceFiles} ${theHeaderFiles})
    list(APPEND BuiltLibraries ${PackageLibrary})
    set(tempPLS ${PackageLibraries})
    list(REMOVE_ITEM tempPLS ${BuiltLibraries})
    if(tempPLS)
      target_link_libraries(${PackageLibrary} ${tempPLS})
    endif(tempPLS)
  endforeach(PackageLibrary)
endif(PackageLibraries)

 message(STATUS "found these Robot Modules: ${RobotModules}")
# build the robot modules
foreach(RobotModule ${RobotModules})
  FILE(GLOB theSourceFiles ${RobotModule}/src/*.cpp ${RobotModule}/src/*.c)
  
  FILE(GLOB theHeaderFiles ${RobotModule}/include/*.h)
  include_directories(${RobotModule}/include)
  add_library(${RobotModule} MODULE ${theSourceFiles} ${theHeaderFiles} ${MOCED_SOURCES} ${BUILT_QT_FORMS})
  target_link_libraries(${RobotModule} ${QT_LIBRARIES})
  # link to libraries that live inside this package
  if(PackageLibraries)
    target_link_libraries(${RobotModule} ${PackageLibraries})
  endif(PackageLibraries)
  target_link_libraries(${RobotModule} ${catkin_LIBRARIES})
endforeach(RobotModule)

message(STATUS "found these World Modules: ${WorldModules}")
#build the world modules
foreach(WorldModule ${WorldModules})
  FILE(GLOB theSourceFiles ${WorldModule}/src/*.cpp)
  FILE(GLOB theHeaderFiles ${WorldModule}/include/*.h)
  include_directories(${WorldModule}/include)
  add_library(${WorldModule} MODULE ${theSourceFiles} ${theHeaderFiles} ${MOCED_SOURCES} ${BUILT_QT_FORMS})
  # link to libraries that live inside this package
  if(PackageLibraries)
    target_link_libraries(${WorldModule} ${PackageLibraries})
  endif(PackageLibraries)
  target_link_libraries(${WorldModule} ${catkin_LIBRARIES})
endforeach(WorldModule)

#create symlinks for the robotmodules
foreach(RobotModule ${RobotModules})
  ADD_CUSTOM_COMMAND(TARGET ${RobotModule} POST_BUILD COMMAND ln -sf ${CATKIN_DEVEL_PREFIX}/lib/lib${RobotModule}.so ${RobotToolKit_STACK_PATH}/module/${RobotModule}.so) 
endforeach(RobotModule)

#create symlinks for the worldmodules
foreach(WorldModule ${WorldModules})
  ADD_CUSTOM_COMMAND(TARGET ${WorldModule} POST_BUILD COMMAND ln -sf ${CATKIN_DEVEL_PREFIX}/lib/lib${WorldModule}.so ${RobotToolKit_STACK_PATH}/module/${WorldModule}.so) 
endforeach(WorldModule)





#create symlinks for data and config..
find_package(catkin REQUIRED)
# first create a target that will always be built, that output s nothing but executes a command..
#add_custom_target(${PROJECT_NAME}_dataAndConfig COMMAND ${rtk_pkg_tools_SOURCE_DIR}/SetupPackage.sh ${PROJECT_NAME})
# make sure this target is built by rosbuild

# or just do this.....
execute_process(COMMAND ${rtk_pkg_tools_SOURCE_DIR}/SetupPackage.sh ${PROJECT_NAME} )

message("
execute_process(COMMAND ${rtk_pkg_tools_SOURCE_DIR}/SetupPackage.sh ${PROJECT_NAME} )
")
