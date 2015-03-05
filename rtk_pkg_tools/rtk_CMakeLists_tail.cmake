###################################
## RobotToolKit specific stuff  ###
###################################
# to comply with the file structure of RobotToolKit, you need to create symlinks 
# to any compiled modules in the RobotToolKit/module folder. This is automated below.
# just add the modules of this package as set(rtk_modules myAwesomeModule1 myAwesomeModule2)
# modules can be world or robot modules. Note that you need to specify your targets using 
# standard cmake/catkin procedures above. This is step is only for linking the build modules.
set(rtk_modules )
execute_process(COMMAND rospack find rtk_pkg_tools OUTPUT_VARIABLE rtk_pkg_tools_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
include(${rtk_pkg_tools_DIR}/rtk_create_symlinks.cmake)

# Uncomment the following line if you want catkin dependencies you specify in CMakeLists.txt to be automatically copied to the package.xml
execute_process(COMMAND ${rtk_pkg_tools_DIR}/scripts/CopyDepsFromCMakeListsToPackageXML.py ${PROJECT_NAME})

