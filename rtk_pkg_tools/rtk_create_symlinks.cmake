set(ROBOTTOOLKIT_PATH ${rtk_pkg_tools_SOURCE_DIR}/..)
foreach(rtk_module ${rtk_modules})
  get_property(MODULE_LIB_DIR TARGET ${rtk_module} PROPERTY LIBRARY_OUTPUT_DIRECTORY)
  add_custom_command(TARGET ${rtk_module} POST_BUILD COMMAND ln -sf ${MODULE_LIB_DIR}/lib${rtk_module}.so ${ROBOTTOOLKIT_PATH}/module/${rtk_module}.so )
endforeach(rtk_module)

message("${PROJECT_NAME}")
# creating symlinks to the data and config folders
add_custom_target(${PROJECT_NAME}_symlink_target ALL)
add_custom_command(TARGET ${PROJECT_NAME}_symlink_target PRE_BUILD COMMAND ${rtk_pkg_tools_SOURCE_DIR}/scripts/SetupPackage4.sh ${PROJECT_NAME})
