#!/bin/bash
# this is a little executable script which can be used to correctly launch robot_simulator from a ros launch file

# find path to rtk_pkg_tools
rtk_path=$(rospack find rtk_pkg_tools)
# strip the path name to get rtk root
rtk_path=${rtk_path%/*}
cd $rtk_path
exec bin/robot_simulator $*

# cd $1
# exec ${@:2}

