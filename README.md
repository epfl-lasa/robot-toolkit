[![Build Status](https://travis-ci.org/epfl-lasa/robot-toolkit.svg?branch=master)](https://travis-ci.org/epfl-lasa/robot-toolkit)

The new version of RobotToolKit has a largely unchanged structure from rosbuild
version. Differences include stuff such as out-of-source build that comes with
catkin. As before, modules need to be linked to the RobotToolKit/module folder
for the executables to find them. Executables should be run from RobotToolKit
folder e.g.

    ./bin/robot_simulator --config packages/myPackage/myConfig

Installation
------------
    cd /path/to/catkin/workspace
    cd src
    git clone git@github.com:epfl-lasa/robot-toolkit.git
    source devel/setup.bash  # See note below
    rosdep install --from-paths src --ignore-src
    catkin_make

### Dependency note: 
robot toolkit has an intricate circular dependency: to compile
RTK you must first source RTK's setup.bash. yikes. The easiest way to do this is
to call catkin_make with an empty src directory (this creates devel/setup.bash)
and then source the setup.bash file (this sets the path correctly) *before*
attempting to compile the robot_toolkit packages.

There is also a dependency on "ncurses" that you can get by running:
`sudo apt-get install libncurses5-dev`

### Minimal compilation

To compile just the barebones robot toolkit (without a simulator or ODE or
Bullet), you can pass in blacklist package to catkin_make:

    catkin_make -DCATKIN_BLACKLIST_PACKAGES="robot_simulator;robot_gui;simulator_dynamics;rtk_libbullet;rtk_libode"

Alternatively extract the `minimal.tgz` file; this will place CATKIN_IGNORE files in the correct packages.


Creating a new RTK package:
---------------------------

There is a new script for creating a rtk package:

    rosrun rtk_pkg_tools CreatePackage.py PACKAGE_NAME

This creates a catkin package with minor addition to the CMakeLists.txt file
which makes it easy to automatically link modules post build. Now roscd to your
new package

    roscd PACKAGE_NAME
    ls


Creating a new RTK module inside your new package:
-------------------------------------------------

you will find a `CreateRobotModule.py` script, which if you run it will create a
robot module inside your newly created package.

    ./CreateRobotModule.py MODULE_NAME

This will create a folder PACKAGE_NAME/MODULE_NAME with boilerplate cpp code for
defining a robot module. Note that modules are just libraries that are part of a
catkin package.


Editing CMakeLists.txt
----------------------

Next, you need to specify what your package should compile by modifying the
CMakeLists.txt. Robot modules should be specified as

    add_libarary(MODULE_NAME PATH_TO_SOURCE_FILES),

where `PATH_TO_SOURCE_FILES` is simply `MODULE_NAME/src` if you use the file
structure given by the CreateRobotModule script. As usual with CMake, you should
also specify include_directories and link to libraries etc. Specifically, to get
catkin includes and link your module with libraries that catkin provides:

    include_directories(${catkin_INCLUDE_DIRS})
    target_link_library(MODULE_NAME ${catkin_LIBRARIES})

At the end of the CMakeLists there is a small RobotToolKit specific part which
will create symlinks for your modules to the correct place. You just need to
list the modules that should be linked and the rest is automatic.

    set(rtk_modules MODULE_NAME)

You also have the option to activate a script that will automatically
synchronize your package.xml with stuff you add as

    find_package(catkin REQUIRED COMPONENTS myDep1 myDep2 ...)

in the CMakeLists. This is useful when developing in cpp and avoids having
manually enter dependencies in several places. To activate this function, just
uncomment the last line according to the instructions in CMakeLists.txt


Config and data
---------------

If you used `rtk_package_tools/scripts/CreatePackage.py` to create your package,
you will also find empty config and data folders in your new package. These will
be linked to to the correct place. If you are convering a package from rosbuild,
keep in mind that to comply with the catkin best practices your package shoudl
follow this_naming_convention which means that you may need to change the
package names at the top of the config file.


A note on history
------------------

In September 2015, during the Great Migration to Github, Felix cleaned up the
history of the RobotToolKit repository to remove binary files (and other
garbage) that found their way into the repository.

The repository cleaner used was BFG
(https://rtyley.github.io/bfg-repo-cleaner/), which preserves history while
removing unecessary files.

If for some reason you need to explore the old repository, it is accessible (as
an archive) here:

    lasasrv2.epfl.ch:/disk1/archive/RobotToolKit.2015.09.15.tgz
