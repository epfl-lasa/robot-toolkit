#!/usr/bin/python

# Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
# Author: Klas Kronander
# email:   klas.kronander@epfl.ch
# website: lasa.epfl.ch
#
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details


import rospy
import rospkg
import shutil
import os
import sys
import subprocess

# define some paths 
rospack = rospkg.RosPack()
rtk_pkg_tools_path=rospack.get_path('rtk_pkg_tools')
rtk_pkg_tools_path += '/'
rtk_path = rtk_pkg_tools_path[:-14]
rtk_addons_path = rtk_path+'packages/addons/'

if(len(sys.argv) < 2):
    raise Exception('No package name provided')
else:
    pkg_name = sys.argv[1]
    print 'creating rtk package with name '+pkg_name
# check if package already exists
existing_pkgs = os.listdir(rtk_addons_path)
if pkg_name in existing_pkgs:
    print 'package already exists, aborting'
    exit(1)
# create catkin_pkg
os.chdir(rtk_addons_path)
subprocess.call(['catkin_create_pkg',pkg_name])
pkg_path = rtk_addons_path+pkg_name+'/'
# add config and data folders
os.chdir(pkg_path)
os.mkdir('config')
os.mkdir('data')
# copy scripts for creating modules
subprocess.call(['cp',rtk_pkg_tools_path+'scripts/CreateRobotModule.py','.'])
# add the rtk specific stuff to the end of the CMakeLists.txt
f = open(rtk_pkg_tools_path+'rtk_CMakeLists_tail.cmake','r')
tailLines = f.readlines()
f.close()
f = open(pkg_path+'CMakeLists.txt','a+')
f.writelines(tailLines)
f.close()




    





