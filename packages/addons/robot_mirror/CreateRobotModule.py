#!/usr/bin/python
import rospy
import rospkg
import shutil
import os
import sys
import subprocess

def replaceTextInFile(fname,oldString,newString):
    f = open(fname,'r')
    text_data = f.read()
    f.close()
    text_data = text_data.replace(oldString,newString)
    f = open(fname,'w')
    f.write(text_data)
    f.close()



if(len(sys.argv) < 2):
    raise Exception('No module name provided')
else:
    module_name = sys.argv[1]
    print 'creating robot module with name '+module_name

rospack = rospkg.RosPack()
rtk_pkg_tools_path=rospack.get_path('rtk_pkg_tools')
# copy from template
res = subprocess.call(['mkdir',module_name])
if(res==1):
    print '\nA module with the specified name already exists in this folder.\nRemove it first if you want to create a new module with the same name.\n'
    exit(1)

subprocess.call(['cp','-r',rtk_pkg_tools_path+'/robot_module_template/include',module_name])
subprocess.call(['cp','-r',rtk_pkg_tools_path+'/robot_module_template/src',module_name])
# rename copied files
module_header = module_name+'/include/'+module_name+'.h'
module_src = module_name+'/src/'+module_name+'.cpp'
subprocess.call(['mv',module_name+'/include/EmptyRobotModule.h',module_header])
subprocess.call(['mv',module_name+'/src/EmptyRobotModule.cpp',module_src])
# replace module name in header and source
replaceTextInFile(module_header,'EmptyRobotModule',module_name)
replaceTextInFile(module_src,'EmptyRobotModule',module_name)

    





