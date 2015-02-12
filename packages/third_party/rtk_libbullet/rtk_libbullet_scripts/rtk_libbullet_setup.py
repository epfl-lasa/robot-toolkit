#!/usr/bin/python

# Copyright (C) 2015 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
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



import os
import subprocess
import glob
import sys


def download_bullet(rtk_libode_path):
    os.chdir(rtk_libbullet_path)
    print 'downloading bullet'
    subprocess.call(['wget','https://github.com/bulletphysics/bullet3/archive/master.zip'])
    subprocess.call(['unzip','master.zip'])
    print 'finished downloading and unpacking bullet'
    
def clean_bullet(rtk_libbullet_path):
    print 'cleaning bullet'
    subprocess.call(['rm','-rf',os.path.join(rtk_libbullet_path,'build')])
    subprocess.call(['rm','-rf',os.path.join(rtk_libbullet_path,'bullet3-master')])
    subprocess.call(['rm','-r',os.path.join(rtk_libbullet_path,'master.zip')])
    print 'finished cleaning'

def build_bullet(rtk_libbullet_path):
    print 'building bullet'
    rtk_libbullet_build_path = os.path.join(rtk_libbullet_path,'build')
    os.mkdir(rtk_libbullet_build_path)
    os.chdir(rtk_libbullet_build_path)
    # calling cmake for the bullet project with a bunch of options for a minimal and suitable build for rtk
    subprocess.call(['cmake','-DUSE_DOUBLE_PRECISION=ON','-DBUILD_SHARED_LIBS=ON','-DBUILD_BULLET2_DEMOS=OFF','-DBUILD_BULLET3=OFF','-DBUILD_CPU_DEMOS=OFF','-DBUILD_OPENGL3_DEMOS=OFF','-DBUILD_EXTRAS=OFF','-DCMAKE_INSTALL_PREFIX='+rtk_libbullet_build_path,'../bullet3-master'])
    # compile with speed!
    subprocess.call(['make','-j8'])
    # dont worry it will only 'install' locally
    subprocess.call(['make','install'])
    print 'finished building'


if __name__ == '__main__':
    rtk_libbullet_path = subprocess.check_output(['rospack','find','rtk_libbullet'])
    # remove the \n at the end
    rtk_libbullet_path = rtk_libbullet_path.rstrip()
    os.chdir(rtk_libbullet_path)
    # check if arguments tell us what to do
    if(len(sys.argv)>1):
        if(sys.argv[1]=='--clean'):
            clean_bullet(rtk_libbullet_path)
        elif(sys.argv[1]=='--build'):
            build_bullet(rtk_libbullet_path)
        elif(sys.argv[1]=='--download'):
            download_bullet(rtk_libbullet_path)
        else:
            # if there were no arguments, try to do the right thing
            if glob.glob('master.zip'):
                print 'libbullet already downloaded'
            else:
                download_bullet(rtk_libbullet_path)

            if glob.glob('build/lib/*.so'):
                print 'libbullet already built'
            else:
                build_bullet(rtk_libbullet_path)

            





