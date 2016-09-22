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

BULLET_RELEASE = '2.83.6'
BULLET_RELEASE_ZIP = BULLET_RELEASE+'.zip'


def download_bullet(rtk_libode_path):
    os.chdir(rtk_libbullet_path)
    print 'downloading bullet'
    subprocess.call(['wget','https://github.com/bulletphysics/bullet3/archive/'+BULLET_RELEASE_ZIP])
    subprocess.call(['unzip',BULLET_RELEASE_ZIP])
    print 'finished downloading and unpacking bullet'

def clean_bullet(rtk_libbullet_path):
    print 'cleaning bullet'
    subprocess.call(['rm','-rf',os.path.join(rtk_libbullet_path,'build')])
    subprocess.call(['rm','-rf',os.path.join(rtk_libbullet_path,'bullet3-'+BULLET_RELEASE)])
    subprocess.call(['rm','-r',os.path.join(rtk_libbullet_path,BULLET_RELEASE_ZIP)])
    # remove "installed" libs and include files, but keep the folder structure and .gitignore files
    for root, dirs, files in os.walk(os.path.join(rtk_libbullet_path,'local_install')):
        # remove files
        for name in files:
            if name != '.gitignore':
                os.remove(os.path.join(root,name))
        # remove empty dirs
        for name in dirs:
            path = os.path.join(root,name)
            if len(os.listdir(path))==0:
                os.rmdir(path)
    print 'finished cleaning'

def build_bullet(rtk_libbullet_path):
    print 'building bullet'
    rtk_libbullet_build_path = os.path.join(rtk_libbullet_path,'build')
    if not os.path.isdir(rtk_libbullet_build_path):
        os.mkdir(rtk_libbullet_build_path)
    os.chdir(rtk_libbullet_build_path)
    rtk_libbullet_install_path = os.path.join(rtk_libbullet_path,'local_install')
    # calling cmake for the bullet project with a bunch of options for a minimal and suitable build for rtk
    subprocess.call(['cmake','-DUSE_DOUBLE_PRECISION=OFF','-DBUILD_SHARED_LIBS=ON','-DBUILD_BULLET2_DEMOS=OFF','-DBUILD_BULLET3=ON','-DBUILD_CPU_DEMOS=OFF','-DBUILD_OPENGL3_DEMOS=OFF','-DBUILD_EXTRAS=ON','-DINSTALL_EXTRA_LIBS=ON','-DCMAKE_INSTALL_PREFIX='+rtk_libbullet_install_path,'../bullet3-'+BULLET_RELEASE])
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
        print "Installing rtk_libbullet."
        if glob.glob(BULLET_RELEASE_ZIP):
            print 'libbullet already downloaded'
        else:
            download_bullet(rtk_libbullet_path)

        if glob.glob('local_install/lib/*.so'):
            print 'libbullet already built'
        else:
            build_bullet(rtk_libbullet_path)
