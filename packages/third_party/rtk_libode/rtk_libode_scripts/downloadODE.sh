#!/bin/bash
#first make sure we are in the right place
rtk_libode_path=$(rospack find rtk_libode)
cd $rtk_libode_path
# download libode source
wget http://sourceforge.net/projects/opende/files/ODE/0.13/ode-0.13.tar.bz2
#unpack
tar -xvf ode-0.13.tar.bz2
