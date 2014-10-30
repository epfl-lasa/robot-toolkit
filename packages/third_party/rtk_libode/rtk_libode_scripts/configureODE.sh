#!/bin/bash
#first make sure we are in the right place
rtk_libode_path=$(rospack find rtk_libode)
cd $rtk_libode_path
cd ode-0.13
pwd
./configure --enable-double-precision --enable-shared --prefix $rtk_libode_path 
