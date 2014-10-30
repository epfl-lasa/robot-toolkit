#!/bin/bash
#first make sure we are in the right place
rtk_libode_path=$(rospack find rtk_libode)
cd $rtk_libode_path
# delete compiled and unpacked stuff
rm -r bin lib ode-0.13 *.tar.bz2*
