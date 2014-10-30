import os
import subprocess
import glob
import sys



rtk_libode_path = subprocess.check_output(['rospack','find','rtk_libode'])
# remove the \n at the end
rtk_libode_path = rtk_libode_path.rstrip()
os.chdir(rtk_libode_path)

if(len(sys.argv)>1):
    if(sys.argv[1]=='--clean'):
        print 'cleaning ODE'
        subprocess.call([rtk_libode_path+'/rtk_libode_scripts/cleanODE.sh'])
        print 'finished cleaning'
        exit(1)

# download and unpack if necessary
if glob.glob('*.tar.bz2'):
    print 'libode source already downloaded'
else: 
    print 'no tarball found, downloading..'
    subprocess.call([rtk_libode_path+'/rtk_libode_scripts/downloadODE.sh'])
    print 'Finished downloading'
# configure and compile if necessary
if glob.glob('lib/libode.so'):
    print 'libode source already compiled'
else:
    print 'libode not compiled. configuring..'
    subprocess.call([rtk_libode_path+'/rtk_libode_scripts/configureODE.sh'])
 #   subprocess.Popen([rtk_libode_path+'/rtk_libode_scripts/configureODE.sh'],stdout=open(os.devnull, 'wb'))
    print 'Finished configuring'
    print 'compiling..'
    subprocess.call([rtk_libode_path+'/rtk_libode_scripts/compileODE.sh'])
    







