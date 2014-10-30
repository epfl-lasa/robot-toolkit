#!/bin/bash

# Version 2 of file handles packages and metapackages
# Nicolas, 28/02/2014
if [[ $# -lt 1 ]] || [[ $# -gt 1 ]]
then
    echo "*Usage: `basename $0` PackageName"
    exit
fi
echo "*********************************************************"
## find the paths to the package and robottoolkit
BASEPATH=. # BASEPATH not used
PKG_NAME=$1
RTK_PATHFOLDER=$(rospack find rtk_pkg_tools)
RTK_PATH=$RTK_PATHFOLDER/..
## if we really cant find it, then give up
if [ -z $RTK_PATH ]; then 
    echo "*Error, could not find RobotToolKit Folder path!"
    exit
else
 echo "*->Found RTK_package"
fi

##find the path to the package that should be set up
PKG_PATHFOLDER=$(rospack find $PKG_NAME)
#echo "PKG_PATHFOLDER: " $RTK_PATHFOLDER

#echo "PKG_PATH: "$RTK_PATH
##make sure that the package was found 
if [  -z $PKG_PATHFOLDER ]; then
    echo "*.....Error: Looks like there no package named: $PKG_NAME"
    echo "*.....Looking for a Metapackage instead ..."

	PKG_PATHFOLDER=$(rosstack find $PKG_NAME)
	PKG_PATH=$PKG_PATHFOLDER/..

	if [  -z $PKG_PATHFOLDER ]; then
    		echo "**Error**: Looks like there is no metapackage named: $PKG_NAME"
    		exit
	else

		PKG_PATH=$PKG_PATHFOLDER/.. #files are in the folder above
 		echo "*->Found metapackage: $PKG_PATH"
	fi
else
	PKG_PATH=$PKG_PATHFOLDER # Not sure wether should be added or not ...
 	echo "*->Found package: $PKG_PATH"

fi


echo "***************************************************"
echo "*SetupPackage: Setting up package for RobotToolKit: $PKG_NAME"

# check if there is a data folder in the package
PKG_DATA_PATH=$PKG_PATH/data
# if there is a data folder, we must link to it from the RTK main data folder
if [ -d $PKG_DATA_PATH ]; then
    echo "*Linking data folder.."
    ln -sfn $PKG_DATA_PATH $RTK_PATH/data/packages/$PKG_NAME
fi
#check if there is a config folder
PKG_CONFIG_PATH=$PKG_PATH/config
# if there is then we must link to it from RTK main config folder
if [ -d $PKG_CONFIG_PATH ]; then
    echo "*Linking config folder.."
    ln -sfn $PKG_CONFIG_PATH $RTK_PATH/config/packages/$PKG_NAME
fi




echo "*SetupPackage: The package $PKG_NAME was successfully setup"
echo "*********************************************************"

exit

