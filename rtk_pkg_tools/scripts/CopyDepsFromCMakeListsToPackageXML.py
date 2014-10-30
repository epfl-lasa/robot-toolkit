#!/usr/bin/python

import sys
import rospkg
from xml.dom import minidom
from collections import deque


def writePackageXMLDeps(xmlpackage,pkgDeps,type='build_depend'):
    for dep in pkgDeps:
        newElement = xmlpackage.createElement(type)
        newText = xmlpackage.createTextNode(dep)
        newElement.appendChild(newText)
        xmlpackage.childNodes[0].appendChild(newElement)
        # find element to put before
        for i,node in enumerate(xmlpackage.childNodes[0].childNodes):
            if node.nodeName == type:
                break
            elif node.nodeName == 'export':
                break
        newline = xmlpackage.createTextNode('\n')
        xmlpackage.childNodes[0].insertBefore(newElement,node)
        xmlpackage.childNodes[0].insertBefore(newline,newElement.nextSibling)
    return xmlpackage



def getPackageXMLDeps(xmlpackage,type='build_depend'):
    #    xmlpackage_deps = dict(build=xmlpackage.getElementsByTagName('build_depend'))
    buildDependNames = []
    buildDependNodes = xmlpackage.getElementsByTagName(type)
    for buildDependNode in buildDependNodes:
        #print buildDependNode.childNodes[0].data
        buildDependNames.append(buildDependNode.childNodes[0].data)
    return buildDependNames

NOT_VALID_PACKAGE_NAMES=['find_package','catkin','REQUIRED','COMPONENTS','']

def checkValidPackageName(st):
    if st in NOT_VALID_PACKAGE_NAMES:
        return False
    else:
        return True

def splitString(st):
    ls = st.split()
    #print ls
    newls=[]
    for el in ls:
        newls.extend(el.split('('))
    newls2=[]
    for el in newls:
        newls2.extend(el.split(')'))
    #print newls2
    return newls2
        

def getCMakeListsDeps(pkgPath):
    fCMakeLists = open(pkgPath+'/CMakeLists.txt','r+')
    lines = fCMakeLists.readlines()
    searchPhrase = 'find_package'
    for startNb,line in enumerate(lines):
        if searchPhrase in line and line[0]!='#':
            break
    for endNb,line in enumerate(lines[startNb:]):
        if ')' in line:
            break
    endNb += startNb
    myLines = lines[startNb:endNb+1]
    #    print myLines
    dependNames = []
    for line in myLines:
        depList = [st for st in splitString(line) if checkValidPackageName(st)]
        dependNames.extend(depList)
    return dependNames

    

if(len(sys.argv) < 2):
    print ('No package name provided, aborting')
    exit(0)
else:
    pkg_name = sys.argv[1]
    print 'Checking and correcting dependency discrepancies between package.xml and CMakeLists.txt for package '+pkg_name

rospack = rospkg.RosPack()
try:
    pkgPath = rospack.get_path(pkg_name)
except:
    print 'the package was not found. Did you give a valid package name? aborting'
    exit(0)


fpackage = open(pkgPath+'/package.xml','r')
xmlpackage = minidom.parse(fpackage)
fpackage.close()

packageDepends = getPackageXMLDeps(xmlpackage,'build_depend')
CMakeListsDepends  = getCMakeListsDeps(pkgPath)
packageDependsToAdd = []
for depend in CMakeListsDepends:
    if not (depend in packageDepends):
        packageDependsToAdd.append(depend)
# modify the xml
bEdited = False
if(len(packageDependsToAdd)>0):
    xmlpackage = writePackageXMLDeps(xmlpackage,packageDependsToAdd,'build_depend')
    bEdited = True
# make run_depend copy of build_depend
buildPackageDepends = getPackageXMLDeps(xmlpackage,'build_depend')
runPackageDepends = getPackageXMLDeps(xmlpackage,'run_depend')

packageDependsToAdd = []
for depend in buildPackageDepends:
    if not (depend in runPackageDepends):
        packageDependsToAdd.append(depend)
if(len(packageDependsToAdd)>0):
    xmlpackage = writePackageXMLDeps(xmlpackage,packageDependsToAdd,'run_depend')
    bEdited=True
#write the output
if bEdited:
    f = open(pkgPath+'/package.xml','w')
    f.write(xmlpackage.toxml())
    f.close()
        

