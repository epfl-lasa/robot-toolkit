/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
 * website: lasa.epfl.ch
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "Robot.h"

#include "StdTools/XmlTree.h"
#include "StdTools/Various.h"

Robot::Robot(bool simMode){
    Free();
    bSimMode = simMode;
    bROSInitialized = false;
}
Robot::~Robot(){
    Free();
}

void Robot::Free(){
    for(int i=0;i<int(mInterfaces.size());i++){
        mInterfaces[i]->Free();
        //delete mInterfaces[i];
    }
    mInterfaces.clear();

    mType               = "";
    mSubType            = "";
    mName               = "";
    mControlMode        = CTRLMODE_DEFAULT;
    mLinks.clear();
    mParents.clear();
    mLinks.clear();
    mJoints.clear();
    mLinksTree.Clear();

    mActuators.clear();
    mSensors.clear();
    mWorldInstance = NULL;
    mWorld = NULL;
}

void Robot::Init(){
    mForwardKinematics.SetRobot(this);
    for(int i=0;i<int(mInterfaces.size());i++){
        mInterfaces[i]->SetRobot(this);
        mInterfaces[i]->SetWorld(mWorld);
        mInterfaces[i]->Init();
    }
}
void Robot::Start(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->Start();
}
void Robot::Stop(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->Stop();
}
void  Robot::Update(){
    for(int i=0;i<int(mInterfaces.size());i++)
        mInterfaces[i]->Update();
}

void  Robot::UpdateCore(){
    UpdateLinks();
    for(int i=0;i<int(mInterfaces.size());i++){
        mInterfaces[i]->UpdateCore();
    }
}
void  Robot::Draw(){
    for(int i=0;i<int(mInterfaces.size());i++){
        mInterfaces[i]->Draw();
    }
}


void  Robot::UpdateLinks(bool initialState){
    mLinksTree.Update();
    if(initialState)
        mLinksTree.SetInitialState();
    mForwardKinematics.Update();
}
/*
void  Robot::UpdateLinks(){
    mLinksTree.Update();
    mForwardKinematics.Update();
}*/

string Robot::GetType(){
    return mType;
}
string Robot::GetSubType(){
    return mSubType;
}

string Robot::GetName(){
    return mName;
}

string Robot::GetPatches(){
    return Serialize(mPatches);
}

World* Robot::GetWorld(){
    return mWorld;
}
void   Robot::SetWorld(World* world){
    mWorld = world;
}
void   Robot::SetWorldInstance(WorldObject* wObject){
    mWorldInstance = wObject;
}

WorldObject* Robot::GetWorldInstance(){
    return mWorldInstance;
}

unsigned int Robot::GetLinksCount(){
    return mLinksCount;
}
LinksList& Robot::GetLinks(){
    return mLinks;
}
JointsList& Robot::GetJoints(){
    return mJoints;
}
vector<int>& Robot::GetParents(){
    return mParents;
}
ActuatorsList&    Robot::GetActuators(){
    return mActuators;
}
SensorsList&      Robot::GetSensors(){
    return mSensors;
}
int Robot::GetDOFCount(){
    int res = 0;
    for(int i=0;i<int(mActuators.size());i++){
        res +=mActuators[i]->GetDOFCount();
    }
    return res;
}
int  Robot::GetLinkIndex(const string name){
  for(unsigned int i=0;i<mLinksCount;i++){
    if(name == mLinks[i]->GetName()){
      return i;
    }
  }
  return -1;
}
int  Robot::GetDOFIndex(const string name){
    int cnt=0;
    for(unsigned int i=0;i<mLinksCount;i++){
        if((RevoluteJoint::Cast(mJoints[i]))||(SliderJoint::Cast(mJoints[i]))){
            if(name == mLinks[i]->GetName())
                return cnt;
            cnt++;
        }
  }
  return -1;
}


void  Robot::AddInterface(RobotInterface *robotInterface){
    if(robotInterface!=NULL)
        mInterfaces.push_back(robotInterface);
}


bool Robot::Load(string robotType, string robotSubType, string patchList){
    string rt=robotType;
    if(robotSubType.length()>0){
        rt+="/";
        rt+=robotSubType;
    }
    return Load(rt,patchList);
}

//#include <execinfo.h>

bool Robot::Load(string robotType, string patchList){


    // Parsing the robot type: type, subtype, and patches
    vector<string> typeList = Tokenize(RemoveSpaces(robotType),"/","/");

    mType       = typeList[0];
    if(int(typeList.size())>1)  mSubType = Serialize(typeList,1);
    else                        mSubType = "";

    gLOG.SetCurrentEntry("Robot");
    gLOG.LockCurrentEntry();
    if(mSubType.length()==0)    gLOG.Append("Setting up Robot %s",mType.c_str());
    else                        gLOG.Append("Setting up Robot %s/%s",mType.c_str(),mSubType.c_str());
    gLOG.SetDeltaIndent(2);

    char filename[256];
    if(mSubType.length()==0)    sprintf(filename,"Robots/%s/structure.xml",mType.c_str());
    else                        sprintf(filename,"Robots/%s/%s/structure.xml",mType.c_str(),mSubType.c_str());

    XmlTree tree;
    if(FileFinder::Find(filename)){
    //if(FileExists(filename)){
        gLOG.Append("Loading file %s",FileFinder::GetCStr());
        if(tree.LoadFromFile(FileFinder::GetCStr())==0){
            gLOG.Append("Error: Parsing file failed");
            gLOG.SetDeltaIndent(-2);
            gLOG.UnlockCurrentEntry();
            return false;
        }
    }else{
        gLOG.Append("Error: File %s not found",filename);
        gLOG.SetDeltaIndent(-2);
        gLOG.UnlockCurrentEntry();
        return false;
    }


    mPatches = Tokenize(RemoveSpaces(patchList));
    mPatches.push_back("bbox");
    mPatches.push_back("shape");
    mPatches.push_back("controlMode");

    for(unsigned int i=0;i<mPatches.size();i++){
        bool bPatchFound = false;
        if(!bPatchFound){
            if(mSubType.length()==0){
                sprintf(filename,"Robots/%s/patches/%s.xml",mType.c_str(),mPatches[i].c_str());
                bPatchFound = FileFinder::Find(filename);
            }else{
                sprintf(filename,"Robots/%s/%s/patches/%s.xml",mType.c_str(),mSubType.c_str(),mPatches[i].c_str());
                bPatchFound = FileFinder::Find(filename);
                if(!bPatchFound){
                    sprintf(filename,"Robots/%s/patches/%s.xml",mType.c_str(),mPatches[i].c_str());
                    bPatchFound = FileFinder::Find(filename);
                }
            }
        }
        if(!bPatchFound){
            if(mSubType.length()==0){
                sprintf(filename,"Robots/%s/%s.xml",mType.c_str(),mPatches[i].c_str());
                bPatchFound = FileFinder::Find(filename);
            }else{
                sprintf(filename,"Robots/%s/%s/%s.xml",mType.c_str(),mSubType.c_str(),mPatches[i].c_str());
                bPatchFound = FileFinder::Find(filename);
                if(!bPatchFound){
                    sprintf(filename,"Robots/%s/%s.xml",mType.c_str(),mPatches[i].c_str());
                    bPatchFound = FileFinder::Find(filename);
                }
            }
        }

        if(bPatchFound){
            gLOG.Append("Patching robot with: %s",FileFinder::GetCStr());
            XmlTree patch;
            if(patch.LoadFromFile(FileFinder::GetCStr())==0){
                gLOG.Append("Error: Parsing file failed");
                gLOG.SetDeltaIndent(-2);
                gLOG.UnlockCurrentEntry();
                return false;
            }
            tree.Patch(&patch);
        }else{
            gLOG.Append("Error: Unable to find patch: %s.xml",mPatches[i].c_str());
        }
    }


    if(tree.GetName()=="Link"){
        mLinksTree.Load(&tree);
        mLinksCount =  mLinksTree.GetTreeSize();

        mParents.resize(mLinksCount);
        mLinks.resize(mLinksCount);
        mJoints.resize(mLinksCount);
        mActuators.resize(mLinksCount);
        mSensors.resize(mLinksCount);

        if(mLinksCount>0){
            int   *parents = new int[mLinksCount];
            pLink *links   = new pLink[mLinksCount];
            mLinksTree.CreateIndex(parents, links, mLinksCount);
            for(unsigned int i=0;i<mLinksCount;i++){
                mParents[i]   = parents[i];
                mLinks[i]     = links[i];
                mJoints[i]    = mLinks[i]->mJoint;
                mActuators[i] = mJoints[i]->mActuator;
                mSensors[i]   = mJoints[i]->mSensor;

                if(mControlMode!=CTRLMODE_DEFAULT){
                    mJoints[i]->mControlMode = (Joint::JointControlMode)mControlMode;
                }
            }
            delete [] parents;
            delete [] links;
        }
        //mTimeSensor.SetName("Time");
        //mSensors[mLinksCount+0]   = &mTimeSensor;

        mLinksTree.Update();
        mForwardKinematics.SetRobot(this);
        mForwardKinematics.Update();

        for(unsigned int i=0;i<mLinksCount;i++){
            mLinks[i]->SetInitialState();
        }

    }else{
        gLOG.Append("Error: No <Link> tag found");
        gLOG.SetDeltaIndent(-2);
        gLOG.UnlockCurrentEntry();
        return false;
    }

    gLOG.SetDeltaIndent(-2);
    gLOG.Append("Robot sucessfully loaded");
    gLOG.UnlockCurrentEntry();
    return true;
}



int Robot::GetSensorsCount(){
    return int(mSensors.size());
}
int Robot::GetActuatorsCount(){
    return int(mActuators.size());
}

pSensor Robot::GetSensor(int index){
    if((index>=0)&&(index<int(mSensors.size()))){
        return mSensors[index];
    }
    return NULL;
}
pActuator Robot::GetActuator(int index){
    if((index>=0)&&(index<int(mActuators.size()))){
        return mActuators[index];
    }
    return NULL;
}

pSensor     Robot::FindSensor(const string name){
    for(unsigned int i=0;i<mSensors.size();i++){
        if(mSensors[i]->GetName()==name) return mSensors[i];
    }
    return NULL;
}
pActuator   Robot::FindActuator(const string name){
    for(unsigned int i=0;i<mActuators.size();i++){
        if(mActuators[i]->GetName()==name) return mActuators[i];
    }
    return NULL;
}
/*
TimeSensor&     Robot::GetTimeSensor(){
    return mTimeSensor;
}*/
ReferenceFrame&   Robot::GetReferenceFrame(unsigned int fromLink, unsigned int toLink){
    return mForwardKinematics.GetReferenceFrame(fromLink, toLink);
}
ReferenceFrame&   Robot::GetReferenceFrame(unsigned int link){
    return mForwardKinematics.GetReferenceFrame(link);
}

void Robot::SetArgs(const vector<string>& args){
    mArgs = args;
}
vector<string>& Robot::GetArgs(){
    return mArgs;
}
void            Robot::SetControlMode(Robot::ControlMode mode){
    mControlMode = mode;
    for(unsigned int i=0;i<mLinksCount;i++){
        if(mControlMode!=CTRLMODE_DEFAULT)
            mJoints[i]->mControlMode = (Joint::JointControlMode)mControlMode;
    }

}
Robot::ControlMode     Robot::GetControlMode(){
    return mControlMode;
}
void       Robot::SetUserInfo(void * userData){
    mUserData = userData;
}
void*      Robot::GetUserInfo(){
    return mUserData;
}
void       Robot::SetName(string name){
    mName = name;
}
const vector<RobotInterface*>&  Robot::GetRobotInterfaces(){
    return mInterfaces;
}
bool            Robot::IsSimulationMode(){
    return bSimMode;
}

ros::NodeHandle *Robot::InitializeROS()
{
    return InitializeROS(mType);
}
ros::NodeHandle *Robot::InitializeROS(string NAME)
{
    if(!bROSInitialized){
        int argc=0;
        char * argv[1];
        ros::init(argc,argv,NAME,ros::init_options::NoSigintHandler);
        mNodeHandle = new(ros::NodeHandle);
        bROSInitialized = true;
    }
    return mNodeHandle;
}

