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

#include "SimulatorDynamics.h"
#include "RobotLib/Link.h"
#include "RobotLib/Joint.h"

BaseDynamicObject::BaseDynamicObject(BaseSimulatorDynamics *world){
    mDynWorld = world;
    mObject = NULL;
}

BaseDynamicObject::~BaseDynamicObject(){
    Free();
}
void    BaseDynamicObject::Free(){}

void    BaseDynamicObject::LinkToObject(WorldObject *object){
    mObject = object;
}

void BaseDynamicObject::Prepare(){
}

void BaseDynamicObject::Update(REALTYPE dt){
}
void BaseDynamicObject::Reset(){
}
BaseDynamicObject*  BaseDynamicObject::GetDynamicObject(WorldObject *object){
    return NULL;
}




BaseSimulatorDynamics::BaseSimulatorDynamics(){
    mWorld              = NULL;
}

BaseSimulatorDynamics::~BaseSimulatorDynamics(){
    Free();
}
void    BaseSimulatorDynamics::Free(){
    for(int i=0;i<int(mDynObjects.size());i++)
        delete mDynObjects[i];
    mDynObjects.clear();
}


BaseDynamicObject*  BaseSimulatorDynamics::CreateDynamicObject(){
    BaseDynamicObject* obj = new BaseDynamicObject(this);
    return obj;
}
BaseDynamicObject*  BaseSimulatorDynamics::CreateDynamicRobot(){
    BaseDynamicObject* obj = new BaseDynamicObject(this);
    return obj;
}
void                BaseSimulatorDynamics::CreateFixedLink(BaseDynamicObject* objA, BaseDynamicObject* objB){
    //cout <<objA<<" "<<objB<<endl;
}

BaseDynamicObject*  BaseSimulatorDynamics::GetDynamicObject(WorldObject *object){
    for(int i=0;i<int(mDynObjects.size());i++){
        BaseDynamicObject* obj;
        //cout <<"BSD:" <<object<<endl;
        if((obj = mDynObjects[i]->GetDynamicObject(object)))
            return obj;
    }
    return NULL;
}

void    BaseSimulatorDynamics::LinkToWorld(World *world){
    bAllowRobotReset    = true;
    mWorld = world;
    if(world!=NULL){
        for(int i=0;i<int(mWorld->mObjects.size());i++){
            BaseDynamicObject* obj;
            if(mWorld->mObjects[i]->IsRobot()){
                obj = CreateDynamicRobot();
            }else{
                obj = CreateDynamicObject();
            }
            obj->LinkToObject(mWorld->mObjects[i]);
            mDynObjects.push_back(obj);
            //cout<<mWorld->mObjects[i]->GetName()<<endl;
        }
        for(int i=0;i<int(mWorld->mObjectLinks.size());i++){
            //cout <<"LINK: "<<mWorld->mObjectLinks[i]->GetObjectA()<<" "<<mWorld->mObjectLinks[i]->GetObjectA()<<endl;
            mWorld->mObjectLinks[i]->Resolve(mWorld);
            CreateFixedLink(GetDynamicObject(mWorld->mObjectLinks[i]->GetObjectA()),
                            GetDynamicObject(mWorld->mObjectLinks[i]->GetObjectB()));
        }
    }
}

void BaseSimulatorDynamics::Update(REALTYPE dt, REALTYPE maxStepTime){

    int nbSteps = int(floor(dt/maxStepTime));
    REALTYPE cDt = maxStepTime;

    for (int i=0;i<=nbSteps;++i){
        if(i==nbSteps){
            if( (cDt = dt - (REALTYPE(nbSteps)*maxStepTime)) < dt*0.1){
                break;
            }
        }
        for(int i=0;i<int(mDynObjects.size());i++)
            mDynObjects[i]->Prepare();
        UpdateCore(cDt);
        for(int i=0;i<int(mDynObjects.size());i++)
            mDynObjects[i]->Update(dt);
    }


}
void    BaseSimulatorDynamics::UpdateCore(REALTYPE dt){
}

void    BaseSimulatorDynamics::Reset(){
    for(int i=0;i<int(mDynObjects.size());i++)
        mDynObjects[i]->Reset();
}
void BaseSimulatorDynamics::AllowRobotReset(bool allow){
    bAllowRobotReset = allow;
}
bool BaseSimulatorDynamics::bAllowRobotReset        = true;

