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

#include "Actuator.h"

BASEOBJECT_TOPDEF(Actuator);
Actuator::Actuator(){Zero();}
Actuator::~Actuator(){}

void      Actuator::SetName(string name){mName = name;};
string    Actuator::GetName(){return mName;};

void      Actuator::Zero(){}
void      Actuator::Set(const Actuator& actuator){}
int       Actuator::GetDOFCount(){return 0;}




BASEOBJECT_DEF(Generic1DOFJointActuator);
Generic1DOFJointActuator::Generic1DOFJointActuator():Actuator(){}
Generic1DOFJointActuator::~Generic1DOFJointActuator(){}

void      Generic1DOFJointActuator::Zero(){
  Set(R_ZERO,R_ZERO,R_ZERO,R_ZERO);
}

void      Generic1DOFJointActuator::Set(const Actuator& actuator){
  pGeneric1DOFJointActuator tmpActuator = Generic1DOFJointActuator::Cast(&actuator);
  if(tmpActuator!=NULL){
    Set(tmpActuator->mPosition,tmpActuator->mVelocity,tmpActuator->mAcceleration,tmpActuator->mTorque);
  }
}

void      Generic1DOFJointActuator::Set(REALTYPE position, REALTYPE velocity, REALTYPE acceleration, REALTYPE torque){
  mPosition     = position;
  mVelocity     = velocity;
  mAcceleration = acceleration;
  mTorque       = torque;
}

int       Generic1DOFJointActuator::GetDOFCount(){return 1;}



Generic1DOFJointActuatorGroup::Generic1DOFJointActuatorGroup(){
    mActuatorList.clear();
    mActuatorCount = 0;
}
Generic1DOFJointActuatorGroup::~Generic1DOFJointActuatorGroup(){
    mActuatorList.clear();
    mActuatorCount = 0;
}

void        Generic1DOFJointActuatorGroup::SetActuatorsList(const ActuatorsList& list){
    mActuatorList.clear();
    mActuatorCount = 0;
    for(unsigned int i=0;i<list.size();i++){
        Generic1DOFJointActuator *actuator = Generic1DOFJointActuator::Cast(list[i]);
        if(actuator){
            mActuatorList.push_back(actuator);
            mActuatorCount ++;
        }
    }
    mPosition.Resize(mActuatorCount,false);
    mVelocity.Resize(mActuatorCount,false);
    mAcceleration.Resize(mActuatorCount,false);
    mTorque.Resize(mActuatorCount,false);

    mPosition.Zero();
    mVelocity.Zero();
    mAcceleration.Zero();
    mTorque.Zero();
}
void        Generic1DOFJointActuatorGroup::WriteActuators(){
    for(int i=0;i<mActuatorCount;i++){
        mActuatorList[i]->mPosition       = mPosition.RefNoCheck(i);
        mActuatorList[i]->mVelocity       = mVelocity.RefNoCheck(i);
        mActuatorList[i]->mAcceleration   = mAcceleration.RefNoCheck(i);
        mActuatorList[i]->mTorque         = mTorque.RefNoCheck(i);
    }
}
void        Generic1DOFJointActuatorGroup::ReadActuators(){
    for(int i=0;i<mActuatorCount;i++){
        mPosition.RefNoCheck(i)     = mActuatorList[i]->mPosition;
        mVelocity.RefNoCheck(i)     = mActuatorList[i]->mVelocity;
        mAcceleration.RefNoCheck(i) = mActuatorList[i]->mAcceleration;
        mTorque.RefNoCheck(i)       = mActuatorList[i]->mTorque;
    }
}
int         Generic1DOFJointActuatorGroup::GetCount(){
    return mActuatorCount;
}
void        Generic1DOFJointActuatorGroup::SetJointAngles(const Vector& vector){
    mPosition       = vector;
}
void        Generic1DOFJointActuatorGroup::SetJointPositions(const Vector& vector){
    mPosition       = vector;
}
void        Generic1DOFJointActuatorGroup::SetJointVelocities(const Vector& vector){
    mVelocity       = vector;
}
void        Generic1DOFJointActuatorGroup::SetJointAccelerations(const Vector& vector){
    mAcceleration       = vector;
}
void        Generic1DOFJointActuatorGroup::SetJointTorques(const Vector& vector){
    mTorque       = vector;
}
Vector&     Generic1DOFJointActuatorGroup::GetJointAngles(){
    return mPosition;
}
Vector&     Generic1DOFJointActuatorGroup::GetJointPositions(){
    return mPosition;
}
Vector&     Generic1DOFJointActuatorGroup::GetJointVelocities(){
    return mVelocity;
}
Vector&     Generic1DOFJointActuatorGroup::GetJointAccelerations(){
    return mAcceleration;
}
Vector&     Generic1DOFJointActuatorGroup::GetJointTorques(){
    return mTorque;
}


void     Generic1DOFJointActuatorGroup::GetJointAngles(Vector& result){
  result = mPosition;
}
void     Generic1DOFJointActuatorGroup::GetJointPositions(Vector& result){
  result = mPosition;
}
void     Generic1DOFJointActuatorGroup::GetJointVelocities(Vector& result){
  result =  mVelocity;
}
void     Generic1DOFJointActuatorGroup::GetJointAccelerations(Vector& result){
  result =  mAcceleration;
}
void     Generic1DOFJointActuatorGroup::GetJointTorques(Vector& result){
  result =  mTorque;
}
