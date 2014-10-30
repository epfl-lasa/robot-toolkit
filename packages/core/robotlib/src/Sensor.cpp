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

#include "Sensor.h"

BASEOBJECT_TOPDEF(Sensor);
Sensor::Sensor(){Zero();}
Sensor::~Sensor(){}

void    Sensor::SetName(string name){mName = name;};
string  Sensor::GetName(){return mName;};

void    Sensor::Zero(){}
void    Sensor::Set(const Sensor& sensor){}
void    Sensor::Set(const Actuator& actuator){}

int     Sensor::StreamSize(){
    int len = mName.length()+1;
    if((len & 0x03)==0)
        return len;
    else
        return len + (4-(len & 0x03));
}
int     Sensor::SetStream(void* memory){
    strcpy((char*)memory,mName.c_str());
    return Sensor::StreamSize();
}
int     Sensor::SetFromStream(const void* memory){
    mName = (char*)memory;
    return Sensor::StreamSize();
}
int     Sensor::SensorsListStreamSize(SensorsList & list){
    int result=0;
    for(size_t i=0;i<list.size();i++)
        result += list[i]->StreamSize();
    return result;
}
int     Sensor::SensorsListSetStream(SensorsList & list,void* memory){
    char* mem = (char*) memory;
    for(size_t i=0;i<list.size();i++)
        mem += list[i]->SetStream(mem);
    return ((char*)mem)-((char*)memory);
}
int     Sensor::SensorsListSetFromStream(SensorsList & list,const void* memory){
    char* mem = (char*) memory;
    for(size_t i=0;i<list.size();i++)
        mem += list[i]->SetFromStream(mem);
    return ((char*)mem)-((char*)memory);
}


BASEOBJECT_DEF(Generic1DOFJointSensor);
Generic1DOFJointSensor::Generic1DOFJointSensor():Sensor(){}
Generic1DOFJointSensor::~Generic1DOFJointSensor(){}

void Generic1DOFJointSensor::Zero(){
  Set(R_ZERO,R_ZERO,R_ZERO,R_ZERO);
}
void Generic1DOFJointSensor::Set(const Sensor& sensor){
  pGeneric1DOFJointSensor tmpSensor = Generic1DOFJointSensor::Cast(&sensor);
  if(tmpSensor!=NULL){
    Set(tmpSensor->mPosition,tmpSensor->mVelocity,tmpSensor->mAcceleration,tmpSensor->mTorque);
  }
}
void Generic1DOFJointSensor::Set(const Actuator& actuator){
  pGeneric1DOFJointActuator tmpActuator = Generic1DOFJointActuator::Cast(&actuator);
  if(tmpActuator!=NULL){
    //cout <<tmpActuator->mPosition<<endl;
    Set(tmpActuator->mPosition,tmpActuator->mVelocity,tmpActuator->mAcceleration,tmpActuator->mTorque);
  }
}
void      Generic1DOFJointSensor::Set(REALTYPE position, REALTYPE velocity, REALTYPE acceleration, REALTYPE torque){
  mPosition     = position;
  mVelocity     = velocity;
  mAcceleration = acceleration;
  mTorque       = torque;
}
void      Generic1DOFJointSensor::SetLimits(REALTYPE limitLow, REALTYPE limitHigh){
    mLimits[0] = MIN(limitLow,limitHigh);
    mLimits[1] = MAX(limitLow,limitHigh);
}
int         Generic1DOFJointSensor::StreamSize(){
    return Sensor::StreamSize() + 4*sizeof(REALTYPE);
}
int         Generic1DOFJointSensor::SetStream(void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetStream(mem);
    REALTYPE *dmem = (REALTYPE*)mem;
    *(dmem++)   = mPosition;
    *(dmem++)   = mVelocity;
    *(dmem++)   = mAcceleration;
    *(dmem++)   = mTorque;
    return ((char*)dmem)-((char*)memory);
}
int         Generic1DOFJointSensor::SetFromStream(const void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetFromStream(mem);
    REALTYPE *dmem = (REALTYPE*)mem;
    mPosition       = *(dmem++);
    mVelocity       = *(dmem++);
    mAcceleration   = *(dmem++);
    mTorque         = *(dmem++);
    return ((char*)dmem)-((char*)memory);
}


Generic1DOFJointSensorGroup::Generic1DOFJointSensorGroup(){
    mSensorList.clear();
    mSensorCount = 0;
}
Generic1DOFJointSensorGroup::~Generic1DOFJointSensorGroup(){
    mSensorList.clear();
    mSensorCount = 0;
}

void        Generic1DOFJointSensorGroup::SetSensorsList(const SensorsList& list){
    mSensorList.clear();
    mSensorCount = 0;
    for(unsigned int i=0;i<list.size();i++){
        Generic1DOFJointSensor *sensor = Generic1DOFJointSensor::Cast(list[i]);
        if(sensor){
            mSensorList.push_back(sensor);
            mSensorCount ++;
        }
    }
    mPosition.Resize(mSensorCount,false);
    mVelocity.Resize(mSensorCount,false);
    mAcceleration.Resize(mSensorCount,false);
    mTorque.Resize(mSensorCount,false);
    mLimits[0].Resize(mSensorCount,false);
    mLimits[1].Resize(mSensorCount,false);

    mPosition.Zero();
    mVelocity.Zero();
    mAcceleration.Zero();
    mTorque.Zero();
    mLimits[0].Zero();
    mLimits[1].Zero();
}
void        Generic1DOFJointSensorGroup::ReadSensors(){
    for(int i=0;i<mSensorCount;i++){
        mPosition.RefNoCheck(i)     = mSensorList[i]->mPosition;
        mVelocity.RefNoCheck(i)     = mSensorList[i]->mVelocity;
        mAcceleration.RefNoCheck(i) = mSensorList[i]->mAcceleration;
        mTorque.RefNoCheck(i)       = mSensorList[i]->mTorque;
        mLimits[0].RefNoCheck(i)    = mSensorList[i]->mLimits[0];
        mLimits[1].RefNoCheck(i)    = mSensorList[i]->mLimits[1];
    }
}
void        Generic1DOFJointSensorGroup::WriteSensors(){
    for(int i=0;i<mSensorCount;i++){
        mSensorList[i]->mPosition       = mPosition.RefNoCheck(i);
        mSensorList[i]->mVelocity       = mVelocity.RefNoCheck(i);
        mSensorList[i]->mAcceleration   = mAcceleration.RefNoCheck(i);
        mSensorList[i]->mTorque         = mTorque.RefNoCheck(i);
    }
}

int         Generic1DOFJointSensorGroup::GetCount(){
    return mSensorCount;
}
Vector&     Generic1DOFJointSensorGroup::GetJointAngles(){
    return mPosition;
}
Vector&     Generic1DOFJointSensorGroup::GetJointPositions(){
    return mPosition;
}
Vector&     Generic1DOFJointSensorGroup::GetJointVelocities(){
    return mVelocity;
}
Vector&     Generic1DOFJointSensorGroup::GetJointAccelerations(){
    return mAcceleration;
}
Vector&     Generic1DOFJointSensorGroup::GetJointTorques(){
    return mTorque;
}
Vector&     Generic1DOFJointSensorGroup::GetJointLimitsHigh(){
    return mLimits[1];
}
Vector&     Generic1DOFJointSensorGroup::GetJointLimitsLow(){
    return mLimits[0];
}
void        Generic1DOFJointSensorGroup::SetJointAngles(const Vector& vector){
    mPosition       = vector;
}
void        Generic1DOFJointSensorGroup::SetJointPositions(const Vector& vector){
    mPosition       = vector;
}
void        Generic1DOFJointSensorGroup::SetJointVelocities(const Vector& vector){
    mVelocity       = vector;
}
void        Generic1DOFJointSensorGroup::SetJointAccelerations(const Vector& vector){
    mAcceleration       = vector;
}
void        Generic1DOFJointSensorGroup::SetJointTorques(const Vector& vector){
    mTorque       = vector;
}

void        Generic1DOFJointSensorGroup::MapFromActuators(const Generic1DOFJointActuatorGroup& actuators){
    if(mSensorCount == actuators.mActuatorCount){
        for(int i=0;i<mSensorCount;i++){
            mSensorList[i]->mPosition       = actuators.mActuatorList[i]->mPosition;
            mSensorList[i]->mVelocity       = actuators.mActuatorList[i]->mVelocity;
            mSensorList[i]->mAcceleration   = actuators.mActuatorList[i]->mAcceleration;
            mSensorList[i]->mTorque         = actuators.mActuatorList[i]->mTorque;
        }
    }
}



/*
BASEOBJECT_CLASS(ForceSensorJoint6DOF,Sensor)

public:
    SpatialForce    mForce;

public:
            ForceSensorJoint6DOF();
    virtual ~ForceSensorJoint6DOF();

    virtual void        Zero();
    virtual void        Set(const Sensor& sensor);
    virtual void        Set(const Actuator& actuator);

            void        Set(const SpatialForce &force);
            void        SetLimits(const SpatialForce &limitLow,const SpatialForce &limitHigh);

    virtual int         StreamSize();
    virtual int         SetStream(void* memory);
    virtual int         SetFromStream(const void* memory);
};
typedef ForceSensorJoint6DOF *pForceSensorJoint6DOF;
*/

BASEOBJECT_DEF(ForceSensorJoint6DOFSensor);
ForceSensorJoint6DOFSensor::ForceSensorJoint6DOFSensor():Sensor(){}
ForceSensorJoint6DOFSensor::~ForceSensorJoint6DOFSensor(){}

void ForceSensorJoint6DOFSensor::Zero(){
    mForce.Zero();
}
void ForceSensorJoint6DOFSensor::Set(const Sensor& sensor){
    pForceSensorJoint6DOFSensor tmpSensor = ForceSensorJoint6DOFSensor::Cast(&sensor);
    if(tmpSensor!=NULL){
        Set(tmpSensor->mForce);
    }
}
void ForceSensorJoint6DOFSensor::Set(const Actuator& actuator){
/*    pForceSensorJoint6DOFActuator tmpActuator = ForceSensorJoint6DOFActuator::Cast(&actuator);
    if(tmpActuator!=NULL){
        Set(tmpActuator->mPosition,tmpActuator->mVelocity,tmpActuator->mAcceleration,tmpActuator->mTorque);
    }
*/
}

void      ForceSensorJoint6DOFSensor::Set(const SpatialForce &force){
    mForce = force;
}
void      ForceSensorJoint6DOFSensor::SetLimits(const SpatialForce &limitLow,const SpatialForce &limitHigh){
}
int         ForceSensorJoint6DOFSensor::StreamSize(){
    return Sensor::StreamSize() + 6*sizeof(REALTYPE);
}
int         ForceSensorJoint6DOFSensor::SetStream(void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetStream(mem);
    REALTYPE *dmem = (REALTYPE*)mem;
    *(dmem++)   = mForce.mLinear.cx();
    *(dmem++)   = mForce.mLinear.cy();
    *(dmem++)   = mForce.mLinear.cz();
    *(dmem++)   = mForce.mAngular.cx();
    *(dmem++)   = mForce.mAngular.cy();
    *(dmem++)   = mForce.mAngular.cz();
    return ((char*)dmem)-((char*)memory);
}
int         ForceSensorJoint6DOFSensor::SetFromStream(const void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetFromStream(mem);
    REALTYPE *dmem = (REALTYPE*)mem;
    mForce.mLinear.x() = *(dmem++);
    mForce.mLinear.y() = *(dmem++);
    mForce.mLinear.z() = *(dmem++);
    mForce.mAngular.x() = *(dmem++);
    mForce.mAngular.y() = *(dmem++);
    mForce.mAngular.z() = *(dmem++);
    return ((char*)dmem)-((char*)memory);
}





BASEOBJECT_DEF(ContactJointSensor);
ContactJointSensor::ContactJointSensor():Sensor(){}
ContactJointSensor::~ContactJointSensor(){}

void ContactJointSensor::Zero(){
    mValue = 0.0;
}
void ContactJointSensor::Set(const Sensor& sensor){
    pContactJointSensor tmpSensor = ContactJointSensor::Cast(&sensor);
    if(tmpSensor!=NULL){
        Set(tmpSensor->mValue);
    }
}
void ContactJointSensor::Set(const Actuator& actuator){
/*    pContactJointActuator tmpActuator = ContactJointActuator::Cast(&actuator);
    if(tmpActuator!=NULL){
        Set(tmpActuator->mPosition,tmpActuator->mVelocity,tmpActuator->mAcceleration,tmpActuator->mTorque);
    }
*/
}

void      ContactJointSensor::Set(const REALTYPE value){
    mValue = value;
}
int         ContactJointSensor::StreamSize(){
    return Sensor::StreamSize() + 1*sizeof(REALTYPE);
}
int         ContactJointSensor::SetStream(void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetStream(mem);
    REALTYPE *dmem = (REALTYPE*)mem;
    *(dmem++)   = mValue;
    return ((char*)dmem)-((char*)memory);
}
int         ContactJointSensor::SetFromStream(const void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetFromStream(mem);
    REALTYPE *dmem = (REALTYPE*)mem;
    mValue = *(dmem++);
    return ((char*)dmem)-((char*)memory);
}







BASEOBJECT_DEF(KeyPressSensor);
KeyPressSensor::KeyPressSensor():Sensor(){}
KeyPressSensor::~KeyPressSensor(){}

void KeyPressSensor::Zero(){
    Set(0);
}
void KeyPressSensor::Set(const Sensor& sensor){
    pKeyPressSensor tmpSensor = KeyPressSensor::Cast(&sensor);
    if(tmpSensor!=NULL){
        Set(tmpSensor->mKeyPressed);
    }
}
void KeyPressSensor::Set(char key){
    mKeyPressed = key;
}
char KeyPressSensor::GetKey(){
    return mKeyPressed;
}
int         KeyPressSensor::StreamSize(){
    return Sensor::StreamSize() + 4;
}
int         KeyPressSensor::SetStream(void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetStream(mem);
    *mem   = mKeyPressed;
    return ((char*)mem)-((char*)memory)+4;
}
int         KeyPressSensor::SetFromStream(const void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetFromStream(mem);
    mKeyPressed = *(mem);
    return ((char*)mem)-((char*)memory)+4;
}


BASEOBJECT_DEF(TimeSensor);
TimeSensor::TimeSensor():Sensor(){}
TimeSensor::~TimeSensor(){}

void TimeSensor::Zero(){
    Set(0);
}
void TimeSensor::Set(const Sensor& sensor){
    pTimeSensor tmpSensor = TimeSensor::Cast(&sensor);
    if(tmpSensor!=NULL){
        Set(tmpSensor->mTime);
    }
}
void TimeSensor::Set(REALTYPE time){
    mTime = time;
}
REALTYPE TimeSensor::GetTime(){
    return mTime;
}
int         TimeSensor::StreamSize(){
    return Sensor::StreamSize() + sizeof(REALTYPE);
}
int         TimeSensor::SetStream(void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetStream(mem);
    REALTYPE *dmem = (REALTYPE*)mem;
    *(dmem++)   = mTime;
    return ((char*)dmem)-((char*)memory);
}
int         TimeSensor::SetFromStream(const void* memory){
    char* mem = (char*) memory;
    mem += Sensor::SetFromStream(mem);
    REALTYPE *dmem = (REALTYPE*)mem;
    mTime           = *(dmem++);
    return ((char*)dmem)-((char*)memory);
}
