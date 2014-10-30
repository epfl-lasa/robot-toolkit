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

#include "Joint.h"

#include "Link.h"

BASEOBJECT_TOPDEF(Joint);

Joint::Joint(){
    ConstructorInit();
}

Joint::Joint(const pXmlTree tree){
    ConstructorInit();
    Load(tree);
}

Joint::~Joint(){
    Free();
}

void  Joint::ConstructorInit(){
    mSensor       = new Sensor();
    mActuator     = new Actuator();
    bIsLocked     = false;
    mParentLink   = NULL;
    mChildLink    = NULL;
    mRefFrame     = NULL;

    mBaseFrame.Identity();
    mJointFrame.Identity();
}

void Joint::Free(){
    bIsLocked     = false;
    mParentLink   = NULL;
    mChildLink    = NULL;
    mRefFrame     = NULL;

    mBaseFrame.Identity();
    mJointFrame.Identity();
    if(mSensor!=NULL)   delete mSensor;   mSensor   = NULL;
    if(mActuator!=NULL) delete mActuator; mActuator = NULL;
}

void Joint::SetParentLink(Link *parent){
    mParentLink = parent;
}
void Joint::SetChildLink(Link *child){
    mChildLink  = child;
    if(mChildLink!=NULL){
        mRefFrame     = &mChildLink->mRefFrame;
        mSensor->SetName(mChildLink->GetName());
        mActuator->SetName(mChildLink->GetName());
    }
}

void Joint::SetBaseFrame(const ReferenceFrame& base){
    mBaseFrame.Set(base);
}

void Joint::Lock(bool lock){
    bIsLocked = lock;
}

void Joint::Update(){
    if(mRefFrame!=NULL){
        mBaseFrame.Mult(mJointFrame,*mRefFrame);
    }
}

void Joint::Load(const pXmlTree tree){
    mBaseFrame.Identity();
    mJointFrame.Identity();
    bIsLocked = false;

    if(tree!=NULL){

        REALTYPE *array;

        Vector3 origin;
        Matrix3 orient;


        if(tree->Find("BaseFrame.Origin")){
            mBaseFrame.SetOrigin(origin.Set(Vector(array,tree->GetArray("BaseFrame.Origin",&array))));
        }

        if(tree->Find("BaseFrame.Orient")){
            mBaseFrame.SetOrient(orient.Set(Matrix(array,tree->GetArray("BaseFrame.Orient",&array)/3,3)));
        }

        if(tree->Find("ControlMode")){
            string mode = tree->Get("ControlMode",string(""));
            if(mode=="Position")
                mControlMode =  JCTRLMODE_POSITION;
            else if(mode=="Velocity")
                mControlMode =  JCTRLMODE_VELOCITY;
            else if(mode=="Acceleration")
                mControlMode =  JCTRLMODE_ACCELERATION;
            else if(mode=="Torque")
                mControlMode =  JCTRLMODE_TORQUE;
            else
                mControlMode =  JCTRLMODE_NONE;
        }else{
            mControlMode = JCTRLMODE_NONE;
        }
    }
    Joint::Update();
}

void Joint::Print(){
/*
  cout << "Joint info:"<<endl;
  cout << "Base Frame:"<<endl;
  mBaseFrame.mPos.Print();
  mBaseFrame.mRot.Print();
  if(mParentLink!=NULL) cout <<"Parent: "<<mParentLink->mName<<endl;
  if(mChildLink!=NULL) cout <<"Child: "<<mChildLink->mName<<endl;
*/
}

void Joint::MapActuatorToSensor(){
  mSensor->Set(*mActuator);
}

void Joint::SetControlMode(Joint::JointControlMode mode){
    mControlMode = mode;
}
Joint::JointControlMode Joint::GetControlMode(){
    return mControlMode;
}





BASEOBJECT_DEF(RevoluteJoint);

RevoluteJoint::RevoluteJoint()
:Joint(){
  ConstructorInit();
}

RevoluteJoint::RevoluteJoint(const pXmlTree tree)
:Joint(){
  ConstructorInit();
  Load(tree);
}

RevoluteJoint::~RevoluteJoint(){
  Free();
}

void  RevoluteJoint::ConstructorInit(){

  if(mSensor!=NULL)   delete mSensor;   mSensor   = NULL;
  if(mActuator!=NULL) delete mActuator; mActuator = NULL;

  mSensor    = new RevoluteJointSensor();
  mActuator  = new RevoluteJointActuator();

  mAxis       = NONE;
  mAxisVector.Zero();
  ((pRevoluteJointSensor)  mSensor  )->Zero();
  ((pRevoluteJointActuator)mActuator)->Zero();
  mZero       = R_ZERO;
  mRange[0]   = R_ZERO;
  mRange[1]   = R_ZERO;
  for(int i=0;i<MAX_FRICTION_COEFS;i++)
    mFrictionCoefs[i] = R_ZERO;

  Update();
}

void RevoluteJoint::Free(){
  Joint::Free();
}


void RevoluteJoint::SetAxis(Axis axis){
  mAxis = axis;
  switch (mAxis){
  case NONE:
    mAxisVector.Zero();
    mRange[0]   = R_ZERO;
    mRange[1]   = R_ZERO;
    ((pRevoluteJointSensor)  mSensor  )->Zero();
    ((pRevoluteJointActuator)mActuator)->Zero();
    mJointFrame.Identity();
    break;
  case EX:
    mAxisVector =  Vector3::EX; break;
  case EY:
    mAxisVector =  Vector3::EY; break;
  case EZ:
    mAxisVector =  Vector3::EZ; break;
  case NEG_EX:
    mAxisVector = -Vector3::EX; break;
  case NEG_EY:
    mAxisVector = -Vector3::EY; break;
  case NEG_EZ:
    mAxisVector = -Vector3::EZ; break;
  case ARBITRARY:
    mAxisVector =  Vector3::EX; break;
  }

  Update();
}

void RevoluteJoint::SetAxis(Vector3 & axisVector){
  mAxis       = ARBITRARY;
  mAxisVector = axisVector;
  mAxisVector.Normalize();

  Update();
}

const Vector3&  RevoluteJoint::GetAxis(){
  return mAxisVector;
}

void RevoluteJoint::SetAngle(REALTYPE angle){
  if(bIsLocked)
    return;

  ((pRevoluteJointActuator)mActuator)->mPosition = TRUNC(angle,mRange[0],mRange[1]);
  //((pRevoluteJointActuator)mActuator)->mPosition = angle;
}

void RevoluteJoint::GetJointLimits(REALTYPE &  low, REALTYPE & high){
  low = mRange[0];
  high = mRange[1];
}

/*
int RevoluteJoint::IsMoveValid(float newDelta){
  float res = angle + newDelta;
  if(res<range[0]-1e-6) return FALSE;
  if(res>range[1]+1e-6) return FALSE;
  return TRUE;
}
*/

void RevoluteJoint::Update(){

  REALTYPE angle = ((pRevoluteJointSensor)mSensor)->mPosition;
  REALTYPE tangle = TRUNC(angle,mRange[0],mRange[1]);
  //REALTYPE tangle = angle;
  if((tangle!=angle)){
    ((pRevoluteJointSensor)mSensor)->mPosition = tangle;
    ((pRevoluteJointSensor)mSensor)->mVelocity = 0.0;
    ((pRevoluteJointSensor)mSensor)->mAcceleration = 0.0;
  }
    ((pRevoluteJointSensor)mSensor)->mLimits[0] = mRange[0];
    ((pRevoluteJointSensor)mSensor)->mLimits[1] = mRange[1];
  angle = tangle;
  switch (mAxis){
  case NONE:
    mJointFrame.GetSetOrient().Identity(); break;
  case EX:
    mJointFrame.GetSetOrient().RotationX( angle); break;
  case EY:
    mJointFrame.GetSetOrient().RotationY( angle); break;
  case EZ:
    mJointFrame.GetSetOrient().RotationZ( angle); break;
  case NEG_EX:
    mJointFrame.GetSetOrient().RotationX(-angle); break;
  case NEG_EY:
    mJointFrame.GetSetOrient().RotationY(-angle); break;
  case NEG_EZ:
    mJointFrame.GetSetOrient().RotationZ(-angle); break;
  case ARBITRARY:
    mJointFrame.GetSetOrient().RotationV(angle,mAxisVector); break;
  }

  Joint::Update();
}



void RevoluteJoint::Load(const pXmlTree tree){

  if(tree==NULL)
    return;

    Joint::Load(tree);

    REALTYPE *array;
    tree->GetArray("Range",&array);
    Vector range = Vector(array,tree->GetArray("Range",&array));
    if(range.Size()>=2){
        mRange[0]    = DEG2RAD(range[0]);
        mRange[1]    = DEG2RAD(range[1]);
    }else{
        mRange[0]    = R_ZERO;
        mRange[1]    = R_ZERO;
    }

    mZero     = DEG2RAD(tree->Get("Zero",R_ZERO));
    mZero     = TRUNC(mZero,mRange[0],mRange[1]);

    ((pRevoluteJointSensor)  mSensor  )->Set(mZero,R_ZERO,R_ZERO,R_ZERO);
    ((pRevoluteJointActuator)mActuator)->Set(mZero,R_ZERO,R_ZERO,R_ZERO);

    pXmlTree tmp = tree->Find("Axis");
    if(tmp!=NULL){
        string axis = tmp->GetData();
        if(axis=="EX")           SetAxis(EX);
        else if(axis=="EY")      SetAxis(EY);
        else if(axis=="EZ")      SetAxis(EZ);
        else if(axis=="NEG_EX")  SetAxis(NEG_EX);
        else if(axis=="NEG_EY")  SetAxis(NEG_EY);
        else if(axis=="NEG_EZ")  SetAxis(NEG_EZ);
    else{
        Vector3 v;
        v.Set(Vector(array,tree->GetArray("Axis",&array)));
        if(v==Vector3::ZERO) SetAxis(NONE);
        else SetAxis(v);
        }
    }else{
        SetAxis(NONE);
    }
    int coefCnt = tree->GetArray("Friction",&array);
    coefCnt = MIN(coefCnt,MAX_FRICTION_COEFS);
    for(int i=0;i<coefCnt;i++)
      mFrictionCoefs[i] = array[i];
    Update();
}

void RevoluteJoint::Print(){
  Joint::Print();
  cout << "Axis:"<< mAxis<< endl;
  cout << "mAngle:" <<((pRevoluteJointSensor)mSensor)->mPosition<<endl;
  cout << "Range:" <<mRange[0]<<" "<<mRange[1]<<endl;
  cout << "mZero:" <<mZero <<endl;
}













BASEOBJECT_DEF(SliderJoint);

SliderJoint::SliderJoint()
:Joint(){
  ConstructorInit();
}

SliderJoint::SliderJoint(const pXmlTree tree)
:Joint(){
  ConstructorInit();
  Load(tree);
}

SliderJoint::~SliderJoint(){
  Free();
}

void  SliderJoint::ConstructorInit(){

  if(mSensor!=NULL)   delete mSensor;   mSensor   = NULL;
  if(mActuator!=NULL) delete mActuator; mActuator = NULL;

  mSensor    = new Generic1DOFJointSensor();
  mActuator  = new Generic1DOFJointActuator();

  mAxis       = NONE;
  mAxisVector.Zero();
  mSensor->Zero();
  mActuator->Zero();
  mZero       = R_ZERO;
  mRange[0]   = R_ZERO;
  mRange[1]   = R_ZERO;
  for(int i=0;i<MAX_FRICTION_COEFS;i++)
    mFrictionCoefs[i] = R_ZERO;

  Update();
}

void SliderJoint::Free(){
  Joint::Free();
}


void SliderJoint::SetAxis(Axis axis){
  mAxis = axis;
  switch (mAxis){
  case NONE:
    mAxisVector.Zero();
    mRange[0]   = R_ZERO;
    mRange[1]   = R_ZERO;
    mSensor  ->Zero();
    mActuator->Zero();
    mJointFrame.Identity();
    break;
  case EX:
    mAxisVector =  Vector3::EX; break;
  case EY:
    mAxisVector =  Vector3::EY; break;
  case EZ:
    mAxisVector =  Vector3::EZ; break;
  case NEG_EX:
    mAxisVector = -Vector3::EX; break;
  case NEG_EY:
    mAxisVector = -Vector3::EY; break;
  case NEG_EZ:
    mAxisVector = -Vector3::EZ; break;
  case ARBITRARY:
    mAxisVector =  Vector3::EX; break;
  }

  Update();
}

void SliderJoint::SetAxis(Vector3 & axisVector){
  mAxis       = ARBITRARY;
  mAxisVector = axisVector;
  mAxisVector.Normalize();

  Update();
}

const Vector3&  SliderJoint::GetAxis(){
  return mAxisVector;
}

void SliderJoint::SetPosition(REALTYPE position){
  if(bIsLocked)
    return;

  ((pGeneric1DOFJointActuator)mActuator)->mPosition = TRUNC(position,mRange[0],mRange[1]);
}

void SliderJoint::Update(){

  REALTYPE pos = ((pGeneric1DOFJointSensor)mSensor)->mPosition;
  REALTYPE tpos = TRUNC(pos,mRange[0],mRange[1]);
  if((tpos!=pos)){
    ((pGeneric1DOFJointSensor)mSensor)->mPosition = tpos;
    ((pGeneric1DOFJointSensor)mSensor)->mVelocity = 0.0;
    ((pGeneric1DOFJointSensor)mSensor)->mAcceleration = 0.0;
  }
    ((pGeneric1DOFJointSensor)mSensor)->mLimits[0] = mRange[0];
    ((pGeneric1DOFJointSensor)mSensor)->mLimits[1] = mRange[1];
  pos = tpos;

  mJointFrame.GetSetOrigin() = mAxisVector*pos;
/*
  switch (mAxis){
  case NONE:
    mJointFrame.GetSetOrient().Identity(); break;
  case EX:
    mJointFrame.GetSetOrient().RotationX( angle); break;
  case EY:
    mJointFrame.GetSetOrient().RotationY( angle); break;
  case EZ:
    mJointFrame.GetSetOrient().RotationZ( angle); break;
  case NEG_EX:
    mJointFrame.GetSetOrient().RotationX(-angle); break;
  case NEG_EY:
    mJointFrame.GetSetOrient().RotationY(-angle); break;
  case NEG_EZ:
    mJointFrame.GetSetOrient().RotationZ(-angle); break;
  case ARBITRARY:
    mJointFrame.GetSetOrient().RotationV(angle,mAxisVector); break;
  }
*/
  Joint::Update();
}



void SliderJoint::Load(const pXmlTree tree){

  if(tree==NULL)
    return;

    Joint::Load(tree);

    REALTYPE *array;
    tree->GetArray("Range",&array);
    Vector range = Vector(array,tree->GetArray("Range",&array));
    if(range.Size()>=2){
        mRange[0]    = range[0];
        mRange[1]    = range[1];
    }else{
        mRange[0]    = R_ZERO;
        mRange[1]    = R_ZERO;
    }

    mZero     = tree->Get("Zero",R_ZERO);
    mZero     = TRUNC(mZero,mRange[0],mRange[1]);

    ((pGeneric1DOFJointSensor)  mSensor  )->Set(mZero,R_ZERO,R_ZERO,R_ZERO);
    ((pGeneric1DOFJointActuator)mActuator)->Set(mZero,R_ZERO,R_ZERO,R_ZERO);

    pXmlTree tmp = tree->Find("Axis");
    if(tmp!=NULL){
        string axis = tmp->GetData();
        if(axis=="EX")           SetAxis(EX);
        else if(axis=="EY")      SetAxis(EY);
        else if(axis=="EZ")      SetAxis(EZ);
        else if(axis=="NEG_EX")  SetAxis(NEG_EX);
        else if(axis=="NEG_EY")  SetAxis(NEG_EY);
        else if(axis=="NEG_EZ")  SetAxis(NEG_EZ);
    else{
        Vector3 v;
        v.Set(Vector(array,tree->GetArray("Axis",&array)));
        if(v==Vector3::ZERO) SetAxis(NONE);
        else SetAxis(v);
        }
    }else{
        SetAxis(NONE);
    }
    int coefCnt = tree->GetArray("Friction",&array);
    coefCnt = MIN(coefCnt,MAX_FRICTION_COEFS);
    for(int i=0;i<coefCnt;i++)
      mFrictionCoefs[i] = array[i];
    Update();
}

void SliderJoint::Print(){
  Joint::Print();
  cout << "Axis:"<< mAxis<< endl;
  cout << "mAngle:" <<((pGeneric1DOFJointSensor)mSensor)->mPosition<<endl;
  cout << "Range:" <<mRange[0]<<" "<<mRange[1]<<endl;
  cout << "mZero:" <<mZero <<endl;
}









BASEOBJECT_DEF(ForceSensorJoint6DOF);

ForceSensorJoint6DOF::ForceSensorJoint6DOF()
:Joint(){
  ConstructorInit();
}

ForceSensorJoint6DOF::ForceSensorJoint6DOF(const pXmlTree tree)
:Joint(){
  ConstructorInit();
  Load(tree);
}
ForceSensorJoint6DOF::~ForceSensorJoint6DOF(){
  Free();
}

void    ForceSensorJoint6DOF::Update(){
    Joint::Update();
}

void    ForceSensorJoint6DOF::Load(const pXmlTree tree){
    if(tree==NULL)
        return;

    Joint::Load(tree);
    ((pForceSensorJoint6DOFSensor)mSensor)->Zero();
}

void    ForceSensorJoint6DOF::Print(){
  Joint::Print();
  SpatialForce &force = ((pForceSensorJoint6DOFSensor)mSensor)->mForce;
  cout << "Force:" <<force.mLinear.cx()<<" "<<force.mLinear.cy()<<" "<<force.mLinear.cz()<<endl;
  cout << "Torque:" <<force.mAngular.cx()<<" "<<force.mAngular.cy()<<" "<<force.mAngular.cz()<<endl;
}

void    ForceSensorJoint6DOF::ConstructorInit(){
    if(mSensor!=NULL)   delete mSensor;   mSensor   = NULL;
    if(mActuator!=NULL) delete mActuator; mActuator = NULL;

    mSensor    = new ForceSensorJoint6DOFSensor();
    mActuator  = new Actuator();

    Update();
}
void    ForceSensorJoint6DOF::Free(){
    Joint::Free();
}











BASEOBJECT_DEF(ContactJoint);

ContactJoint::ContactJoint()
:Joint(){
  ConstructorInit();
}

ContactJoint::ContactJoint(const pXmlTree tree)
:Joint(){
  ConstructorInit();
  Load(tree);
}
ContactJoint::~ContactJoint(){
  Free();
}

void    ContactJoint::Update(){
    Joint::Update();
}

void    ContactJoint::Load(const pXmlTree tree){
    if(tree==NULL)
        return;

    Joint::Load(tree);
    ((pContactJointSensor)mSensor)->Zero();
}

void    ContactJoint::Print(){
  Joint::Print();
  cout << "Contact value:" <<((pContactJointSensor)mSensor)->mValue<<endl;
}

void    ContactJoint::ConstructorInit(){
    if(mSensor!=NULL)   delete mSensor;   mSensor   = NULL;
    if(mActuator!=NULL) delete mActuator; mActuator = NULL;

    mSensor    = new ContactJointSensor();
    mActuator  = new Actuator();

    Update();
}
void    ContactJoint::Free(){
    Joint::Free();
}






pJoint JointConstructor::Create(const pXmlTree tree){
    if(tree==NULL)
        return new Joint(tree);

    if(tree->GetName()!="Joint"){
        return NULL;
    }

    pJoint result;
    string type = tree->GetParamValue("type");
    //cout << "JOINT: type: "<<type<<endl;
    if(type=="Revolute"){
        result = new RevoluteJoint(tree);
    }else if(type=="Slider"){
        result = new SliderJoint(tree);
    }else if(type=="ForceSensor6DOF"){
        result = new ForceSensorJoint6DOF(tree);
    }else if(type=="Contact"){
        result = new ContactJoint(tree);
    }else{
        result = new Joint(tree);
    }

    return result;
}

