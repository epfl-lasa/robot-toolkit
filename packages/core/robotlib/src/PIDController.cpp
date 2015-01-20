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

#include "PIDController.h"

PIDController::PIDController(){
  mState = PIDCTRL_NONE;
}
PIDController::~PIDController(){
}

void    PIDController::Init(pXmlTree tree){
  if(tree==NULL){
    Resize(1);
  }else{
    Resize(0);
    if(tree->GetName() != "PIDController"){
      return;
    }
    pXmlTreeList tlist = tree->GetSubTrees();
    int cnt=0;
    for(unsigned int i=0;i<tlist->size();i++){
      if(tlist->at(i)->GetName()=="PIDEntry"){
        REALTYPE bound;
        Resize(mInput.Size()+1,true);
        mKp(cnt)    = tlist->at(i)->CGet("KP",R_ZERO);
        mKd(cnt)    = tlist->at(i)->CGet("KD",R_ZERO);
        mKi(cnt)    = tlist->at(i)->CGet("KI",R_ZERO);
        bound       = tlist->at(i)->CGet("Max",R_ZERO);
        mOutputLowBound(cnt)  = -fabs(bound);
        mOutputHighBound(cnt) =  fabs(bound);
        cnt++;
      }
    }
    
    // cout <<"********** PID Values"<<endl;
    // mKp.Print();
    // mKd.Print();
    // mKi.Print();
    // mOutputLowBound.Print();
    // mOutputHighBound.Print();
    // cout << "***************"<<endl;

  }
}
//#include <syslog.h>

void    PIDController::Resize(int size, bool Ks){
  // syslog(LOG_ERR,"PID Resize %d->%d",mTarget.Size(),size);
  mTarget.Resize(size,false); mTarget.Zero();
  mInput.Resize(size,false);  mInput.Zero();

  if(Ks){
    mKp.Resize(size);
    mKd.Resize(size);
    mKi.Resize(size);

    mOutputLowBound.Resize(size);
    mOutputHighBound.Resize(size);
  }

  mErrors.Resize(size,false);       mErrors.Zero();
  mErrorsDiff.Resize(size,false);   mErrorsDiff.Zero();
  mErrorsInt.Resize(size,false);    mErrorsInt.Zero();
  mLastErrors.Resize(size,false);   mLastErrors.Zero();

  mBuffer.Resize(size,false);       mBuffer.Zero();

  mOutput.Resize(size,false);       mOutput.Zero();

  mState = PIDCTRL_STOP;
}


void    PIDController::Free(){
}

void    PIDController::Start(){
  if(mState!=PIDCTRL_START){
    Update(R_ZERO);
    mState = PIDCTRL_START;
  }
}
void    PIDController::Stop(){
  mState = PIDCTRL_STOP;
  Update(1);
}
void    PIDController::Reset(){
  Stop();
  Start();
}


void    PIDController::Update(REALTYPE dt){
  mTarget.Sub(mInput,mErrors);

  if(dt<=R_ZERO){
    mOutput.Zero();
    return;
  }

  switch(mState){
  case PIDCTRL_NONE:
  case PIDCTRL_STOP:
    mLastErrors.Set(mErrors);
    //mErrors.Zero();
    mErrorsDiff.Zero();
    mErrorsInt.Zero();
    //mLastErrors.Zero();
    mOutput.Zero();
    break;
  case PIDCTRL_START:
    mTarget.Sub(mInput,mErrors);
    //mErrors

    mErrors.Sub(mLastErrors,mErrorsDiff);
    mErrorsDiff *= (R_ONE/dt);
    mErrors.Mult(dt,mOutput);
    mErrorsInt += mOutput;


    mKp.PMult(mErrors,mOutput);
    mKd.PMult(mErrorsDiff,mBuffer);
    mOutput+=mBuffer;
    mKi.PMult(mErrorsInt,mBuffer);
    mOutput+=mBuffer;

    mLastErrors.Set(mErrors);
//    cout <<"InPID";
//    mOutput.Print("mOutput");
    break;
  }
//  mOutput.Print("outBef");
  mOutput.Trunc(mOutputLowBound,mOutputHighBound);
//  mOutput.Print("outAft");

}


void    PIDController::Process(){
}

void    PIDController::SetTarget(const Vector & target){
  if(mTarget.Size()!=target.Size()){
    Resize(target.Size());
  }
  mTarget = target;
}
void    PIDController::SetInput(const Vector & input){
  if(mInput.Size()!=input.Size()){
    Resize(input.Size());
  }
  mInput = input;
}
Vector& PIDController::GetOutput(Vector & result){
//  cout <<"GetO";mOutput.Print();

  result.Set(mOutput);
//  cout <<"GetR";
//    result.Print();
  return result;
}
Vector&  PIDController::GetOutput(){
  return mOutput;
}

void PIDController::AddOutput(Vector & result){
  result += mOutput;
}

void    PIDController::SetKP(const Vector& kp){
  mKp.Set(kp);
}
void    PIDController::SetKD(const Vector& kd){
  mKd.Set(kd);
}
void    PIDController::SetKI(const Vector& ki){
  mKi.Set(ki);
}
void    PIDController::SetKs(const Vector& kp, const Vector& kd, const Vector& ki){
  SetKP(kp);
  SetKD(kd);
  SetKI(ki);
}

void    PIDController::SetBounds(const Vector& low,const Vector& high){
  mOutputLowBound.Set(low);
  mOutputHighBound.Set(high);
}




