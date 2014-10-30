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

#include "InverseDynamics.h"

InverseDynamics::InverseDynamics(){
  mRobot                      = NULL;
  mLinks                      = NULL;
  mJoints                     = NULL;
  mLinksCount                 = 0;

  mExternalForces             = NULL;

  bGravityCompensationOnly    = true;
  mFrictionCompensationMethod = 0;
  mFrictionCompensationRatio  = 1.0;

  m_Ic = NULL;

}

InverseDynamics::~InverseDynamics(){
  Free();
}

void InverseDynamics::SetRobot(pRobot robot){
  if(mRobot==robot) return;
  Free();
  mRobot = robot;
  if(mRobot!=NULL){
    mOutputTorques.Resize(mRobot->GetDOFCount());
    mOutputTorques.Zero();
    mLinksCount =  mRobot->GetLinksCount();
    mLinks      = &mRobot->GetLinks();
    mJoints     = &mRobot->GetJoints();

    mExternalForces = new SpatialForce[mLinksCount];
    mJSIM.Resize(mRobot->GetDOFCount(),mRobot->GetDOFCount(),false);
    mJSIM.Zero();

    m_Ic = new TMatrix<6>[mLinksCount];

  }
}

void    InverseDynamics::SetGravity(const Vector3& gravity){
  mGravity = gravity;
}
void    InverseDynamics::SetGravityCompensationOnly(bool only){
  bGravityCompensationOnly = only;
}
void    InverseDynamics::SetFrictionCompensation(int method, REALTYPE ratio){
  mFrictionCompensationMethod = method;
  mFrictionCompensationRatio  = ratio;
}


void    InverseDynamics::Init(XmlTree *tree){
  bGravityCompensationOnly = true;
  mGravity.Set(0.0, 0.0, -9.81);

  if(tree!=NULL){
    bGravityCompensationOnly = tree->CGet("StaticModel",false);
    if(tree->Find("Gravity")){
      REALTYPE *array;
      mGravity.Set(Vector(array,tree->GetArray("Gravity",&array)));
    }
  }
}

void    InverseDynamics::Free(){
  mLinks      = NULL;
  mJoints     = NULL;
  mLinksCount = 0;
  if(mExternalForces!=NULL) delete [] mExternalForces;
  mExternalForces=NULL;
  if(m_Ic!=NULL) delete [] m_Ic;
  m_Ic = NULL;

}

void    InverseDynamics::ClearWrenches(){
    for(int i=0;i<mLinksCount;i++){
        mExternalForces[i].Zero();
    }
}

void    InverseDynamics::SetForce(unsigned int linkId, const Vector3& force){
  if((linkId>=0)&&(linkId<mLinksCount)){
    mExternalForces[linkId].mLinear = force;
    mExternalForces[linkId].mAngular.Zero();
  }
}
void    InverseDynamics::SetWrench(unsigned int linkId, const Vector& force){
  if((linkId>=0)&&(linkId<mLinksCount)&&force.Size()==6){
    for(int i=0;i<3;i++){
      mExternalForces[linkId].mLinear(i) = force.At(i);
      mExternalForces[linkId].mAngular(i) = force.At(i+3);
    }
  }
}

void    InverseDynamics::UpdateForward(){
  if(mRobot==NULL) return;

  Vector3 v3_0, v3_1, v3_2;

  mLinks->at(0)->mPos.Set(mLinks->at(0)->mRefFrame);

  mLinks->at(0)->mVelocity.mAngular.Zero();
  mLinks->at(0)->mAccel.mAngular.Zero();
  mLinks->at(0)->mVelocity.mLinear.Zero();
  mLinks->at(0)->mAccel.mAngular.Zero();

  // Should set gravity here
  mGravity.Minus(mLinks->at(0)->mAccel.mLinear);

  /*
    cout <<"-------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
    cout <<"*******************************************************************************************************************************************"<<endl;
    cout <<"-------------------------------------------------------------------------------------------------------------------------------------------"<<endl;
  */
  /*cout <<"%%%%%%%%%%%%%%%%%%%%%"<<endl;
    mLinks->at(0)->mAccel.mLinear.Print();
    cout <<"%%%%%%%%%%%%%%%%%%%%%"<<endl;
  */
  for(unsigned int i=1;i<mLinksCount;i++){
    pLink           currLink  = mLinks->at(i);

    pRevoluteJoint        currRevoluteJoint   = RevoluteJoint::Cast(mJoints->at(i));
    //pSliderJoint          currSliderJoint     = SliderJoint::Cast(mJoints->at(i));
    pJoint                currJoint           = mJoints->at(i);
    pLink                 prevLink            = currJoint->mParentLink;
    pRevoluteJointSensor  currRevoluteSensor  = RevoluteJointSensor::Cast(currJoint->mSensor);
    //pSliderJointSensor    currSliderSensor    = SliderJointSensor::Cast(currJoint->mSensor);

    SpatialInertia & inertia = currLink->GetSpatialInertia();

    //currLink->mSpFrame.GetReferenceFrame().GetHMatrix().Print();
        
    if(!bGravityCompensationOnly){ //if not only gravity:

      // w_i = R_i/p(i) * w_p(i)
      currLink->mRefFrame.GetInverse().GetOrient().Mult(prevLink->mVelocity.mAngular,
                                                        currLink->mVelocity.mAngular);
      // w_i += EZ * dq_i
      if(currRevoluteJoint){
        Vector3::EZ.Mult(currRevoluteSensor->mVelocity,v3_0);
        currLink->mVelocity.mAngular+=(v3_0);
      }/*else if(currSliderJoint){
         Vector3::EZ.Mult(currSliderSensor->mVelocity,v3_0);
         currLink->mVelocity.mLinear+=(v3_0);
         }*/

      // dw_i = R_i/p(i) * dw_p(i)
      currLink->mRefFrame.GetInverse().GetOrient().Mult(prevLink->mAccel.mAngular,
                                                        currLink->mAccel.mAngular);

      // dw_i += [R_i/p(i) * w_p(i)] x EZ*dq_i
      currLink->mRefFrame.GetInverse().GetOrient().Mult(prevLink->mVelocity.mAngular,
                                                        v3_1);
      v3_1.Cross(v3_0,v3_2);
      currLink->mAccel.mAngular+=(v3_2);

      // dw_i += EZ*ddq_i
      Vector3::EZ.Mult((currRevoluteSensor?currRevoluteSensor->mAcceleration:0.0),v3_0);
      currLink->mAccel.mAngular+=(v3_0);
    }

    // dv_i =
    v3_0.Set(prevLink->mAccel.mLinear);

    if(!bGravityCompensationOnly){ //if not only gravity
      // --
      prevLink->mAccel.mAngular.Cross(currLink->mRefFrame.GetOrigin(), v3_1);
      v3_0+=(v3_1);
      // --
      prevLink->mVelocity.mAngular.Cross(currLink->mRefFrame.GetOrigin(), v3_1);
      prevLink->mVelocity.mAngular.Cross(v3_1, v3_2);
      v3_0+=(v3_2);
    }
    currLink->mRefFrame.GetInverse().GetOrient().Mult(v3_0,
                                                      currLink->mAccel.mLinear);

    // f_i =
    v3_0.Set(currLink->mAccel.mLinear);
    if(!bGravityCompensationOnly){
      currLink->mAccel.mAngular.Cross(inertia.mCenterOfMass,v3_1);
      v3_0+=(v3_1);
      currLink->mVelocity.mAngular.Cross(inertia.mCenterOfMass, v3_1);
      currLink->mVelocity.mAngular.Cross(v3_1, v3_2);
      v3_0+=(v3_2);
    }
    v3_0.Mult(inertia.mMass,
              currLink->mForce.mLinear);


    // n_i =
    if(!bGravityCompensationOnly){
      inertia.mInertiaMoment.Mult(currLink->mAccel.mAngular,
                                  currLink->mForce.mAngular);
      inertia.mInertiaMoment.Mult(currLink->mVelocity.mAngular,
                                  v3_0);
      currLink->mVelocity.mAngular.Cross(v3_0,v3_1);
      currLink->mForce.mAngular+=(v3_1);


      inertia.mCenterOfMass.Cross(currLink->mForce.mLinear,v3_0);
      currLink->mForce.mAngular+=(v3_0);

      //currLink->mForce.mAngular += mExternalForces[i].mAngular;

      if(currRevoluteJoint){
        if(mFrictionCompensationMethod>0){
          REALTYPE f=0.0;
          //    yn = (a0n + a1n*exp(-(x/v0n).^2) ).*sign(x)+a2n*x;
          // 	yp = (a0p + a1p*exp(-(x/v0p).^2) ).*sign(x)+a2p*x;
          if(currRevoluteSensor->mVelocity>0.0){
            REALTYPE exp2 = currRevoluteSensor->mVelocity/currRevoluteJoint->mFrictionCoefs[3];
            exp2 = exp(-exp2*exp2);
            f += currRevoluteJoint->mFrictionCoefs[0] + currRevoluteJoint->mFrictionCoefs[1] * exp2;
            f += currRevoluteJoint->mFrictionCoefs[2] * currRevoluteSensor->mVelocity;
          }else if(currRevoluteSensor->mVelocity<0.0){
            REALTYPE exp2 = currRevoluteSensor->mVelocity/currRevoluteJoint->mFrictionCoefs[7];
            exp2 = exp(-exp2*exp2);
            f -= currRevoluteJoint->mFrictionCoefs[4] + currRevoluteJoint->mFrictionCoefs[5] * exp2;
            f += currRevoluteJoint->mFrictionCoefs[6] * currRevoluteSensor->mVelocity;
          }
          f *= mFrictionCompensationRatio;
          //cout << i<<"  id "<<f<<endl;
          currLink->mForce.mAngular += Vector3::EZ*f;
        }
      }

      // Friction test
      //if(currRevoluteJoint)
      //currLink->mForce.mAngular-= Vector3::EZ*(-currRevoluteJoint->mFrictionCoefs[1]*currRevoluteSensor->mVelocity);
      //currLink->mForce += mExternalForces[i];

    }else{

      //inertia.mCenterOfMass.Cross(currLink->mForce.mLinear,currLink->mForce.mAngular);

      ReferenceFrame & rf = mRobot->GetReferenceFrame(0,i);
      rf.GetOrient().Mult(mExternalForces[i].mLinear,v3_0);
      currLink->mForce.mLinear += v3_0;

      inertia.mCenterOfMass.Cross(currLink->mForce.mLinear,currLink->mForce.mAngular);

      //mExternalForces[i].mLinear.Cross(rf.GetOrigin(),v3_0);
      //rf.GetOrigin().Cross(mExternalForces[i].mLinear,v3_0);
      v3_0 = mExternalForces[i].mAngular;

      rf.GetOrient().Mult(v3_0,v3_1);

      currLink->mForce.mAngular += v3_1;

      //currLink->mForce += mExternalForces[i];
      //cout << i<< endl;
      //mExternalForces[i].mAngular.Print();
      //v3_1.Print();
    }
  }
  //cout << mLinksCount<< endl;
}

void    InverseDynamics::UpdateBackward(){
  if(mRobot==NULL) return;
  Vector3 v3_0, v3_1, v3_2;
  int cnt = mRobot->GetDOFCount()-1;
  for(unsigned int i=mLinksCount-1;i>0;i--){

    pLink                   currLink              = mLinks->at(i);
    pJoint                  currJoint             = mJoints->at(i);
    pLink                   prevLink              = currJoint->mParentLink;
    pRevoluteJoint          currRevoluteJoint     = RevoluteJoint::Cast(mJoints->at(i));
    pSliderJoint            currSliderJoint       = SliderJoint::Cast(mJoints->at(i));
    //pRevoluteJointActuator  currRevoluteActuator  = RevoluteJointActuator::Cast(currJoint->mActuator);
    //pSliderJointActuator    currSliderActuator    = SliderJointActuator::Cast(currJoint->mActuator);

    // t_i = EZ^T * n_i
    if(currRevoluteJoint){
      mOutputTorques[cnt--] = Vector3::EZ.Dot(currLink->mForce.mAngular);
      //currRevoluteActuator->mTorque = Vector3::EZ.Dot(currLink->mForce.mAngular);
    }else if(currSliderJoint){
      mOutputTorques[cnt--] = Vector3::EZ.Dot(currLink->mForce.mLinear);
    }

    if(i>0){
      // f_p(i) += R^T*f_i
      currLink->mRefFrame.GetInverse().GetOrient().TransposeMult(currLink->mForce.mLinear,
                                                                 v3_0);
      prevLink->mForce.mLinear+=(v3_0);

      // n_p(i) +=
      currLink->mRefFrame.GetInverse().GetOrient().TransposeMult(currLink->mForce.mAngular,
                                                                 v3_0);
      prevLink->mForce.mAngular+=(v3_0);
      currLink->mRefFrame.GetInverse().GetOrient().TransposeMult(currLink->mForce.mLinear,
                                                                 v3_0);
      // --
      currLink->mRefFrame.GetOrigin().Cross(v3_0,v3_1);
      prevLink->mForce.mAngular+=(v3_1);
    }
  }
}
void    InverseDynamics::Update(){
  if(mRobot==NULL) return;

  UpdateForward();
  UpdateBackward();

  /*
    for(unsigned int i=1;i<mLinksCount;i++){
    pLink                   currLink              = mLinks->at(i);
    cout << i <<" ";
    cout << currLink->mForce.mAngular[0] << " "<< currLink->mForce.mAngular[1] << " "<< currLink->mForce.mAngular[2]<< " ";
    cout << currLink->mForce.mLinear[0] << " "<< currLink->mForce.mLinear[1] << " "<< currLink->mForce.mLinear[2];
    cout << endl;
    }
    cout <<"************************"<<endl;

    cout << "Pos ";
    for(unsigned int i=1;i<mLinksCount;i++){
    pLink           currLink  = mLinks->at(i);

    pRevoluteJoint        currRevoluteJoint   = RevoluteJoint::Cast(mJoints->at(i));
    pJoint                currJoint           = mJoints->at(i);
    pLink                 prevLink            = currJoint->mParentLink;
    pRevoluteJointSensor  currRevoluteSensor  = RevoluteJointSensor::Cast(currJoint->mSensor);

    cout << (currRevoluteSensor?currRevoluteSensor->mPosition:0.0)<<" ";
    }
    cout <<endl;

    cout << "Vel ";
    for(unsigned int i=1;i<mLinksCount;i++){
    pLink           currLink  = mLinks->at(i);

    pRevoluteJoint        currRevoluteJoint   = RevoluteJoint::Cast(mJoints->at(i));
    pJoint                currJoint           = mJoints->at(i);
    pLink                 prevLink            = currJoint->mParentLink;
    pRevoluteJointSensor  currRevoluteSensor  = RevoluteJointSensor::Cast(currJoint->mSensor);

    cout << (currRevoluteSensor?currRevoluteSensor->mVelocity:0.0)<<" ";
    }
    cout <<endl;

    cout << "Acc ";
    for(unsigned int i=1;i<mLinksCount;i++){
    pLink           currLink  = mLinks->at(i);

    pRevoluteJoint        currRevoluteJoint   = RevoluteJoint::Cast(mJoints->at(i));
    pJoint                currJoint           = mJoints->at(i);
    pLink                 prevLink            = currJoint->mParentLink;
    pRevoluteJointSensor  currRevoluteSensor  = RevoluteJointSensor::Cast(currJoint->mSensor);

    cout << (currRevoluteSensor?currRevoluteSensor->mAcceleration:0.0)<<" ";
    }
    cout <<endl;
  */
}

void    InverseDynamics::GetTorques(Vector & output){
  output = mOutputTorques;
}

void    InverseDynamics::AddTorques(Vector & output){
  output += mOutputTorques;
}

inline TMatrix<6>& RefFrameToTMatrix6(ReferenceFrame &ref, TMatrix<6> & result){
  Matrix3 tmpM;  tmpM.Set(ref.GetOrient());
  Matrix3 tmpM2; tmpM2.SkewSymmetric(ref.GetOrigin()); //tmpM2.STranspose();
  Matrix3 tmpM3; tmpM.Mult(tmpM2,tmpM3);
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++){
      result._[(i  )*6+j]   = tmpM.          _[i*3+j];
      result._[(i+3)*6+j]   = tmpM3.         _[i*3+j];
      result._[(i  )*6+j+3] = R_ZERO;
      result._[(i+3)*6+j+3] = tmpM.          _[i*3+j];
    }
  }
  return result;
}


Matrix & InverseDynamics::ComputeJSIM(){
  //  cout << "Warning: code still in debug"<<endl;
  TVector<6> F,F2;
  TVector<6> RJointAxis;
  TMatrix<6> R,Rt;
  TMatrix<6> m6tmp;
  TMatrix<6> m6tmp2;
  
  RJointAxis.Zero();
  RJointAxis.RefNoCheck(2) = 1.0;

  vector<int> & parents = mRobot->GetParents();
  // H=0
  mJSIM.Zero();

  for(unsigned int i=0;i<mLinksCount;i++){
    pLink           currLink = mLinks->at(i);
    //        I_i^C = I_i
    currLink->GetSpatialInertia().ToTMatrix6(m_Ic[i]);
       m_Ic[i].Print();
       cout<<"dd"<<i<<endl;
  }

  int icnt = mRobot->GetDOFCount()-1;

  for(unsigned int i=mLinksCount;i>0;i--){
    //pLink           currLink = mLinks->at(i);
    //pJoint          currJoint             = mJoints->at(i);
    pRevoluteJoint  currRevoluteJoint     = RevoluteJoint::Cast(mJoints->at(i));
    //F = I_c^C \theta_i
    m_Ic[i].Mult(RJointAxis,F);
    if(currRevoluteJoint!=NULL){
        cout << i<<":"<<icnt<<" set "<<endl;
      // H_ii = \theta_i^T*F
      //      mJSIM.RefNoCheck(i,i) = m_Ic[i].At(2,2);
      mJSIM.RefNoCheck(i,i) = RJointAxis.Dot(F);
    }

    if(parents[i]>0){
      //cout << icnt<<" -- "<<i<<"->"<<parents[i]<<endl;
      //RefFrameToTMatrix6(prevLink->mRefFrame.GetInverse(), R);
      // I_p(i)^C = I_p(i)^C + R_ip(i)^T * I_i^C * R_ip(i)
      RefFrameToTMatrix6(mRobot->GetReferenceFrame(i,parents[i]), R);
      R.Transpose(Rt);
      Rt.Mult(m_Ic[i],m6tmp);
      m6tmp.Mult(R,m6tmp2);
      m_Ic[parents[i]] += m6tmp2;
      //m_Ic[parents[i]].Add(m6tmp2,m_Ic[parents[i]]);
    }
    //    }
    icnt = 0;
    //for(unsigned int i=0;i<mLinksCount;i++){
    //cout << i<<":"<<icnt<<endl;
    //pLink           currLink = mLinks->at(i);
    //pJoint          currJoint             = mJoints->at(i);
    //        pRevoluteJoint  currRevoluteJoint     = RevoluteJoint::Cast(mJoints->at(i));
    //pLink           prevLink              = currJoint->mParentLink;
    if(currRevoluteJoint!=NULL){
      //      cout << i<<":"<<icnt<<" set2 "<<endl;
      int j = i;
      int jcnt = icnt;
      while(parents[j]>0){
        //        cout << "asd"<<endl;
        // F = j^X^T_p(i)*F
        //RefFrameToTMatrix6(mLinks->at(parents[j])->mRefFrame.GetInverse(), R);
        RefFrameToTMatrix6(mRobot->GetReferenceFrame(j,parents[j]), R);
        R.Transpose(Rt);
        Rt.Mult(F,F2);
        F = F2;
        j = parents[j];
        if(RevoluteJoint::Cast(mJoints->at(j))!=NULL){
          jcnt --;
          //          cout << "JSIM:"<<icnt<<" "<<jcnt<<endl;
          //Hij = F^T*\theta_j
          //Hji = Hij^T
          //but these are scalars
          // mJSIM.RefNoCheck(icnt,jcnt) = F.AtNoCheck(2);
          // mJSIM.RefNoCheck(jcnt,icnt) = F.AtNoCheck(2);
          mJSIM.RefNoCheck(i,j) = F.Dot(RJointAxis);
          mJSIM.RefNoCheck(j,i) = F.Dot(RJointAxis);
        }
      }

      icnt++;
    }
  }
  //  mJSIM.Print();
  return mJSIM;
}
