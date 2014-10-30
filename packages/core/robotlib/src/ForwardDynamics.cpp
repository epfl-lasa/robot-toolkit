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

#include "ForwardDynamics.h"

ForwardDynamics::ForwardDynamics(){
    mRobot                  = NULL;
    mLinks                  = NULL;
    mJoints                 = NULL;
    mLinksCount             = 0;
    mLinkP                  = NULL;
    mFrictionMethod         = 0;
}

ForwardDynamics::~ForwardDynamics(){
    Free();
}
void ForwardDynamics::SetRobot(pRobot robot){
    if(mRobot==robot) return;

    Free();
    mRobot = robot;
    if(mRobot!=NULL){
        mOutputAccel.Resize(mRobot->GetDOFCount());
        mOutputAccel.Zero();
        mLinksCount =  mRobot->GetLinksCount();
        mLinks      = &mRobot->GetLinks();
        mJoints     = &mRobot->GetJoints();
        mLinkP      = new LinkProperty[mLinksCount];
    }
}

void    ForwardDynamics::SetGravity(const Vector3& gravity){
    mGravity = gravity;
}


void    ForwardDynamics::Init(XmlTree *tree){
    mGravity.Set(0.0, 0.0, -9.81);
    //mGravity.Set(0.0, 0.0, 0.0);

    if(tree!=NULL){
        if(tree->Find("Gravity")){
            REALTYPE *array;
            mGravity.Set(Vector(array,tree->GetArray("Gravity",&array)));
        }
    }
}

void    ForwardDynamics::Free(){
    if(mLinkP) delete [] mLinkP;
    mLinkP      = NULL;
    mLinks      = NULL;
    mJoints     = NULL;
    mLinksCount = 0;
}


inline TMatrix<6>& ToCrossMotionTMatrix6(TVector<6> &vec, TMatrix<6> &result){
  Vector3 v1,v2;
  for(int i=0;i<3;i++){
    v1[i] = vec[i];
    v2[i] = vec[i+3];
  }

  Matrix3 sk1,sk2;
  sk1.SkewSymmetric(v1);
  sk2.SkewSymmetric(v2);

  for(int j=0;j<3;j++){
    for(int i=0;i<3;i++){
      result._[(i  )*6+j  ]   = sk1._[i*3+j];
      result._[(i+3)*6+j  ]   = sk2._[i*3+j];
      result._[(i  )*6+j+3]   = R_ZERO;
      result._[(i+3)*6+j+3]   = sk1._[i*3+j];
    }
  }
  return result;
}
inline TMatrix<6>& ToCrossForceTMatrix6(TVector<6> &vec, TMatrix<6> & result){
  Vector3 v1,v2;
  for(int i=0;i<3;i++){
    v1[i] = vec[i];
    v2[i] = vec[i+3];
  }

  Matrix3 sk1,sk2;
  sk1.SkewSymmetric(v1);
  sk2.SkewSymmetric(v2);

  for(int j=0;j<3;j++){
    for(int i=0;i<3;i++){
      result._[(i  )*6+j  ]   = sk1._[i*3+j];
      result._[(i+3)*6+j  ]   = R_ZERO;
      result._[(i  )*6+j+3]   = sk2._[i*3+j];
      result._[(i+3)*6+j+3]   = sk1._[i*3+j];
    }
  }
  return result;
}

inline TMatrix<6>& RefFrameToTMatrix6(ReferenceFrame &ref, TMatrix<6> & result){
  Matrix3 tmpM;  tmpM.Set(ref.GetOrient());
  Matrix3 tmpM2; tmpM2.SkewSymmetric(ref.GetOrigin()); tmpM2.STranspose();
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


void    ForwardDynamics::Update(){
  if(mRobot==NULL) return;

  TMatrix<6> tmpM1;
  TVector<6> tmpV1;

  TVector<6> ez; ez.Zero(); ez(2) = R_ONE;

  vector<int> & parents = mRobot->GetParents();



  // Initialization
  mLinkP[0].m_v.Zero();
  mLinkP[0].m_a.Zero();
  mLinkP[0].m_a[3] = -mGravity[0];
  mLinkP[0].m_a[4] = -mGravity[1];
  mLinkP[0].m_a[5] = -mGravity[2];
  mLinkP[0].m_q       = R_ZERO;
  mLinkP[0].m_dq      = R_ZERO;
  mLinkP[0].m_ddq     = R_ZERO;
  mLinkP[0].m_bJoint  = 0;

  for(unsigned int i=1;i<mLinksCount;i++){
    pLink                 currLink            = mLinks->at(i);
    pRevoluteJoint        currRevoluteJoint   = RevoluteJoint::Cast(mJoints->at(i));
    pJoint                currJoint           = mJoints->at(i);
    //pLink                 prevLink            = currJoint->mParentLink;
    pRevoluteJointSensor  currRevoluteSensor  = RevoluteJointSensor::Cast(currJoint->mSensor);
    pRevoluteJointActuator currRevoluteActuator = RevoluteJointActuator::Cast(currJoint->mActuator);

    mLinkP[i].m_t       = (currRevoluteJoint?currRevoluteActuator->mTorque:R_ZERO);
    mLinkP[i].m_q       = (currRevoluteJoint?currRevoluteSensor->mPosition:R_ZERO);
    mLinkP[i].m_dq      = (currRevoluteJoint?currRevoluteSensor->mVelocity:R_ZERO);
    mLinkP[i].m_ddq     = (currRevoluteJoint?currRevoluteSensor->mAcceleration:R_ZERO);
    mLinkP[i].m_bJoint  = (currRevoluteJoint?1:0);


    if(mLinkP[i].m_bJoint){
        if(mLinkP[i].m_q<=currRevoluteJoint->mRange[0]){
            //cout << "MIN: "<<i<<endl;
            mLinkP[i].m_q = currRevoluteJoint->mRange[0];
            if(mLinkP[i].m_dq < R_ZERO) mLinkP[i].m_dq = 0;
        }else if(mLinkP[i].m_q>=currRevoluteJoint->mRange[1]){
            //cout << "MAX: "<<i<<endl;
            mLinkP[i].m_q = currRevoluteJoint->mRange[1];
            if(mLinkP[i].m_dq > R_ZERO) mLinkP[i].m_dq = 0;
        }
    }

    ReferenceFrame ref;
    ref.SetOrient(currLink->mRefFrame.GetInverse().GetOrient());
    ref.SetOrigin(currLink->mRefFrame.GetOrigin());
    /*ref.SetOrient(currLink->mRefFrame.GetInverse().GetOrient());
    ref.SetOrigin(currLink->mRefFrame.GetInverse().GetOrigin());
     */
    //ref.SetOrient(currLink->mRefFrame.GetInverse().GetOrient());
    //ref.SetOrigin(currLink->mRefFrame.GetInverse().GetOrigin());
    RefFrameToTMatrix6(ref,mLinkP[i].m_X);

    mLinkP[i].m_X.Transpose(mLinkP[i].m_Xt);

    mLinkP[i].m_vdq.Zero();
    mLinkP[i].m_vdq[2] = mLinkP[i].m_dq;

    mLinkP[i].m_vddq.Zero();
    mLinkP[i].m_vddq[2] = mLinkP[i].m_ddq;
    //cout << i<<" "<<currLink->m.mMass<<endl;
    /*
    if(i==2){
        double a = currLink->mInertia.mMass;
        Vector3 m = currLink->mInertia.mLinearMoment;
        currLink->mInertia.mLinearMoment = m*0.1;
        currLink->mInertia.mMass = a*0.1;
        currLink->mInertia.ToTMatrix6(mLinkP[i].m_I);
        currLink->mInertia.mMass = a;
        currLink->mInertia.mLinearMoment = m;
    }else{
        currLink->mInertia.ToTMatrix6(mLinkP[i].m_I);

    }*/
        currLink->GetSpatialInertia().ToTMatrix6(mLinkP[i].m_I);
    //mLinkP[i].m_I.Print();
  }
  /*cout << "Pos "; for(unsigned int i=0;i<mLinksCount;i++) cout << mLinkP[i].m_q << " "; cout << endl;
  cout << "Vel "; for(unsigned int i=0;i<mLinksCount;i++) cout << mLinkP[i].m_dq << " "; cout << endl;
  cout << "Acc "; for(unsigned int i=0;i<mLinksCount;i++) cout << mLinkP[i].m_ddq << " "; cout << endl;
  cout << "Trq "; for(unsigned int i=0;i<mLinksCount;i++) cout << mLinkP[i].m_t << " "; cout << endl;
*/
/*
  for(unsigned int i=1;i<mLinksCount;i++){

    mLinkP[i].m_v = mLinkP[i].m_X * mLinkP[parents[i]].m_v;
    mLinkP[i].m_v+= mLinkP[i].m_vdq;

    mLinkP[i].m_s = ToCrossMotionTMatrix6(mLinkP[i].m_v,tmpM1) * mLinkP[i].m_vdq;

    mLinkP[i].m_a = mLinkP[i].m_X * mLinkP[parents[i]].m_a;
    mLinkP[i].m_a+= mLinkP[i].m_vddq;
    mLinkP[i].m_a+= mLinkP[i].m_s;


    mLinkP[i].m_f = mLinkP[i].m_I * mLinkP[i].m_a;
    mLinkP[i].m_f+= ToCrossForceTMatrix6(mLinkP[i].m_v,tmpM1) * (mLinkP[i].m_I * mLinkP[i].m_v);

  }

  for(unsigned int i=mLinksCount-1;i>0;i--){
    if(mLinkP[i].m_bJoint){
      mLinkP[i].m_t = ez.Dot(mLinkP[i].m_f);
    }else{
      mLinkP[i].m_t = 0.0;
    }
    mLinkP[parents[i]].m_f += mLinkP[i].m_Xt * mLinkP[i].m_f;
  }

  cout << "Trq "; for(unsigned int i=0;i<mLinksCount;i++) cout << mLinkP[i].m_t << " "; cout << endl;
  cout << "Acc "; for(unsigned int i=0;i<mLinksCount;i++) cout << mLinkP[i].m_ddq << " "; cout << endl;
  */
  int ccnt = 0;
  for(unsigned int i=1;i<mLinksCount;i++){

    mLinkP[i].m_v = mLinkP[i].m_X * mLinkP[parents[i]].m_v;
    mLinkP[i].m_v+= mLinkP[i].m_vdq;

    mLinkP[i].m_s = ToCrossMotionTMatrix6(mLinkP[i].m_v,tmpM1) * mLinkP[i].m_vdq;

    mLinkP[i].m_p = ToCrossForceTMatrix6(mLinkP[i].m_v,tmpM1) * (mLinkP[i].m_I * mLinkP[i].m_v);
    if(mLinkP[i].m_bJoint){
        pRevoluteJoint        currRevoluteJoint   = RevoluteJoint::Cast(mJoints->at(i));
        if(mFrictionMethod>0){
	        //yn = (a0n + a1n*exp(-(x/v0n).^2) ).*sign(x)+a2n*x;
            // 	yp = (a0p + a1p*exp(-(x/v0p).^2) ).*sign(x)+a2p*x;
            REALTYPE f=0.0;
            REALTYPE fThres = 0.01;
            if(mLinkP[i].m_dq>fThres){
                REALTYPE exp2 = mLinkP[i].m_dq/currRevoluteJoint->mFrictionCoefs[3];
                exp2 = exp(-exp2*exp2);
                f += currRevoluteJoint->mFrictionCoefs[0] + currRevoluteJoint->mFrictionCoefs[1] * exp2;
                f += currRevoluteJoint->mFrictionCoefs[2] * mLinkP[i].m_dq;
            }else if(mLinkP[i].m_dq<-fThres){
                REALTYPE exp2 = mLinkP[i].m_dq/currRevoluteJoint->mFrictionCoefs[7];
                exp2 = exp(-exp2*exp2);
                f -= currRevoluteJoint->mFrictionCoefs[4] + currRevoluteJoint->mFrictionCoefs[5] * exp2;
                f += currRevoluteJoint->mFrictionCoefs[6] * mLinkP[i].m_dq;
            }else{
                if(mLinkP[i].m_dq>0.0){
                    f += 0.1 * mLinkP[i].m_dq/fThres * currRevoluteJoint->mFrictionCoefs[0];
                }else if(mLinkP[i].m_dq<0.0){
                    f += 0.1 * mLinkP[i].m_dq/fThres * currRevoluteJoint->mFrictionCoefs[4];
                }
                mLinkP[i].m_dq = 0;
                mLinkP[i].m_ddq = 0;
            }
            mLinkP[i].m_p += ez*f;
            //double f = -(0.15681 +0.75241*exp(-(mLinkP[i].m_dq/1.16341)*(mLinkP[i].m_dq/1.16341)))*RSIGN(mLinkP[i].m_dq) - 0.63643*mLinkP[i].m_dq;
            //double f = -(0.15681 +0.75241*exp(-(mLinkP[i].m_dq/1.16341)*(mLinkP[i].m_dq/1.16341)))*RSIGN(mLinkP[i].m_dq) - 0.63643*mLinkP[i].m_dq;
            //mLinkP[i].m_p -= ez*f;
            //if(ccnt==6)
                //cout <<f<<" "<< mLinkP[i].m_dq<<endl;
        }
        ccnt++;
        //cout << i <<" "<<currRevoluteJoint->mFrictionCoefs[1]<<endl;
        //mLinkP[i].m_p -= ez*(-currRevoluteJoint->mFrictionCoefs[1]*mLinkP[i].m_dq);
        //cout <<"FRICT "<<currRevoluteJoint->mFrictionCoefs[1]*mLinkP[i].m_dq<<endl;
/*        if(ccnt==3){
            if(fabs(mLinkP[i].m_dq)<5.0*PI/180.0){
                cout <<"Testing "<<ccnt<< " "<<zeroGTorques[ccnt]<<" "<<mLinkP[i].m_t<<endl;
                if(fabs(zeroGTorques[ccnt]-mLinkP[i].m_t) < currRevoluteJoint->mFrictionCoefs[0]){
                    cout <<"BUSTED"<<endl;
                    mLinkP[i].m_p -= ez*((zeroGTorques[ccnt]-mLinkP[i].m_t));
                    //mLinkP[i].m_p *= 0.0;//= ez*(-.m_dq);
                }else{
                    cout << endl;
                }
            }else{
                cout <<"Dynamic only "<<endl<<endl;
                mLinkP[i].m_p -= ez*(-currRevoluteJoint->mFrictionCoefs[1]*mLinkP[i].m_dq);
            }

        }*/
    }
  }
    int ccnt2=6;
  for(unsigned int i=mLinksCount-1;i>0;i--){

    if(mLinkP[i].m_bJoint){
      mLinkP[i].m_U = mLinkP[i].m_I * ez;
            //mLinkP[i].m_I.Print();
            //mLinkP[i].m_U.Print();
      mLinkP[i].m_D = ez.Dot(mLinkP[i].m_U);
      mLinkP[i].m_D = R_ONE / mLinkP[i].m_D;
        //cout << "asdasd "<<mLinkP[i].m_U.Dot(mLinkP[i].m_s)<<endl;
      mLinkP[i].m_u = mLinkP[i].m_t - mLinkP[i].m_U.Dot(mLinkP[i].m_s) - ez.Dot(mLinkP[i].m_p);
      if(ccnt2==3){
          //cout << "EEE "<<mLinkP[i].m_u<<" "<< zeroGTorques[ccnt2]<< " "<< mLinkP[i].m_t<<endl;
      }
      ccnt2--;
      //mLinkP[i].m_u = mLinkP[i].m_t - ez.Dot(mLinkP[i].m_p);
    }else{
      mLinkP[i].m_U.Zero();
      mLinkP[i].m_D = R_ZERO;
      mLinkP[i].m_u = R_ZERO;
    }

    (mLinkP[i].m_U* (-mLinkP[i].m_D)).MultTranspose(mLinkP[i].m_U,tmpM1);
    tmpM1+= mLinkP[i].m_I;

    mLinkP[parents[i]].m_I += mLinkP[i].m_Xt *tmpM1 * mLinkP[i].m_X;

    tmpV1 = mLinkP[i].m_p + mLinkP[i].m_I * mLinkP[i].m_s + mLinkP[i].m_U * mLinkP[i].m_D * mLinkP[i].m_u;
    mLinkP[parents[i]].m_p += mLinkP[i].m_Xt * tmpV1;
  }


  ccnt = 0;
  for(unsigned int i=1;i<mLinksCount;i++){
    pRevoluteJoint        currRevoluteJoint   = RevoluteJoint::Cast(mJoints->at(i));
    TVector<6> dqVec; dqVec.Zero();
    dqVec[2] = mLinkP[i].m_dq;

    mLinkP[i].m_a   = mLinkP[i].m_X * mLinkP[parents[i]].m_a;// + mLinkP[i].m_s;
    mLinkP[i].m_ddq = mLinkP[i].m_D*(mLinkP[i].m_u-mLinkP[i].m_U.Dot(mLinkP[i].m_a));
    if(ccnt == 3){
        //REALTYPE f = mLinkP[i].m_u-mLinkP[i].m_U.Dot(mLinkP[i].m_a);
        /*cout << f <<endl;
        if(fabs(mLinkP[i].m_dq)<15.0*PI/180.0){
            cout <<"Testing "<< f <<" "<<currRevoluteJoint->mFrictionCoefs[0]<<endl;
            if(fabs(f) < currRevoluteJoint->mFrictionCoefs[0]){
                cout <<"BUSTED "<<endl;
                //mLinkP[i].m_ddq = mLinkP[i].m_ddq;
            }
        }*/
    }
    if(mLinkP[i].m_bJoint){
        if(mLinkP[i].m_q<=currRevoluteJoint->mRange[0]){
            if(mLinkP[i].m_ddq < R_ZERO) mLinkP[i].m_ddq = 0;
            //cout << "MINQ: "<<i<<endl;
        }else if(mLinkP[i].m_q>=currRevoluteJoint->mRange[1]){
            if(mLinkP[i].m_ddq > R_ZERO) mLinkP[i].m_ddq = 0;
            //cout << "MAXQ: "<<i<<endl;
        }
    }
    ccnt++;

    mLinkP[i].m_a  += ez * mLinkP[i].m_ddq + mLinkP[i].m_s;
  }

  //cout << "Acc "; for(unsigned int i=0;i<mLinksCount;i++) cout << mLinkP[i].m_ddq << " "; cout << endl;

  int cnt = 0;
  for(unsigned int i=1;i<mLinksCount;i++){
    /*
    pLink                  currLink             = mLinks->at(i);
    pRevoluteJoint         currRevoluteJoint    = RevoluteJoint::Cast(mJoints->at(i));
    pJoint                 currJoint            = mJoints->at(i);
    pLink                  prevLink             = currJoint->mParentLink;
    pRevoluteJointSensor   currRevoluteSensor   = RevoluteJointSensor::Cast(currJoint->mSensor);
    pRevoluteJointActuator currRevoluteActuator = RevoluteJointActuator::Cast(currJoint->mActuator);
    */
    if(mLinkP[i].m_bJoint){
      mOutputAccel[cnt] = mLinkP[i].m_ddq;
      cnt++;
    }
  }

}

void    ForwardDynamics::GetAccelerations(Vector & output){
  output = mOutputAccel;
}

