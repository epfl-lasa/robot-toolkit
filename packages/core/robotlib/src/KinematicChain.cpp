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

#include "KinematicChain.h"


KinematicChain::KinematicChain(){
    mRobot                          = NULL;
    mLinks                          = NULL;
    mJoints                         = NULL;
    mParents                        = NULL;
    mLinksCount                     = 0;

    mRootLink                       = 0;
    mTargetLink                     = 0;
    mBaseLink                       = 0;

    mJacobianGradient               = NULL;
    mJacobianAxis                   = NULL;
    mJacobianPos                    = NULL;
}
KinematicChain::~KinematicChain(){
    if(mJacobianGradient != NULL)
        delete [] mJacobianGradient;
    mJacobianGradient = NULL;
    if(mJacobianAxis != NULL)
        delete [] mJacobianAxis;
    mJacobianAxis = NULL;
    if(mJacobianPos != NULL)
        delete [] mJacobianPos;
    mJacobianPos = NULL;

}

void KinematicChain::SetRobot(pRobot robot){
    if(mRobot==robot) return;

    mRobot = robot;
    if(mRobot!=NULL){
        mLinksCount     =  mRobot->GetLinksCount();
        mLinks          = &mRobot->GetLinks();
        mJoints         = &mRobot->GetJoints();
        mParents        = &mRobot->GetParents();

        mValidJoints.clear();
        for(unsigned int i=0;i<mLinksCount;i++){
            if(RevoluteJoint::Cast(mJoints->at(i)) || SliderJoint::Cast(mJoints->at(i)) ){
                mValidJoints.push_back(i);
            }
        }
    }else{
        mLinksCount     = 0;
        mLinks          = NULL;
        mJoints         = NULL;
        mParents        = NULL;
        mValidJoints.clear();
    }
}


void  KinematicChain::Create(int base, int root, int target){

    root    = TRUNC(root,0,mLinksCount-1);
    base    = TRUNC(base,-1,mLinksCount-1);
    target  = TRUNC(target,0,mLinksCount-1);

    if((mRootLink==root)&&(mTargetLink==target)&&(mBaseLink==base))
        return;

    mRootLink   = root;
    mTargetLink = target;
    mBaseLink   = base;

    mChain.clear();

    int currLink = mTargetLink;
    while((currLink!=mRootLink)&&(currLink>=0)){
        mChain.push_back(currLink);
        currLink = mParents->at(currLink);
    }

    if(currLink==mRootLink){
        mChain.push_back(currLink);
    }else{
        vector<int> backChain;
        bool stopFlag = false;
        currLink = mRootLink;

        while(!stopFlag){
            backChain.push_back(currLink);
            currLink = mParents->at(currLink);
            for(unsigned int i=0;i<mChain.size();i++){
                if(int(mChain[i])==currLink){
                    stopFlag = true;
                }
            }
        }
        if(backChain.size()>0){
            for(int i=int(backChain.size())-1;i>=0;i--){
                mChain.push_back(backChain[i]);
            }
        }
    }
    mChainSize = int(mChain.size());


    for(int i=0;i<mChainSize/2;i++){
        unsigned int tmp;
        tmp=mChain[i];
        mChain[i] = mChain[mChainSize-1-i];
        mChain[mChainSize-1-i] = tmp;
    }

    mJointMapping.clear();
    int size=0;

    for(int i=0;i<mChainSize;i++){
        unsigned int id=mLinksCount;
        for(unsigned int j=0;j<mValidJoints.size();j++){
            if(mChain[i]==mValidJoints[j]){
                id = j;
                break;
            }
        }
        if(id<mLinksCount){
            mJointMapping.push_back(id);
            size++;
        }
    }
    if(mJacobianGradient != NULL)
        delete [] mJacobianGradient;
    mJacobianGradient = new Matrix[mJointMapping.size()];
    if(mJacobianAxis != NULL)
        delete [] mJacobianAxis;
    mJacobianAxis = new Vector3[mJointMapping.size()];
    if(mJacobianPos != NULL)
        delete [] mJacobianPos;
    mJacobianPos = new Vector3[mJointMapping.size()];


    mChainPos.Resize(size,false); mChainPos.Zero();
    mChainVel.Resize(size,false); mChainVel.Zero();
    mChainAcc.Resize(size,false); mChainAcc.Zero();
}
int KinematicChain::GetTargetLinkId(){
    return mTargetLink;
}
/*
Vector& KinematicChain::GetJointAngles(Vector & result){
    result = mChainPos;
    return result;
}
Vector& KinematicChain::GetJointVelocity(Vector & result){
    result = mChainVel;
    return result;
}
Vector& KinematicChain::GetJointAcceleration(Vector & result){
    result = mChainAcc;
    return result;
}
Vector& KinematicChain::GetTargetPos(Vector & result){
    result = mTargetPos;
    return result;
}
Vector& KinematicChain::GetTargetVel(Vector & result){
    result = mTargetVel;
    return result;
}
Vector& KinematicChain::GetTargetAcc(Vector & result){
    result = mTargetAcc;
    return result;
}
Vector& KinematicChain::GetTargetAngularPos(Vector & result){
    result.Set(mTargetAPos);
    return result;
}
Vector& KinematicChain::GetTargetAngularVel(Vector & result){
    result.Set(mTargetAVel);
    return result;
}
Vector& KinematicChain::GetTargetAngularAcc(Vector & result){
    result.Set(mTargetAAcc);
    return result;
}
*/

int     KinematicChain::GetDOFCount(){
    return int(mJointMapping.size());
}

void  KinematicChain::BuildJacobian(){

     mJacobian.Resize(6,mJointMapping.size(),false);

    Vector3      pos;
    Vector3      axis;
    Vector3      effect;

    const Vector3& tPos = (mBaseLink>=0? mRobot->GetReferenceFrame(mTargetLink,mBaseLink).GetOrigin()
                                       : mRobot->GetReferenceFrame(mTargetLink).GetOrigin());

    int currId = 0;
    for(int i=0;i<mChainSize;i++){

        pRevoluteJoint rJoint = RevoluteJoint::Cast(mJoints->at(mChain[i]));
        if(rJoint){
            ReferenceFrame& ref  = (mBaseLink>=0? mRobot->GetReferenceFrame(mChain[i],mBaseLink)
                                                : mRobot->GetReferenceFrame(mChain[i]));
            const Vector3&        jPos = ref.GetOrigin();
            const Matrix3&        jOri = ref.GetOrient();
            tPos.Sub(jPos,pos);


            jOri.Mult(rJoint->GetAxis(),axis);

            if(mChain[i]>mChain[i+1])
                axis.SMinus();

            axis.Cross(pos,effect);

            mJacobianAxis[currId] = axis;
            mJacobianPos[currId]  = pos;

            mJacobian(0,currId) = effect.x();
            mJacobian(1,currId) = effect.y();
            mJacobian(2,currId) = effect.z();
            mJacobian(3,currId) = axis.x();
            mJacobian(4,currId) = axis.y();
            mJacobian(5,currId) = axis.z();

            currId++;
        }
        else
        {
        	pSliderJoint sJoint = SliderJoint::Cast(mJoints->at(mChain[i]));
        	if(sJoint)
        	{
        		  ReferenceFrame& ref  = (mBaseLink>=0? mRobot->GetReferenceFrame(mChain[i],mBaseLink)
        		                                                : mRobot->GetReferenceFrame(mChain[i]));
        		  	  	  	const Vector3&        jPos = ref.GetOrigin();
        		            const Matrix3&        jOri = ref.GetOrient();

        		            tPos.Sub(jPos,pos);
        		            jOri.Mult(sJoint->GetAxis(),axis);

        		            if(mChain[i]>mChain[i+1])
        		                axis.SMinus();


        		            mJacobianAxis[currId] = axis;
        		            mJacobianPos[currId]  = pos;

        		            mJacobian(0,currId) = axis.x();
        		            mJacobian(1,currId) = axis.y();
        		            mJacobian(2,currId) = axis.z();
        		            mJacobian(3,currId) = 0;
        		            mJacobian(4,currId) = 0;
        		            mJacobian(5,currId) = 0;

        		            currId++;
        	}
        }
    }
}

void    KinematicChain::BuildJacobianGradient(){


    Vector3     pPart;
    Vector3     oPart;

    Vector3     tmp;


    for(unsigned int j=0;j<mJointMapping.size();j++){
        mJacobianGradient[j].Resize(6,mJointMapping.size(),false);
        mJacobianGradient[j].Zero();
        for(unsigned int i=0;i<mJointMapping.size();i++){

            Vector3 daxis;
            Vector3 dpos;

            if(j<i){
                mJacobianAxis[j].Cross(mJacobianAxis[i],daxis);
                mJacobianAxis[j].Cross(mJacobianPos[i],dpos);
            }else{
                daxis.Zero();
                mJacobianAxis[j].Cross(mJacobianPos[j],dpos);
            }


            daxis.Cross(mJacobianPos[i],pPart);
            mJacobianAxis[i].Cross(dpos,tmp);

            pPart +=tmp;
            oPart = daxis;

            mJacobianGradient[j](0,i) = pPart.x();
            mJacobianGradient[j](1,i) = pPart.y();
            mJacobianGradient[j](2,i) = pPart.z();
            mJacobianGradient[j](3,i) = oPart.x();
            mJacobianGradient[j](4,i) = oPart.y();
            mJacobianGradient[j](5,i) = oPart.z();

        }
    }
}
Matrix& KinematicChain::GetJacobianGradient(int dof){
    return mJacobianGradient[dof];
}


void KinematicChain::Update(){
    BuildJacobian();

    /*
    mJacobian.Resize(jSize,mJacobian.ColumnSize(),false);    mJacobian.Zero();
    mJacobian.Resize(jSize,m2ndJacobian.ColumnSize(),false); m2ndJacobian.Zero();


    ReferenceFrame F_0N;
    ReferenceFrame PF_0N;
    ReferenceFrame F_tmp;

    Vector3 V_0N;
    Vector3 V_Loc;
    Vector3 V_Abs;
    Vector3 V_tmp;
    Vector3 V_axis;

    Vector3 VA_axis;

    Vector3 aV_0N;

    Vector3 PV_0N;
    Vector3 PaV_0N;

    Vector3 A_0N;
    Vector3 A_Loc;
    Vector3 A_Abs;
    Vector3 A_tmp;
    Vector3 A_axis;

    Vector3 aA_0N;

    F_0N.Set(mLinks->at(mChain[mChainSize-1])->mRefFrame);
    F_0N.Identity();
    V_0N.Zero();
    A_0N.Zero();

    aV_0N.Zero();
    aA_0N.Zero();

    int currId = mJacobian.ColumnSize()-1;
    for(int i=mChainSize-1;i>=1;i--){
        pLink                 currLink            = mLinks->at(mChain[i]) ;
        pJoint                currJoint           = mJoints->at(mChain[i]);
        pRevoluteJointSensor  currRevoluteSensor  = RevoluteJointSensor::Cast(currJoint->mSensor);
        ReferenceFrame& ref  = mBody->GetReferenceFrame(mChain[i],mBaseLink);

        // saving previous...
        PF_0N.Set(F_0N);
        PV_0N = V_0N;
        PaV_0N = aV_0N;

        // Position
        currLink->mRefFrame.Mult(F_0N, F_tmp);
        F_0N.Set(F_tmp);


        // Velocity
        if(currRevoluteSensor){
            V_axis = Vector3::EZ;
            V_axis.Cross(PF_0N.GetOrigin(),V_Loc);

            ref.GetOrient().Mult(V_Loc,V_Abs);
            ref.GetOrient().Mult(V_axis,VA_axis);

            mJacobian(0,currId) = V_Abs(0);
            mJacobian(1,currId) = V_Abs(1);
            mJacobian(2,currId) = V_Abs(2);
            mJacobian(3,currId) = VA_axis(0);
            mJacobian(4,currId) = VA_axis(1);
            mJacobian(5,currId) = VA_axis(2);

            V_Loc*= currRevoluteSensor->mVelocity;
            V_0N += V_Loc;


            aV_0N += V_axis *currRevoluteSensor->mVelocity;;

        }
        currLink->mRefFrame.GetOrient().Mult(V_0N,V_tmp);
        V_0N = V_tmp;

        currLink->mRefFrame.GetOrient().Mult(aV_0N,V_tmp);
        aV_0N = V_tmp;

        // Acc
        if(currRevoluteSensor){
            V_axis.Cross(PV_0N,A_Loc);
            A_Loc *= 2.0;

            V_axis.Cross(PF_0N.GetOrigin(),V_tmp);
            V_axis.Cross(V_tmp,A_tmp);
            A_tmp *= currRevoluteSensor->mVelocity;
            A_Loc += A_tmp;

            ref.GetOrient().Mult(A_Loc,A_Abs);
            m2ndJacobian(0,currId) = A_Abs(0);
            m2ndJacobian(1,currId) = A_Abs(1);
            m2ndJacobian(2,currId) = A_Abs(2);

            Vector3::EZ.Mult(currRevoluteSensor->mAcceleration,A_axis);
            A_axis.Cross(PF_0N.GetOrigin(),A_tmp);
            A_0N += A_tmp;

            A_Loc *= currRevoluteSensor->mVelocity;
            A_0N += A_Loc;

            V_axis.Cross(PaV_0N,A_Loc);
            aA_0N += A_Loc*currRevoluteSensor->mVelocity;

            aA_0N += A_axis;
        }
        currLink->mRefFrame.GetOrient().Mult(A_0N,A_tmp);
        A_0N = A_tmp;

        currLink->mRefFrame.GetOrient().Mult(aA_0N,A_tmp);
        aA_0N = A_tmp;

        if(currRevoluteSensor){
            currId--;
        }
    }



    int cnt=0;
    for(int i=0;i<mChainSize;i++){
        pRevoluteJoint rJoint = RevoluteJoint::Cast(mJoints->at(mChain[i]));
        if(rJoint){
            mChainPos(cnt) = ((pRevoluteJointSensor)rJoint->mSensor)->mPosition;
            mChainVel(cnt) = ((pRevoluteJointSensor)rJoint->mSensor)->mVelocity;
            mChainAcc(cnt) = ((pRevoluteJointSensor)rJoint->mSensor)->mAcceleration;
            cnt++;
        }
    }

    mTargetPos = F_0N.GetOrigin();
    mTargetVel = V_0N;
    mTargetAcc = A_0N;

    mTargetAPos = F_0N.GetOrient().GetExactRotationAxis();
    mTargetAVel = aV_0N;
    mTargetAAcc = aA_0N;
    */
}

Matrix& KinematicChain::GetJacobian(){
    return mJacobian;
}
IndicesVector& KinematicChain::GetJointMapping(){
    return mJointMapping;
}


KinematicChainIndicesManager::KinematicChainIndicesManager(){
}
KinematicChainIndicesManager::~KinematicChainIndicesManager(){
}
int KinematicChainIndicesManager::GetSize(){
    return mGlobalIndices.size();
}

void KinematicChainIndicesManager::AddKinematicChain(KinematicChain *kchain){

    mGlobalIndices.clear();
    mGlobalIndicesSet.insert(kchain->GetJointMapping().begin(),kchain->GetJointMapping().end());
    {
        set<unsigned int>::iterator it = mGlobalIndicesSet.begin();
        int cnt = 0;
        while(it != mGlobalIndicesSet.end()){
            mGlobalIndices.push_back(*it);
            mGlobalIndicesMap[*it] = cnt++;
            it++;
        }
    }
    mLocalIndices[kchain] = IndicesVector();
    {
        map<KinematicChain*,IndicesVector>::iterator it = mLocalIndices.begin();
        while(it != mLocalIndices.end()){
            it->second.clear();
            for(unsigned int i=0;i<it->first->GetJointMapping().size();i++){
                it->second.push_back(mGlobalIndicesMap[it->first->GetJointMapping()[i]]);
            }
            it++;
        }
    }
}
IndicesVector & KinematicChainIndicesManager::GetLocalIndices(KinematicChain *kchain){
    return mLocalIndices[kchain];
}
IndicesVector & KinematicChainIndicesManager::GetGlobalIndices(){
    return mGlobalIndices;
}

