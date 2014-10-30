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

#include "IKGroupSolver.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

IKGroupSolver::IKGroupSolver(){
	mIKItems.clear();
	mSortedPriorityIds.clear();

	mConstraintsSize    = 0;
	mDofs               = 0;
	bVerbose            = false;
	bComputePriorities  = false;
	bUseNullTarget		= false;
}

IKGroupSolver::~IKGroupSolver(){
	mIKItems.clear();
	mSortedPriorityIds.clear();
}

void    IKGroupSolver::SetVerbose(bool verbose){
	bVerbose = verbose;
	for(size_t i=0;i<mIKItems.size();i++){
		IKSolverItem &item = mIKItems[i];
		item.mSolver.SetVerbose(verbose);
	}
}
void    IKGroupSolver::SetSizes(int dofs){
	mDofs = MAX(0,dofs);
	Resize();
}
int     IKGroupSolver::GetDOFSize(){
	return mDofs;
}
int     IKGroupSolver::AddSolverItem(const int constraintsSize){
	mIKItems.resize(mIKItems.size()+1);
	IKSolverItem &item = mIKItems[mIKItems.size()-1];

	item.mSolver.SetSizes(mDofs,constraintsSize);
	item.mDofsIndex.clear();
	item.mPriority = 0;
	item.mDesiredTarget.Resize(constraintsSize,false);
	item.mDesiredTarget.Zero();
	item.mOutputTarget.Resize(constraintsSize,false);
	item.mOutputTarget.Zero();
	item.mActualTarget.Resize(constraintsSize,false);
	item.mActualTarget.Zero();
	item.mErrorTarget.Resize(constraintsSize,false);
	item.mErrorTarget.Zero();

	item.mOutput.Resize(mDofs,false);
	item.mOutput.Zero();
	item.bEnabled   = true;
	item.bSuspended = false;
	bComputePriorities = true;

	return int(mIKItems.size())-1;
}

void    IKGroupSolver::SetDofsIndices(const vector<unsigned int> & dofsIndex, int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		item.mDofsIndex.clear();
		for(size_t i=0;i<dofsIndex.size();i++)
			item.mDofsIndex.push_back(dofsIndex[i]);
	}
}
void    IKGroupSolver::SetPriority(int priority, int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		item.mPriority = priority;
		bComputePriorities = true;
	}
}

void    IKGroupSolver::SetThresholds(REALTYPE loose, REALTYPE cut, int solverId){
	if(solverId<0){
		for(size_t i=0;i<mIKItems.size();i++){
			mIKItems[i].mSolver.SetThresholds(loose,cut);
		}
	}else if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		item.mSolver.SetThresholds(loose,cut);
	}
}

void    IKGroupSolver::SetJacobian(const Matrix & j, int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		if(item.mDofsIndex.size()>0){
			item.mSolver.GetJacobian().Zero();
			item.mSolver.GetJacobian().SetColumnSpace(item.mDofsIndex, j);
			//item.mSolver.GetJacobian().Print();
		}else
			item.mSolver.SetJacobian(j);
	}
}
Matrix * IKGroupSolver::GetJacobian(int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		return &item.mSolver.GetJacobian();
	}
	return NULL;
}
void    IKGroupSolver::SetConstraintsWeights(Matrix &m, int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		item.mSolver.SetConstraintsWeights(m);
	}
}
void    IKGroupSolver::SetConstraintsWeights(Vector &v, int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		item.mSolver.SetConstraintsWeights(v);
	}
}
void    IKGroupSolver::SetTarget(const Vector &v, int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		item.mDesiredTarget = v;
	}
}
void    IKGroupSolver::SetNullTarget(const Vector &null){
	mNullTarget.Zero();
	mNullTarget.SetSubVector(0,null);
	bUseNullTarget = true;
}
void	IKGroupSolver::RemoveNullTarget(){
	bUseNullTarget = false;
}
void    IKGroupSolver::Enable(bool enable, int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		if(item.bEnabled != enable){
			bComputePriorities  = true;
			item.bEnabled       = enable;
		}
		if(enable){
			if(item.bSuspended){
				bComputePriorities  = true;
				item.bSuspended     = false;
			}
		}else{
			if(!item.bSuspended){
				bComputePriorities  = true;
				item.bSuspended     = true;
			}
		}
	}
}
bool    IKGroupSolver::IsEnabled(int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		return item.bEnabled;
	}
	return false;
}

void    IKGroupSolver::Suspend(bool suspend, int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		if(item.bSuspended != suspend){
			bComputePriorities = true;
			item.bSuspended = suspend;
		}
	}
}


void    IKGroupSolver::ComputePriorities(){
	if(bVerbose) cerr << "IKGroupSolver: Computing Priorities"<<endl;
	mSortedPriorityIds.clear();
	for(size_t i=0;i<mIKItems.size();i++){
		if((mIKItems[i].bEnabled)&&(!mIKItems[i].bSuspended))
			mSortedPriorityIds.push_back(i);
	}
	for(size_t i=0;i<mSortedPriorityIds.size();i++){
		for(size_t j=0;j<i;j++){
			if(mIKItems[mSortedPriorityIds[i]].mPriority<mIKItems[mSortedPriorityIds[j]].mPriority){
				int tmp               = mSortedPriorityIds[i];
				mSortedPriorityIds[i] = mSortedPriorityIds[j];
				mSortedPriorityIds[j] = tmp;
			}
		}
	}
	if(bVerbose){
		for(size_t i=0;i<mSortedPriorityIds.size();i++){
			cerr << "  "<<i<<": Solver <"<<mSortedPriorityIds[i]<<"> "<<mIKItems[mSortedPriorityIds[i]].mPriority<<endl;
		}
	}

	bComputePriorities = false;
}

/*void    IKGroupSolver::SetDofsWeights(Matrix &m){
    mDofsWeights = m;
}*/
void    IKGroupSolver::SetDofsWeights(Vector &v){
	mDofsWeights.Resize(mDofs,mDofs,false);
	mDofsWeights.Zero();
	mInvDofsWeights.Resize(mDofs,mDofs,false);
	mInvDofsWeights.Zero();
	int len = MIN(int(v.Size()),mDofs);
	for(int i=0;i<len;i++){
		mDofsWeights(i,i) = v(i);
		if(v(i)>1e-6)
			mInvDofsWeights(i,i) = 1.0/v(i);
	}
}

void    IKGroupSolver::Solve(){
	if(bVerbose) cerr << "IKGroupSolver: Solving"<<endl;

	if(bComputePriorities)
		ComputePriorities();

	mCurrDofsWeights  = mDofsWeights;

	mCurrLimits[0] = mLimits[0];
	mCurrLimits[1] = mLimits[1];


	//int cCnt;

	// Compute constraints size
	mConstraintsSize = 0;
	for(size_t i=0;i<mSortedPriorityIds.size();i++){
		IKSubSolver& cSolver = mIKItems[mSortedPriorityIds[i]].mSolver;
		mConstraintsSize += cSolver.mConstraintsSize;
	}

	// Set target vectors
	for(size_t i=0;i<mSortedPriorityIds.size();i++){
		IKSolverItem &item = mIKItems[mSortedPriorityIds[i]];
		item.mSolver.SetTarget(item.mDesiredTarget);
	}

	mOutput.Zero();
	mCurrNullTarget = mNullTarget;

	// Check initial limits
	bool bHasInitialLimits = false;
	mLimitsOffset.Zero();
	for(int i=0;i<mDofs;i++){
		if(mCurrLimits[0][i]<=mCurrLimits[1][i]){
			if(mCurrLimits[0][i]>0.0){
				mCurrLimits[1][i]  -= mCurrLimits[0][i];
				mLimitsOffset[i]    = mCurrLimits[0][i];
				mCurrLimits[0][i]   = 0.0;
				bHasInitialLimits   = true;
			}else if(mCurrLimits[1][i]<0.0){
				mCurrLimits[0][i]  -= mCurrLimits[1][i];
				mLimitsOffset[i]    = mCurrLimits[1][i];
				mCurrLimits[1][i]   = 0.0;
				bHasInitialLimits   = true;
			}
		}
	}


	//Vector tmpCsV(mConstraintsSize);
	//Vector tmpDsV(mDofs);
	if(bUseNullTarget)
	{
		// Apply null space opt first
		for(size_t i=0;i<mSortedPriorityIds.size();i++){
			IKSubSolver& cSolver = mIKItems[mSortedPriorityIds[i]].mSolver;
			Vector nt;
			cSolver.mJacobian.Mult(mNullTarget,nt);
			cSolver.mDesiredTarget  -= nt;
		}
		mOutput += mNullTarget;
	}

	// Apply initial limits
	if(bHasInitialLimits){
		mLimitsOffsetTarget.Zero();
		for(size_t i=0;i<mSortedPriorityIds.size();i++){
			IKSubSolver& cSolver = mIKItems[mSortedPriorityIds[i]].mSolver;
			cSolver.mJacobian.Mult(mLimitsOffset,mLimitsOffsetTarget);
			cSolver.mDesiredTarget  -= mLimitsOffsetTarget;
		}
		mOutput         += mLimitsOffset;
		mCurrNullTarget -= mLimitsOffset;
	}

	int stepCnt =0;
	while(1){
		if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<">"<<endl;

		mStepOutput.Zero();

		mCurrWeights   = mCurrDofsWeights;

		// Solve each sub system
		for(size_t i=0;i<mSortedPriorityIds.size();i++){
			IKSolverItem &item = mIKItems[mSortedPriorityIds[i]];
			item.mActualTarget = item.mSolver.mDesiredTarget;
			if(i>0){
				//item.mSolver.mDesiredTarget -= item.mSolver.mJacobian * mStepOutput;
				item.mSolver.mJacobian.Mult(mStepOutput,tmpCsV);
				item.mSolver.mDesiredTarget -= tmpCsV;
			}
			item.mSolver.SetDofsWeights(mCurrWeights);
			item.mSolver.Solve();
			// Retrieve each sub output
			mStepOutput += item.mSolver.mOutput;

			tmpMat.Set(mCurrWeights.Array(),mDofs,mDofs);
			//Matrix tmp(mCurrWeights);
			tmpMat.Mult(item.mSolver.GetNullSpace(),mCurrWeights);

			item.mSolver.mDesiredTarget = item.mActualTarget;
		}
		/*
        //if(stepCnt ==0){
            mCurrWeights.Transpose(mCurrWeightsTranspose);

            mInvDofsWeights.Mult(mCurrNullTarget,tmpDsV);
            mInvDofsWeights.Mult(tmpDsV,tmpDsV2);
            //tmpDsV2 = mCurrNullTarget;
            mCurrWeightsTranspose.Mult(tmpDsV2,tmpDsV);
            mCurrWeights.Mult(tmpDsV,tmpDsV2);
            if(bVerbose){
                cerr << mStepOutput<<endl;
            }
            mStepOutput+=tmpDsV2;
            if(bVerbose){
                cerr << "IKGroupSolver: Null Pass <"<< stepCnt <<" "<<endl;
                cerr << tmpDsV2<<endl;
                cerr << mCurrNullTarget<<endl;
                cerr << mStepOutput<<endl;
            }
        //}
		 */

		// Checking Limits
		for(int i=0;i<mDofs;i++){
			mOutputLimitsError[i] = 1.0;
			if(mCurrLimits[0][i]<=mCurrLimits[1][i]){
				if(mStepOutput[i]<mCurrLimits[0][i])
					mOutputLimitsError[i] = fabs(mCurrLimits[0][i]/mStepOutput[i]);
				else if(mStepOutput[i]>mCurrLimits[1][i])
					mOutputLimitsError[i] = fabs(mCurrLimits[1][i]/mStepOutput[i]);
			}
		}
		double minOutputLimitError=1.0;
		int minId=-1;
		for(int i=0;i<mDofs;i++){
			if(minOutputLimitError>mOutputLimitsError[i]){
				minOutputLimitError = mOutputLimitsError[i];
				minId = i;
			}
		}
		if(minId>=0){
			if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<"> Limits reached..."<<endl;
			for(int i=0;i<mDofs;i++){
				if(minOutputLimitError == mOutputLimitsError[i]){
					if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<"> Locking DOF <"<<i<<">"<<endl;
					for(int j=0;j<mDofs;j++){
						mCurrDofsWeights(i,j)   = 0.0;
					}

				}
			}
		}

		mStepOutput *= minOutputLimitError;
		for(int i=0;i<mDofs;i++){
			if(mCurrLimits[0][i]<=mCurrLimits[1][i]){
				if     (mStepOutput[i]>0) mCurrLimits[1][i] -= mStepOutput[i];
				else if(mStepOutput[i]<0) mCurrLimits[0][i] -= mStepOutput[i];
				mCurrLimits[1][i] = MAX(0,mCurrLimits[1][i]);
				mCurrLimits[0][i] = MIN(0,mCurrLimits[0][i]);
			}
		}

		if(minOutputLimitError==1.0){
			mOutput += mStepOutput;
			if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<"> Success"<<endl;
			break;
		}else{
			stepCnt ++;
			for(size_t i=0;i<mSortedPriorityIds.size();i++){
				IKSolverItem &item = mIKItems[mSortedPriorityIds[i]];
				item.mSolver.mJacobian.Mult(mStepOutput,tmpCsV);
				item.mSolver.mDesiredTarget -= tmpCsV;
			}
			// Update desired target
			mOutput += mStepOutput;

			if(bUseNullTarget)
			{
				if(stepCnt ==0){
					mCurrNullTarget += mStepOutput;
				}
			}

		}
	}

	//mNullTarget.Sub(mOutput,mCurrNullTarget);
	mCurrWeights.Transpose(mCurrWeightsTranspose);

	bool bNullOptDone = false;
	while(!bNullOptDone){
		mInvDofsWeights.Mult(mCurrNullTarget,tmpDsV);
		mInvDofsWeights.Mult(tmpDsV,tmpDsV2);
		mCurrWeightsTranspose.Mult(tmpDsV2,tmpDsV);
		mCurrWeights.Mult(tmpDsV,tmpDsV2);
		mStepOutput=tmpDsV2;
		for(int i=0;i<mDofs;i++){
			if(fabs(mStepOutput[i])<1e-6) mStepOutput[i] = 0.0;
		}
		/*cout << mCurrNullTarget<<endl;
        cout << mStepOutput<<endl;
        cout << mCurrLimits[0]<<endl;
        cout << mCurrLimits[1]<<endl;
		 */
		for(int i=0;i<mDofs;i++){
			mOutputLimitsError[i] = 1.0;
			if(mCurrLimits[0][i]<=mCurrLimits[1][i]){
				if(mStepOutput[i]<mCurrLimits[0][i]){
					mOutputLimitsError[i] = fabs(mCurrLimits[0][i]/mStepOutput[i]);
					if(bVerbose) cerr << "IKGroupSolver: Null Space Pass: Locking DOF <"<<i<<">"<<endl;
				}
				else if(mStepOutput[i]>mCurrLimits[1][i]){
					mOutputLimitsError[i] = fabs(mCurrLimits[1][i]/mStepOutput[i]);
					if(bVerbose) cerr << "IKGroupSolver: Null Space Pass: Locking DOF <"<<i<<">"<<endl;
				}
			}
		}

		double minOutputLimitError=1.0;
		int minId=-1;
		for(int i=0;i<mDofs;i++){
			if(minOutputLimitError>mOutputLimitsError[i]){
				minOutputLimitError = mOutputLimitsError[i];
				minId = i;
			}
		}

		if(minId>=0){
			//cout << "Null factor: "<<minOutputLimitError<<endl;
			//if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<"> Limits reached..."<<endl;
			for(int i=0;i<mDofs;i++){
				if(minOutputLimitError == mOutputLimitsError[i]){
					//if(bVerbose) cerr << "IKGroupSolver: Pass <"<< stepCnt <<"> Locking DOF <"<<i<<">"<<endl;
					for(int j=0;j<mDofs;j++){
						mCurrDofsWeights(i,j)   = 0.0;
					}

				}
			}
		}
		//    d = W*n
		//mStepOutput *= minOutputLimitError;


		//mStepOutput ^= mOutputLimitsError;
		for(int i=0;i<mDofs;i++){
			if(mCurrLimits[0][i]<=mCurrLimits[1][i]){
				if     (mStepOutput[i]>0) mCurrLimits[1][i] -= mStepOutput[i];
				else if(mStepOutput[i]<0) mCurrLimits[0][i] -= mStepOutput[i];
				mCurrLimits[1][i] = MAX(0,mCurrLimits[1][i]);
				mCurrLimits[0][i] = MIN(0,mCurrLimits[0][i]);
			}
		}

		mCurrNullTarget -= mStepOutput;
		mOutput += mStepOutput;
		//cout << mStepOutput<<endl;
		if(minId<0){
			bNullOptDone = true;
		}
	}

	if(bVerbose) cerr << "IKGroupSolver: Done"<<endl;
}
void    IKGroupSolver::ClearLimits(){
	mLimits[0].Zero();  mLimits[0] += 1.0;
	mLimits[1].Zero();  mLimits[1] -= 1.0;
}

void    IKGroupSolver::SetLimits(const Vector &low,const Vector &high){
	//int len;
	mLimits[0].Zero();
	mLimits[0].SetSubVector(0,low);
	mLimits[1].Zero();
	mLimits[1].SetSubVector(0,high);
}
Vector&     IKGroupSolver::GetOutput(){
	return mOutput;
}
Vector&     IKGroupSolver::GetTargetError(int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		//item.mOutputTarget = item.mSolver.mJacobian * mOutput;
		item.mSolver.mJacobian.Mult(mOutput,item.mOutputTarget);
		//item.mErrorTarget  = item.mDesiredTarget - item.mOutputTarget;
		item.mDesiredTarget.Sub(item.mOutputTarget,item.mErrorTarget);
		return item.mErrorTarget;
	}
	return mIKItems[0].mErrorTarget;
}
Vector&     IKGroupSolver::GetTargetOutput(int solverId){
	if((solverId>=0)&&(solverId<int(mIKItems.size()))){
		IKSolverItem &item = mIKItems[solverId];
		//item.mOutputTarget = item.mSolver.mJacobian * mOutput;
		item.mSolver.mJacobian.Mult(mOutput,item.mOutputTarget);
		//item.mErrorTarget  = item.mDesiredTarget - item.mOutputTarget;
		item.mDesiredTarget.Sub(item.mOutputTarget,item.mErrorTarget);
		return item.mOutputTarget;
	}
	return mIKItems[0].mOutputTarget;
}
REALTYPE    IKGroupSolver::GetTargetErrorNorm(){
	return 0;
}
REALTYPE    IKGroupSolver::GetTargetErrorNorm2(){
	return 0;
}

void IKGroupSolver::Resize(){  

	mLimits[0].Resize(mDofs);
	mLimits[1].Resize(mDofs);
	mLimitsOffset.Resize(mDofs);

	mDofsWeights.Resize(mDofs,mDofs,false);
	mInvDofsWeights.Resize(mDofs,mDofs,false);
	mCurrDofsWeights.Resize(mDofs,mDofs,false);

	mCurrWeightsPtr.Resize(10*mDofs,10*mDofs,false);
	mCurrWeights.SetSharedPtr(mCurrWeightsPtr.Array(),mDofs*10,mDofs*10);


	mCurrWeightsTransposePtr.Resize(10*mDofs,10*mDofs,false);
	mCurrWeightsTranspose.SetSharedPtr(mCurrWeightsTransposePtr.Array(),mDofs*10,mDofs*10);

	mDofsWeights.Identity();
	mInvDofsWeights.Identity();

	mOutputLimitsError.Resize(mDofs,false);

	mNullTarget.Resize(mDofs,false);
	mNullTarget.Zero();
	mCurrNullTarget.Resize(mDofs,false);
	mCurrNullTarget.Zero();

	mOutput.Resize(mDofs,false);
	mStepOutput.Resize(mDofs,false);
	mOutputOffset.Resize(mDofs,false);
	mStepOutput.Resize(mDofs,false);

	tmpMat.Resize(mDofs,mDofs,false);

	tmpCsV.Resize(mConstraintsSize,false);
	tmpDsVPtr.Resize(mDofs,false);
	tmpDsV2Ptr.Resize(mDofs,false);

	tmpDsV.SetSharedPtr(tmpDsVPtr.Array(),mDofs);
	tmpDsV2.SetSharedPtr(tmpDsV2Ptr.Array(),mDofs);


}
