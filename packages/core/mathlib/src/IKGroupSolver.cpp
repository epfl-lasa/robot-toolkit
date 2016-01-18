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



QPSolver::QPSolver(): mMultiplierRotationJ(1){
}
QPSolver::~QPSolver(){}


Vector QPSolver::getOutput()const{
	return mu.GetSubVector(0,mDOF);
}

void QPSolver::setDimensions(unsigned int DOF, unsigned int EndEffectorDim){
	mDOF =DOF;
	mEndEffectorDim = EndEffectorDim;
	mu.Resize(DOF+EndEffectorDim);
	mJ = Matrix(mEndEffectorDim, mDOF);
}


void QPSolver::setW(Matrix const& W){
	mW = W;
}

void QPSolver::setMultiplierRotationJ(double MultiplierRotationJ){
	mMultiplierRotationJ = MultiplierRotationJ;
}


void QPSolver::setJacobian(Matrix const& J){
	mJ = J;
	for(unsigned int i(0); i<3; ++i){
		for(unsigned int j(0); j<mDOF; ++j){
			mJ(i,j)*= mMultiplierRotationJ;
		}
	}
}


void QPSolver::setDeltaT(double const& dt){
	mDeltaT = dt;
}


void QPSolver::setGamma(double const& gamma){
	mGamma = gamma;
}

Vector QPSolver::POmega(Vector handle){
	Vector XiM = XiMinus();
	Vector XiP = XiPlus();

	for(unsigned int i(0); i<mDOF; ++i){
		if(handle[i] < XiM[i]){
			handle[i] = XiM[i];
		}
		if(handle[i] > XiP[i]){
			handle[i] = XiP[i];
		}
	}
	return handle;
}




Vector QPSolver::q(){
	Vector q(mDOF+mEndEffectorDim);
	Vector b=mb();
	Vector d=md();

	for(unsigned int i(0); i<mDOF; ++i){
		q[i] = b[i];
	}
	for(unsigned int i(mDOF); i<mEndEffectorDim; ++i){
		q[i] = -d[i];
	}
	return q;
}


void QPSolver::Solve(Vector y){

	//Updates u
	for(unsigned int i(0); i<mEndEffectorDim; ++i){
		mu[mDOF + i] = y[i];
	}

	//creates the M matrix
	Matrix M(mDOF + mEndEffectorDim,mDOF + mEndEffectorDim);

	Matrix tmp_mJT = -mJ.Transpose();

	for(unsigned int i(0); i < mDOF + mEndEffectorDim; ++i){
		for(unsigned int j(0); j < mDOF + mEndEffectorDim; ++j){
			if(i<mDOF && j<mDOF){
				M(i,j)=mW(i,j);
			}
			else if(i>=mDOF && j<mDOF){
				M(i,j) = mJ(i-mDOF,j);
			}
			else if(i<mDOF && j>=mDOF){
				M(i,j) = tmp_mJT(i,j-mDOF);
			}
		}
	}

	Vector handle =  mu - (M*mu + q());

	Matrix I(M);
	I.Identity();

	Matrix tmpSum= I + M.Transpose();
	Vector tmpVect = POmega(handle)-mu;


	Vector udot = tmpSum*tmpVect;
	udot = udot.Mult(mGamma,udot);

	mu = mu + udot * mDeltaT;

}

MANSolver::MANSolver(){}
MANSolver::~MANSolver(){}


void MANSolver::setW(){
	Matrix I(mDOF,mDOF);
}

void MANSolver::setConstraints(Vector ThetaMinus, Vector ThetaPlus, Vector ThetaDotMinus, Vector ThetaDotPlus, Vector Theta2Minus, Vector Theta2Plus){
	mThetaMinus = ThetaMinus;
	mThetaPlus = ThetaPlus;
	mThetaDotMinus = ThetaDotMinus;
	mThetaDotPlus = ThetaDotPlus;
	mTheta2Minus = Theta2Minus;
	mTheta2Plus = Theta2Plus;
}

void MANSolver::setMuP(double const& mup){
	mMuP = mup;
}

void MANSolver::setEtaP(double const& etap){
	mEtaP = etap;
}

void MANSolver::setMuV(double const& muv){
	mMuV = muv;
}

void MANSolver::SetCurrent(Vector Theta, Vector ThetaDot){
	mCurrentTheta = Theta;
	mCurrentThetaDot = ThetaDot;
}

void MANSolver::Solve(Vector rdot, Vector Theta, Vector ThetaDot){
	SetCurrent(Theta, ThetaDot);
	QPSolver::Solve(rdot);
}

Vector MANSolver::XiMinus(){
	Vector tempMinus = (mThetaMinus*mEtaP - mCurrentTheta)*mMuP;
	Vector tempMinusDot = (mThetaDotMinus - mCurrentThetaDot)*mMuV;

	Vector XiMinus(mDOF + mEndEffectorDim);

	for (unsigned int i(0); i< mDOF; ++i){
		if(tempMinus[i] > tempMinusDot[i]){
			XiMinus[i] =tempMinus[i];
		}
		else{
			XiMinus[i] =tempMinusDot[i];
		}
		if(mTheta2Minus[i] > XiMinus[i]){
			XiMinus[i] =mTheta2Minus[i];
		}
	}
	return XiMinus;
}

Vector MANSolver::XiPlus(){
	Vector tempPlus = (mThetaPlus*mEtaP - mCurrentTheta)*mMuP;
	Vector tempPlusDot = (mThetaDotPlus - mCurrentThetaDot)*mMuV;

	Vector XiPlus(mDOF + mEndEffectorDim);

	for (unsigned int i(0); i< mDOF; ++i){
		if(tempPlus[i] < tempPlusDot[i]){
			XiPlus[i] =tempPlus[i];
		}
		else{
			XiPlus[i] =tempPlusDot[i];
		}
		if(mTheta2Plus[i] < XiPlus[i]){
			XiPlus[i] =mTheta2Plus[i];
		}
	}
	return XiPlus;
}

Vector MANSolver::mb(){
	Vector b(mDOF);
	return b;
}


Vector MANSolver::md(){
	Vector d(mEndEffectorDim);

	Matrix Jdot = ((mJ - mOldJ)*(1./mDeltaT));

	d=mu.GetSubVector(mDOF, mEndEffectorDim) - Jdot * mCurrentThetaDot;
	return d;
}

void MANSolver::setJacobian(Matrix const& J){
	mOldJ = mJ;
	QPSolver::setJacobian(J);
}





MKESolver::MKESolver():QPSolver(){}
MKESolver::~MKESolver(){}



void MKESolver::setH(Matrix const& H){
	setW(H);
}


void MKESolver::setConstraints(Vector ThetaMinus, Vector ThetaPlus, Vector ThetaDotMinus, Vector ThetaDotPlus){
	mThetaMinus = ThetaMinus;
	mThetaPlus = ThetaPlus;
	mThetaDotMinus = ThetaDotMinus;
	mThetaDotPlus = ThetaDotPlus;
}

void MKESolver::SetCurrentTheta(Vector Theta){
	mCurrentTheta= Theta;
}

void MKESolver::setMuP(double const& mup){
	mMuP = mup;
}

Vector MKESolver::XiMinus(){
	Vector tempMinus = (mThetaMinus - mCurrentTheta)*mMuP;
	Vector XiMinus(mDOF + mEndEffectorDim);

	for (unsigned int i(0); i< mDOF; ++i){
		if(tempMinus[i] > mThetaDotMinus[i]){
			XiMinus[i] =tempMinus[i];
		}
		else{
			XiMinus[i] =mThetaDotMinus[i];
		}
	}
	return XiMinus;
}

Vector MKESolver::XiPlus(){
	Vector tempPlus = (mThetaPlus - mCurrentTheta)*mMuP;
	Vector XiPlus(mDOF + mEndEffectorDim);

	for (unsigned int i(0); i< mDOF; ++i){
		if(tempPlus[i] < mThetaDotPlus[i]){
			XiPlus[i] =tempPlus[i];
		}
		else{
			XiPlus[i] =mThetaDotPlus[i];
		}
	}
	return XiPlus;
}

Vector MKESolver::mb(){
	Vector b(mDOF);
	return b;
}


Vector MKESolver::md(){
	Vector d(mEndEffectorDim);
	d=mu.GetSubVector(mDOF, mEndEffectorDim);
	return d;
}



void MKESolver::Solve(Vector rdot, Vector Theta){
	SetCurrentTheta(Theta);
	QPSolver::Solve(rdot);

}














/*

MKESolver::MKESolver(): mMultiplierRotationJ(1){}
MKESolver::~MKESolver(){}


void MKESolver::setDimensions(unsigned int DOF, unsigned int EndEffectorDim){
	mDOF =DOF;
	mEndEffectorDim = EndEffectorDim;
	mu.Resize(DOF+EndEffectorDim);
}

void MKESolver::setH(Matrix const& H){
	mH = H;
}

void MKESolver::setMultiplierRotationJ(double MultiplierRotationJ){
	mMultiplierRotationJ = MultiplierRotationJ;
}


void MKESolver::setJacobian(Matrix const& J){
	mJ = J;
	for(unsigned int i(0); i<3; ++i){
		for(unsigned int j(0); j<mDOF; ++j){
			mJ(i,j)*= mMultiplierRotationJ;
		}
	}
}

void MKESolver::setConstraints(Vector ThetaMinus, Vector ThetaPlus, Vector ThetaDotMinus, Vector ThetaDotPlus){
	mThetaMinus = ThetaMinus;
	mThetaPlus = ThetaPlus;
	mThetaDotMinus = ThetaDotMinus;
	mThetaDotPlus = ThetaDotPlus;
}

void MKESolver::setDeltaT(double const& dt){
	mDeltaT = dt;
}

void MKESolver::setMuP(double const& mup){
	mMuP = mup;
}


void MKESolver::setGamma(double const& gamma){
	mGamma = gamma;
}



Vector MKESolver::getOutput()const{
	return mu.GetSubVector(0,mDOF);
}


void MKESolver::Solve(Vector rdot, Vector Theta){

	//creates the vector q and update u
	Vector q(mDOF + mEndEffectorDim);
	for(unsigned int i(0); i<mEndEffectorDim; ++i){
		mu[mDOF + i] = rdot[i];
		q[mDOF + i] = -rdot[i];
	}

	//creates the M matrix
	Matrix M(mDOF + mEndEffectorDim,mDOF + mEndEffectorDim);

	Matrix tmp_mJT = -mJ.Transpose();


	for(unsigned int i(0); i < mDOF + mEndEffectorDim; ++i){
		for(unsigned int j(0); j < mDOF + mEndEffectorDim; ++j){
			if(i<mDOF && j<mDOF){
				M(i,j)=mH(i,j);
			}
			else if(i>=mDOF && j<mDOF){
				M(i,j) = mJ(i-mDOF,j);
			}
			else if(i<mDOF && j>=mDOF){
				M(i,j) = tmp_mJT(i,j-mDOF);
			}
		}
	}


//	M.InsertSubMatrix(0,0,mH,0,mDOF, 0,mDOF);
//	M.InsertSubMatrix(0,mDOF, -mJ.Transpose(), 0, mDOF, mDOF, mEndEffectorDim);
//	M.InsertSubMatrix(mDOF, 0, mJ, mDOF, mEndEffectorDim, 0,mDOF);


	Vector handle =  mu - (M*mu + q);

	Matrix I(M);
	I.Identity();

	Matrix tmpSum= I + M.Transpose();
	//Vector tmpVect = POmega(handle, Theta)-mu; //////////////////////////////////////////////////////////////////////////////////////////
	Vector tmpVect = handle-mu;

	Vector udot = tmpSum*tmpVect;
	udot = udot.Mult(mGamma,udot);

	mu = mu + udot * mDeltaT;

}


Vector MKESolver::POmega(Vector handle, Vector const& Theta) {
	Vector tempMinus = (mThetaMinus - Theta)*mMuP;
	Vector XiMinus(mDOF + mEndEffectorDim);

	Vector tempPlus = (mThetaPlus - Theta)*mMuP;
	Vector XiPlus(mDOF + mEndEffectorDim);
	for (unsigned int i(0); i< mDOF; ++i){
		if(tempMinus[i] > mThetaDotMinus[i]){
			XiMinus[i] =tempMinus[i];
		}
		else{
			XiMinus[i] =mThetaDotMinus[i];
		}

		if(tempPlus[i] < mThetaDotPlus[i]){
			XiPlus[i] =tempPlus[i];
		}
		else{
			XiPlus[i] =mThetaDotPlus[i];
		}
	}
	for(unsigned int i(mDOF); i < mDOF+ mEndEffectorDim; ++i){
		XiPlus[i]= 1e30;
		XiMinus[i]=-1e30;
	}

	for(unsigned int i(0); i < mDOF + mEndEffectorDim;++i){
		if(handle[i] < XiMinus[i]){
			handle[i] = XiMinus[i];
		}
		if(handle[i] > XiPlus[i]){
			handle[i] = XiPlus[i];
		}
	}
	return handle;
}



*/



