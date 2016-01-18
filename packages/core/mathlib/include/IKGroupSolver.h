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

#ifndef IKGROUPSOLVER_H_
#define IKGROUPSOLVER_H_

#include "MathLib.h"
#include "IKSubSolver.h"

//#include "RobotLib/ForwardDynamics.h"
//#include "RobotLib/InverseDynamics.h"



#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

    
/**
 * \class IKGroupSolver
 * 
 * \brief A class for solving inverse kinematic in a pseudo-inverse fashion with optimization
 * 
 * This is a loose inverse kinematic algorithm... By loose, I mean that constraint are only satisfied if possible...
 * This way some singularities can be solved pretty well...
 */

class IKGroupSolver
{
public:
        
            /// Constructor
            IKGroupSolver();
            /// Destructor
    virtual ~IKGroupSolver();

            /// Allows to print out debug message
            void    SetVerbose(bool verbose=true);

            /// Set the solver number of DOFs
            void    SetSizes(int dofs);
            /// Get the solver number of DOFs
            int     GetDOFSize();

            /// Sets the weights on the degrees of freedom (useful if the redundancies are on the DOFs)
            void    SetDofsWeights(Vector &v);

            /// Add a solver item with given constraints size and return the sovler id
            int     AddSolverItem(const int constraintsSize);
            /// Set jacobian dofs indices for the given solver
            void    SetDofsIndices(const vector<unsigned int> & dofsIndex, int solverId = 0);
            /// Set the priority of the given solver
            void    SetPriority(int priority, int solverId = 0);
            /// Set the thresholds for the given solver (all by default)
            void    SetThresholds(REALTYPE loose, REALTYPE cut, int solverId = -1);

            /// Set the jacobian for the given solver (if provided, dofs indices will be used)
            void    SetJacobian(const Matrix & j, int solverId = 0);
            /// Get the jacobian from the given solver
            Matrix* GetJacobian(int solverId = 0);

            /// Sets the weights on the constraints for the given solver (useful if the redundancies are on the constraints)
            void    SetConstraintsWeights(Matrix &m, int solverId = 0);
            void    SetConstraintsWeights(Vector &v, int solverId = 0);

            /// Sets the target values to reach for the given solver
            void    SetTarget(const Vector &v, int solverId = 0);
            /// Enable or disable the given solver
            void    Enable(bool enable=true, int solverId = 0);
            /// Suspend or resume the given solver (used in conjunction with Enable)
            void    Suspend(bool suspend=true, int solverId = 0);
            /// Get if the given solver is enabled
            bool    IsEnabled(int solverId = 0);
        

            /// Sets the target for the null space (Size given by the number of DOFs)
            void    SetNullTarget(const Vector &null);

            /// Removes the null space behavior
            void    RemoveNullTarget();

            /// Removes all constraints limits on the outputs
            void    ClearLimits();
            
            /**
             * \brief Sets the constraints limits on the putput
             * \param low      Vector for low bounds values
             * \param high     Vector for high bounds values
             */  
            void    SetLimits(const Vector &low, const Vector &high);
        
            void    ComputePriorities();
        
            void    Solve();
        
            void    Resize();
            
            /// Get the result
            Vector&     GetOutput();        
            /// Get the error between produced target and the requested one
            Vector&     GetTargetError(int solverId = 0);        
            /// Get the actual target that output values produces
            Vector&     GetTargetOutput(int solverId = 0);
            /// Get the squared norm of the error between produced target and the requested one
            REALTYPE    GetTargetErrorNorm();
            REALTYPE    GetTargetErrorNorm2();
protected:
    typedef struct{
        IKSubSolver     mSolver;
        IndicesVector   mDofsIndex;
        int             mPriority;
        Vector          mDesiredTarget;
        Vector          mActualTarget;
        Vector          mOutputTarget;
        Vector          mErrorTarget;
        Vector          mOutput;
        bool            bEnabled;
        bool            bSuspended;
    }IKSolverItem;

    vector<IKSolverItem>    mIKItems;

    vector<int>             mSortedPriorityIds;

    bool                    bVerbose;

    bool					bUseNullTarget;

    bool                    bComputePriorities;

    int                     mConstraintsSize;
    int                     mDofs;

    Vector                  mNullTarget;
    Vector                  mCurrNullTarget;

    Vector                  mLimits[2];
    Vector                  mCurrLimits[2];
    Vector                  mLimitsOffset;

    Matrix                  mDofsWeights;
    Matrix                  mInvDofsWeights;
    Matrix                  mCurrDofsWeights;
    SharedMatrix            mCurrWeights;
    SharedMatrix            mCurrWeightsTranspose;

    Matrix                  mCurrWeightsPtr;
    Matrix                  mCurrWeightsTransposePtr;

    Vector              mLimitsOffsetTarget;

    Vector              mOutput;
    Vector              mStepOutput;
    Vector              mOutputOffset;
    Vector              mOutputLimitsError;

    Vector              tmpCsV;
    Vector              tmpDsVPtr;
    Vector              tmpDsV2Ptr;
    SharedVector        tmpDsV;
    SharedVector        tmpDsV2;

    Matrix tmpMat;

};




class QPSolver {
public:

	QPSolver();
	virtual ~QPSolver();

	Vector getOutput()const;


	void setDimensions(unsigned int DOF, unsigned int EndEffectorDim);

	void setW(Matrix const&);
	virtual void setJacobian(Matrix const&);

	void setDeltaT(double const&);
	void setGamma(double const&);
	void setMultiplierRotationJ(double MultiplierRotationJ);

	void Solve(Vector y);

protected:
	virtual Vector XiMinus()=0;
	virtual Vector XiPlus()=0;

	//Depends on the scheme used
	virtual Vector mb()=0;
	virtual Vector md()=0;

	virtual Vector q();


	//Projector. The position and the velocity(only in the acceleration level) have to be given in the children classes
	Vector POmega(Vector handle);

	//W matrix
	Matrix mW;
	//Jacobian
	Matrix mJ;

	//timestep
	double mDeltaT;

	double mGamma;

	//Vector u containing rdot and thetadot
	Vector mu;

	//Number of degrees of freedom
	unsigned int mDOF;
	//Dimension of the end effector phase space
	unsigned int mEndEffectorDim;

	//Multiplier for the rotation coordinates of the jacobian, close to 0 means we don't care about orientation of the end effector and 1 that we do.
	double mMultiplierRotationJ;

};



//Solver for the inverse kinematic based on the minimum acceleration norm:
class MANSolver: public QPSolver {
public:
	MANSolver();
	virtual ~MANSolver();


	void setW();

	void setConstraints(Vector ThetaMinus, Vector ThetaPlus, Vector ThetaDotMinus, Vector ThetaDotPlus, Vector Theta2Minus, Vector Theta2Plus);

	void setMuP(double const&);
	void setEtaP(double const&);
	void setMuV(double const&);


	void Solve(Vector rdot, Vector Theta, Vector ThetaDot);

	virtual void setJacobian(Matrix const&);

protected:


	void SetCurrent(Vector Theta, Vector ThetaDot);


	virtual Vector XiMinus();
	virtual Vector XiPlus();

	//Depends on the scheme used
	virtual Vector mb();
	virtual Vector md();

	Vector mCurrentTheta;
	Vector mCurrentThetaDot;

	double mMuP;
	double mEtaP;
	double mMuV;


	//Limits of the constraints
	Vector mThetaMinus;
	Vector mThetaPlus;
	Vector mThetaDotMinus;
	Vector mThetaDotPlus;
	Vector mTheta2Minus;
	Vector mTheta2Plus;


	Matrix mOldJ;
};



class MKESolver: public QPSolver {
public:
	MKESolver();
	virtual ~MKESolver();


	void setH(Matrix const&);

	void setConstraints(Vector ThetaMinus, Vector ThetaPlus, Vector ThetaDotMinus, Vector ThetaDotPlus);
	void setMuP(double const&);


	//Solver, needs the wanted velocity of the end effector and the current position (to comupute the jacobian)
	void Solve(Vector rdot, Vector Theta);



protected:
	void SetCurrentTheta(Vector Theta);

	virtual Vector XiMinus();
	virtual Vector XiPlus();

	//Depends on the scheme used
	virtual Vector mb();
	virtual Vector md();

	Vector mCurrentTheta;

	double mMuP;


	//Limits of the constraints
	Vector mThetaMinus;
	Vector mThetaPlus;
	Vector mThetaDotMinus;
	Vector mThetaDotPlus;


};


/*
//Solver for the inverse kinematic based on the minimum kinematic energy
class MKESolver {
public:
	MKESolver();
	virtual ~MKESolver();

	void setDimensions(unsigned int DOF, unsigned int EndEffectorDim);

	void setH(Matrix const&);
	void setJacobian(Matrix const&);

	void setConstraints(Vector ThetaMinus, Vector ThetaPlus, Vector ThetaDotMinus, Vector ThetaDotPlus);
	void setDeltaT(double const&);
	void setMuP(double const&);
	void setGamma(double const&);
	void setMultiplierRotationJ(double MultiplierRotationJ);

	//returns the velocities of the joints thetadot
	Vector getOutput()const;

	//Solver, needs the wanted velocity of the end effector and the current position (to comupute the jacobian)
	void Solve(Vector rdot, Vector Theta);



protected:
	//Inertia matrix
	Matrix mH;
	//Jacombian
	Matrix mJ;

	//timestep
	double mDeltaT;

	double mMuP;
	double mGamma;

	//Vector u containing rdot and thetadot
	Vector mu;

	//Limits of the constraints
	Vector mThetaMinus;
	Vector mThetaPlus;
	Vector mThetaDotMinus;
	Vector mThetaDotPlus;

	//Projector on the constraints
	Vector POmega(Vector handle, Vector const& Theta);

	//Number of degrees of freedom
	unsigned int mDOF;
	//Dimension of the end effector phase space
	unsigned int mEndEffectorDim;

	//Multiplier for the rotation coordinates of the jacobian, close to 0 means we don't care about orientation of the end effector and 1 that we do.
	double mMultiplierRotationJ;

};
*/



#ifdef USE_MATHLIB_NAMESPACE
}
#endif
#endif
