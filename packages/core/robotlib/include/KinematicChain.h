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

#ifndef KINEMATICCHAIN_H_
#define KINEMATICCHAIN_H_

#include "Robot.h"

#include <algorithm>
#include <set>
#include <map>
using namespace std;

/**
 * \class KinematicChain
 *
 * \brief Represent a Kinematic Chain, compute jacobian and more
 *
 * This class links to a robot object and allows setting up a kinematic chain object,
 * usually to get the Jacobian and more.
 */
class KinematicChain
{

protected:
    Robot              *mRobot;
    LinksList          *mLinks;
    JointsList         *mJoints;
    vector<int>        *mParents;
    unsigned int        mLinksCount;

    MathLib::Matrix              mJacobian;
    MathLib::Matrix              m2ndJacobian;

    MathLib::Matrix*             mJacobianGradient;
    Vector3*            mJacobianAxis;
    Vector3*            mJacobianPos;


    IndicesVector       mValidJoints;
    IndicesVector       mJointMapping;

    int                 mChainSize;
    IndicesVector       mChain;

    int                 mBaseLink;
    int                 mRootLink;
    int                 mTargetLink;

    Vector              mChainPos;
    Vector              mChainVel;
    Vector              mChainAcc;

    Vector              mTargetPos;
    Vector              mTargetVel;
    Vector              mTargetAcc;

    Vector              mTargetAPos;
    Vector              mTargetAVel;
    Vector              mTargetAAcc;



public:

    /// Constructor
    KinematicChain();
    /// Destructor
    ~KinematicChain();

    /// Sets the robot to link to
    void    SetRobot(pRobot robot);
    /// Create a kinematic chain that starts from root to target (in robot'd links indices), represented in base link frame
    void    Create(int base, int root, int target);
    /// Return the number of DOF inside the chain
    int     GetDOFCount();

    /*
    Vector& GetJointAngles(Vector & result);
    Vector& GetJointVelocity(Vector & result);
    Vector& GetJointAcceleration(Vector & result);
    Vector& GetTargetPos(Vector & result);
    Vector& GetTargetVel(Vector & result);
    Vector& GetTargetAcc(Vector & result);
    Vector& GetTargetAngularPos(Vector & result);
    Vector& GetTargetAngularVel(Vector & result);
    Vector& GetTargetAngularAcc(Vector & result);
    */

    /// Build the jacobian
    void    Update();
    /// Build the jacobian
    void    BuildJacobian();

    /// Get the jacobian
    MathLib::Matrix& GetJacobian();

    /// Build the gradient of the jacobian wih respect to the angle(theta)
    void    BuildJacobianGradient();
    /// Get this gradiet of the jacobian
    MathLib::Matrix& GetJacobianGradient(int dof);

    /// Get a vector of indices that maps each dof of the chain to the dof indices of the full robot
    IndicesVector& GetJointMapping();
    /// Get the target link ID in robot indices
    int GetTargetLinkId();
};

/**
 * \class KinematicChainIndicesManager
 *
 * \brief A helper class to managê the indices of multiple chains
 *
 */
class KinematicChainIndicesManager
{
protected:
    map<KinematicChain*,IndicesVector>  mLocalIndices;
    map<unsigned int,unsigned int>      mGlobalIndicesMap;
    set<unsigned int>                   mGlobalIndicesSet;
    IndicesVector                       mGlobalIndices;

public:
    /// Constructor
    KinematicChainIndicesManager();
    /// Destructor
    ~KinematicChainIndicesManager();

    /// Add a chain to the set
    void AddKinematicChain(KinematicChain *kchain);
    /// Get the number of total number of dof in registered chains
    int GetSize();

    /// Get the local indices with respect to the full set
    IndicesVector & GetLocalIndices(KinematicChain *kchain);
    /// Get the globalindices of the full set
    IndicesVector & GetGlobalIndices();
};

#endif /*KINEMATICCHAIN_H_*/
