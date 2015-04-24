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

#ifndef iCubIK_H_
#define iCubIK_H_

#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"

#include "RobotLib/RobotInterface.h"
#include "RobotLib/KinematicChain.h"




class iCubIK : public RobotInterface
{
protected:


    /// Internal robot for open loop position control
    Robot                       mInternalRobot;
    /// Corresponding sensor group
    RevoluteJointSensorGroup    mInternalSensorsGroup;
    /// Corresponding actuator group
    RevoluteJointActuatorGroup  mInternalActuatorsGroup;

    /// Reading from real robot
    RevoluteJointSensorGroup    mSensorsGroup;
    /// Actuation from real robot
    RevoluteJointActuatorGroup  mActuatorsGroup;

    /// Kinematic chain manager
    KinematicChainIndicesManager mKinChainManager;

    /// Kinematc chain for the iCub head
    KinematicChain              mKinematicChainHead;
    /// Kinematc chain for the iCub right arm
    KinematicChain              mKinematicChain;

    /// Kinematic chain solver
    IKGroupSolver               mIKSolver;

    /// IDs of the solver linked to the hand cartesian position
    int                         mIKSolverHandCartID;
    /// IDs of the solver linked to the hand orientation
    int                         mIKSolverHandOrientID;
    /// IDs of the solver linked to the head
    int                         mIKSolverHeadID;

    /// Do we want to control the orientation of the hand
    bool                        bEnableHandOrient;
    /// Do we want to control the head
    bool                        bEnableHead;
    /// Do we want to map the hand target to the head target automatically
    bool                        bHeadMapTarget;




    /// Joint pos at state switches
    Vector                      mStartJointPos;

    /// Rest position
    Vector                      mTargetRestPos;
    /// Second version of rest position (due to some flaw in the code :)
    Vector                      mTargetRestPosBackup;


    /// Memory of last velocity for a bit of trajectory smoothing
    Vector                      mLastVel;

    /// Hand target in global frame
    Vector                      mGlobalTargetCart;
    /// Head target in global frame
    Vector                      mGlobalHeadTargetCart;

    /// Hand target in body frame
    Vector                      mTargetCart;
    /// Head target in body frame
    Vector                      mHeadTargetCart;

    /// Current hand cartesian position in world frame
    Vector                      mGlobalPosCart;
    /// Current hand cartesian position in robot frame
    Vector                      mPosCart;

    /// Local hand target for possible preprocessing of the global one
    Vector                      mPostGlobalTargetCart;




    /// Current state
    int                         mState;
    /// Next state variable
    int                         mNextState;

    /// Time at state switches
    double                      mStartTime;
    /// Current stime since last state stwiches
    double                      mDataTime;

    /// Tells the system that current position is the rest position
    bool                        bSetRest;


    /// ID of the target (given by the world file)
    int                         mTargetID;
    /// ID of the head target (given by the world file)
    int                         mHeadTargetID;


    /// Start flag
    bool                        bFirst;


public:
            iCubIK();
    virtual ~iCubIK();

    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);


            /// Perform inverse kinematics with contact avoidance calculation
            void                DoIK();
        
};



#endif 
