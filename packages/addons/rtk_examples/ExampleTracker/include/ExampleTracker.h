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

#ifndef ExampleTracker_H_
#define ExampleTracker_H_

#include "RobotLib/RobotInterface.h"

#include "MathLib/MathLib.h"
#include "MathLib/IKGroupSolver.h"

#include "RobotLib/ForwardDynamics.h"
#include "RobotLib/InverseDynamics.h"
#include "RobotLib/KinematicChain.h"
#include "RobotLib/PIDController.h"

class ExampleTracker : public RobotInterface
{
protected:

    Robot                       mInternalRobot;
    RevoluteJointSensorGroup    mInternalSensorsGroup;
    RevoluteJointActuatorGroup  mInternalActuatorsGroup;
    KinematicChain              mInternalKinematicChain;


    RevoluteJointSensorGroup    mSensorsGroup;
    RevoluteJointActuatorGroup  mActuatorsGroup;

    KinematicChain              mKinematicChain;
    InverseDynamics             mInvDynamics;

    IKGroupSolver               mIKSolver;

    PIDController               mPIDCtrl;




    int                         mEndEffectorId;
    int							DOFCount;
    WorldObject                *mTarget;

    Vector                      mTorques;
    Vector                      mGravityTorques;

    Vector                      mJointPos;
    Vector                      mJointDesPos;
    Vector                      mJointTargetPos;

    Vector                      mJointVel;
    Vector                      mJointTmp;

    Vector                      mJointVelLimits[2];

    int                         mState;
    int                         mMode;

    double                      mStartTime;
    Vector3                     mTargetZero;

    Vector3                     mTargetError;
public:
            ExampleTracker();
    virtual ~ExampleTracker();
  
    virtual Status              RobotInit();
    virtual Status              RobotFree();
  
    virtual Status              RobotStart();    
    virtual Status              RobotStop();
  
    virtual Status              RobotUpdate();
    virtual Status              RobotUpdateCore();

    virtual int                 RespondToConsoleCommand(const string cmd, const vector<string> &args);
};



#endif 
