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

#include "ExampleTracker.h"


ExampleTracker::ExampleTracker()
:RobotInterface(){
}
ExampleTracker::~ExampleTracker(){
}

RobotInterface::Status ExampleTracker::RobotInit(){

    if(mInternalRobot.Load(mRobot->GetType(),mRobot->GetSubType(),"")){
        //cout << "Internal robot loaded as a copy of main robbot"<<endl;
    }else{
        cout << "Failed loading internal robot"<<endl;
        return STATUS_ERROR;
    }
    mInternalSensorsGroup.SetSensorsList(mInternalRobot.GetSensors());
    mInternalActuatorsGroup.SetActuatorsList(mInternalRobot.GetActuators());


    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

    // Inverse dynamics
    mInvDynamics.SetRobot(mRobot);
    mInvDynamics.Init();
    mInvDynamics.SetGravityCompensationOnly(true);

    // KChain
    mEndEffectorId = mRobot->GetLinksCount()-1;
    mKinematicChain.SetRobot(mRobot);
    mKinematicChain.Create(0,2,mEndEffectorId);

    mInternalKinematicChain.SetRobot(&mInternalRobot);
    mInternalKinematicChain.Create(0,2,mEndEffectorId);


    std::cout<< " DOFCount: " << mRobot->GetDOFCount() << std::endl;
    DOFCount = mRobot->GetDOFCount() - 2;

    // Inverse kinematics
    mIKSolver.SetSizes(DOFCount);  // Dof counts
    mIKSolver.AddSolverItem(3);                 // One solver with 3 constraints (x,y,z for instance)
    mIKSolver.SetVerbose(false);                // No comments
    mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
    mIKSolver.Enable(true,0);                   // Enable first solver
    mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver

    // PID
    XmlTree tree;
    tree.LoadFromFile("./data/Misc/WAMDefaultPID.xml");
    mPIDCtrl.Init(&tree);


    mTarget = GetWorld()->Find("Ball");

    mTorques.Resize(DOFCount);
    mGravityTorques.Resize(DOFCount);
    mJointPos.Resize(DOFCount);
    mJointDesPos.Resize(DOFCount);
    mJointTargetPos.Resize(DOFCount);
    mJointVel.Resize(DOFCount);
    mJointTmp.Resize(DOFCount);
    mJointVelLimits[0].Resize(DOFCount);
    mJointVelLimits[1].Resize(DOFCount);

    mState  = 0;
    mMode=0;

    AddConsoleCommand("rest");
    AddConsoleCommand("goPID");
    AddConsoleCommand("goForce");

    return STATUS_OK;
}
RobotInterface::Status ExampleTracker::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status ExampleTracker::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status ExampleTracker::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status ExampleTracker::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status ExampleTracker::RobotUpdateCore(){
    mSensorsGroup.ReadSensors();
    mJointPos       = mSensorsGroup.GetJointAngles();

    

    mTorques.Zero();

    mInvDynamics.SetGravityCompensationOnly(true);
    mInvDynamics.Update();
    mInvDynamics.GetTorques(mGravityTorques);

    int nextstate = mState;

    switch(mState){
    case 0:
        mJointDesPos    = mJointPos;
        mJointTargetPos = mJointPos;

        mJointVel.Zero();

        mPIDCtrl.SetInput(mJointPos);
        mPIDCtrl.SetTarget(mJointPos);
        mPIDCtrl.Start();

        mInternalSensorsGroup.SetJointAngles(mJointDesPos);
        mInternalSensorsGroup.WriteSensors();
        mInternalRobot.UpdateLinks();

        if(mMode!=0)
            nextstate = 1;

        break;
    case 1:

        mInternalSensorsGroup.ReadSensors();
        mJointDesPos = mInternalSensorsGroup.GetJointAngles();

        mJointTargetPos(0) = PI/4.0;
        mJointTargetPos(1) = -PI/3.0;
        mJointTargetPos(2) = 0.0;
        mJointTargetPos(3) = PI/2.0;
        mJointTargetPos(4) = 0.0;
        mJointTargetPos(5) = 0.0;
        mJointTargetPos(6) = 0.0;

        mJointTargetPos.Sub(mJointDesPos,mJointVel);
        mJointVel *= 0.3;
        mJointVel.Trunc(-DEG2RAD(20.0),DEG2RAD(20.0));
        //mJointVel.Print();
        
        mJointVel.ScaleAddTo(GetClock().GetDt(),mJointDesPos);

        mPIDCtrl.SetInput(mJointPos);
        mPIDCtrl.SetTarget(mJointDesPos);
        mPIDCtrl.Update(GetClock().GetDt());

        mPIDCtrl.GetOutput(mTorques);

        mTargetZero = mRobot->GetReferenceFrame(mEndEffectorId).GetOrigin();

        mStartTime = GetClock().GetTime();
        

        mJointTmp  = mJointPos;
        mJointTmp -= mJointTargetPos;
        mJointTmp.SAbs();

        if(mJointTmp.Max()<DEG2RAD(5.0)){
            
            if(mMode==1){
                nextstate = 2;
            }else if(mMode==2){
                nextstate = 3;
            }else{
                nextstate = 0;
            }
        }

        mInternalSensorsGroup.SetJointAngles(mJointDesPos);
        mInternalSensorsGroup.WriteSensors();
        mInternalRobot.UpdateLinks();

        
        break;
    case 2:
        {
        Vector3 tpos = mTargetZero;
        tpos(1) += 0.3*sin(GetClock().GetTime()-mStartTime);
        if(mTarget){
            mTarget->GetReferenceFrame().SetOrigin() =  tpos;
        }

        mInternalSensorsGroup.ReadSensors();
        mJointDesPos = mInternalSensorsGroup.GetJointAngles();

        mInternalKinematicChain.Update();
    

        // Set jacobian
        mIKSolver.SetJacobian(mInternalKinematicChain.GetJacobian(),0);

    
        // Dealing with joint limits: Limiting output: Max 35 deg per seconds
        mJointVelLimits[0] = 20*DEG2RAD(-180.0);
        mJointVelLimits[1] = 20*DEG2RAD( 180.0);
        for(int i=0;i<mRobot->GetDOFCount();i++){
            // Limiting speed when approaching 5 deg from joint range
            if(mJointDesPos[i]-mSensorsGroup.GetJointLimitsLow()(i)< DEG2RAD(5.0)){
                mJointVelLimits[0][i] *= (mJointDesPos[i]-mSensorsGroup.GetJointLimitsLow()(i))/DEG2RAD(5.0);
            }else if(mSensorsGroup.GetJointLimitsHigh()(i)-mJointDesPos[i]<DEG2RAD(5.0)){
                mJointVelLimits[1][i] *= (mSensorsGroup.GetJointLimitsHigh()(i)-mJointDesPos[i])/DEG2RAD(5.0); 
            }
        }
        mIKSolver.SetLimits(mJointVelLimits[0],mJointVelLimits[1]);
    
        mTargetError  = tpos;
        mTargetError -= mInternalRobot.GetReferenceFrame(mEndEffectorId).GetOrigin();
        mTargetError *= 5.0;

        mIKSolver.SetTarget(SharedVector(mTargetError),0);
    
        mIKSolver.Solve();

        

        mIKSolver.GetOutput().ScaleAddTo(GetClock().GetDt(),mJointDesPos);

        mInternalSensorsGroup.SetJointAngles(mJointDesPos);
        mInternalSensorsGroup.WriteSensors();
        mInternalRobot.UpdateLinks();


        mPIDCtrl.SetInput(mJointPos);
        mPIDCtrl.SetTarget(mJointDesPos);
        mPIDCtrl.Update(GetClock().GetDt());

        mPIDCtrl.GetOutput(mTorques);

        if(mMode!=1)
            nextstate = 0;
        }
        break;
    case 3:
    
        {
        Vector3 tpos = mTargetZero;
        //tpos(1) += 0.3*sin(GetClock().GetTime()-mStartTime);
        if(mTarget){
            mTarget->GetReferenceFrame().SetOrigin() =  tpos;
        }

        mTargetError  = tpos;
        mTargetError -= mRobot->GetReferenceFrame(mEndEffectorId).GetOrigin();
        mTargetError*=40.0;
        mKinematicChain.Update();
        mKinematicChain.GetJacobian().TransposeMult(SharedVector(mTargetError),mTorques);
        //mKinematicChain.GetJacobian().Print();
        //mTargetError.Print();
        //mTorques.Print();
        if(mMode!=2)
            nextstate = 0;

        }
        break;
    }
    if(mState != nextstate)
        cout << "next: "<<nextstate<<endl;

    mState = nextstate;

    mTorques += mGravityTorques;
    mActuatorsGroup.SetJointTorques(mTorques);
    mActuatorsGroup.SetJointAngles(mJointDesPos);
    mActuatorsGroup.WriteActuators();
    return STATUS_OK;
}
int ExampleTracker::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    if(cmd=="rest"){
        mMode = 0;
    }
    if(cmd=="goPID"){
        mMode = 1;
    }
    if(cmd=="goForce"){
        mMode = 2;
    }
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    ExampleTracker* create(){return new ExampleTracker();}
    void destroy(ExampleTracker* module){delete module;}
}

