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

#include "iCubIK.h"

#include <algorithm>
#include <set>
#include <map>

// Default time for a trial
#define TRIAL_TIME 6.0

// Kinda tool function for prjecting a 3d target to a 2d target for head tracking
Matrix GetProjection(Vector3 & axis){
    Vector3 freeAxis = axis;
    freeAxis.Normalize();
    Matrix jProj(3,3);
    jProj.Identity();
    Matrix D(3,3);
    Matrix F(3,1);
    F.SetColumn(freeAxis,0);
    D=(jProj.Transpose()*F)*F.Transpose();
    jProj -= D;
    jProj.GramSchmidt();
    jProj.RemoveZeroColumns();
    jProj.Transpose(D);
    jProj = D;
    return jProj;
}


// Project the target into tthe reduced head tracking space
Vector ProjectTarget(Vector target, Vector3 & freeAxis){
    Matrix p = GetProjection(freeAxis);
    return p*target;
}

// Project the jacobian into tthe reduced head tracking space
Matrix ProjectJacobian(Matrix jacobian, Vector3 & freeAxis){
    Matrix p = GetProjection(freeAxis);
    return p*jacobian;
}

// Another tool function for a dynamic sytsem in cylindrical space
// From current position and desired target, gives desired velocity
Vector3 CylindricalDynSys(Vector3 pos, Vector3 target){

    Vector3 start,end,v;
    start[0] = atan2(pos[1],pos[0]);
    if(start[0]<0.0) start[0]+=2.0*M_PI;
    start[1] = sqrt(pos[1]*pos[1] + pos[0]*pos[0]);
    start[2] = pos[2];

    end[0] = atan2(target[1],target[0]);
    if(end[0]<0.0) end[0]+=2.0*M_PI;
    end[1] = sqrt(target[1]*target[1] + target[0]*target[0]);
    end[2] = target[2];

    v = (end-start)*2.0;

    Vector3 res;
    res[0] = -sin(start[0])*v[0]*start[1] + cos(start[0])*v[1] + (target[0]-pos[0])*1.0;
    res[1] =  cos(start[0])*v[0]*start[1] + sin(start[0])*v[1] + (target[1]-pos[1])*1.0;
    res[2] = v[2];

    return res;
}









iCubIK::iCubIK()
:RobotInterface(){
}
iCubIK::~iCubIK(){
}

RobotInterface::Status iCubIK::RobotInit(){

    // Sensors and actuators
    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

    // Copy to the internal robot
    mInternalRobot.SetWorldInstance(mRobot->GetWorldInstance());
    if(mInternalRobot.Load(mRobot->GetType(),mRobot->GetSubType(),mRobot->GetPatches())){
        cout << "Internal robot loaded as a copy of main robbot"<<endl;
    }else{
        cout << "Failed loading internal robot"<<endl;
        exit(0);
    }
    // Copy the base frame from robot to ihternaél robot
    mInternalRobot.GetLinks()[0]->GetReferenceFrame(true) = mRobot->GetLinks()[0]->GetReferenceFrame(true);
    mInternalRobot.GetLinks()[0]->GetReferenceFrame(false) = mRobot->GetLinks()[0]->GetReferenceFrame(false);

    // Sensors and actuators
    mInternalSensorsGroup.SetSensorsList(mInternalRobot.GetSensors());
    mInternalActuatorsGroup.SetActuatorsList(mInternalRobot.GetActuators());

    // Pass the internal robot as user info (useful if you use the iCubSkinReader)
    mRobot->SetUserInfo(&mInternalRobot);



    // Some commands
    AddConsoleCommand("idle");
    AddConsoleCommand("rest");
    AddConsoleCommand("move");

    AddConsoleCommand("t");
    AddConsoleCommand("target");

    AddConsoleCommand("h");
    AddConsoleCommand("headtarget");

    AddConsoleCommand("ik_head");
    AddConsoleCommand("ik_orient");

    AddConsoleCommand("setRest");





    // Creating kinematic chains: first right arm
    mKinematicChain.SetRobot(&mInternalRobot);
    mKinematicChain.Create(0,mInternalRobot.GetLinkIndex("Torso_0"),mInternalRobot.GetLinkIndex("RHand"));

    // Creating kinematic chains: then head
    mKinematicChainHead.SetRobot(&mInternalRobot);
    mKinematicChainHead.Create(0,mInternalRobot.GetLinkIndex("Torso_0"),mInternalRobot.GetLinkIndex("Eye_0"));

    // Setting up the kinematic chain manager
    mKinChainManager.AddKinematicChain(&mKinematicChainHead);
    mKinChainManager.AddKinematicChain(&mKinematicChain);


    // Setting up the Inverse Kinematics  Solver

    mIKSolver.SetSizes(mKinChainManager.GetSize()); // Dof counts
    mIKSolver.SetVerbose(false);                    // No comments
    mIKSolver.SetThresholds(0.0001,0.00001);        // Singularities thresholds

    // add hand cartesian solver
    mIKSolverHandCartID = mIKSolver.AddSolverItem(3);               // Solver number of constraints
    mIKSolver.Enable(false,mIKSolverHandCartID);                     // Enabling this solver
    mIKSolver.SetPriority(100,mIKSolverHandCartID);                 // Low priority
    mIKSolver.SetDofsIndices(mKinChainManager.GetLocalIndices(&mKinematicChain),mIKSolverHandCartID); // Joint maps for first solver

    // add hand orientation solver
    mIKSolverHandOrientID = mIKSolver.AddSolverItem(2);
    mIKSolver.Enable(false,mIKSolverHandOrientID);
    mIKSolver.SetPriority(120,mIKSolverHandOrientID);
    mIKSolver.SetDofsIndices(mKinChainManager.GetLocalIndices(&mKinematicChain),mIKSolverHandOrientID);

    // add head solver
    mIKSolverHeadID = mIKSolver.AddSolverItem(2);
    mIKSolver.Enable(false,mIKSolverHeadID);
    mIKSolver.SetPriority(150,mIKSolverHeadID);
    mIKSolver.SetDofsIndices(mKinChainManager.GetLocalIndices(&mKinematicChainHead),mIKSolverHeadID);


    mTargetID       = 0;
    mHeadTargetID   = 0;


    return STATUS_OK;
}
RobotInterface::Status iCubIK::RobotFree(){

    return STATUS_OK;
}

RobotInterface::Status iCubIK::RobotStart(){
    bEnableHandOrient   = false;
    bEnableHead         = false;
    bHeadMapTarget      = false;

    bSetRest            = true;

    mState              = 0;
    mNextState          = 0;
    bFirst              = true;
    return STATUS_OK;
}    
RobotInterface::Status iCubIK::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status iCubIK::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status iCubIK::RobotUpdateCore(){

    // Read from the robot
    mSensorsGroup.ReadSensors();
    mInternalSensorsGroup.ReadSensors();

    //////////////////////////////////////////////////////////////////////////////////////
    // In case of state switches
    if((bFirst)||(mNextState!=mState)){
        // Reset the internal robot
        mInternalSensorsGroup.SetJointAngles(mSensorsGroup.GetJointAngles());
        mInternalSensorsGroup.SetJointVelocities(mSensorsGroup.GetJointVelocities());
        mInternalSensorsGroup.WriteSensors();
        mInternalRobot.UpdateLinks();

        // Reset smoothing variable
        mLastVel.Resize(mInternalSensorsGroup.GetCount());
        mLastVel.Zero();

        mState          = mNextState;
        // Set start state variables
        mStartTime      = GetClock().GetTime();
        mStartJointPos  = mSensorsGroup.GetJointAngles();

        GetConsole()->Print(string("Switching to new state: ")+Int02ToString(mState));
    }
    // In case of setting rest position
    if((bFirst)||(bSetRest)){
        mTargetRestPos       = mSensorsGroup.GetJointAngles();
        mTargetRestPosBackup = mTargetRestPos;
        bSetRest        = false;

        GetConsole()->Print(string("Using current position to set rest position"));
    }
    bFirst = false;

    mTargetRestPos = mTargetRestPosBackup;

    // Current time since state change
    mDataTime = GetClock().GetTime() - mStartTime ;


    //////////////////////////////////////////////////////////////////////////////////////
    // Setting head and hand targets
    pWorldObject obj;

    mGlobalTargetCart.Resize(3);
    mGlobalHeadTargetCart.Resize(3);

    if(bHeadMapTarget)
        mHeadTargetID = mTargetID;

    if((mTargetID>=0)&&(mTargetID<=4)){
        obj = GetWorld()->Find(string("Target")+Int01ToString(mTargetID));
    }else{
        obj = GetWorld()->Find("Target");
    }
    if(obj) mGlobalTargetCart = SharedVector(obj->GetReferenceFrame().GetOrigin());

    if((mHeadTargetID>=0)&&(mHeadTargetID<=4)){
        obj = GetWorld()->Find(string("Target")+Int01ToString(mHeadTargetID));
    }else{
        obj = GetWorld()->Find("Target");
    }
    if(obj) mGlobalHeadTargetCart = SharedVector(obj->GetReferenceFrame().GetOrigin());



    // Setting global variable of the hand position in world and in robot
    mGlobalPosCart  = SharedVector(mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinkIndex("RHand")).GetOrigin().Array(),3);
    mPosCart        = SharedVector(mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinkIndex("RHand"),0).GetOrigin().Array(),3);


    switch(mState){
    case 0:
        // Do nothing
        break;

    case 1:{
        // Move to a default rest position in 3 sec
        double restArray[]= {  0.0,  0.0,  0.0,
                               0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                             -40.0, 40.0,  40.0, 60.0,  0.0,  0.0,  0.0,
                             -10.0, 20.0,  0.0, 15.0,  0.0,  0.0,  0.0};

        Vector targetDeg(restArray,mInternalSensorsGroup.GetCount());
        Vector target = targetDeg * DEG2RAD(1.0);

        // Move to rest pos
        double deltaT = (mDataTime) / 3.0 ;
        deltaT = TRUNC(deltaT,0.0,1.0);
        deltaT = 0.5*(1.0-cos(deltaT* M_PI));
        target *=  deltaT;
        target += mStartJointPos * (1.0-deltaT);

        mInternalSensorsGroup.SetJointAngles(target);
        mInternalSensorsGroup.WriteSensors();
        mInternalRobot.UpdateLinks();
        }break;

    // All other cases
    case 2:
    case 3:{
        mPostGlobalTargetCart = mGlobalTargetCart;



        // Prepare local target positions
        Vector3 tgt;
        tgt = mInternalRobot.GetReferenceFrame(0).GetInverse().GetHMatrix().Transform(Vector3(mPostGlobalTargetCart(0),mPostGlobalTargetCart(1),mPostGlobalTargetCart(2)));
        mTargetCart = SharedVector(tgt.Array(),3);

        tgt = mInternalRobot.GetReferenceFrame(0).GetInverse().GetHMatrix().Transform(Vector3(mGlobalHeadTargetCart(0),mGlobalHeadTargetCart(1),mGlobalHeadTargetCart(2)));
        mHeadTargetCart = SharedVector(tgt.Array(),3);


        // Hum... do IK...
        DoIK();

        // Integrate IK's output
        Vector jOutputPos(mInternalSensorsGroup.GetJointAngles());
        Vector jOutputVel(mInternalSensorsGroup.GetCount());

        jOutputVel.SetSubVector(mKinChainManager.GetGlobalIndices(),mIKSolver.GetOutput());

        // A bit of smoothing
        jOutputVel  = mLastVel + (jOutputVel-mLastVel)*0.1;
        mLastVel    = jOutputVel;

        // Integrate IK's output
        jOutputPos += (jOutputVel * GetClock().GetDt());
        mInternalSensorsGroup.SetJointAngles(jOutputPos);
        mInternalSensorsGroup.SetJointVelocities(jOutputVel);
        mInternalSensorsGroup.WriteSensors();
        }break;
    }

    // Update the internal robot and send orders to the real one
    mInternalRobot.UpdateLinks();
    mActuatorsGroup.SetJointAngles(mInternalSensorsGroup.GetJointAngles());
    mActuatorsGroup.WriteActuators();




    // Move some virtual object for drawing
    obj = GetWorld()->Find("Ball");
    if(obj) obj->GetReferenceFrame().SetOrigin().Set(mGlobalTargetCart.Array());
    obj = GetWorld()->Find("Ball2");
    if(obj) obj->GetReferenceFrame().SetOrigin().Set(mPostGlobalTargetCart.Array());


    return STATUS_OK;
}


int iCubIK::RespondToConsoleCommand(const string cmd, const vector<string> &args){


    if(cmd=="idle"){ // Do nothing
        mNextState = 0;
    }else if(cmd=="rest"){ // Go to rest postion
        mNextState = 1;
    }else if(cmd=="move"){ // Go violently to the target
        mNextState = 2;
    }else if((cmd=="t")||(cmd=="target")){ // Set the target <id>
        if(args.size()==1){
            mTargetID = atoi(args[0].c_str());
            mTargetID = TRUNC(mTargetID,0,5);
            GetConsole()->Print(string("Hand target: ")+Int01ToString(mTargetID));
        }else{
            GetConsole()->Print(string("Error: No hand target provided"));
        }
    }else if((cmd=="h")||(cmd=="headtarget")){ // Set the head target <id>. 
                                               // If no id provided, target is mapped to the hand's target
        if(args.size()==1){
            mHeadTargetID = atoi(args[0].c_str());
            mHeadTargetID = TRUNC(mHeadTargetID,0,5);
            bHeadMapTarget = false;
            GetConsole()->Print(string("Head target: ")+Int01ToString(mHeadTargetID));
        }else{
            bHeadMapTarget = true;
            GetConsole()->Print("Head target: hand target");
        }
    }else if(cmd=="ik_head"){ // Enable head IK
        bEnableHead = !bEnableHead;
        if(bEnableHead) GetConsole()->Print("IK: Head enabled");
        else            GetConsole()->Print("IK: Head disabled");
    }else if(cmd=="ik_orient"){ // Enable hand orientation IK
        bEnableHandOrient = !bEnableHandOrient;
        if(bEnableHandOrient)   GetConsole()->Print("IK: Hand orient enabled");
        else                    GetConsole()->Print("IK: Hand orient disabled");
    }else if(cmd=="setRest"){   // Set to the rest position
        bSetRest = true;
    }
    return 0;
}

void iCubIK::DoIK(){

    // Update the kinematic chains
    mKinematicChain.Update();
    mKinematicChainHead.Update();

    int ksize = mKinChainManager.GetSize();

    Vector weights(ksize); 
    Vector rest(ksize); 
    Vector targetCart(3);


    // Get position rest position, and limits in using KChain ids
    Vector cPos;
    cPos   .Set(mKinChainManager.GetGlobalIndices(),mInternalSensorsGroup.GetJointAngles());

    Vector tRest;
    tRest  .Set(mKinChainManager.GetGlobalIndices(),mTargetRestPos);

    Vector limLow,limHigh;
    limLow .Set(mKinChainManager.GetGlobalIndices(),mInternalSensorsGroup.GetJointLimitsLow());
    limHigh.Set(mKinChainManager.GetGlobalIndices(),mInternalSensorsGroup.GetJointLimitsHigh());


    // Dealing with joint limits: Limiting output: Max 35 deg per seconds
    Vector lim1(ksize);
    Vector lim2(ksize);
    lim1= DEG2RAD(-70.0);
    lim2= DEG2RAD( 70.0);
    lim1(0) = lim1(1) = lim1(2) = DEG2RAD(-20.0);
    lim2(0) = lim2(1) = lim2(2) = DEG2RAD( 20.0);
    for(int i=0;i<ksize;i++){
        // Limiting speed when approaching 5 deg from joint range limit
        double deltaLow  = cPos[i]-limLow[i];
        double deltaHigh = limHigh[i]-cPos[i];
        if(deltaLow < 0.0)                lim1[i] *= 0.0;
        else if(deltaLow  < DEG2RAD(5.0)) lim1[i] *= deltaLow /DEG2RAD(5.0);
        if(deltaHigh < 0.0)               lim2[i] *= 0.0;
        else if(deltaHigh < DEG2RAD(5.0)) lim2[i] *= deltaHigh/DEG2RAD(5.0);
    }
    mIKSolver.SetLimits(lim1,lim2);

    // Set the weights of each dof
    weights    = 2.0;
    weights(0) = 1.0;
    weights(1) = 1.0;
    weights(2) = 2.0;
    weights(6) = 3.0;
    mIKSolver.SetDofsWeights(weights);


    // Set null space optimization
    rest = tRest - cPos;
    rest *= 1.0;
    rest(0) *= 2.0;
    rest(1) *= 2.0;
    rest(2) *= 1.0;
    //rest.Trunc(-1.5,1.5);
    mIKSolver.SetNullTarget(rest);

    ///////////////////////////////////
    // Hand cartesian target

    targetCart  = mTargetCart;
    targetCart -= mInternalRobot.GetReferenceFrame(mKinematicChain.GetTargetLinkId(),0).GetOrigin();
    // Add some gain
    if((mState>2)&&(mState<5)){
        targetCart *= 5.0;
    }else{
        // Cylindrical dynamical system
        targetCart *= 2.5;
        targetCart = CylindricalDynSys(mInternalRobot.GetReferenceFrame(mKinematicChain.GetTargetLinkId(),0).GetOrigin(), Vector3(mTargetCart[0],mTargetCart[1],mTargetCart[2]));
        targetCart*= 0.3;
    }
    // Trunc speed
    if(targetCart.Norm()>0.5){
        targetCart *= 0.5/targetCart.Norm();
    }

    mIKSolver.SetJacobian(mKinematicChain.GetJacobian().GetRows(0,2),mIKSolverHandCartID);
    mIKSolver.SetTarget(targetCart,mIKSolverHandCartID);
    mIKSolver.Enable(true,mIKSolverHandCartID);

    /////////////////////////////////////
    // Hand orient target
    if(bEnableHandOrient){

        Vector3 handAxis(0,0,-1);
        Vector3 handDir = mInternalRobot.GetReferenceFrame(mKinematicChain.GetTargetLinkId(),0).GetOrient().GetColumn(1);

        Vector3 tgtDir = handDir.Cross(handAxis);
        if(handDir.Dot(handAxis)<0.0) tgtDir.SMinus();

        mIKSolver.SetJacobian(ProjectJacobian(mKinematicChain.GetJacobian().GetRows(3,5),handAxis),mIKSolverHandOrientID);
        mIKSolver.SetTarget(ProjectTarget(SharedVector(tgtDir),handAxis),mIKSolverHandOrientID);

        mIKSolver.Enable(true,mIKSolverHandOrientID);
    }else{
        mIKSolver.Enable(false,mIKSolverHandOrientID);
    }
    /////////////////////////////////////


    ///////////////////////////////////// 
    // Head target
    if(bEnableHead){
        Vector3 headAxis;

        Vector3 tgtDir;
        tgtDir.Set(mHeadTargetCart);
        tgtDir -= mInternalRobot.GetReferenceFrame(mKinematicChainHead.GetTargetLinkId(),0).GetOrigin();
        tgtDir.SMinus();
        tgtDir.Normalize();
        Vector3 td = tgtDir;
        headAxis = mInternalRobot.GetReferenceFrame(mKinematicChainHead.GetTargetLinkId(),0).GetOrient().GetColumn(0);
        headAxis = tgtDir;


        tgtDir = tgtDir.Cross(mInternalRobot.GetReferenceFrame(mKinematicChainHead.GetTargetLinkId(),0).GetOrient().GetColumn(0));
        tgtDir*=-1.0;
        if(tgtDir.Norm()>0.5){
            tgtDir.Normalize();
            tgtDir*=0.5;
        }


        mIKSolver.SetJacobian(ProjectJacobian(mKinematicChainHead.GetJacobian().GetRows(3,5),headAxis),mIKSolverHeadID);
        mIKSolver.SetTarget(ProjectTarget(SharedVector(tgtDir),headAxis),mIKSolverHeadID);

        mIKSolver.Enable(true,mIKSolverHeadID);
    }else{
        mIKSolver.Enable(false,mIKSolverHeadID);
    }



    // We solve what we can
    mIKSolver.Solve();


}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    iCubIK* create(){return new iCubIK();}
    void destroy(iCubIK* module){delete module;}
}



