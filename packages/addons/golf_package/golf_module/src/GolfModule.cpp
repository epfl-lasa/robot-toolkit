/*
 * Copyright (C) 2010 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author: Klas Kronander
 * email:   klas.kronander@epfl.ch
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

#include "GolfModule.h"
#include "aux/GolfLearn.cpp"
#include "aux/GolfAux.cpp"//contains auxilliary functions
#include "aux/GolfCommand.cpp"//contains the intialization of commands and RespondToConsoleCommand
#include "aux/GolfInit.cpp"//contains RobotInit


void Mirror(ReferenceFrame& ref, ReferenceFrame& res);

GolfModule::GolfModule()
  :RobotInterface(){
}
GolfModule::~GolfModule(){
}
RobotInterface::Status GolfModule::RobotFree(){
  return STATUS_OK;
}
RobotInterface::Status GolfModule::RobotStart(){
  return STATUS_OK;
}    
RobotInterface::Status GolfModule::RobotStop(){
  return STATUS_OK;
}

static int cntU = 0;
static int cntUC = 0;

RobotInterface::Status GolfModule::RobotUpdate(){


  cntU++;
  if(simulatorMode){
    pWorldObject obj = GetWorld()->Find("Ball");
    Ball_pos=obj->GetReferenceFrame().GetOrigin();
    obj = GetWorld()->Find("hole");
    Target_pos=obj->GetReferenceFrame().GetOrigin();
  }
  //if autoadjust is enabled, adjust the hitting parameters automatically
  if(autoadjust){
    adjust();
  }
  //is the exploremode enabled?
  if (simulatorMode && exploreMode)
    {
      exploreModeDo();
    }
  return STATUS_OK;		
}


RobotInterface::Status GolfModule::RobotUpdateCore(){
  EnableWrappers(false);
  //printf("-----------------------------------------------\n");

  cntUC++;//counter that can be used to keep track of nb core updates per update etc
  //  cout << "UC: "<<cntUC<<" "<<cntU<<" "<<double(cntUC)/double(cntU)<<endl;
  // Current time
  mClock.SetInternal(false);
  mClock.SetTime(GetClock().GetTime());
  mClock.Update();
	
  if(mClock.GetDt()<=0) return STATUS_OK;
	
  // Read sensors
  mSensorsGroup.ReadSensors();
  mInternalSensorsGroup.ReadSensors();
  // Get current joint position
  currPos = mInternalSensorsGroup.GetJointAngles();
  // Update Kinematic chainb
  mKinematicChain.Update();


  //////////////////////////////////////////////////////////
  ///////control of end-effector movement//////////////////
  /////////////////////////////////////////////////////////
  
  int nextState = mState;
  switch(mState){ //take action according to mState

  case 1: // Going to the initial position

    eventID=1; //this is only for recording purposes. 
    // reading end-effector position
    //    xd = Initial_point - mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin();
    Initial_point.Sub(SharedVector(mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin()),xd);
    //set the speed of movement:

    if(simulatorMode && xd.Norm()>0.1)
      xd *= (10/xd.Norm());
    else if(xd.Norm()>0.2){
      //      xd *=(0.2/xd.Norm());
      xd.Mult(0.2/xd.Norm(),xd);
    }
    if(xd.Norm()<0.01){
      eventID=2; //ie reached initial position

    }

    ///////control of end-effector orientation///////////////


    //transformation matrix from local coordinates in the endeffector to the global coordinate system
    T_L_G =SharedMatrix(mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrient());	
    //current racket direction in global coordinates:
    //  Racket_dir_proj=T_L_G*Racket_dir;
    T_L_G.Mult(Racket_dir,Racket_dir_proj);
    //printf("14-----------------------------------------------\n");
    //calculate cross product to get rotation vector 
    //    w_rotation[0]  = Racket_dir_proj[1] * hitting_dir[2] - Racket_dir_proj[2] * hitting_dir[1];
    w_rotation[1]  = Racket_dir_proj[2] * hitting_dir[0] - Racket_dir_proj[0] * hitting_dir[2];
    w_rotation[2]  = Racket_dir_proj[0] * hitting_dir[1] - Racket_dir_proj[1] * hitting_dir[0];

    w_rotation[0]=T_L_G(1,1)*sin(-15*PI/180)-T_L_G(1,2)*cos(-15*PI/180);

    break;

    {
    case 3: // Dynamical Systems Module with non-zero final velocity
      

      xd=R*R_DS*DS_model.doRegression(R_DS.Transpose()*R.Transpose()*targetCart);

      //get the strength factor
      double hej;
      Vector temp1(1);
      temp1(0)=targetCart.Norm()/(Initial_point-hitting_point).Norm();
      hej=DS_model_v.doRegression(temp1)(0);
      //multiply target field and strengthfactor and hitting gain. 
      xd*=fabs(hej)*hitting_gain/xd.Norm();




      eventID=3;
      //Using forward kinematics to get end effector position
      /*
        targetCart=mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin();
    
        // Substract the ball position (or hitting point)
        targetCart -= hitting_point + R*target_offset;
      */
      //more efficient implementation of commented stuff:
      // R is rotation with respect to default hitting dir in robot global coord. 
      // R_DS is alignment of Robot global coord system and DS
      R.Mult(target_offset,targetCart);

      targetCart += hitting_point;
      //printf("---------2.1---------------------\n");
      targetCart.SMinus();
      //printf("---------2.2---------------------\n");
      targetCart +=SharedVector(mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin());
      //printf("3----------------------------------------------\n");
      if (targetCart.Norm() < 0.01) //i.e. the ball is hit
        {
          xd_switching = xd;//copy current ee velociy to xd_switching
          nextState = 4;//go to sqitching dynamics
          //	cout<<"hit!"<<endl;
          hit_time=GetClock().GetTime();
          eventID=4;//record event 
        }


      ///////control of end-effector orientation///////////////

      //transformation matrix from local coordinates in the endeffector to the global coordinate system
      T_L_G =SharedMatrix(mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrient());	
      //current racket direction in global coordinates:
      //  Racket_dir_proj=T_L_G*Racket_dir;
      T_L_G.Mult(Racket_dir,Racket_dir_proj);
      //calculate cross product to get rotation vector 
      //    w_rotation[0]  = Racket_dir_proj[1] * hitting_dir[2] - Racket_dir_proj[2] * hitting_dir[1];
      w_rotation[1]  = Racket_dir_proj[2] * hitting_dir[0] - Racket_dir_proj[0] * hitting_dir[2];
      w_rotation[2]  = Racket_dir_proj[0] * hitting_dir[1] - Racket_dir_proj[1] * hitting_dir[0];

      w_rotation[0]=T_L_G(1,1)*sin(-15*PI/180)-T_L_G(1,2)*cos(-15*PI/180);

      break;
    }
	
  }	















  //////////////////////////////////////////////////////////
  /////// control in joint space///////////////////////////
  /////////////////////////////////////////////////////////

  mJointTarget_acc=mJointTarget_vel;

  switch(mState){
  case 0: // Idle ie gravity compensation 
    mJointSetPos = mSensorsGroup.GetJointAngles();
    mInternalSensorsGroup.SetJointAngles(mJointSetPos);
    mInternalSensorsGroup.WriteSensors();
    mInternalRobot.UpdateLinks();
    mPIDCtrl.SetInput(mJointSetPos);
    mPIDCtrl.SetTarget(mJointSetPos);
    mPIDCtrl.Start();
    mInvDynamics.SetGravityCompensationOnly(true);
    mInvDynamics.Update();
    mInvDynamics.GetTorques(mJointTorques);

    break;
	
    //  if(mState==1 || mState==3){
  case 1:
  case 3:
  
    /////// Inverse Kinematics ///////////////////////////////////


    // //put a safety limit on the speed:
    if(xd.Norm()>2 && !simulatorMode){
      xd*=2/xd.Norm();
      cout<<"safety limit reached!"<<endl;
    }
    //set the target velocities for IK
    target_vel[0] = xd[0];
    target_vel[1] = xd[1];
    target_vel[2] = xd[2];
    target_vel[3] = w_rotation[0];
    target_vel[4] = w_rotation[1];
    target_vel[5] = w_rotation[2];
    // Dealing with joint limits: Limiting output: Max 35 deg per seconds
    lim1 = DEG2RAD(-180.0);
    lim2 = DEG2RAD( 180.0);
    //printf("6-----------------------------------------------\n");
    for(int i=0;i<mRobot->GetDOFCount();i++){
      // Limiting speed when approaching 5 deg from joint range
      if(currPos[i]-mSensorsGroup.GetJointLimitsLow()(i)< DEG2RAD(5.0)){
        lim1[i] *= (currPos[i]-mSensorsGroup.GetJointLimitsLow()(i))/DEG2RAD(5.0);
      }else if(mSensorsGroup.GetJointLimitsHigh()(i)-currPos[i]<DEG2RAD(5.0)){
        lim2[i] *= (mSensorsGroup.GetJointLimitsHigh()(i)-currPos[i])/DEG2RAD(5.0); 
      }
    }
    if(simulatorMode) // THIS IS A MANUALLY SET LIMIT THAT SHOULD NORMALLY NOT BE HERE. ONLY ADDED FOR SIMULATION PURPOSES 
      lim2(3)=2;

    mIKSolver.SetJacobian(mKinematicChain.GetJacobian(),0);
    mIKSolver.SetLimits(lim1,lim2);
    // Def the weights of each dof
    weights=1;
    mIKSolver.SetDofsWeights(weights);
    // Set IK velocity target
    mIKSolver.SetTarget(target_vel,0);
    // Solve Inverse Kinematics
    mIKSolver.Solve();
    mJointTarget_vel=mIKSolver.GetOutput();

    double t;
    t = GetClock().GetTime() - command_time;
    if (2*t < (PI/2.0)){
      t  = cos(2*t);
      //      mJointTarget_vel *= 1.0-t;
      mJointTarget_vel.Mult(1.0-t,mJointTarget_vel);
    }
    //mJointTarget_acc=mJointTarget_vel-mJointTarget_acc;
    mJointTarget_vel.Sub(mJointTarget_acc,mJointTarget_acc);
    //  mJointTarget_acc/=GetClock().GetDt();
    mJointTarget_acc.Div(GetClock().GetDt(),mJointTarget_acc);

    //integrating to find joint target
    mJointTarget = mInternalSensorsGroup.GetJointAngles();
    mJointTarget_vel.ScaleAddTo(GetClock().GetDt(),mJointTarget);

    //save the joint target in the internal sensors 
    mInternalSensorsGroup.SetJointAngles(mJointTarget);
    // Applying changes to the internal model
    mInternalSensorsGroup.WriteSensors();
    mInternalRobot.UpdateLinks();

    mSensorsGroup.SetJointVelocities(mJointTarget_vel);
    mSensorsGroup.SetJointAccelerations(mJointTarget_acc);
    mSensorsGroup.WriteSensors();

    mInvDynamics.SetGravityCompensationOnly(false);
    mInvDynamics.SetFrictionCompensation(0,1.0);

    mInvDynamics.Update();
    mInvDynamics.GetTorques(mJointTorques);

    if(mState!=0){
      //set the joint angles as input
      mPIDCtrl.SetInput(mSensorsGroup.GetJointAngles());
      //set the target
      mPIDCtrl.SetTarget(mJointTarget);
      //Update the controller
      mPIDCtrl.Update(GetClock().GetDt());
      mJointTorques += mPIDCtrl.GetOutput();
    }

    break;

  case 4:

    eventID=5;
    double temp_time; 
    temp_time=GetClock().GetTime()-hit_time;//this is the time from the ball impact
    double temp_tmax;
    temp_tmax=0.5; //this is the time during which the new speed in z-axis is gradually applied 
    if(temp_time<temp_tmax){
      double br;
      br=(0.5+0.5*sin(temp_time/temp_tmax*PI-PI/2));
      mJointTarget_vel(1)=(1-br)*mJointTarget_vel(1)+br*hitting_gain*0.8;
      if(mRobot->GetType()=="KUKA")
	mJointTarget_vel(1)*=-1;

	  
    } 

    double tmax2;
    tmax2=2;
    double br2;
    br2=1-0.05*temp_time*temp_time*temp_time;
    if(br2<0.01)
      br2=0;


    mJointTarget_vel*=br2;

    //mJointTarget_acc=mJointTarget_vel-mJointTarget_acc;
    mJointTarget_vel.Sub(mJointTarget_acc,mJointTarget_acc);
    //  mJointTarget_acc/=GetClock().GetDt();
    mJointTarget_acc.Div(GetClock().GetDt(),mJointTarget_acc);

    //integrating to find joint target
    mJointTarget = mInternalSensorsGroup.GetJointAngles();
    mJointTarget_vel.ScaleAddTo(GetClock().GetDt(),mJointTarget);


    //save the joint target in the internal sensors 
    mInternalSensorsGroup.SetJointAngles(mJointTarget);
    // Applying changes to the internal model
    mInternalSensorsGroup.WriteSensors();
    mInternalRobot.UpdateLinks();


    mSensorsGroup.SetJointVelocities(mJointTarget_vel);
    mSensorsGroup.SetJointAccelerations(mJointTarget_acc);
    mSensorsGroup.WriteSensors();

    mInvDynamics.SetGravityCompensationOnly(false);
    mInvDynamics.SetFrictionCompensation(0,1.0);

    mInvDynamics.Update();
    mInvDynamics.GetTorques(mJointTorques);

    if(mState!=0){
      //set the joint angles as input
      mPIDCtrl.SetInput(mSensorsGroup.GetJointAngles());
      //set the target
      mPIDCtrl.SetTarget(mJointTarget);
      //Update the controller
      mPIDCtrl.Update(GetClock().GetDt());
      mJointTorques += mPIDCtrl.GetOutput();
    }

    break;
    {
    case 2:

      mJointTarget_vel=JointHomePos;
      mJointTarget_vel-=mSensorsGroup.GetJointAngles();
      double t;
      t = GetClock().GetTime() - command_time;
      if (0.5*t < (PI/2.0)){
        t  = cos(0.5*t);
        mJointTarget_vel *= 1.0-t;
      }
      if(mJointTarget_vel.Norm()>1)
        mJointTarget_vel/=mJointTarget_vel.Norm();

      //mJointTarget_acc=mJointTarget_vel-mJointTarget_acc;
      mJointTarget_vel.Sub(mJointTarget_acc,mJointTarget_acc);
      //  mJointTarget_acc/=GetClock().GetDt();
      mJointTarget_acc.Div(GetClock().GetDt(),mJointTarget_acc);

      //integrating to find joint target
      mJointTarget = mInternalSensorsGroup.GetJointAngles();
      mJointTarget_vel.ScaleAddTo(GetClock().GetDt(),mJointTarget);
      //save the joint target in the internal sensors 
      mInternalSensorsGroup.SetJointAngles(mJointTarget);
      // Applying changes to the internal model
      mInternalSensorsGroup.WriteSensors();
      mInternalRobot.UpdateLinks();

      mSensorsGroup.SetJointVelocities(mJointTarget_vel);
      mSensorsGroup.SetJointAccelerations(mJointTarget_acc);
      mSensorsGroup.WriteSensors();

      mInvDynamics.SetGravityCompensationOnly(true);
      mInvDynamics.SetFrictionCompensation(0,1.0);

      mInvDynamics.Update();
      mInvDynamics.GetTorques(mJointTorques);

      if(mState!=0){
        //set the joint angles as input
        mPIDCtrl.SetInput(mSensorsGroup.GetJointAngles());
        //set the target
        mPIDCtrl.SetTarget(mJointTarget);
        //Update the controller
        mPIDCtrl.Update(GetClock().GetDt());
        mJointTorques += mPIDCtrl.GetOutput();
      }
 
      break;
    }
  }
  //Set the torques
  mActuatorsGroup.SetJointTorques(mJointTorques);
  mActuatorsGroup.SetJointPositions(mJointTarget);
  //send orders to WAM
  mActuatorsGroup.WriteActuators();
  mState = nextState;

  EnableWrappers(false);
  return STATUS_OK;
}
		
		
extern "C"{
  // These two "C" functions manage the creation and destruction of the class
  GolfModule* create()                          {return new GolfModule();}
  void                 destroy(GolfModule* mod) {delete mod;}
}

