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

RobotInterface::Status GolfModule::RobotInit(){


  string Hitting_mod, Hitting_mod_v;
  //load config-file:
  XmlTree conf; 
  int check=conf.LoadFromFile("./data/packages/GolfPackage/Misc/GolfModule/config.xml");
  if(check==1){
    REALTYPE *array;
    //load the default hitting direction
    conf.GetArray("Default_dir",&array);
    Default_dir.Set(array,3);
    //load the directin of the dynamical system used for hitting motion
    conf.GetArray("Dynamic_dir",&array);
    Dynamic_dir.Set(array,3);
    //load ball position
    conf.GetArray("hitting_point",&array);
    hitting_point.Set(array,3);
    //load the direction of the gold club
    conf.GetArray("Racket_dir",&array);
    Racket_dir.Set(array,3);
    string expT,expP,balle;
    //expNbT=conf.Get("exploreNbTarget",0);
    //    expNbP=conf.Get("exploreNbParams",0);
    expT=conf.Get("exploreTargetPos",balle);
    expP=conf.Get("exploreParams",balle);
    exploreFileName=conf.Get("exploreFileName",balle);
    char txt[256];
    sprintf(txt,"./data/packages/GolfPackage/Misc/GolfModule/%s",expT.c_str());
    cout<<txt<<endl;
    exploreTargetPos.Load(txt);
    sprintf(txt,"./data/packages/GolfPackage/Misc/GolfModule/%s",expP.c_str());
    exploreParams.Load(txt);
    min_dist=10;


    cout<<exploreFileName<<endl;

    //    exploreTargetPos.Print();
    //    exploreParams.Print();

    exploreParamIter=0;
    exploreTargetIter=0;

    //    swing_dir.Set(array,3);
    //    swing_dir/=swing_dir.Norm();
    //load the dynamical system used for hitting motion
    string default_mod="DS_model.txt";
    Hitting_mod=conf.Get("Hitting_model",default_mod);
    string default_mod_v="DS_model.txt";
    Hitting_mod_v=conf.Get("Hitting_model_v",default_mod_v);
    cout<<Hitting_mod<<Hitting_mod_v<<endl;
   
    
    //load the intiial position(if so specified in conf file)
    bool li=conf.Get("manual_initial_position",false);
    if(li){
      conf.GetArray("Initial_point",&array);
      Initial_point.Set(array,3);
    }
   else{
      Initial_point.Resize(3);
      //first, set offset in negative default hitting direction. 
      Initial_point[0]=hitting_point[0]-0.5*Default_dir[0];  
      Initial_point[1]=hitting_point[1]-0.5*Default_dir[1];
      //secondly, set the offset in the vertical direction
      Initial_point[2]=hitting_point[2]+0.1;
    }

        
  }else
    cout<<"failed loading configuration file!"<<endl;
    
  //load the internal robot model:	
  if(mInternalRobot.Load(mRobot->GetType(),mRobot->GetSubType(),"")){
    cout << "Internal robot loaded as a copy of main robot"<<endl;
  }else{
    cout << "Failed loading internal robot"<<endl;
    exit(0);
  }

  // Sensors and actuators
  mSensorsGroup.SetSensorsList(mRobot->GetSensors());
  mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

  // Kinematic chain for inverse kinematics
  mKinematicChain.SetRobot(&mInternalRobot);
  mKinematicChain.Create(0,0,mInternalRobot.GetLinksCount()-1);

  // Inverse kinematics
  mIKSolver.SetSizes(mRobot->GetDOFCount());  // Dof counts
  mIKSolver.AddSolverItem(6);                 // One solver with 6 constraints (x,y,z, wx, wy, wz)
  mIKSolver.SetVerbose(false);                // No comments
  mIKSolver.SetThresholds(0.0001,0.00001);    // Singularities thresholds
  mIKSolver.Enable(true,0);                   // Enable first solver
  mIKSolver.SetDofsIndices(mKinematicChain.GetJointMapping(),0); // Joint maps for first solver

  // Inverse dynamics
  mInvDynamics.SetRobot(mRobot);
  mInvDynamics.Init();
  mInvDynamics.SetGravityCompensationOnly(true);
	
  //Setup some vectors used for robot control
  mJointTorques.Resize(mRobot->GetDOFCount());
  mJointTarget.Resize(mRobot->GetDOFCount());

  mJointSetPos.Resize(mRobot->GetDOFCount());
  //setup the vector containing the WAM torque limits 
  TorqueLimits.Resize(7);
  TorqueLimits(0)=15;
  TorqueLimits(1)=25;
  TorqueLimits(2)=8;
  TorqueLimits(3)=8;
  TorqueLimits(4)=2;
  TorqueLimits(5)=3;
  TorqueLimits(6)=2;

	
  // load the PID parameters
  XmlTree tree; 
  tree.LoadFromFile("./data/packages/WAMRobotModel/Misc/WAMDefaultPID.xml");
  mPIDCtrl.Init(&tree);
	
  //Setup internal sensors and actuators
  mInternalSensorsGroup.SetSensorsList(mInternalRobot.GetSensors());
  mInternalActuatorsGroup.SetActuatorsList(mInternalRobot.GetActuators());

  //set the size of some vectors and matrices	
  lim1.Resize(7); //vector used to limit the joint velocities when approaching joint limits
  lim2.Resize(7); //;;
  weights.Resize(7); //vector used to set weights of the joints, ie how much should each joint be moved relatively to the others
  rest.Resize(7); //vector used for null-space determination of the joint angles
  targetCart.Resize(3);//vector used to track the endeffector
  targetCart_last.Resize(3);//;;
  xd.Resize(3);//vector used to determine the desired velocity of the endeffector
  Racket_dir_proj.Resize(3);
  target_vel.Resize(6); //used to send the target velocity to inverse kinematics

  mJointTarget_vel.Resize(7); //jointarget velocities and accelerations
  mJointTarget_acc.Resize(7);

  target_offset.Resize(3); //the offset distance to hit the ball
  target_offset=Default_dir;
  target_offset*=-0.01/Default_dir.Norm();
  R.Resize(3,3); //Rotation Matrix
  R.Identity();// initialize R

  Target_pos.Resize(3); //target position, ie the position of the hole 
  Target_pos.Resize(3);


  currPos.Resize(7);//vetor used to store the current joint angles
  w_rotation.Resize(3);//rotation vector used to align the racket perpendiculairly to the hitting direction
  T_L_G.Resize(3,3);//matrix that transforms oordinates from racket to global

  // Dynamical System stuff
  char txt[256];
  sprintf(txt,"./data/packages/GolfPackage/Misc/GolfModule/%s",Hitting_mod.c_str());
  DS_model.loadParams(txt);
  sprintf(txt,"./data/packages/GolfPackage/Misc/GolfModule/%s",Hitting_mod_v.c_str());
  DS_model_v.loadParams(txt);
  xd_switching.Resize(3); //this keeps the velocity value at the switching point - used in conjunction with non-zero final velocity

  hitting_dir.Resize(3);//direction of hitting
	
  mState = 0;//begin in state 0

  simulatorMode=true; //this can be chaged on command, to set the operation to simulator mode
  
  exploreMode=false;
  exploreclock=0;
  rw=false;

  R_DS.Resize(3,3);
  R_DS = Compute_Rotation_Matrix(Dynamic_dir, Default_dir); //initialize the rotation used for regression. 
  Dynamic_dir.Print();
  Default_dir.Print();
  R_DS.Print();

  hitting_dir = Default_dir; // initialize hitting_dir to the default hitting dir.

  if(simulatorMode){
    pWorldObject obj = GetWorld()->Find("Ball");
    Ball_pos=obj->GetReferenceFrame().GetOrigin();
    obj = GetWorld()->Find("hole");
    Target_pos=obj->GetReferenceFrame().GetOrigin();
    Target_pos_original=Target_pos;
  }

  eventID=0;// event id is an integer used to specify what action the robot is doing at each time.

  hitting_gain=1;
  beta=0; //hitting angle, user input variable
	
  //initialize the adjustData matrix, matrix used to store demonstrations of beta and hitting_gain:
  adjustData.Resize(25,4);
  demoCount=0;

  autoadjust=false;


  InitializeCommands();//this initializes the commands that can be used etc. 

  hpball=false; //per default the hitting_point should be the ball position. this is ok for simulator and low speed real robot hitting motions but should be avoided for high speed hits for security reasons. The hitting point can instead be set by going to idle mode, showing the hittinh point by manually moving the end effector and then typing hp set
  
  JointHomePos.Resize(7);
  JointHomePos.Zero();
  JointHomePos(1)=-2;
  JointHomePos(3)=3;

  
  return STATUS_OK;
}
