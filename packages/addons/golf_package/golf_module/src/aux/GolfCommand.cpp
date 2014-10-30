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
void GolfModule::InitializeCommands(){
  // Did someone give us a console?
  if(GetConsole()){
    // The simplest way to add a command that will be sent in RespondToConsoleCommand
    AddConsoleCommand("t"); //to start hitting motion
    AddConsoleCommand("i");//to go to inital position
    AddConsoleCommand("b");//specify angle and magnitude
    AddConsoleCommand("h");//get help

    AddConsoleCommand("o"); //go to gravity compensation mode
    AddConsoleCommand("target"); //to specify a target position
    AddConsoleCommand("use");//to use current setting as traing data
    AddConsoleCommand("learn");//use saved data to learn a model    
    AddConsoleCommand("load");// load a  data set    
    AddConsoleCommand("adjust");// estimate beta and hitting_gain for current target    
    AddConsoleCommand("clear");// clear the current data set    
    AddConsoleCommand("saveModel");// save the trained model. 
    
    AddConsoleCommand("simulator");// set operating mode to simulator
    AddConsoleCommand("save");// save current training data to file
    AddConsoleCommand("p");// print the current end effector position
    AddConsoleCommand("ball");// to specify ball position
    AddConsoleCommand("initial");// to specify initial position
    AddConsoleCommand("offset");// to specify angular offset

    AddConsoleCommand("hp");// to read the hitting point from the end effector
    AddConsoleCommand("home");// to read the hitting point from the end effector

    AddConsoleCommand("explore");//to start the exploring mode. specify files containg target positions and paramter combinations in the config file. 
    // Let's print out some stuff on screem
    GetConsole()->Print("---GolfModule---");
    GetConsole()->Print("type <h> for help");
  }

}


int GolfModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){

  // Answering to console command
  if(GetConsole()){
    if(cmd=="t"){
      mState=3;
      command_time=GetClock().GetTime();
    }else if(cmd=="home"){
      if(args.size()>0)
        {
          if(args[0]=="start")
            {
              JointHomePos(0)=0.532912;
              JointHomePos(1)= -0.824358;
              JointHomePos(2)= -2.221722;
              JointHomePos(3)=   1.600284;
              JointHomePos(4)= -1.154692;
              JointHomePos(5)= 0.970712;
              JointHomePos(6) = -0.366080;
            }
          if(args[0]=="home")
            {
              JointHomePos.Resize(7);
              JointHomePos.Zero();
              JointHomePos(1)=-2;
              JointHomePos(3)=3;

            }
	  
	  
        }

      GetConsole()->Print("Returning to home position");
      mState=2;
      command_time=GetClock().GetTime();

    }else if(cmd=="i"){
      mState = 1;
      command_time=GetClock().GetTime();
    }else if(cmd=="o"){
      mState = 0;
    }else if(cmd=="target"){
      if(args.size()==2 && !simulatorMode){
        Target_pos(0)=atof(args[0].c_str());
        Target_pos(1)=atof(args[1].c_str());
      }
      else if(args.size()>0 && simulatorMode){
        if(args.size()==2){
          Target_pos(0)=Target_pos_original(0)+sin(PI/4)*atof(args[0].c_str())+sin(3*PI/4)*atof(args[1].c_str());
          Target_pos(1)=Target_pos_original(1)+cos(PI/4)*atof(args[0].c_str())+cos(3*PI/4)*atof(args[1].c_str());
        }
        if(args.size()==3){
          Target_pos(0)=atof(args[0].c_str());
          Target_pos(1)=atof(args[1].c_str());
        }
        else if(args.size()==1){
          Target_pos(0)=Target_pos_original(0)+sin(PI/4)*atof(args[0].c_str());
          Target_pos(1)=Target_pos_original(1)+cos(PI/4)*atof(args[0].c_str());
        }
        updateTargetPosition();
      }
      char txt[256];
      sprintf(txt,"Target position is (%.3f ; %.3f) ",Target_pos(0),Target_pos(1));
      GetConsole()->Print(txt);
    }else if(cmd=="ball"){
      if(simulatorMode && args.size()==3){
        Ball_pos(0)=atof(args[0].c_str());
        Ball_pos(1)=atof(args[1].c_str());
        Ball_pos(2)=atof(args[2].c_str());
      }
      //if the robot is in idle mode, and not in simulator mode, the ball psoition can be set to the current endeffector position.
      if(mState==0 && args.size()>0 &&args[0].compare("set")==0 && !simulatorMode){ 
        Ball_pos=mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin();
      }
      char txt[256];
      sprintf(txt,"Ball position is (%.3f ; %.3f ; %.3f) ",Ball_pos(0),Ball_pos(1),Ball_pos(2));
      GetConsole()->Print(txt);
    }else if(cmd=="hp"){
      //if the robot is in idle mode, and not in simulator mode, the ball psoition can be set to the current endeffector position.
      if(mState==0 && args.size()>0 &&args[0].compare("set")==0){ 
        hitting_point=mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin();
      }else if(args.size()>0 && args[0].compare("ball")==0){
        hitting_point=Ball_pos;
      }
      char txt[256];
      sprintf(txt,"hitting point is (%.3f ; %.3f ; %.3f) ",hitting_point(0),hitting_point(1),hitting_point(2));
      GetConsole()->Print(txt);
    }else if(cmd=="initial"){
    if(simulatorMode && args.size()==3){
      Initial_point(0)=atof(args[0].c_str());
      Initial_point(1)=atof(args[1].c_str());
      Initial_point(2)=atof(args[2].c_str());
    }
    //if the robot is in idle mode, and not in simulator mode, the initial psoition can be set to the current endeffector position.
    if(mState==0 && args.size()>0 &&args[0].compare("set")==0 && !simulatorMode){ 
      Initial_point=mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin();

    }

    char txt[256];
    sprintf(txt,"Initialposition is (%.1f ; %.1f ; %.1f) ",Initial_point(0),Initial_point(1),Initial_point(2));
    GetConsole()->Print(txt);

    }else if(cmd=="use"){
    addCurrent();
  }else if(cmd=="learn"){
    if(args.size()<1){
      learn(1);
    }
    else{
      learn(atoi(args[0].c_str()));
    }
  }else if(cmd=="simulator"){
    simulatorMode=!simulatorMode; //toggle simulatormode
    if(simulatorMode){ //when simulator mode is activated the simulator should look for the ball
      GetConsole()->Print("Simulator mode activated");
      pWorldObject obj = GetWorld()->Find("Ball");
      Ball_pos=obj->GetReferenceFrame().GetOrigin();
      obj = GetWorld()->Find("hole");
      Target_pos=obj->GetReferenceFrame().GetOrigin();
      Target_pos_original=Target_pos; //needed for moving the target along a specific axis
    }
    else
      GetConsole()->Print("Simulator mode deactivated");
      
    }else if(cmd=="load"){
    if(args.size()<2)
      GetConsole()->Print("no filename or democount specified. Load failed.");
    else{
      loadTrainingData(args[0].c_str(),atoi(args[1].c_str()));
    }
  }else if(cmd=="save"){
    if(args.size()<1)
      GetConsole()->Print("no filename specified. Save failed.");
    else{
      saveTrainingData(args[0].c_str());
    }
  }else if(cmd=="saveModel"){
    if(args.size()<1)
      GetConsole()->Print("no filename specified. Save failed.");
    else{
      saveModel(args[0].c_str());
    }
  }else if(cmd=="clear"){
    clearTrainingData();
  }else if(cmd=="adjust"){
    if(args.size()>0 && args[0].compare("auto")==0 )
      autoadjust=!autoadjust;
    adjust();
  }
  else if(cmd=="b"){ //read angle beta from args
    double maxspeed;
    maxspeed=1.2;

    if(args.size()>1){
      beta=DEG2RAD(atof(args[0].c_str()));
      if(atof(args[1].c_str())<=150)
        hitting_gain=atof(args[1].c_str())/100*maxspeed;
      changeHittingDirection();
    }
    char txt[256];
    sprintf(txt,"current settings: %f degrees",RAD2DEG(beta));
    GetConsole()->Print(txt);
    sprintf(txt,"and a gain of %f",hitting_gain*100/maxspeed);
    GetConsole()->Print(txt);



  }else if(cmd=="h"){
    GetConsole()->Print("<i> return to initial position");
    GetConsole()->Print("<t> hit ball, <r> start recording");
    GetConsole()->Print("<e> to stop recording");
    GetConsole()->Print("<b arg0 arg1> change hitting angle to arg0 degrees");
    GetConsole()->Print("and hitting gain to arg1");
  }else if(cmd=="p"){
    mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin().Print();
  }else if(cmd=="offset"){
    if(args.size()>0)
      angular_offset=DEG2RAD(atof(args[0].c_str()));
    char txt[256];
    sprintf(txt,"angular offset is: %.1f",RAD2DEG(angular_offset));
    GetConsole()->Print(txt);
    changeHittingDirection();
  }else if(cmd=="explore"){
    exploreMode=!exploreMode;
    char txt[256];
    if (exploreMode)
      {
        sprintf(txt,"explore mode on");
      }
    else
      {
        sprintf(txt,"explore mode off");
      }
    GetConsole()->Print(txt);
  }
}
return 0;
}


