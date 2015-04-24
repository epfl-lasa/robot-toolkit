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

#include "ExampleOpenLoopPIDControlModule.h"


ExampleOpenLoopPIDControlModule::ExampleOpenLoopPIDControlModule()
:RobotInterface(){
}
ExampleOpenLoopPIDControlModule::~ExampleOpenLoopPIDControlModule(){
}

RobotInterface::Status ExampleOpenLoopPIDControlModule::RobotInit(){

    // Sensors and actuators
    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());


    mJointTorques.Resize(mRobot->GetDOFCount());
    mJointTarget.Resize(mRobot->GetDOFCount());
    //mJointTorques.Print();



    if(mInternalRobot.Load(mRobot->GetType(),mRobot->GetSubType(),"")){
        cout << "Internal robot loaded as a copy of main robbot"<<endl;
    }else{
        cout << "Failed loading internal robot"<<endl;
        exit(0);
    }


    mInternalSensorsGroup.SetSensorsList(mInternalRobot.GetSensors());
    mInternalActuatorsGroup.SetActuatorsList(mInternalRobot.GetActuators());

    // Inverse dynamics
    mInvDynamics.SetRobot(mRobot);
    mInvDynamics.Init();
    mInvDynamics.SetGravityCompensationOnly(true);

    XmlTree tree;
    tree.LoadFromFile("./data/packages/WAMRobotModel/Misc/WAMDefaultPID.xml");
    mPIDCtrl.Init(&tree);

    mState  = 0;


    if(GetConsole()){
        AddConsoleCommand("Rest");
        AddConsoleCommand("Hit");
        AddConsoleCommand("GComp");
        GetConsole()->Print("Available commands are GComp Rest and Hit");
    }

    return STATUS_OK;
}
RobotInterface::Status ExampleOpenLoopPIDControlModule::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status ExampleOpenLoopPIDControlModule::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status ExampleOpenLoopPIDControlModule::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status ExampleOpenLoopPIDControlModule::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status ExampleOpenLoopPIDControlModule::RobotUpdateCore(){

    mSensorsGroup.ReadSensors();

    /////////////////////////////
    // Internal model update
    /////////////////////////////

    // Getting internal state
    mInternalSensorsGroup.ReadSensors();
    mJointTarget = mInternalSensorsGroup.GetJointAngles();

    // Updating some joints
    if(mState <2){
        mJointTarget(0) = 0;
        mJointTarget(1) = PI/4;
        mJointTarget(3) = 3*PI/4+PI/6;
        mStartTime = GetClock().GetTime();
    }else{
        mJointTarget(0) = 0;
        mJointTarget(1) = PI/4;
        mJointTarget(3) = 3*PI/4+PI/6*(cos(10*(GetClock().GetTime()-mStartTime)));
    }
    // Setting the joints 
    // Only in position: if you want some velocity control here,
    // you should do the integration yourself
    mInternalSensorsGroup.SetJointAngles(mJointTarget);
    // Applying changes to the internal model
    mInternalSensorsGroup.WriteSensors();
    mInternalRobot.UpdateLinks();

    // Btw, forward knematic can aslo be used here
    //mInternalRobot.GetReferenceFrame(mInternalRobot.GetLinksCount()-1,0).GetOrigin().Print();
    // to compare with the real robot position
    //mRobot->GetReferenceFrame(mRobot->GetLinksCount()-1,0).GetOrigin().Print();


    // General update part to the real robot

    switch(mState){
    case 0: // Idle
        mJointTarget = mSensorsGroup.GetJointAngles();

        mPIDCtrl.SetInput(mJointTarget);
        mPIDCtrl.SetTarget(mJointTarget);
        mPIDCtrl.Start();

        mInvDynamics.SetGravityCompensationOnly(true);
        mInvDynamics.Update();
        mInvDynamics.GetTorques(mJointTorques);
        mActuatorsGroup.SetJointTorques(mJointTorques);
        break;
    case 1: // PID
    case 2: // PID

        mInvDynamics.SetGravityCompensationOnly(true);
        mInvDynamics.Update();
        mInvDynamics.GetTorques(mJointTorques);

        mJointTarget = mInternalSensorsGroup.GetJointAngles();

        mPIDCtrl.SetInput(mSensorsGroup.GetJointAngles());
        mPIDCtrl.SetTarget(mJointTarget);
        mPIDCtrl.Update(GetClock().GetDt());

        mJointTorques += mPIDCtrl.GetOutput();
        mActuatorsGroup.SetJointTorques(mJointTorques);
    }


    // Sends order to robot
    mActuatorsGroup.WriteActuators();

    return STATUS_OK;
}

int ExampleOpenLoopPIDControlModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    if(cmd=="GComp")
        mState = 0;
    else if(cmd=="Rest")
        mState = 1;
    else if(cmd=="Hit")
        mState = 2;
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    ExampleOpenLoopPIDControlModule* create(){return new ExampleOpenLoopPIDControlModule();}
    void destroy(ExampleOpenLoopPIDControlModule* module){delete module;}
}

