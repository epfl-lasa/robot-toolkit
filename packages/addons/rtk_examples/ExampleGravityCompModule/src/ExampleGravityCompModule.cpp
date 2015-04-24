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

#include "ExampleGravityCompModule.h"


ExampleGravityCompModule::ExampleGravityCompModule()
:RobotInterface(){
}
ExampleGravityCompModule::~ExampleGravityCompModule(){
}

RobotInterface::Status ExampleGravityCompModule::RobotInit(){

    AddConsoleCommand("GCompOn");
    AddConsoleCommand("GCompOff");
    // Sensors and actuators
    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

    // Inverse dynamics
    mInvDynamics.SetRobot(mRobot);
    mInvDynamics.Init();
    mInvDynamics.SetGravityCompensationOnly(true);

    mJointTorques.Resize(mRobot->GetDOFCount());

    bGComp = true;

    return STATUS_OK;
}
RobotInterface::Status ExampleGravityCompModule::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status ExampleGravityCompModule::RobotStart(){
    return STATUS_OK;
}    
RobotInterface::Status ExampleGravityCompModule::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status ExampleGravityCompModule::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status ExampleGravityCompModule::RobotUpdateCore(){

    mSensorsGroup.ReadSensors();

    if(bGComp){
        mInvDynamics.SetGravityCompensationOnly(true);

        mInvDynamics.Update();
        mInvDynamics.GetTorques(mJointTorques);
    }else{
        mJointTorques.Zero();
    }

    // Sends order to robot
    mActuatorsGroup.SetJointTorques(mJointTorques);
    mActuatorsGroup.WriteActuators();

    return STATUS_OK;
}
int ExampleGravityCompModule::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    if(cmd=="GCompOn"){
        GetConsole()->Print("Gravity compensation On");
        bGComp = true;
    }else if(cmd=="GCompOff"){
        GetConsole()->Print("Gravity compensation Off");
        bGComp = false;
    }
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    ExampleGravityCompModule* create(){return new ExampleGravityCompModule();}
    void destroy(ExampleGravityCompModule* module){delete module;}
}

