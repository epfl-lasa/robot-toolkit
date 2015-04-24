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

#include "ExampleVelocityControl.h"


ExampleVelocityControl::ExampleVelocityControl()
:RobotInterface(){
}
ExampleVelocityControl::~ExampleVelocityControl(){
}

RobotInterface::Status ExampleVelocityControl::RobotInit(){
    AddConsoleCommand("MoveLeft");
    AddConsoleCommand("MoveRight");
    AddConsoleCommand("Reset");
    AddConsoleCommand("Freeze");

    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

    mCurrentPosition.Resize(mRobot->GetDOFCount());

    mLeftIndex  = mRobot->GetDOFIndex("LArm_0");
    mRightIndex = mRobot->GetDOFIndex("RArm_0");
    mState = 0;
    return STATUS_OK;
}
RobotInterface::Status ExampleVelocityControl::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status ExampleVelocityControl::RobotStart(){
    mSensorsGroup.ReadSensors();
    mRestPosition = mSensorsGroup.GetJointPositions();
    return STATUS_OK;
}    
RobotInterface::Status ExampleVelocityControl::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status ExampleVelocityControl::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status ExampleVelocityControl::RobotUpdateCore(){
    mSensorsGroup.ReadSensors();

    mCurrentPosition = mSensorsGroup.GetJointPositions();
    Vector velocity(mRobot->GetDOFCount());
    velocity.Zero();

    Vector target(mRestPosition);

    switch(mState){
    case 0:
        target = mRestPosition;
        break;
    case 1:
        if(mLeftIndex>=0){
            target[mLeftIndex] = -PI/3.0+PI/3.0*sin(GetClock().GetTime());
            target[mLeftIndex+3] = PI/6.0+PI/6.0*sin(GetClock().GetTime()+1.0);
        }
        break;
    case 2:
        if(mRightIndex>=0){
            target[mRightIndex] = -PI/3.0+PI/3.0*sin(GetClock().GetTime());
            target[mRightIndex+3] = PI/6.0+PI/6.0*sin(GetClock().GetTime()+1.0);
        }
        break;
    case 3:
        target = mCurrentPosition;
        break;
    }

    velocity = (target-mCurrentPosition)*2.0;
    mActuatorsGroup.SetJointVelocities(velocity);
    mActuatorsGroup.WriteActuators();

    return STATUS_OK;
}
int ExampleVelocityControl::RespondToConsoleCommand(const string cmd, const vector<string> &args){

    if(cmd=="MoveLeft"){
        mState = 1;
    }else if(cmd=="MoveRight"){
        mState = 2;
    }else if(cmd=="Reset"){
        mState = 0;
    }else if(cmd=="Freeze"){
        mState = 3;
    }
    return 0;
    return 0;
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    ExampleVelocityControl* create(){return new ExampleVelocityControl();}
    void destroy(ExampleVelocityControl* module){delete module;}
}

