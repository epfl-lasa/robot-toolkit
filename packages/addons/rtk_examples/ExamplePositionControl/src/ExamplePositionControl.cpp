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

#include "ExamplePositionControl.h"


ExamplePositionControl::ExamplePositionControl()
:RobotInterface(){
}
ExamplePositionControl::~ExamplePositionControl(){
}

RobotInterface::Status ExamplePositionControl::RobotInit(){
    AddConsoleCommand("MoveLeft");
    AddConsoleCommand("MoveRight");
    AddConsoleCommand("Reset");
    AddConsoleCommand("Freeze");

    mSensorsGroup.SetSensorsList(mRobot->GetSensors());
    mActuatorsGroup.SetActuatorsList(mRobot->GetActuators());

    mCurrentPosition.Resize(mRobot->GetDOFCount());

    mLeftIndex  = mRobot->GetDOFIndex("L_SFE");
    mRightIndex = mRobot->GetDOFIndex("R_SFE");

    mState = 0;

    return STATUS_OK;
}
RobotInterface::Status ExamplePositionControl::RobotFree(){
    return STATUS_OK;
}
RobotInterface::Status ExamplePositionControl::RobotStart(){
    mSensorsGroup.ReadSensors();
    mRestPosition = mSensorsGroup.GetJointPositions();

    return STATUS_OK;
}    
RobotInterface::Status ExamplePositionControl::RobotStop(){
    return STATUS_OK;
}
RobotInterface::Status ExamplePositionControl::RobotUpdate(){
    return STATUS_OK;
}
RobotInterface::Status ExamplePositionControl::RobotUpdateCore(){
    mSensorsGroup.ReadSensors();

    mCurrentPosition = mSensorsGroup.GetJointPositions();

    switch(mState){
    case 0:
        mCurrentPosition = mRestPosition;
        break;
    case 1:
        if(mLeftIndex>=0){
            mCurrentPosition[mLeftIndex] = PI/3.0*sin(GetClock().GetTime());
            mCurrentPosition[mLeftIndex+3] = -PI/2.0+PI/6.0*sin(GetClock().GetTime()+1.0);
        }
        break;
    case 2:
        if(mRightIndex>=0){
            mCurrentPosition[mRightIndex] = PI/3.0*sin(GetClock().GetTime());
            mCurrentPosition[mRightIndex+3] = -PI/2.0+PI/6.0*sin(GetClock().GetTime()+1.0);
        }
        break;
    case 3:
        //mActuatorsGroup.SetJointPositions(mCurrentPosition);
        break;
    }
    mActuatorsGroup.SetJointPositions(mCurrentPosition);
    mActuatorsGroup.WriteActuators();

    return STATUS_OK;
}
int ExamplePositionControl::RespondToConsoleCommand(const string cmd, const vector<string> &args){

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
}


extern "C"{
    // These two "C" functions manage the creation and destruction of the class
    ExamplePositionControl* create(){return new ExamplePositionControl();}
    void destroy(ExamplePositionControl* module){delete module;}
}

