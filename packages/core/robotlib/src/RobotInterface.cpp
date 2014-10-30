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

#include "RobotInterface.h"
#include "Robot.h"

RobotInterface::RobotInterface(bool isControlInterface)
:ModuleInterface(isControlInterface){
    mRobot              = NULL;
    mWorld              = NULL;
}

RobotInterface::~RobotInterface(){
}
void RobotInterface::SetRobot(Robot* robot){
    if(GetSystemState() == SYSSTATE_NONE){
        mRobot = robot;
        //SetName(mRobot->GetName());
    }
}
void  RobotInterface::SetWorld(World* world){
    if(GetSystemState() == SYSSTATE_NONE)
        mWorld = world;
}


RobotInterface::Status RobotInterface::InterfaceFree(){
    Status res = RobotFree();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Free();
        }
    }
    return res;
}

RobotInterface::Status RobotInterface::InterfaceInit(){
    Status res = STATUS_ERROR;
    if((mRobot!=NULL)&&(mWorld!=NULL)){
        res = RobotInit();
    }
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Init();
        }
    }
    return res;
}

RobotInterface::Status RobotInterface::InterfaceStart(){
    Status res = STATUS_ERROR;
    res = RobotStart();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Start();
        }
    }
    return res;
}

RobotInterface::Status RobotInterface::InterfaceStop(){
    Status res = STATUS_ERROR;
    res = RobotStop();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Stop();
        }
    }
    return res;
}


RobotInterface::Status RobotInterface::InterfaceUpdate(){
    Status res = STATUS_ERROR;
    res = RobotUpdate();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->Update();
        }
    }
    return res;
}
RobotInterface::Status RobotInterface::InterfaceUpdateCore(){
    Status res = STATUS_ERROR;
    if(IsControlInterface())
        UpdateSensors();
    res = RobotUpdateCore();
    if(res==STATUS_OK){
        if(IsControlInterface()){
            mWorld->UpdateCore();
            UpdateActuators();
        }
    }
    return res;
}
void RobotInterface::InterfaceDraw(){
    RobotDraw();
    if(IsControlInterface()){
        mWorld->Draw();
    }
}
            
Robot* RobotInterface::GetRobot(){
    return mRobot;
}
World* RobotInterface::GetWorld(){
    return mWorld;
}



RobotInterface::Status RobotInterface::RobotInit()             {return STATUS_OK;}
RobotInterface::Status RobotInterface::RobotFree()             {return STATUS_OK;}
RobotInterface::Status RobotInterface::RobotStart()            {return STATUS_OK;}
RobotInterface::Status RobotInterface::RobotStop()             {return STATUS_OK;}
RobotInterface::Status RobotInterface::RobotUpdate()           {return STATUS_OK;}
RobotInterface::Status RobotInterface::RobotUpdateCore()       {return STATUS_OK;}
void                   RobotInterface::RobotDraw()             {}

void RobotInterface::UpdateActuators(){}
void RobotInterface::UpdateSensors(){}
int RobotInterface::RespondToConsoleCommand(const string cmd, const vector<string> &args){
    return 0;
}

